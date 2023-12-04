#include <ep_training/campaign_orchestrator_gui.hpp>
#include <ep_training/campaign_orchestrator_gui_p.hpp>
#include "ui_campaign_orchestrator_gui.h"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

#include <QApplication>

namespace {
    std::vector<std::string> csv_to_vector(const QString& csv)
    {
        if (csv.isEmpty())
        {
            return {};
        }

        const QStringList values_qstring = csv.split(",");
        std::vector<std::string> values_stdstring(values_qstring.size());
        std::transform(
            values_qstring.cbegin(),
            values_qstring.cend(),
            values_stdstring.begin(),
            [](const QString& v){ return v.toStdString(); }
        );
        return values_stdstring;
    };
}

namespace ep_training {

CampaignOrchestratorGuiPrivate::CampaignOrchestratorGuiPrivate(CampaignOrchestratorGui* const parent) :
    ui(std::make_unique<Ui::CampaignOrchestratorGui>()),
    node(std::make_shared<rclcpp::Node>("campaign_orchestrator")),
    execute_training_campaign_acl(rclcpp_action::create_client<CampaignOrchestratorGui::TrainAction>(
        node.get(),
        "execute_training_campaign"
    )),
    handle_campaign_goal_response_ros_thread_callback_func(std::bind(
        &ep_training::CampaignOrchestratorGui::handle_campaign_goal_response_ros_thread_callback,
        parent,
        std::placeholders::_1
    )),
    handle_campaign_feedback_ros_thread_callback_func(std::bind(
        &ep_training::CampaignOrchestratorGui::handle_campaign_feedback_ros_thread_callback,
        parent,
        std::placeholders::_1,
        std::placeholders::_2
    )),
    handle_campaign_result_ros_thread_callback_func(std::bind(
        &ep_training::CampaignOrchestratorGui::handle_campaign_result_ros_thread_callback,
        parent,
        std::placeholders::_1
    ))
{
    qRegisterMetaType<TrainActionGoalHandle::SharedPtr>("TrainActionGoalHandle::SharedPtr");
    qRegisterMetaType<std::shared_ptr<const TrainAction::Feedback>>("std::shared_ptr<const TrainAction::Feedback>");
    qRegisterMetaType<TrainActionGoalHandle::WrappedResult>("TrainActionGoalHandle::WrappedResult");
}

CampaignOrchestratorGui::CampaignOrchestratorGui() :
    CampaignOrchestratorGui(*(new CampaignOrchestratorGuiPrivate(this)))
{
}

CampaignOrchestratorGui::CampaignOrchestratorGui(CampaignOrchestratorGuiPrivate& d_) :
    QMainWindow(),
    d_ptr(&d_)
{
    Q_D(CampaignOrchestratorGui);

    d->ui->setupUi(this);

    // Connect signals emitted in the ROS thread to Qt slots, so work can be
    // done to the GUI in the Qt event loop
    connect(
        this, &ep_training::CampaignOrchestratorGui::announce_campaign_goal_response,
        this, &ep_training::CampaignOrchestratorGui::handle_campaign_goal_response_gui_thread_callback
    );
    connect(
        this, &ep_training::CampaignOrchestratorGui::announce_campaign_feedback,
        this, &ep_training::CampaignOrchestratorGui::handle_campaign_feedback_gui_thread_callback
    );
    connect(
        this, &ep_training::CampaignOrchestratorGui::announce_campaign_result,
        this, &ep_training::CampaignOrchestratorGui::handle_campaign_result_gui_thread_callback
    );

    // Connect internal signals+slots
    connect(
        d->ui->start, &QPushButton::clicked,
        this,         &ep_training::CampaignOrchestratorGui::send_campaign_goal
    );
    connect(
        d->ui->cancel, &QPushButton::clicked,
        this,          &ep_training::CampaignOrchestratorGui::cancel_campaign_goal
    );
}

CampaignOrchestratorGui::~CampaignOrchestratorGui()
{
}

std::shared_ptr<rclcpp::Node> CampaignOrchestratorGui::get_ros_node()
{
    Q_D(CampaignOrchestratorGui);
    return d->node;
}

void CampaignOrchestratorGui::send_campaign_goal()
{
    Q_D(CampaignOrchestratorGui);

    if (!d->execute_training_campaign_acl->wait_for_action_server(std::chrono::duration<int64_t, std::milli>(1000)))
    {
        RCLCPP_ERROR_STREAM(d->node->get_logger(), "Campaign action server not available after waiting!");
        return;
    }

    auto campaign_goal_msg = CampaignOrchestratorGui::TrainAction::Goal();
    campaign_goal_msg.generation_solutions = d->ui->generation_solutions->value();
    campaign_goal_msg.convergence_threshold = d->ui->convergence_threshold->value();
    campaign_goal_msg.generation_limit = d->ui->generation_limit->value();
    campaign_goal_msg.solution_time_limit = d->ui->solution_time_limit->value();
    campaign_goal_msg.crossover_offspring = d->ui->crossover_offspring->value();
    campaign_goal_msg.mutability = d->ui->mutability->value();
    campaign_goal_msg.command_interface_names = csv_to_vector(d->ui->command_interface_names->text());
    campaign_goal_msg.state_interface_names = csv_to_vector(d->ui->state_interface_names->text());
    campaign_goal_msg.desired_parallel_agent_count.value = d->ui->desired_parallel_agent_count->value();
    campaign_goal_msg.parameters_file_url = d->ui->parameters_file_url->text().toStdString();

    auto campaign_send_goal_options = rclcpp_action::Client<TrainAction>::SendGoalOptions();
    campaign_send_goal_options.goal_response_callback = d->handle_campaign_goal_response_ros_thread_callback_func;
    campaign_send_goal_options.feedback_callback = d->handle_campaign_feedback_ros_thread_callback_func;
    campaign_send_goal_options.result_callback = d->handle_campaign_result_ros_thread_callback_func;

    RCLCPP_INFO_STREAM(d->node->get_logger(), "Sending campaign goal!");
    d->execute_training_campaign_acl->async_send_goal(
        campaign_goal_msg,
        campaign_send_goal_options
    );
}

void CampaignOrchestratorGui::cancel_campaign_goal()
{
    // TODO: prompt confirmation message then cancel if so
}

void CampaignOrchestratorGui::handle_campaign_goal_response_ros_thread_callback(
    CampaignOrchestratorGui::TrainActionGoalHandle::SharedPtr future
)
{
    emit announce_campaign_goal_response(future);

    // TODO: handle campaign goal response callback in ROS thread
}

void CampaignOrchestratorGui::handle_campaign_feedback_ros_thread_callback(
    CampaignOrchestratorGui::TrainActionGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const TrainAction::Feedback> feedback
)
{
    emit announce_campaign_feedback(goal_handle, feedback);

    // TODO: handle campaign feedback callback in ROS thread
}

void CampaignOrchestratorGui::handle_campaign_result_ros_thread_callback(
    const CampaignOrchestratorGui::TrainActionGoalHandle::WrappedResult& result
)
{
    emit announce_campaign_result(result);

    // TODO: handle campaign result callback in ROS thread
}

void CampaignOrchestratorGui::handle_campaign_goal_response_gui_thread_callback(
    CampaignOrchestratorGui::TrainActionGoalHandle::SharedPtr future
)
{
    // TODO: handle campaign goal response callback in GUI thread
}

void CampaignOrchestratorGui::handle_campaign_feedback_gui_thread_callback(
    CampaignOrchestratorGui::TrainActionGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const TrainAction::Feedback> feedback
)
{
    Q_D(CampaignOrchestratorGui);

    d->ui->feedback_generation_number->setText(QString::number(feedback->generations));
    d->ui->feedback_solution_number->setText(QString::number(feedback->solutions_exercised));
    d->ui->feedback_best_fitness->setText(QString::number(feedback->best_fitness));
    d->ui->feedback_simulation_active->setText(feedback->simulation_active ? "True" : "False");
}

void CampaignOrchestratorGui::handle_campaign_result_gui_thread_callback(
    const CampaignOrchestratorGui::TrainActionGoalHandle::WrappedResult& result
)
{
    // TODO: handle campaign result callback in GUI thread
}

}   // namespaces

int main(int argc, char** argv)
{
    // Capture arguments and remove those specifically for ROS
    rclcpp::init_and_remove_ros_arguments(argc, argv);

    // Create the Qt application and, when the last window is closed, ensure
    // that the ROS context is shutdown
    QApplication app(argc, argv);
    QObject::connect(
        &app, &QGuiApplication::lastWindowClosed,
        [] () { if (rclcpp::ok()) rclcpp::shutdown(); }
    );

    // Create the GUI and register a callback to have it close when the ROS
    // context is shutdown
    ep_training::CampaignOrchestratorGui gui;
    rclcpp::on_shutdown( [&gui] () { gui.close(); } );
    gui.show();

    // Spin a thread dedicated to ROS callbacks
    std::thread ros_thread( [&gui] () { rclcpp::spin(gui.get_ros_node()); } );

    // Start the Qt application
    const int rc = app.exec();

    // Wait until the ROS thread is done, which happens when rclcpp::shutdown()
    // is called, and can happen in this app if the user does CTRL+C in the
    // terminal or closes the Qt window(s)
    ros_thread.join();

    // Done
    return rc;
}

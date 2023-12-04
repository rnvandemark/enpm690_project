#ifndef EP_TRAINING__CAMPAIGN_ORCHESTRATOR_GUI_HPP
#define EP_TRAINING__CAMPAIGN_ORCHESTRATOR_GUI_HPP

#include <ep_training_interfaces/action/execute_training_campaign.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <QMainWindow>

// Forward declare
namespace rclcpp {
    class Node;
}

namespace ep_training {

// Forward declare
class CampaignOrchestratorGuiPrivate;

class CampaignOrchestratorGui : public QMainWindow
{
    Q_OBJECT;

protected:
    // Shorthands for types for the campaign action
    using TrainAction = ep_training_interfaces::action::ExecuteTrainingCampaign;
    using TrainActionClient = rclcpp_action::Client<TrainAction>;
    using TrainActionGoalHandle = rclcpp_action::ClientGoalHandle<TrainAction>;

public:
    CampaignOrchestratorGui();
    virtual ~CampaignOrchestratorGui();

    // Getter for the ROS node in this class' private data.
    std::shared_ptr<rclcpp::Node> get_ros_node();

Q_SIGNALS:
    // A signal used to handle GUI updates for handling a campaign goal
    // response in the Qt event thread.
    void announce_campaign_goal_response(TrainActionGoalHandle::SharedPtr future);

    // A signal used to handle GUI updates for handling campaign feedbacks in
    // in the Qt event thread.
    void announce_campaign_feedback(
        TrainActionGoalHandle::SharedPtr goal_handle,
        const std::shared_ptr<const TrainAction::Feedback> feedback
    );

    // A signal used to handle GUI updates for handling a campaign result in
    // in the Qt event thread.
    void announce_campaign_result(const TrainActionGoalHandle::WrappedResult& result);

protected:
    // Handle campaign goal responses in the ROS thread.
    void handle_campaign_goal_response_ros_thread_callback(TrainActionGoalHandle::SharedPtr future);

    // Handle campaign feedback in the ROS thread.
    void handle_campaign_feedback_ros_thread_callback(
        TrainActionGoalHandle::SharedPtr goal_handle,
        const std::shared_ptr<const TrainAction::Feedback> feedback
    );

    // Handle a campaign result in the ROS thread.
    void handle_campaign_result_ros_thread_callback(const TrainActionGoalHandle::WrappedResult& result);

protected slots:
    // Build a campaign goal response from UI inputs and send the goal.
    void send_campaign_goal();
    // Cancel an active campaign.
    void cancel_campaign_goal();

    // Handle campaign goal responses in the GUI thread.
    void handle_campaign_goal_response_gui_thread_callback(TrainActionGoalHandle::SharedPtr future);

    // Handle campaign feedback in the GUI thread.
    void handle_campaign_feedback_gui_thread_callback(
        TrainActionGoalHandle::SharedPtr goal_handle,
        const std::shared_ptr<const TrainAction::Feedback> feedback
    );

    // Handle a campaign result in the GUI thread.
    void handle_campaign_result_gui_thread_callback(const TrainActionGoalHandle::WrappedResult& result);

protected:
    CampaignOrchestratorGui(CampaignOrchestratorGuiPrivate& d_);
    QSharedPointer<CampaignOrchestratorGuiPrivate> d_ptr;

private:
    Q_DECLARE_PRIVATE(CampaignOrchestratorGui);
};

}   // namespaces

#endif  // #ifndef EP_TRAINING__CAMPAIGN_ORCHESTRATOR_GUI_HPP

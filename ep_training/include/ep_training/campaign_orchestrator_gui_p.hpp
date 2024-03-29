#ifndef EP_TRAINING__CAMPAIGN_ORCHESTRATOR_GUI_P_HPP
#define EP_TRAINING__CAMPAIGN_ORCHESTRATOR_GUI_P_HPP

#include <ep_training_interfaces/action/execute_training_campaign.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Forward declare
namespace Ui {
    class CampaignOrchestratorGui;
}

namespace ep_training {

// Forward declare
class CampaignOrchestratorGui;

class CampaignOrchestratorGuiPrivate
{
protected:
    // Shorthands for types for the campaign action
    using TrainAction = ep_training_interfaces::action::ExecuteTrainingCampaign;
    using TrainActionClient = rclcpp_action::Client<TrainAction>;
    using TrainActionGoalHandle = rclcpp_action::ClientGoalHandle<TrainAction>;

public:
    // The UI of the window.
    const std::unique_ptr<Ui::CampaignOrchestratorGui> ui;

    // A node for the GUI to use to interact with the ROS network.
    std::shared_ptr<rclcpp::Node> node;
    // A client to the training campaign action server.
    std::shared_ptr<TrainActionClient> execute_training_campaign_acl;

    // Constructor.
    // @param parent The owner of this private data, used to bind the ROS
    // thread callback functions in this class.
    CampaignOrchestratorGuiPrivate(CampaignOrchestratorGui* const parent);

    // Default callback function for campaign goal responses.
    const TrainActionClient::GoalResponseCallback handle_campaign_goal_response_ros_thread_callback_func;

    // Default callback function for campaign feedback.
    const TrainActionClient::FeedbackCallback handle_campaign_feedback_ros_thread_callback_func;

    // Default callback function for a campaign's result.
    const TrainActionClient::ResultCallback handle_campaign_result_ros_thread_callback_func;
};

}   // namespaces

#endif  // #ifndef EP_TRAINING__CAMPAIGN_ORCHESTRATOR_GUI_P_HPP

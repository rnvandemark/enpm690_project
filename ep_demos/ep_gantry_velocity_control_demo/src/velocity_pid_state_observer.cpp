#include "ep_gantry_velocity_control_demo/velocity_pid_state_observer.hpp"

#include <rclcpp/executors.hpp>

namespace ep_gantry_velocity_control_demo {

VelocityPidStateObserver::VelocityPidStateObserver(const uint8_t id) :
    ep_training::StateObserverI(id),
    joint_name(declare_parameter("joint_name", "slider_to_cart")),
    target_position(declare_parameter("target_position", 0.0)),
    target_displacement_is_positive(declare_parameter("target_displacement_is_positive", false)),
    joint_states_sub(create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        1,
        std::bind(
            &ep_gantry_velocity_control_demo::VelocityPidStateObserver::joint_states_callback,
            this,
            std::placeholders::_1
        )
    ))
{
}

bool VelocityPidStateObserver::prepare_for_starting_exercising_solution(std::string& error_message)
{
    bool rc = ep_training::StateObserverI::prepare_for_starting_exercising_solution(error_message);
    if (!rc)
    {
        // Call to base class should have set the error message
        goto END;
    }
    rc = get_parameter("target_displacement_is_positive", target_displacement_is_positive);
    if (!rc)
    {
        error_message = "Failed to get parameter 'target_displacement_is_positive'";
        goto END;
    }
    rc = get_parameter("target_position", target_position);
    if (!rc)
    {
        error_message = "Failed to get parameter 'target_position'";
        goto END;
    }
END:
    return rc;
}

void VelocityPidStateObserver::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    const auto name_iter = std::find(msg->name.cbegin(), msg->name.cend(), joint_name);
    if (msg->name.cend() != name_iter)
    {
        const double position = msg->position[std::distance(msg->name.cbegin(), name_iter)];
        if ((target_displacement_is_positive && (position >= target_position))
            || (!target_displacement_is_positive && (position <= target_position)))
        {
            ep_training_interfaces::msg::ExercisingSolutionFinishedReason reason;
            reason.value = ep_training_interfaces::msg::ExercisingSolutionFinishedReason::TASK_GOAL_REACHED;
            set_finished_reason(reason);
        }
    }
}

}   // namespaces

int main(int argc, char** argv)
{
    rclcpp::init_and_remove_ros_arguments(argc, argv);
    rclcpp::spin(std::make_shared<ep_gantry_velocity_control_demo::VelocityPidStateObserver>(0));
    return 0;
}

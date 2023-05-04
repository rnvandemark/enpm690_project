#ifndef EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_PID_STATE_OBSERVER_HPP
#define EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_PID_STATE_OBSERVER_HPP

#include <ep_training/state_observer_i.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ep_gantry_velocity_control_demo {

class VelocityPidStateObserver : public ep_training::StateObserverI
{
protected:
    // The name of the single joint in the gantry. If unspecified, this defaults
    // to "slider_to_cart".
    const std::string joint_name;

    // The target position of the joint named @a joint_name for the current
    // solution's simulation.
    double target_position;
    // Whether the displacement from the initial position to the
    // @a target_position is positive (true), or not (false).
    bool target_displacement_is_positive;

    // A subscription to the joint states, which this node uses to get joint
    // state telemetry on the joint with name @a joint_name.
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;

public:
    explicit VelocityPidStateObserver(const uint8_t id);

protected:
    // Perform any necessary tasks to reset/clear any containers used to
    // collect telemetry. Also do any other work needed to start observing a
    // solution's performance. This is called before
    // @a handle_starting_exercising_solution_requests() returns successfully.
    // If this returns false, this should overwrite the error message with a
    // description as to why.
    virtual bool prepare_for_starting_exercising_solution(std::string& error_message);

    // An update on an agent's joint state.
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}   // namespaces

#endif  // #ifndef EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_PID_STATE_OBSERVER_HPP

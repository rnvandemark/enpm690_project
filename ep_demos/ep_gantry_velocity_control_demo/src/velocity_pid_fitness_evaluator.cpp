#include "ep_gantry_velocity_control_demo/velocity_pid_fitness_evaluator.hpp"

#include <rclcpp/executors.hpp>

namespace ep_gantry_velocity_control_demo {

VelocityPidFitnessEvaluator::VelocityPidFitnessEvaluator(const uint8_t id) :
    ep_training::TimeDependentFitnessEvaluatorI(id),
    joint_name(declare_parameter("joint_name", "slider_to_cart")),
    joint_state_sub(create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        10,
        std::bind(
            &ep_gantry_velocity_control_demo::VelocityPidFitnessEvaluator::joint_state_callback,
            this,
            std::placeholders::_1
        )
    )),
    motion_profile(
        declare_parameter("profile.t_accelerating", 0.0),
        declare_parameter("profile.t_constant_velocity", 0.0),
        declare_parameter("profile.acceleration", 0.0)
    )
{
}

void VelocityPidFitnessEvaluator::handle_restart_requests(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    TimeDependentFitnessEvaluatorI::handle_restart_requests(req, res);

    if (res->success && !get_parameter("profile.t_accelerating", motion_profile.t_accelerating))
    {
        res->message = "Could not get the parameter 'profile.t_accelerating'.";
        res->success = false;
    }
    if (res->success && !get_parameter("profile.t_constant_velocity", motion_profile.t_constant_velocity))
    {
        res->message = "Could not get the parameter 'profile.t_constant_velocity'.";
        res->success = false;
    }
    if (res->success && !get_parameter("profile.acceleration", motion_profile.acceleration))
    {
        res->message = "Could not get the parameter 'profile.acceleration'.";
        res->success = false;
    }
    if (res->success)
    {
        meas_vel_errors.clear();
    }
}

double VelocityPidFitnessEvaluator::calculate_fitness() const
{
    const double cumulative_vel_error = std::accumulate(
        meas_vel_errors.cbegin(),
        meas_vel_errors.cend(),
        0.0,
        [] (const double lhs, const double rhs) { return lhs + std::abs(rhs); }
    );
    const double contribution_vel_errors = (
        (cumulative_vel_error <= 1e-6)
        ? 1000000.0
        : (1.0 / cumulative_vel_error)
    );
    return contribution_vel_errors;
}

void VelocityPidFitnessEvaluator::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    const auto name_iter = std::find(msg->name.cbegin(), msg->name.cend(), joint_name);
    if (msg->name.cend() != name_iter)
    {
        const double meas_vel = msg->velocity[std::distance(msg->name.cbegin(), name_iter)];
        const double desi_vel = motion_profile.evaluate_at(
            (rclcpp::Time(msg->header.stamp) - init_timestamp).seconds()
        );
        meas_vel_errors.push_back(desi_vel - meas_vel);
    }
}

}   // namespaces

int main(int argc, char** argv)
{
    rclcpp::init_and_remove_ros_arguments(argc, argv);
    rclcpp::spin(std::make_shared<ep_gantry_velocity_control_demo::VelocityPidFitnessEvaluator>(0));
    return 0;
}

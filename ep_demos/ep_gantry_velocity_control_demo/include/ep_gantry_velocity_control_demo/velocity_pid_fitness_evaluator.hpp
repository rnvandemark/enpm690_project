#ifndef EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_PID_FITNESS_EVALUATOR_HPP
#define EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_PID_FITNESS_EVALUATOR_HPP

#include "ep_gantry_velocity_control_demo/velocity_trapezoidal_profile.hpp"

#include <ep_training/time_dependent_fitness_evaluator_i.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ep_gantry_velocity_control_demo {

class VelocityPidFitnessEvaluator : public ep_training::TimeDependentFitnessEvaluatorI
{
protected:
    // The name of the single joint in the gantry. If unspecified, this defaults
    // to "slider_to_cart".
    const std::string joint_name;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;

    std::vector<double> meas_vel_errors;

public:
    explicit VelocityPidFitnessEvaluator(const uint8_t id);

protected:
    VelTrapProfile motion_profile;

    virtual void handle_restart_requests(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res
    ) override;

    virtual double calculate_fitness() const override;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}   // namespaces

#endif  // #ifndef EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_PID_FITNESS_EVALUATOR_HPP

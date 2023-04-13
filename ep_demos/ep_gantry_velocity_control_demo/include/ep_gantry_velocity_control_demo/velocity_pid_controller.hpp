#ifndef EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_PID_CONTROLLER_HPP
#define EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_PID_CONTROLLER_HPP

#include "ep_gantry_velocity_control_demo/pid_constants.hpp"
#include "ep_gantry_velocity_control_demo/velocity_trapezoidal_profile.hpp"

#include <ep_common/command_interface_controller_i.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ep_gantry_velocity_control_demo {

class VelocityPidController : public ep_common::CommandInterfaceControllerI
{
public:
    explicit VelocityPidController(const uint8_t id);
    explicit VelocityPidController();

    virtual LifecycleCallbackReturn on_init() override;

    virtual LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

protected:
    VelTrapProfile motion_profile;
    PidConstants pid;

    rclcpp::Time motion_start_time;
    double motion_cumulative_error_vel;
    double motion_prev_error_vel;

    virtual void set_solution_from_parameters() override;

    virtual std::vector<double> evaluate_fitness(
        const rclcpp::Time& update_time,
        const rclcpp::Duration& period
    ) override;
};
}   // namespaces

#endif  // EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_PID_CONTROLLER_HPP

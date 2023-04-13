#include "ep_gantry_velocity_control_demo/velocity_pid_controller.hpp"

namespace ep_gantry_velocity_control_demo {

VelocityPidController::VelocityPidController(const uint8_t id) :
    ep_common::CommandInterfaceControllerI(id)
{
}
VelocityPidController::VelocityPidController() :
    VelocityPidController::VelocityPidController(0)
{
}

ep_common::CommandInterfaceControllerI::LifecycleCallbackReturn VelocityPidController::on_init()
{
    ep_common::CommandInterfaceControllerI::LifecycleCallbackReturn rc = ep_common::CommandInterfaceControllerI::on_init();

    if (rc == ep_common::CommandInterfaceControllerI::LifecycleCallbackReturn::SUCCESS)
    {
        try {
            auto_declare<double>("solution.kp", 0.0);
            generate_param_callback("solution.kp");

            auto_declare<double>("solution.ki", 0.0);
            generate_param_callback("solution.ki");

            auto_declare<double>("solution.kd", 0.0);
            generate_param_callback("solution.kd");

            auto_declare<double>("profile.t_accelerating", 0.0);
            generate_param_callback("profile.t_accelerating");

            auto_declare<double>("profile.t_constant_velocity", 0.0);
            generate_param_callback("profile.t_constant_velocity");

            auto_declare<double>("profile.acceleration", 0.0);
            generate_param_callback("profile.acceleration");

        } catch (const std::exception& e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s\n", e.what());
            rc = ep_common::CommandInterfaceControllerI::LifecycleCallbackReturn::ERROR;
        }
    }

    return rc;
}

ep_common::CommandInterfaceControllerI::LifecycleCallbackReturn VelocityPidController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    const auto rc = ep_common::CommandInterfaceControllerI::on_activate(previous_state);
    motion_cumulative_error_vel = 0.0;
    motion_prev_error_vel = 0.0;
    motion_start_time = node_->now();
    return rc;
}

void VelocityPidController::set_solution_from_parameters()
{
    ep_common::CommandInterfaceControllerI::set_solution_from_parameters();

    pid.kp = node_->get_parameter("solution.kp").as_double();
    pid.ki = node_->get_parameter("solution.ki").as_double();
    pid.kd = node_->get_parameter("solution.kd").as_double();

    motion_profile.t_accelerating = node_->get_parameter("profile.t_accelerating").as_double();
    motion_profile.t_constant_velocity = node_->get_parameter("profile.t_constant_velocity").as_double();
    motion_profile.acceleration = node_->get_parameter("profile.acceleration").as_double();
}

std::vector<double> VelocityPidController::evaluate_fitness(
    const rclcpp::Time& update_time,
    const rclcpp::Duration& period
)
{
    // Calculate the time elapsed since the start of the motion
    const double motion_dt = (update_time - motion_start_time).seconds();

    // Given the time elapsed, calculate the desired velocity
    const double desi_vel = motion_profile.evaluate_at(motion_dt);

    // Get the actual velocity and calculate the error
    const double actual_vel = state_interfaces_[0].get_value();
    const double error_vel = desi_vel - actual_vel;
    motion_cumulative_error_vel += error_vel;

    // Compute the corrected velocity
    const double cmnd_vel = (
        (pid.kp * error_vel)
        + (pid.ki * motion_cumulative_error_vel * period.seconds())
        + (pid.kd * (error_vel - motion_prev_error_vel) / period.seconds())
    );

    // Latch this error for next cycle
    motion_prev_error_vel = error_vel;

    RCLCPP_INFO(
        node_->get_logger(),
        "- %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", update_time.seconds(), pid.kp, pid.ki, pid.kd, motion_dt, desi_vel, actual_vel, error_vel
    );

    return std::vector<double>({cmnd_vel});
}
}   // namespaces

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(
    ep_gantry_velocity_control_demo::VelocityPidController,
    controller_interface::ControllerInterface
)

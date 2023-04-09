#ifndef EP_COMMON__COMMAND_INTERFACE_CONTROLLER_I_HPP
#define EP_COMMON__COMMAND_INTERFACE_CONTROLLER_I_HPP

#include <controller_interface/controller_interface.hpp>
#include <ep_common/node_id.hpp>
#include <ep_common_interfaces/srv/set_string.hpp>
#include <rclcpp/service.hpp>

namespace ep_common {

class CommandInterfaceControllerI : public controller_interface::ControllerInterface
{
protected:
    using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    const ep_common::NodeId node_id;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler;
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> param_callback_handles;

public:
    explicit CommandInterfaceControllerI(const uint8_t id);

    LifecycleCallbackReturn on_init() override;

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period
    ) override;

protected:
    std::vector<std::string> state_interface_names;
    std::vector<std::string> command_interface_names;

    rclcpp::Service<ep_common_interfaces::srv::SetString>::SharedPtr dump_controller_parameters_ssv;

    void generate_param_callback(const std::string& param_name);

    virtual bool reset();

    virtual void set_solution_from_parameters();

    virtual void handle_dump_controller_parameters_requests(
        const std::shared_ptr<ep_common_interfaces::srv::SetString::Request> req,
        std::shared_ptr<ep_common_interfaces::srv::SetString::Response> res
    );

    virtual std::vector<double> evaluate_fitness(
        const rclcpp::Time& update_time,
        const rclcpp::Duration& period
    ) = 0;
};
}   // namespaces

#endif  // EP_COMMON__COMMAND_INTERFACE_CONTROLLER_I_HPP

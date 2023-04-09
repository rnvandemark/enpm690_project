#include "ep_common/command_interface_controller_i.hpp"

using namespace std::chrono_literals;

namespace {
    std::string join(const std::vector<std::string>& elems)
    {
        return elems.empty() ? "[]" : (std::string("[") + std::accumulate(
            std::next(elems.cbegin()),
            elems.cend(),
            elems.front(),
            [](const std::string& a, const std::string& b)
            {
                return std::move(a) + ", " + b;
            }
        )) + "]";
    }
}

namespace ep_common {

CommandInterfaceControllerI::CommandInterfaceControllerI(const uint8_t id) :
    controller_interface::ControllerInterface(),
    node_id(id),
    state_interface_names(),
    command_interface_names()
{
}

CommandInterfaceControllerI::LifecycleCallbackReturn CommandInterfaceControllerI::on_init()
{
    try {
        param_event_handler = std::make_shared<rclcpp::ParameterEventHandler>(node_);

        auto_declare<std::vector<std::string>>("state_interface_names", std::vector<std::string>());
        generate_param_callback("state_interface_names");

        auto_declare<std::vector<std::string>>("command_interface_names", std::vector<std::string>());
        generate_param_callback("command_interface_names");
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s\n", e.what());
        return CommandInterfaceControllerI::LifecycleCallbackReturn::ERROR;
    }

    return CommandInterfaceControllerI::LifecycleCallbackReturn::SUCCESS;
}

CommandInterfaceControllerI::LifecycleCallbackReturn CommandInterfaceControllerI::on_configure(const rclcpp_lifecycle::State&)
{
    dump_controller_parameters_ssv = node_->create_service<ep_common_interfaces::srv::SetString>(
        node_id.get_formatted_topic("dump_controller_parameters"),
        std::bind(
            &ep_common::CommandInterfaceControllerI::handle_dump_controller_parameters_requests,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    set_solution_from_parameters();

    if (command_interface_names.empty())
    {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "Command interface names parameter is empty!"
        );
        return CommandInterfaceControllerI::LifecycleCallbackReturn::ERROR;
    }

    RCLCPP_INFO_STREAM(
        node_->get_logger(),
        "Command interface names: " << join(command_interface_names)
    );

    RCLCPP_INFO_STREAM(
        node_->get_logger(),
        "State interface names: " << join(state_interface_names)
    );

    return reset()
        ? CommandInterfaceControllerI::LifecycleCallbackReturn::SUCCESS
        : CommandInterfaceControllerI::LifecycleCallbackReturn::ERROR
    ;
}

CommandInterfaceControllerI::LifecycleCallbackReturn CommandInterfaceControllerI::on_cleanup(const rclcpp_lifecycle::State&)
{
    return reset()
        ? CommandInterfaceControllerI::LifecycleCallbackReturn::SUCCESS
        : CommandInterfaceControllerI::LifecycleCallbackReturn::ERROR
    ;
}

CommandInterfaceControllerI::LifecycleCallbackReturn CommandInterfaceControllerI::on_error(const rclcpp_lifecycle::State&)
{
    return reset()
        ? CommandInterfaceControllerI::LifecycleCallbackReturn::SUCCESS
        : CommandInterfaceControllerI::LifecycleCallbackReturn::ERROR
    ;
}

controller_interface::InterfaceConfiguration CommandInterfaceControllerI::state_interface_configuration() const
{
    return {
        controller_interface::interface_configuration_type::INDIVIDUAL,
        state_interface_names
    };
}
controller_interface::InterfaceConfiguration CommandInterfaceControllerI::command_interface_configuration() const
{
    return {
        controller_interface::interface_configuration_type::INDIVIDUAL,
        command_interface_names
    };
}

controller_interface::return_type CommandInterfaceControllerI::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period
)
{
    // Evaluate fitness function
    const std::vector<double> fitness_results = evaluate_fitness(time, period);
    if (fitness_results.size() != command_interfaces_.size())
    {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "Fitness results count (" << fitness_results.size()
                << ") != command interface count (" << command_interfaces_.size() << ")"
        );
        return controller_interface::return_type::ERROR;
    }

    // Send each result to its corresponding command interface
    for (size_t i = 0; i < command_interfaces_.size(); i++)
    {
        command_interfaces_[i].set_value(fitness_results[i]);
    }

    return controller_interface::return_type::OK;
}

void CommandInterfaceControllerI::generate_param_callback(const std::string& param_name)
{
    param_callback_handles.push_back(param_event_handler->add_parameter_callback(
        param_name,
        [this] (const rclcpp::Parameter& p)
        {
            RCLCPP_INFO_STREAM(
                node_->get_logger(),
                "Received an update to parameter '" << p.get_name() << "' of type '"
                    << p.get_type_name() << "': '" << p.value_to_string() << "'."
            );
        }
    ));
}

bool CommandInterfaceControllerI::reset()
{
    release_interfaces();
    param_callback_handles.clear();
    param_event_handler = nullptr;
    return true;
}

void CommandInterfaceControllerI::set_solution_from_parameters()
{
    // Capture specified state and command interfaces
    state_interface_names = node_->get_parameter("state_interface_names").as_string_array();
    command_interface_names = node_->get_parameter("command_interface_names").as_string_array();
}

void CommandInterfaceControllerI::handle_dump_controller_parameters_requests(
    const std::shared_ptr<ep_common_interfaces::srv::SetString::Request> req,
    std::shared_ptr<ep_common_interfaces::srv::SetString::Response> res
)
{
    // TODO
}
}   // namespaces

// NO! This is an interface
//#include "class_loader/register_macro.hpp"
//CLASS_LOADER_REGISTER_CLASS(
//    ep_common::CommandInterfaceControllerI,
//    controller_interface::ControllerInterface
//)

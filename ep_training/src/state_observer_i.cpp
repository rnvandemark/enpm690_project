#include "ep_training/state_observer_i.hpp"

namespace ep_training {

StateObserverI::StateObserverI(const uint8_t id) :
    node_id(id),
    rclcpp::Node(node_id.get_formatted_topic("state_observer")),
    starting_exercising_solution_ssv(create_service<ep_common_interfaces::srv::SetDouble>(
        node_id.get_formatted_topic("inform_starting_exercising_solution"),
        std::bind(
            &ep_training::StateObserverI::handle_starting_exercising_solution_requests,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    )),
    finished_reason_pub(create_publisher<ep_training_interfaces::msg::ExercisingSolutionFinishedReason>(
        node_id.get_formatted_topic("exercising_solution_finished_reason"),
        1
    )),
    completion_criteria_timeout(0.0),
    completion_criteria_timeout_tmr(nullptr)
{
    latched_reason.value = ep_training_interfaces::msg::ExercisingSolutionFinishedReason::UNKNOWN;
}

void StateObserverI::handle_starting_exercising_solution_requests(
    const std::shared_ptr<ep_common_interfaces::srv::SetDouble::Request> req,
    std::shared_ptr<ep_common_interfaces::srv::SetDouble::Response> res)
{
    res->success = (req->value > 0.0);
    if (res->success)
    {
        // Ensure the error message is clear, then give
        // prepare_for_starting_exercising_solution() a chance to overwrite it
        res->message = "";
        res->success = prepare_for_starting_exercising_solution(res->message);
        if (res->success)
        {
            completion_criteria_timeout = req->value;
            completion_criteria_timeout_tmr = create_wall_timer(
                std::chrono::duration<double>(completion_criteria_timeout),
                std::bind(
                    &ep_training::StateObserverI::completion_criteria_timeout_callback,
                    this
                )
            );
        }
        // else prepare_for_starting_exercising_solution() set the error msg
    }
    else
    {
        res->message = "Invalid timeout value, must be greater than 0 seconds!";
    }
}

bool StateObserverI::prepare_for_starting_exercising_solution(std::string& error_message)
{
    latched_reason.value = ep_training_interfaces::msg::ExercisingSolutionFinishedReason::UNKNOWN;
    return true;
}

void StateObserverI::completion_criteria_timeout_callback()
{
    ep_training_interfaces::msg::ExercisingSolutionFinishedReason reason;
    reason.value = ep_training_interfaces::msg::ExercisingSolutionFinishedReason::TIME_LIMIT_REACHED;
    set_finished_reason(reason);

    completion_criteria_timeout_tmr->cancel();
    completion_criteria_timeout_tmr = nullptr;
}

void StateObserverI::set_finished_reason(const ep_training_interfaces::msg::ExercisingSolutionFinishedReason& reason)
{
    if (ep_training_interfaces::msg::ExercisingSolutionFinishedReason::UNKNOWN == latched_reason.value)
    {
        latched_reason = reason;
        finished_reason_pub->publish(latched_reason);
    }
}

}   // namespaces

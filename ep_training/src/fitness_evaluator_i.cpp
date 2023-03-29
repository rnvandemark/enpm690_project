#include "ep_training/fitness_evaluator_i.hpp"

namespace ep_training {

FitnessEvaluatorI::FitnessEvaluatorI(const uint8_t id) :
    node_id(id),
    rclcpp::Node(node_id.get_formatted_topic("fitness_evaluator")),
    reset_ssv(create_service<std_srvs::srv::Trigger>(
        node_id.get_formatted_topic("restart"),
        std::bind(
            &ep_training::FitnessEvaluatorI::handle_reset_requests,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    )),
    evaluate_solution_ssv(create_service<ep_common_interfaces::srv::GetDouble>(
        node_id.get_formatted_topic("evaluate"),
        std::bind(
            &ep_training::FitnessEvaluatorI::handle_evaluate_solution_requests,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    ))
{
}

void FitnessEvaluatorI::handle_reset_requests(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    res->success = true;
    res->message = "";
}

void FitnessEvaluatorI::handle_evaluate_solution_requests(
    const std::shared_ptr<ep_common_interfaces::srv::GetDouble::Request> req,
    std::shared_ptr<ep_common_interfaces::srv::GetDouble::Response> res) const
{
    (void)req;
    res->value = calculate_fitness();
    res->success = true;
    res->message = "";
}

}   // namespaces

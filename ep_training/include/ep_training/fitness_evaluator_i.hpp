#ifndef EP_TRAINING__FITNESS_EVALUATOR_I_HPP
#define EP_TRAINING__FITNESS_EVALUATOR_I_HPP

#include <ep_common/node_id.hpp>
#include <ep_common_interfaces/srv/get_double.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ep_training {

class FitnessEvaluatorI : public rclcpp::Node
{
protected:
    const ep_common::NodeId node_id;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_ssv;

    rclcpp::Service<ep_common_interfaces::srv::GetDouble>::SharedPtr evaluate_solution_ssv;

public:
    explicit FitnessEvaluatorI(const uint8_t id);

protected:
    virtual void handle_restart_requests(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res
    );

    virtual void handle_evaluate_solution_requests(
        const std::shared_ptr<ep_common_interfaces::srv::GetDouble::Request> req,
        std::shared_ptr<ep_common_interfaces::srv::GetDouble::Response> res
    ) const;

    virtual double calculate_fitness() const = 0;
};

}   // namespaces

#endif  // #ifndef EP_TRAINING__FITNESS_EVALUATOR_I_HPP

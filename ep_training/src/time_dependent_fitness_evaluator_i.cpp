#include "ep_training/time_dependent_fitness_evaluator_i.hpp"

namespace ep_training {

TimeDependentFitnessEvaluatorI::TimeDependentFitnessEvaluatorI(const uint8_t id) :
    FitnessEvaluatorI(id),
    init_timestamp_set(false),
    simulator_clock_sub(create_subscription<rosgraph_msgs::msg::Clock>(
        "clock",
        1,
        std::bind(
            &ep_training::TimeDependentFitnessEvaluatorI::simulator_clock_callback,
            this,
            std::placeholders::_1
        )
    ))
{
}

void TimeDependentFitnessEvaluatorI::simulator_clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    last_timestamp = msg->clock;
    if (!init_timestamp_set)
    {
        init_timestamp = last_timestamp;
        init_timestamp_set = true;
    }
}

void TimeDependentFitnessEvaluatorI::handle_reset_requests(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    FitnessEvaluatorI::handle_reset_requests(req, res);

    init_timestamp_set = false;
}

}   // namespaces

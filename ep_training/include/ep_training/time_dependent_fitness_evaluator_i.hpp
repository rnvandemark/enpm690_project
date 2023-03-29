#ifndef EP_TRAINING__TIME_DEPENDENT_FITNESS_EVALUATOR_I_HPP
#define EP_TRAINING__TIME_DEPENDENT_FITNESS_EVALUATOR_I_HPP

#include <ep_training/fitness_evaluator_i.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace ep_training {

class TimeDependentFitnessEvaluatorI : public FitnessEvaluatorI
{
protected:
    bool init_timestamp_set;
    rclcpp::Time init_timestamp;
    rclcpp::Time last_timestamp;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr simulator_clock_sub;

public:
    explicit TimeDependentFitnessEvaluatorI(const uint8_t id);

protected:
    virtual void simulator_clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    virtual void handle_reset_requests(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res
    ) override;
};

}   // namespaces

#endif  // #ifndef EP_TRAINING__TIME_DEPENDENT_FITNESS_EVALUATOR_I_HPP

#ifndef EP_TRAINING__STATE_OBSERVER_I_HPP
#define EP_TRAINING__STATE_OBSERVER_I_HPP

#include <ep_common/node_id.hpp>
#include <ep_common_interfaces/srv/set_double.hpp>
#include <ep_training_interfaces/msg/exercising_solution_finished_reason.hpp>
#include <rclcpp/node.hpp>

namespace ep_training {

class StateObserverI : public rclcpp::Node
{
protected:
    const ep_common::NodeId node_id;

    // Handle the fact that the solution that this node should observe is about
    // to be exercised. The double that is being set is the timeout for the
    // solution to reach its goal, in seconds.
    rclcpp::Service<ep_common_interfaces::srv::SetDouble>::SharedPtr starting_exercising_solution_ssv;

    // A publisher for @a latched_reason, a reason describing why this solution
    // has met its completion criteria.
    rclcpp::Publisher<ep_training_interfaces::msg::ExercisingSolutionFinishedReason>::SharedPtr finished_reason_pub;

    // The reason as to why the active solution met its completion criteria. If
    // this is UNKNOWN while a solution is being ran, then no completion
    // criteria have been met yet.
    ep_training_interfaces::msg::ExercisingSolutionFinishedReason latched_reason;

    // The timeout for a solution to try and reach its goal, in seconds.
    double completion_criteria_timeout;
    // The timer used to respect the above timeout. This is set to null while a
    // solution is not active, and is (re)initialized when a call to
    // @a starting_exercising_solution_ssv is heard. A callback to
    // @a completion_criteria_timeout_callback() sells this back to null. This
    // is essentially used as a oneshot timer for each solution.
    rclcpp::TimerBase::SharedPtr completion_criteria_timeout_tmr;

public:
    explicit StateObserverI(const uint8_t id);

protected:
    // Handle the fact that the solution that this node should observe is about
    // to be exercised.
    virtual void handle_starting_exercising_solution_requests(
        const std::shared_ptr<ep_common_interfaces::srv::SetDouble::Request> req,
        std::shared_ptr<ep_common_interfaces::srv::SetDouble::Response> res
    );

    // Perform any necessary tasks to reset/clear any containers used to
    // collect telemetry. Also do any other work needed to start observing a
    // solution's performance. This is called before
    // @a handle_starting_exercising_solution_requests() returns successfully.
    // If this returns false, this should overwrite the error message with a
    // description as to why.
    virtual bool prepare_for_starting_exercising_solution(std::string& error_message);

    // The callback for @a completion_criteria_timeout_tmr, signfiying that the
    // solution has ran out of time (met the 'timeout' completion criteria).
    virtual void completion_criteria_timeout_callback();

    // Finalize the reason as to why this solution has met its completion
    // criteria. This will set a value for @a latched_reason, publish it via
    // @a finished_reason_pub, and not allow any updates to it until telemetry
    // in this node has been cleared via @a starting_exercising_solution_ssv.
    void set_finished_reason(const ep_training_interfaces::msg::ExercisingSolutionFinishedReason& reason);
};

}   // namespaces

#endif  // #ifndef EP_TRAINING__STATE_OBSERVER_I_HPP

#ifndef EP_TRAINING__GENETIC_ALGORITHM_CONTROLLER_I_HPP
#define EP_TRAINING__GENETIC_ALGORITHM_CONTROLLER_I_HPP

#include <ep_common_interfaces/srv/get_double.hpp>
#include <ep_common_interfaces/srv/set_double.hpp>
#include <ep_common_interfaces/srv/set_string.hpp>
#include <ep_training_interfaces/action/execute_training_campaign.hpp>
#include <ep_training_interfaces/msg/exercising_solution_finished_reason.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ep_training {

template <typename SolnT>
class GeneticAlgorithmControllerI : public rclcpp::Node
{
protected:
    using TrainAction = ep_training_interfaces::action::ExecuteTrainingCampaign;
    using TrainActionGoalHandleSharedPtr = std::shared_ptr<rclcpp_action::ServerGoalHandle<TrainAction>>;

protected:
    // A solution and its corresponding fitness.
    struct ScoredSolnT
    {
    public:
        // A solution.
        SolnT soln;
        // Its corresponding fitness.
        double fitness;
    };

    // A shorthand for a population, i.e. a list of solutions.
    using ScoredPopulationT = std::vector<ScoredSolnT>;

    // The maximum number of parallel agents that can be orchestrated in the
    // simulators. Requested campaigns can have a parallel agent count less
    // than or equal to this.
    const ep_training_interfaces::msg::ParallelAgentCount::_value_type max_parallel_agent_count;

    // The current campaign (nullptr if not active).
    TrainAction::Goal::ConstSharedPtr current_goal;
    // A container for the current campaign's feedback (nullptr if not active).
    TrainAction::Feedback::SharedPtr current_goal_feedback;

    // The action server to handle new campaign requests.
    rclcpp_action::Server<TrainAction>::SharedPtr execute_training_campaign_asv;

    // A list of service clients to set the parameters of a command interface
    // controller, used to set the initial conditions and the control constants
    // of each active controller.
    std::vector<rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr> set_controller_parameters_scl;
    // A list of service clients used to have a command interface controller
    // dump the relevant controller parameters to a file URL. This is not
    // really 'setting' a string, the value is the file URL to use.
    std::vector<rclcpp::Client<ep_common_interfaces::srv::SetString>::SharedPtr> dump_controller_parameters_scl;

    // A list of service clients used to clear the accumulated telemetry of a
    // command interface's performance.
    std::vector<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> restart_fitness_evaluator_scl;
    // A list of service clients used to query the performance of a command
    // interface controller using its current set of control constants.
    std::vector<rclcpp::Client<ep_common_interfaces::srv::GetDouble>::SharedPtr> evaluate_solution_scl;

    // A list of service clients used to inform the monitors of the parallel
    // agents' completion criteria that a new solution is about to start.
    std::vector<rclcpp::Client<ep_common_interfaces::srv::SetDouble>::SharedPtr> inform_starting_exercising_solution_scl;
    // A list of subscribers of signals as to when and why a parallel agent has
    // met its completion criteria.
    std::vector<rclcpp::Subscription<ep_training_interfaces::msg::ExercisingSolutionFinishedReason>::SharedPtr> exercising_solution_finished_reason_sub;

    // A service client to reset the simulated world, once all of the current
    // agents running in parallel have finished.
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_world_scl;
    // A service client to pause the simulation world for all parallel agents.
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_simulation_scl;
    // A service client to resume the simulation world for all parallel agents.
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr play_simulation_scl;

    // A list of mutexes to access finished_reasons at the same index.
    std::vector<std::mutex> finished_reason_mutexes;
    // A list of reasons as to why an agent's exercise finished.
    std::vector<ep_training_interfaces::msg::ExercisingSolutionFinishedReason> finished_reasons;

public:
    explicit GeneticAlgorithmControllerI();

protected:
    // Handle campaign goal requests. TODO handle the scenario where a goal is
    // already being executed and another request is received.
    rclcpp_action::GoalResponse handle_campaign_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const TrainAction::Goal> goal
    );

    // Cancel the current campaign.
    rclcpp_action::CancelResponse handle_campaign_cancel(const TrainActionGoalHandleSharedPtr goal_handle);

    // Handle a campaign request having been accepted.
    void handle_campaign_accepted(const TrainActionGoalHandleSharedPtr goal_handle);

    // Execute a training campaign from start to finish.
    void execute_campaign(const TrainActionGoalHandleSharedPtr goal_handle);

    // Randomly initialize the population with the given number of solutions.
    void init_population(ScoredPopulationT& population, const size_t num_solutions);
    // Evaluate all individuals in the population by sorting them in order from
    // best fitness to worst.
    void select(ScoredPopulationT& population);
    // Perform crossover to reproduce the specified number of offspring.
    void crossover(
        ScoredPopulationT& population,
        const TrainAction::Goal::_crossover_offspring_type& num_offspring
    );
    // Randomly mutate each solution in the given population to the given
    // degree of mutability.
    void mutate(
        ScoredPopulationT& population,
        const double mutability
    );

    // A generic callback to one of exercising_solution_finished_reason_sub,
    // describing why an agent has reached its completion criteria for the
    // simulation objective (not the training's completion criteria).
    void exercising_solution_finished_reason_callback(
        const ep_training_interfaces::msg::ExercisingSolutionFinishedReason::SharedPtr msg,
        const ep_training_interfaces::msg::ParallelAgentCount::_value_type agent_id
    );
};

}   // namespaces

// Because this is a class template, include implementation
#include "ep_training/genetic_algorithm_controller_i_impl.hpp"

#endif  // #ifndef EP_TRAINING__GENETIC_ALGORITHM_CONTROLLER_I_HPP

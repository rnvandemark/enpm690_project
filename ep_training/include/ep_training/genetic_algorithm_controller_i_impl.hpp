#ifndef EP_TRAINING__GENETIC_ALGORITHM_CONTROLLER_I_IMPL_HPP
#define EP_TRAINING__GENETIC_ALGORITHM_CONTROLLER_I_IMPL_HPP

#include "ep_training/genetic_algorithm_controller_i.hpp"
#include <ep_common/node_id.hpp>
#include <rclcpp/executors.hpp>

#include <random>

namespace {
    rclcpp::SubscriptionOptions get_sub_options_with_callback_group(rclcpp::CallbackGroup::SharedPtr cb_group)
    {
        rclcpp::SubscriptionOptions options;
        options.callback_group = cb_group;
        return options;
    }
}

using namespace std::placeholders;

namespace ep_training {

// Join the elements in a vector of strings with commas.
std::string join(const std::vector<std::string>& vec)
{
    return (vec.empty() ? "" : std::accumulate(
        std::next(vec.cbegin()),
        vec.cend(),
        vec.front(),
        [](const std::string& l, const std::string& r)
        {
            return std::move(l) + ", " + r;
        }
    ));
}

// Compare the fitness of two scored solutions.
bool compare_fitness(const double lhs, const double rhs);

// Build a list of parameter messages from a solution given its type.
template <typename SolnT>
std::vector<rcl_interfaces::msg::Parameter> node_parameters_from_solution(const SolnT& soln);

// Generate a random solution.
template <typename SolnT>
SolnT get_random_solution();

// Have the two given parents mate at the specified crossover point (some value
// in the range [0.0, 1.0] and store the results in the two offspring.
template <typename SolnT>
void do_crossover_at(
    const SolnT& solnParent1,
    const SolnT& solnParent2,
    const double crossover_point,
    SolnT& solnOffspring1,
    SolnT& solnOffspring2
);

// Mutate the given solution to a random degree within the given mutability.
template <typename SolnT>
void mutate_solution(SolnT& soln, const double mutability);

template <typename SolnT>
GeneticAlgorithmControllerI<SolnT>::GeneticAlgorithmControllerI() :
    rclcpp::Node("genetic_algorithm_controller"),
    callback_group_action_server(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    callback_group_execution(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    max_parallel_agent_count(declare_parameter("max_parallel_agent_count", 1)),
    current_goal(nullptr),
    current_goal_feedback(nullptr),
    execute_training_campaign_asv(rclcpp_action::create_server<GeneticAlgorithmControllerI<SolnT>::TrainAction>(
        this,
        "execute_training_campaign",
        std::bind(&ep_training::GeneticAlgorithmControllerI<SolnT>::handle_campaign_goal, this, _1, _2),
        std::bind(&ep_training::GeneticAlgorithmControllerI<SolnT>::handle_campaign_cancel, this, _1),
        std::bind(&ep_training::GeneticAlgorithmControllerI<SolnT>::handle_campaign_accepted, this, _1),
        rcl_action_server_get_default_options(),
        callback_group_action_server
    )),
    set_controller_parameters_scl(0), // Set in constructor body
    restart_fitness_evaluator_scl(0), // Set in constructor body
    evaluate_solution_scl(0), // Set in constructor body
    inform_starting_exercising_solution_scl(0), // Set in constructor body
    exercising_solution_finished_reason_sub(0), // Set in constructor body
    reset_world_scl(create_client<std_srvs::srv::Empty>(
        "reset_world",
        rmw_qos_profile_services_default,
        callback_group_execution
    )),
    pause_simulation_scl(create_client<std_srvs::srv::Empty>(
        "pause_physics",
        rmw_qos_profile_services_default,
        callback_group_execution
    )),
    play_simulation_scl(create_client<std_srvs::srv::Empty>(
        "unpause_physics",
        rmw_qos_profile_services_default,
        callback_group_execution
    )),
    finished_reason_mutexes(max_parallel_agent_count),
    finished_reasons(max_parallel_agent_count)
{
    set_controller_parameters_scl.reserve(max_parallel_agent_count);
    restart_fitness_evaluator_scl.reserve(max_parallel_agent_count);
    evaluate_solution_scl.reserve(max_parallel_agent_count);
    inform_starting_exercising_solution_scl.reserve(max_parallel_agent_count);
    exercising_solution_finished_reason_sub.reserve(max_parallel_agent_count);
    for (ep_training_interfaces::msg::ParallelAgentCount::_value_type i = 0; i < max_parallel_agent_count; i++)
    {
        set_controller_parameters_scl.push_back(
            create_client<rcl_interfaces::srv::SetParameters>(
                ep_common::NodeId::get_formatted_topic("command_interface_controller", i) + "/set_parameters",
                rmw_qos_profile_services_default,
                callback_group_execution
            )
        );
        restart_fitness_evaluator_scl.push_back(
            create_client<std_srvs::srv::Trigger>(
                ep_common::NodeId::get_formatted_topic("restart_fitness_evaluator", i),
                rmw_qos_profile_services_default,
                callback_group_execution
            )
        );
        evaluate_solution_scl.push_back(
            create_client<ep_common_interfaces::srv::GetDouble>(
                ep_common::NodeId::get_formatted_topic("evaluate_solution", i),
                rmw_qos_profile_services_default,
                callback_group_execution
            )
        );
        inform_starting_exercising_solution_scl.push_back(
            create_client<ep_common_interfaces::srv::SetDouble>(
                ep_common::NodeId::get_formatted_topic("inform_starting_exercising_solution", i),
                rmw_qos_profile_services_default,
                callback_group_execution
            )
        );
        exercising_solution_finished_reason_sub.push_back(
            create_subscription<ep_training_interfaces::msg::ExercisingSolutionFinishedReason>(
                ep_common::NodeId::get_formatted_topic("exercising_solution_finished_reason", i),
                1,
                [this, i](const ep_training_interfaces::msg::ExercisingSolutionFinishedReason::SharedPtr msg)
                {
                    exercising_solution_finished_reason_callback(msg, i);
                },
                get_sub_options_with_callback_group(callback_group_execution)
            )
        );
    }
}

template <typename SolnT>
rclcpp_action::GoalResponse GeneticAlgorithmControllerI<SolnT>::handle_campaign_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const GeneticAlgorithmControllerI<SolnT>::TrainAction::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO_STREAM(get_logger(), "Received goal request with the following properties:");
    RCLCPP_INFO_STREAM(get_logger(), "- generation_solutions = " << goal->generation_solutions);
    RCLCPP_INFO_STREAM(get_logger(), "- convergence_threshold = " << goal->convergence_threshold);
    RCLCPP_INFO_STREAM(get_logger(), "- generation_limit = " << goal->generation_limit);
    RCLCPP_INFO_STREAM(get_logger(), "- solution_time_limit = " << goal->solution_time_limit);
    RCLCPP_INFO_STREAM(get_logger(), "- mutability = " << goal->mutability);
    RCLCPP_INFO_STREAM(get_logger(), "- command_interface_names = [" << join(goal->command_interface_names) << "]");
    RCLCPP_INFO_STREAM(get_logger(), "- state_interface_names = [" << join(goal->state_interface_names) << "]");
    RCLCPP_INFO_STREAM(get_logger(), "- desired_parallel_agent_count = " << static_cast<int>(goal->desired_parallel_agent_count.value));
    RCLCPP_INFO_STREAM(get_logger(), "- parameters_file_url = " << goal->parameters_file_url);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename SolnT>
rclcpp_action::CancelResponse GeneticAlgorithmControllerI<SolnT>::handle_campaign_cancel(
    const GeneticAlgorithmControllerI<SolnT>::TrainActionGoalHandleSharedPtr goal_handle
)
{
    (void)goal_handle;
    current_goal = nullptr;
    RCLCPP_INFO_STREAM(get_logger(), "Goal cancelled by request.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename SolnT>
void GeneticAlgorithmControllerI<SolnT>::handle_campaign_accepted(
    const GeneticAlgorithmControllerI<SolnT>::TrainActionGoalHandleSharedPtr goal_handle
)
{
    // This needs to return quickly to avoid blocking the executor, so spin up
    // a new thread.
    std::thread{
        std::bind(
            &GeneticAlgorithmControllerI<SolnT>::execute_campaign,
            this,
            _1
        ),
        goal_handle
    }.detach();
}

template <typename SolnT>
void GeneticAlgorithmControllerI<SolnT>::execute_campaign(
    const GeneticAlgorithmControllerI<SolnT>::TrainActionGoalHandleSharedPtr goal_handle
)
{
    current_goal = goal_handle->get_goal();

    // Publish initial feedback.
    current_goal_feedback.reset(new GeneticAlgorithmControllerI<SolnT>::TrainAction::Feedback);
    const auto pub_feedback = [this, &goal_handle](
        const TrainAction::Feedback::_generations_type& generations,
        const TrainAction::Feedback::_solutions_exercised_type& solutions_exercised,
        const TrainAction::Feedback::_best_fitness_type& best_fitness,
        const TrainAction::Feedback::_simulation_active_type& simulation_active,
        const TrainAction::Feedback::_agent_pausing_simulation_type& agent_pausing_simulation
    )
    {
        this->current_goal_feedback->generations = generations;
        this->current_goal_feedback->solutions_exercised = solutions_exercised;
        this->current_goal_feedback->best_fitness = best_fitness;
        this->current_goal_feedback->simulation_active = simulation_active;
        this->current_goal_feedback->agent_pausing_simulation = agent_pausing_simulation;
        goal_handle->publish_feedback(this->current_goal_feedback);
    };

    auto result = std::make_shared<TrainAction::Result>();

#define DEMUX_PARAM(NAME) const auto& NAME = current_goal->NAME

    // Demux each parameter in the campaign goal for simplicity.
    DEMUX_PARAM(generation_solutions);
    DEMUX_PARAM(convergence_threshold);
    DEMUX_PARAM(generation_limit);
    DEMUX_PARAM(solution_time_limit);
    DEMUX_PARAM(crossover_offspring);
    DEMUX_PARAM(mutability);
    DEMUX_PARAM(command_interface_names);
    DEMUX_PARAM(state_interface_names);
    DEMUX_PARAM(desired_parallel_agent_count);
    DEMUX_PARAM(parameters_file_url);

#undef DEMUX_PARAM

#define CREATE_REQ(NAME, TYPE) auto NAME##_req = std::make_shared<TYPE::Request>()

    // Prepare containers that will be used often.
    CREATE_REQ(set_parameters, rcl_interfaces::srv::SetParameters);
    CREATE_REQ(set_string, ep_common_interfaces::srv::SetString);
    CREATE_REQ(trigger, std_srvs::srv::Trigger);
    CREATE_REQ(get_double, ep_common_interfaces::srv::GetDouble);
    CREATE_REQ(set_double, ep_common_interfaces::srv::SetDouble);
    CREATE_REQ(empty, std_srvs::srv::Empty);
    std::vector<rcl_interfaces::msg::Parameter> command_interface_controller_params;

#undef CREATE_REQ

    // Whether we have converged to an acceptable solution (true), or not
    // (false).
    bool convergence_reached = false;
    // Whether the generation limit was reached (true), or not (false).
    bool generation_limit_reached = false;
    // Whether a fatal, unexpected error has occurred (true), or not (false).
    bool error_occurred = false;
    // Whether any of the three above flags have been set high.
    bool campaign_finished = false;

    // Whether the simulation is currently paused or active.
    bool simulation_active = true;
    // Each solutions' completion criteria status.
    std::vector<bool> solution_completion_criteria_met(desired_parallel_agent_count.value, false);
    // Rate at which to query is a service response was received.
    const std::chrono::duration<double> response_wait(0.02);
    // Rate at which to query for solution completion criteria.
    rclcpp::Rate poll_completion_rate(0.25);

    // Init the population of solutions.
    GeneticAlgorithmControllerI<SolnT>::ScoredPopulationT population;

#define HELP_PLACE_REQUEST(SRV_TYPE, REQ_NAME, RES_NAME, SCL) \
    RCLCPP_INFO_STREAM(get_logger(), "Placing request with '" #SCL "'"); \
    auto RES_NAME = SCL->async_send_request(REQ_NAME); \
    while (std::future_status::ready != RES_NAME.wait_for(response_wait)) {} \
    RCLCPP_INFO_STREAM(get_logger(), "Received response from '" #SCL "'")

#define ENSURE_REQ_SUCCESS(RES_NAME, ERR_MSG) \
    if (!(RES_NAME.get()->success)) \
    { \
        result->message = std::string(ERR_MSG) + ": " + (RES_NAME.get()->message) + "."; \
        error_occurred = true; \
        goto END_CAMPAIGN; \
    }

    // Everything is ready to start execution.
    RCLCPP_INFO_STREAM(get_logger(), "Executing goal!");

    // Train until any of the following occur:
    // - the ROS context is shutdown
    // - we have converged to an acceptable solution
    // - the generation limit was reached
    // - we have encountered an error and set the error message
    TrainAction::Goal::_generation_limit_type generation_number = 0;
    double best_fitness_so_far = 0.0;

#define VERIFY_NAMED_SERVICE_READY(SCL, NAME) \
    if (SCL->wait_for_service(std::chrono::duration<double>(0.5))) \
    { \
        RCLCPP_INFO_STREAM(get_logger(), "Service '" << NAME << "' is ready."); \
    } \
    else \
    { \
        result->message = std::string("Service client '") + NAME + "' is not ready."; \
        error_occurred = true; \
        goto END_CAMPAIGN; \
    }

#define VERIFY_SERVICE_READY(SCL) VERIFY_NAMED_SERVICE_READY(SCL, #SCL)

    // Real quick, make sure all of the service clients are connected to their
    // respective servers.
    VERIFY_SERVICE_READY(reset_world_scl)
    VERIFY_SERVICE_READY(pause_simulation_scl)
    VERIFY_SERVICE_READY(play_simulation_scl)
    for (ep_training_interfaces::msg::ParallelAgentCount::_value_type i = 0; i < desired_parallel_agent_count.value; i++)
    {
        VERIFY_NAMED_SERVICE_READY(set_controller_parameters_scl[i], "set_controller_parameters_scl[" + std::to_string(i) + "]")
        VERIFY_NAMED_SERVICE_READY(restart_fitness_evaluator_scl[i], "restart_fitness_evaluator_scl[" + std::to_string(i) + "]")
        VERIFY_NAMED_SERVICE_READY(evaluate_solution_scl[i], "evaluate_solution_scl[" + std::to_string(i) + "]")
        VERIFY_NAMED_SERVICE_READY(inform_starting_exercising_solution_scl[i], "inform_starting_exercising_solution_scl[" + std::to_string(i) + "]")
    }

#undef VERIFY_SERVICE_READY
#undef VERIFY_NAMED_SERVICE_READY

    while (rclcpp::ok() && (!campaign_finished))
    {
        pub_feedback(generation_number, 0, best_fitness_so_far, simulation_active, 0);

        // Pause simulation.
        HELP_PLACE_REQUEST(std_srvs::srv::Empty, empty_req, pause_simulation_res, pause_simulation_scl);
        simulation_active = false;
        pub_feedback(generation_number, 0, best_fitness_so_far, simulation_active, 0);

        // If this is the first generation, initialize the set of solutions. If
        // not, perform the selection/crossover/mutation process.
        if (0 == generation_number)
        {
            init_population(population, generation_solutions);
        }
        else
        {
            select(population);
            crossover(population, crossover_offspring);
            mutate(population, mutability);
        }

        // Iterate through the expected solution count.
        for (
            TrainAction::Goal::_generation_solutions_type solution_number = 0;
            rclcpp::ok() && (!campaign_finished) && (solution_number < generation_solutions);
            solution_number++
        )
        {
            pub_feedback(generation_number, solution_number, best_fitness_so_far, simulation_active, 0);

            // Get the next set of solution parameters and append the command
            // and state interface names from the campaign goal.
            command_interface_controller_params = node_parameters_from_solution(population[solution_number].soln);
            rcl_interfaces::msg::Parameter interface_names_param;
            interface_names_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            interface_names_param.name = "command_interfaces";
            interface_names_param.value.string_array_value = command_interface_names;
            command_interface_controller_params.push_back(interface_names_param);
            interface_names_param.name = "state_interfaces";
            interface_names_param.value.string_array_value = state_interface_names;
            command_interface_controller_params.push_back(interface_names_param);

            // Set the parameters.
            HELP_PLACE_REQUEST(
                rcl_interfaces::srv::SetParameters,
                set_parameters_req,
                set_controller_parameters_res,
                set_controller_parameters_scl[0]
            );
            for (
                auto iter = set_controller_parameters_res.get()->results.cbegin();
                iter != set_controller_parameters_res.get()->results.cend();
                ++iter
            )
            {
                if (!iter->successful)
                {
                    result->message = "Failed to set a command interface controller parameter: " + iter->reason + ".";
                    error_occurred = true;
                    goto END_CAMPAIGN;
                }
            }

            // Reset the simulation world, state observer, and fitness
            // evaluators, then resume the simulation.
            HELP_PLACE_REQUEST(std_srvs::srv::Empty, empty_req, reset_world_res, reset_world_scl);
            set_double_req->value = solution_time_limit;
            HELP_PLACE_REQUEST(
                ep_common_interfaces::srv::SetDouble,
                set_double_req,
                inform_starting_exercising_solution_res,
                inform_starting_exercising_solution_scl[0]
            );
            ENSURE_REQ_SUCCESS(inform_starting_exercising_solution_res, "Failed to inform state observer of solution starting")
            HELP_PLACE_REQUEST(
                std_srvs::srv::Trigger,
                trigger_req,
                restart_fitness_evaluator_res,
                restart_fitness_evaluator_scl[0]
            );
            ENSURE_REQ_SUCCESS(restart_fitness_evaluator_res, "Failed to restart fitness evaluator")
            HELP_PLACE_REQUEST(std_srvs::srv::Empty, empty_req, play_simulation_res, play_simulation_scl);
            simulation_active = true;
            pub_feedback(generation_number, solution_number, best_fitness_so_far, simulation_active, 0);

            // Wait until some completion criteria for this agent has been met.
            {
                const std::lock_guard<std::mutex> lock(finished_reason_mutexes[0]);
                finished_reasons[0].value = ep_training_interfaces::msg::ExercisingSolutionFinishedReason::UNKNOWN;
            }
            solution_completion_criteria_met.assign(solution_completion_criteria_met.size(), false);
            while (!solution_completion_criteria_met[0])
            {
                poll_completion_rate.sleep();
                const std::lock_guard<std::mutex> lock(finished_reason_mutexes[0]);
                solution_completion_criteria_met[0] = (
                    ep_training_interfaces::msg::ExercisingSolutionFinishedReason::UNKNOWN != finished_reasons[0].value
                );
            }

            // Pause simulation again.
            HELP_PLACE_REQUEST(std_srvs::srv::Empty, empty_req, loop_pause_simulation_res, pause_simulation_scl);
            simulation_active = false;

            // Evaluate the performance of this agent with this solution.
            HELP_PLACE_REQUEST(
                ep_common_interfaces::srv::GetDouble,
                get_double_req,
                evaluate_performance_res,
                evaluate_solution_scl[0]
            );
            ENSURE_REQ_SUCCESS(evaluate_performance_res, "Failed to evaluate performance")
            population[solution_number].fitness = evaluate_performance_res.get()->value;

            // Is this the best solution so far?
            if ((0 == solution_number) || compare_fitness(population[solution_number].fitness, best_fitness_so_far))
            {
                best_fitness_so_far = population[solution_number].fitness;
            }

            // Send some feedback now that this solution is finished.
            pub_feedback(generation_number, solution_number, best_fitness_so_far, simulation_active, 0);

            // Determine if this solution is acceptable to with the specified
            // threshold. If it is, then populate the result with the data and
            // call a service so the command interface controller's parameters
            // (relevant to the solution) get dumped to a file.
            if (compare_fitness(population[solution_number].fitness, convergence_threshold))
            {
                result->solution = solution_number;
                result->fitness = population[solution_number].fitness;
                result->generation = generation_number;

                set_string_req->value = parameters_file_url;
                HELP_PLACE_REQUEST(
                    ep_common_interfaces::srv::SetString,
                    set_string_req,
                    dump_params_res,
                    dump_controller_parameters_scl[0]
                );
                ENSURE_REQ_SUCCESS(dump_params_res, "Failed to dump controller parameters")

                convergence_reached = true;

                goto END_CAMPAIGN;
            }
        }

        // Increment the generation number and set if this breaches the max.
        generation_number++;
        generation_limit_reached = (generation_number >= generation_limit);

        // Have any of the end conditions been met?
        campaign_finished = (convergence_reached || generation_limit_reached || error_occurred);
    }

#undef ENSURE_REQ_SUCCESS
#undef HELP_PLACE_REQUEST

END_CAMPAIGN:
    // If an error occurred, log the error message and abort the campaign.
    if (error_occurred)
    {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "Training campaign failed with a fatal error: " << result->message
        );
        result->success = false;
        goal_handle->abort(result);
    }
    // If we found an acceptable solution, the campaign was succesful. Check
    // this before whether or not the generation limit was reached because
    // both could have been set to true in the same cycle.
    else if (convergence_reached)
    {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "Training campaign was successful after " << generation_number << " generations."
        );
        result->success = true;
        goal_handle->succeed(result);
    }
    // If the generation limt was reached, the campaign was not successul, but
    // we still consider the action goal to be successful in the sense that it
    // finished under nominal conditions (no fatal errors).
    else if (generation_limit_reached)
    {
        RCLCPP_WARN_STREAM(get_logger(), "Training campaign did not converge to an acceptable solution.");
        result->success = false;
        goal_handle->succeed(result);
    }
    // This should be impossible.
    else if (rclcpp::ok())
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Unknown campaign termination!?");
        result->success = false;
        goal_handle->abort(result);
    }
}

template <typename SolnT>
void GeneticAlgorithmControllerI<SolnT>::init_population(
    GeneticAlgorithmControllerI<SolnT>::ScoredPopulationT& population,
    const size_t num_solutions
)
{
    population.resize(num_solutions);
    for (size_t i = 0; i < population.size(); i++)
    {
        population[i].soln = get_random_solution<SolnT>();
        population[i].fitness = 0.0;
    }
}

template <typename SolnT>
void GeneticAlgorithmControllerI<SolnT>::select(
    GeneticAlgorithmControllerI<SolnT>::ScoredPopulationT& population
)
{
    std::sort(
        population.begin(),
        population.end(),
        [] (
            const GeneticAlgorithmControllerI<SolnT>::ScoredSolnT& lhs,
            const GeneticAlgorithmControllerI<SolnT>::ScoredSolnT& rhs
        )
        {
            return compare_fitness(lhs.fitness, rhs.fitness);
        }
    );
}

template <typename SolnT>
void GeneticAlgorithmControllerI<SolnT>::crossover(
    GeneticAlgorithmControllerI<SolnT>::ScoredPopulationT& population,
    const TrainAction::Goal::_crossover_offspring_type& num_offspring
)
{
    const size_t num_solutions = population.size();
    std::random_device rd;
    std::mt19937 rng_engine(rd());
    std::uniform_real_distribution<> urd(0.0, 1.0);

    for (size_t i = 0; i < num_offspring; i+=2)
    {
        const size_t solnParentIndex1 = i;
        const size_t solnParentIndex2 = i + 1;
        const size_t solnOffspringIndex1 = num_solutions - solnParentIndex1;
        const size_t solnOffspringIndex2 = num_solutions - solnParentIndex2;
        do_crossover_at(
            population[solnParentIndex1].soln,
            population[solnParentIndex2].soln,
            urd(rng_engine),
            population[solnOffspringIndex1].soln,
            population[solnOffspringIndex2].soln
        );
    }
}

template <typename SolnT>
void GeneticAlgorithmControllerI<SolnT>::mutate(
    GeneticAlgorithmControllerI<SolnT>::ScoredPopulationT& population,
    const double mutability
)
{
    std::for_each(
        population.begin(),
        population.end(),
        [&mutability] (ScoredSolnT& soln) { mutate_solution(soln.soln, mutability); }
    );
}

template <typename SolnT>
void GeneticAlgorithmControllerI<SolnT>::exercising_solution_finished_reason_callback(
    const ep_training_interfaces::msg::ExercisingSolutionFinishedReason::SharedPtr msg,
    const uint8_t agent_id
)
{
    const std::lock_guard<std::mutex> lock(finished_reason_mutexes[agent_id]);
    finished_reasons[agent_id] = *msg;
}

}   // namespaces

#endif  // #ifndef EP_TRAINING__GENETIC_ALGORITHM_CONTROLLER_I_IMPL_HPP

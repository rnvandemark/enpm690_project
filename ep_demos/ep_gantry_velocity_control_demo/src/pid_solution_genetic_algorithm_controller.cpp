#include "ep_gantry_velocity_control_demo/pid_constants.hpp"

#include <ep_common/node_id.hpp>
#include <ep_training/genetic_algorithm_controller_i.hpp>
#include <rclcpp/executors.hpp>

using SolnT = ep_gantry_velocity_control_demo::PidConstants;

namespace ep_training {

bool compare_fitness(const double lhs, const double rhs)
{
    return lhs > rhs;
}

template <>
std::vector<rcl_interfaces::msg::Parameter> node_parameters_from_solution(const SolnT& soln)
{
    std::vector<rcl_interfaces::msg::Parameter> parameters;

    rcl_interfaces::msg::Parameter p;
    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    p.name = "solution.kp";
    p.value.double_value = soln.kp;
    parameters.push_back(p);

    p.name = "solution.ki";
    p.value.double_value = soln.ki;
    parameters.push_back(p);

    p.name = "solution.kd";
    p.value.double_value = soln.kd;
    parameters.push_back(p);

    return parameters;
}

template <>
SolnT get_random_solution()
{
    std::random_device rd;
    std::mt19937 rng_engine(rd());
    std::uniform_real_distribution<> urd(-1.0, 1.0);
    return SolnT(
        50 * urd(rng_engine),
        50 * urd(rng_engine),
        50 * urd(rng_engine)
    );
}

template <>
void do_crossover_at(
    const SolnT& solnParent1,
    const SolnT& solnParent2,
    const double crossover_point,
    SolnT& solnOffspring1,
    SolnT& solnOffspring2
)
{
    // solnParent1 = a b c
    // solnParent2 = x y z

    double kp, ki, kd;
    if (crossover_point < (1.0 / 3))
    {
        // solnOffspring1 = a y z
        solnOffspring1.kp = solnParent1.kp;
        solnOffspring1.ki = solnParent2.ki;
        solnOffspring1.kd = solnParent2.kd;
        // solnOffspring2 = x b c
        solnOffspring2.kp = solnParent2.kp;
        solnOffspring2.ki = solnParent1.ki;
        solnOffspring2.kd = solnParent1.kd;
    }
    else if (crossover_point < (2.0 / 3))
    {
        // solnOffspring1 = a b z
        solnOffspring1.kp = solnParent1.kp;
        solnOffspring1.ki = solnParent1.ki;
        solnOffspring1.kd = solnParent2.kd;
        // solnOffspring2 = x y c
        solnOffspring2.kp = solnParent2.kp;
        solnOffspring2.ki = solnParent2.ki;
        solnOffspring2.kd = solnParent1.kd;
    }
    else
    {
        // solnOffspring1 = a y c
        solnOffspring1.kp = solnParent1.kp;
        solnOffspring1.ki = solnParent2.ki;
        solnOffspring1.kd = solnParent1.kd;
        // solnOffspring2 = x b z
        solnOffspring2.kp = solnParent2.kp;
        solnOffspring2.ki = solnParent1.ki;
        solnOffspring2.kd = solnParent2.kd;
    }
}

template <>
void mutate_solution(SolnT& soln, const double mutability)
{
    std::random_device rd;
    std::mt19937 rng_engine(rd());
    std::uniform_real_distribution<> urd(-1.0, 1.0);
    soln.kp *= (1.0 + (mutability * urd(rng_engine)));
    soln.ki *= (1.0 + (mutability * urd(rng_engine)));
    soln.kd *= (1.0 + (mutability * urd(rng_engine)));
}
}

int main(int argc, char** argv)
{
    rclcpp::init_and_remove_ros_arguments(argc, argv);
    auto node = std::make_shared<ep_training::GeneticAlgorithmControllerI<SolnT>>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    return 0;
}

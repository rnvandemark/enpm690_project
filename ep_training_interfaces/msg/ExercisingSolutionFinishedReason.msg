# A reason describing why exercising a solution has finished.

# Unknown/unset.
uint8 UNKNOWN = 0
# The time limit was reached.
uint8 TIME_LIMIT_REACHED = 1
# The task's goal was reached.
uint8 TASK_GOAL_REACHED = 2
# The solution's behavior converged such that the state of the agent is no
# longer changing significantly, but the goal was not reached.
uint8 CONVERGED_TO_INCOMPLETE_GOAL = 3
# There was a critical failure such that the agent has no chance of completing
# the goal.
uint8 CRITICAL_FAILURE = 4

# One of the above constants describing the reason.
uint8 value

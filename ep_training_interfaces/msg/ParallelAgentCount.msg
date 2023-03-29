# A number specifying how many agents can run in parallel.

# The maximum number of parallel agents allowed. There is a 32-bit mask in
# Gazebo which defines whether inter-entity collisions are enabled or not, so
# this means there can be 31 different agents that ignore collisions with one
# another but will all respect collisions with the rest of the world (e.g. the
# ground plane).
uint8 MAX_PARALLEL_AGENTS = 31

# The value in the bounds [0, MAX_PARALLEL_AGENTS]. A value of 0 indicates a
# 'default' value should be used.
uint8 value

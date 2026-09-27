# AI Robot Navigation Simulation

This project demonstrates an autonomous robot navigation system using A* pathfinding, sensor-based obstacle detection, automated decision-making, and performance monitoring.

## Features

- A* pathfinding for efficient robot navigation
- 10x10 simulated environment
- Sensor-based obstacle detection
- Dynamic obstacle avoidance
- Automatic route replanning
- Finite State Machine (FSM) style decision-making
- Automated movement from start to goal
- Performance and debugging data logging

## Robot States

The robot uses different states to control its behavior:

- SEARCHING - Calculates a route to the goal
- MOVING - Travels along the calculated route
- OBSTACLE DETECTED - Responds when a new obstacle blocks the route
- REPLANNING - Uses A* to calculate a new safe route
- SCANNING - Checks nearby locations for obstacles
- GOAL REACHED - Completes the navigation task
- STOPPED - Used when no safe route is available

## Obstacle Avoidance

The robot begins with a known map of obstacles. During navigation, a simulated sensor detects an unexpected obstacle. If the obstacle interferes with the planned path, the robot automatically changes states and recalculates a safe route using A*.

## Performance Monitoring

The program records information including:

- Original path length
- Updated path length
- Number of movement steps
- Sensor scans
- Total known obstacles
- Dynamic obstacles detected
- Replanning events
- Final robot position and state

This information can be used to monitor navigation efficiency and verify that the robot successfully completes its task.

## How to Run

1. Open `robot_simulation.py` using Python 3.
2. Run the program.
3. View the initial A* route.
4. Observe the simulated sensor detecting a new obstacle.
5. Watch the robot replan its route and continue toward the goal.
6. Review the performance report at the end.

## Symbols

- `R` = Robot
- `G` = Goal
- `X` = Obstacle
- `*` = Calculated path
- `.` = Open space

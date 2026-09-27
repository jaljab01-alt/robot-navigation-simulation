# AI Robot Navigation - Obstacle Avoidance and Automation
# Uses A* pathfinding, sensor-based obstacle detection,
# finite state decision-making, and performance logging.

import heapq

ROWS = 10
COLS = 10

start = (0, 0)
goal = (9, 9)

# Known obstacles
obstacles = {
    (1, 2), (2, 2), (3, 2),
    (4, 2), (5, 2),
    (5, 3), (5, 4), (5, 5),
    (7, 6), (7, 7), (7, 8)
}


# Estimate distance to the goal
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# A* pathfinding algorithm
def astar(start_position, goal_position, blocked):
    open_list = []
    heapq.heappush(open_list, (0, start_position))

    came_from = {}
    cost_so_far = {start_position: 0}

    while open_list:
        current = heapq.heappop(open_list)[1]

        if current == goal_position:
            break

        row, col = current

        neighbors = [
            (row - 1, col),
            (row + 1, col),
            (row, col - 1),
            (row, col + 1)
        ]

        for neighbor in neighbors:
            r, c = neighbor

            # Ignore positions outside the grid
            if r < 0 or r >= ROWS or c < 0 or c >= COLS:
                continue

            # Ignore obstacles
            if neighbor in blocked:
                continue

            new_cost = cost_so_far[current] + 1

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost

                priority = new_cost + heuristic(neighbor, goal_position)
                heapq.heappush(open_list, (priority, neighbor))

                came_from[neighbor] = current

    if goal_position not in cost_so_far:
        return None

    # Reconstruct path
    path = []
    current = goal_position

    while current != start_position:
        path.append(current)
        current = came_from[current]

    path.append(start_position)
    path.reverse()

    return path


# Display the environment and calculated route
def display_grid(robot, route):
    for row in range(ROWS):
        for col in range(COLS):
            position = (row, col)

            if position == robot:
                print("R", end=" ")
            elif position == goal:
                print("G", end=" ")
            elif position in obstacles:
                print("X", end=" ")
            elif route and position in route:
                print("*", end=" ")
            else:
                print(".", end=" ")

        print()


print("=== AI ROBOT NAVIGATION SYSTEM ===")

# STATE 1: SEARCHING
robot_state = "SEARCHING"
robot_position = start

print("\nRobot State:", robot_state)
print("Calculating initial A* route...")

initial_path = astar(robot_position, goal, obstacles)

if initial_path is None:
    robot_state = "STOPPED"
    print("No safe path available.")

else:
    print("Initial path found!")
    print("Initial path length:", len(initial_path) - 1)

    display_grid(robot_position, initial_path)

    # STATE 2: MOVING
    robot_state = "MOVING"
    print("\nRobot State:", robot_state)

    # Simulate sensor detecting an unexpected obstacle
    new_obstacle = (0, 5)

    print("\nSensor detected unexpected obstacle at:", new_obstacle)
    obstacles.add(new_obstacle)

    # STATE 3: OBSTACLE DETECTED
    if new_obstacle in initial_path:
        robot_state = "OBSTACLE DETECTED"
        print("Robot State:", robot_state)

        # STATE 4: REPLANNING
        robot_state = "REPLANNING"
        print("Robot State:", robot_state)

        updated_path = astar(robot_position, goal, obstacles)

    else:
        updated_path = initial_path

    if updated_path is None:
        robot_state = "STOPPED"
        print("No safe alternative route found.")

    else:
        print("Safe route available!")
        print("Updated path length:", len(updated_path) - 1)

        print("\n=== UPDATED ROUTE ===")
        display_grid(robot_position, updated_path)

        # Automated movement
        print("\n=== AUTOMATED TASK EXECUTION ===")

        steps_taken = 0
        sensor_scans = 0

        for next_position in updated_path[1:]:
            robot_state = "MOVING"

            print(
                "State:",
                robot_state,
                "| Moving:",
                robot_position,
                "->",
                next_position
            )

            robot_position = next_position
            steps_taken += 1

            # Sensor checks surrounding cells
            row, col = robot_position

            nearby_positions = [
                (row - 1, col),
                (row + 1, col),
                (row, col - 1),
                (row, col + 1)
            ]

            nearby_obstacles = [
                position
                for position in nearby_positions
                if position in obstacles
            ]

            if nearby_obstacles:
                robot_state = "SCANNING"
                sensor_scans += 1

                print(
                    "State:",
                    robot_state,
                    "| Nearby obstacle(s):",
                    nearby_obstacles
                )

        # Final state
        if robot_position == goal:
            robot_state = "GOAL REACHED"

        print("\nRobot State:", robot_state)
        print("Automation task completed!")

        # Performance / debugging data
        print("\n=== PERFORMANCE REPORT ===")

        original_length = len(initial_path) - 1
        updated_length = len(updated_path) - 1

        print("Original path length:", original_length)
        print("Updated path length:", updated_length)
        print("Movement steps:", steps_taken)
        print("Sensor scans triggered:", sensor_scans)
        print("Total known obstacles:", len(obstacles))
        print("Dynamic obstacles detected: 1")
        print("Replanning events: 1")

        if updated_length <= original_length:
            print("Navigation result: Efficient alternative route found")
        else:
            print("Navigation result: Safe route adjusted around obstacle")

        print("Final position:", robot_position)
        print("Final state:", robot_state)
        print("Task status: SUCCESS")

# Robotic Simulation Prototype Report

## AI Techniques Used

For this prototype, I created a basic two-dimensional mobile robot simulation in Python. The robot moves through a grid environment that contains several obstacles. I used Dijkstra's algorithm as the AI-based path-planning method. The algorithm searches for the shortest available path from the robot's starting position to the goal while avoiding blocked grid cells.

I also added simulated proximity sensor readings. The sensors check the distance from the robot to the nearest obstacle or wall in four directions: up, down, left, and right. These readings are displayed while the robot moves so the simulation includes basic sensor input in addition to path planning.

## Challenges and Solutions

One challenge was making sure the robot did not move through obstacles. I handled this by treating every obstacle as a blocked grid location when Dijkstra's algorithm searches for neighboring cells.

Another challenge was showing the robot's movement instead of immediately displaying the final result. I used Tkinter's timed update feature so the robot moves through the planned path one step at a time. This makes the path-planning process easier to demonstrate in a short screen recording.

Overall, the prototype shows how a robot can use an AI-based search algorithm and simulated sensor data to move safely from a starting point to a goal.

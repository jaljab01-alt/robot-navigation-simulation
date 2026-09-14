import tkinter as tk
import heapq

# -----------------------------
# Grid settings
# -----------------------------
ROWS = 12
COLS = 16
CELL = 40

START = (1, 1)
GOAL = (10, 14)

# Obstacles
OBSTACLES = {
    (2, 3), (3, 3), (4, 3), (5, 3), (6, 3),
    (6, 4), (6, 5), (6, 6), (6, 7),
    (3, 8), (4, 8), (5, 8), (6, 8), (7, 8),
    (8, 8), (8, 9), (8, 10), (8, 11),
    (2, 12), (3, 12), (4, 12)
}

# -----------------------------
# Dijkstra path planning
# -----------------------------
def dijkstra(start, goal):
    queue = [(0, start)]
    distances = {start: 0}
    previous = {}

    while queue:
        current_distance, current = heapq.heappop(queue)

        if current == goal:
            break

        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nr, nc = current[0] + dr, current[1] + dc
            neighbor = (nr, nc)

            if not (0 <= nr < ROWS and 0 <= nc < COLS):
                continue
            if neighbor in OBSTACLES:
                continue

            new_distance = current_distance + 1

            if new_distance < distances.get(neighbor, float("inf")):
                distances[neighbor] = new_distance
                previous[neighbor] = current
                heapq.heappush(queue, (new_distance, neighbor))

    if goal not in distances:
        return []

    path = []
    node = goal
    while node != start:
        path.append(node)
        node = previous[node]

    path.append(start)
    path.reverse()
    return path


# -----------------------------
# Simulated sensor readings
# -----------------------------
def sensor_readings(position):
    """Simulate four simple proximity sensors.
    Returns distance in grid cells to nearest obstacle or wall
    in the up, down, left, and right directions.
    """
    r, c = position
    readings = {}

    directions = {
        "Up": (-1, 0),
        "Down": (1, 0),
        "Left": (0, -1),
        "Right": (0, 1)
    }

    for name, (dr, dc) in directions.items():
        distance = 0
        nr, nc = r, c

        while True:
            nr += dr
            nc += dc
            distance += 1

            if not (0 <= nr < ROWS and 0 <= nc < COLS):
                break
            if (nr, nc) in OBSTACLES:
                break

        readings[name] = distance

    return readings


# -----------------------------
# GUI
# -----------------------------
class RobotSimulation:
    def __init__(self, root):
        self.root = root
        self.root.title("AI Robot Navigation - Dijkstra Simulation")

        self.canvas = tk.Canvas(
            root,
            width=COLS * CELL,
            height=ROWS * CELL,
            bg="white"
        )
        self.canvas.pack()

        self.sensor_label = tk.Label(
            root,
            text="Sensor readings will appear here.",
            font=("Arial", 11)
        )
        self.sensor_label.pack(pady=6)

        self.start_button = tk.Button(
            root,
            text="Start Robot",
            command=self.start_robot
        )
        self.start_button.pack(pady=6)

        self.path = dijkstra(START, GOAL)
        self.step = 0
        self.robot_position = START

        self.draw_grid()
        self.draw_scene()

    def draw_grid(self):
        for r in range(ROWS):
            for c in range(COLS):
                x1 = c * CELL
                y1 = r * CELL
                x2 = x1 + CELL
                y2 = y1 + CELL

                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    outline="gray"
                )

    def draw_scene(self):
        self.canvas.delete("object")

        # Draw obstacles
        for r, c in OBSTACLES:
            self.canvas.create_rectangle(
                c * CELL, r * CELL,
                (c + 1) * CELL, (r + 1) * CELL,
                fill="black",
                tags="object"
            )

        # Draw planned path
        for r, c in self.path:
            if (r, c) not in (START, GOAL):
                self.canvas.create_rectangle(
                    c * CELL + 10, r * CELL + 10,
                    (c + 1) * CELL - 10, (r + 1) * CELL - 10,
                    fill="lightblue",
                    outline="",
                    tags="object"
                )

        # Start
        sr, sc = START
        self.canvas.create_rectangle(
            sc * CELL, sr * CELL,
            (sc + 1) * CELL, (sr + 1) * CELL,
            fill="green",
            tags="object"
        )
        self.canvas.create_text(
            sc * CELL + CELL / 2,
            sr * CELL + CELL / 2,
            text="S",
            fill="white",
            font=("Arial", 14, "bold"),
            tags="object"
        )

        # Goal
        gr, gc = GOAL
        self.canvas.create_rectangle(
            gc * CELL, gr * CELL,
            (gc + 1) * CELL, (gr + 1) * CELL,
            fill="red",
            tags="object"
        )
        self.canvas.create_text(
            gc * CELL + CELL / 2,
            gr * CELL + CELL / 2,
            text="G",
            fill="white",
            font=("Arial", 14, "bold"),
            tags="object"
        )

        # Robot
        rr, rc = self.robot_position
        self.canvas.create_oval(
            rc * CELL + 7, rr * CELL + 7,
            (rc + 1) * CELL - 7, (rr + 1) * CELL - 7,
            fill="orange",
            outline="brown",
            width=2,
            tags="object"
        )

    def start_robot(self):
        if not self.path:
            self.sensor_label.config(text="No valid path to the goal.")
            return

        self.step = 0
        self.robot_position = self.path[0]
        self.move_robot()

    def move_robot(self):
        if self.step >= len(self.path):
            self.sensor_label.config(text="Goal reached!")
            return

        self.robot_position = self.path[self.step]
        readings = sensor_readings(self.robot_position)

        self.sensor_label.config(
            text=(
                f"Robot: {self.robot_position} | "
                f"Sensors - Up: {readings['Up']}, "
                f"Down: {readings['Down']}, "
                f"Left: {readings['Left']}, "
                f"Right: {readings['Right']}"
            )
        )

        self.draw_scene()
        self.step += 1

        self.root.after(300, self.move_robot)


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulation(root)
    root.mainloop()

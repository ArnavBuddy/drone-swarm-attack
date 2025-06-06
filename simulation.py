import random
import logging
from collections import deque

# Logging
logging.basicConfig(filename="drone_log.txt", level=logging.INFO,
                    format="%(asctime)s - %(message)s")

class DroneSimulation:
    def __init__(self, max_steps=20, jamming_radius=1, grid_size=6):
        self.grid_size = grid_size
        self.max_steps = max_steps
        self.jamming_radius = jamming_radius
        self.num_drones = 3
        self.reset()

    def reset(self):
        positions = [(i, j) for i in range(self.grid_size) for j in range(self.grid_size)]
        random.shuffle(positions)

        self.drones = positions[:self.num_drones]
        self.targets = positions[self.num_drones:2 * self.num_drones]
        self.jamming_zones = positions[2 * self.num_drones:2 * self.num_drones + 3]

        self.step_count = 0
        self.jammed_positions = []
        self.logged_jams = set()
        self.jam_log = []
        self.avoidance_memory = {i: set() for i in range(self.num_drones)}
        self.success_attempts = [0] * self.num_drones
        self.total_attempts = [0] * self.num_drones

    def is_near_jamming_zone(self, x, y):
        for jx, jy in self.jamming_zones:
            if abs(jx - x) + abs(jy - y) <= self.jamming_radius:
                return True
        return False

    def should_jam(self, i, x, y):
        if self.is_near_jamming_zone(x, y):
            return random.random() < 0.4
        return random.random() < 0.05

    def is_valid(self, x, y):
        return 0 <= x < self.grid_size and 0 <= y < self.grid_size

    def bfs_path(self, start, goal, blocked):
        queue = deque([(start, [])])
        visited = set([start])
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while queue:
            (x, y), path = queue.popleft()
            if (x, y) == goal:
                return path

            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if self.is_valid(nx, ny) and (nx, ny) not in visited and (nx, ny) not in blocked:
                    visited.add((nx, ny))
                    queue.append(((nx, ny), path + [(nx, ny)]))
        return []

    def next_step(self):
        if self.step_count >= self.max_steps:
            logging.info("Simulation ended.")
            logging.info(f"Total steps: {self.step_count}")
            self.save_summary()
            return None

        self.step_count += 1
        new_positions = []
        self.jammed_positions = []

        for i, (x, y) in enumerate(self.drones):
            self.total_attempts[i] += 1

            if i == 1:
                directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
                random.shuffle(directions)
                moved = False
                for dx, dy in directions:
                    nx, ny = x + dx, y + dy
                    if self.is_valid(nx, ny):
                        next_x, next_y = nx, ny
                        moved = True
                        break
                if not moved:
                    next_x, next_y = x, y
            else:
                target = self.targets[i]
                blocked = set(self.jamming_zones) | self.avoidance_memory[i]
                path = self.bfs_path((x, y), target, blocked)
                if path:
                    next_x, next_y = path[0]
                else:
                    next_x, next_y = x, y

            if self.should_jam(i, next_x, next_y):
                self.jammed_positions.append((next_x, next_y))
                jammed_pos = (next_x + 1, next_y + 1)
                log_entry = f"Drone {i} is jammed at position {jammed_pos}!"
                if log_entry not in self.logged_jams:
                    logging.info(log_entry)
                    self.logged_jams.add(log_entry)
                self.jam_log.append((self.step_count, i, jammed_pos))

                if i != 1:
                    self.avoidance_memory[i].add((next_x, next_y))

                new_positions.append((x, y))
                print(f"[STEP {self.step_count}] Drone {i} JAMMED at {next_x},{next_y}")
            else:
                new_positions.append((next_x, next_y))
                print(f"[STEP {self.step_count}] Drone {i} moved to {next_x},{next_y}")

                if (next_x, next_y) == self.targets[i]:
                    self.success_attempts[i] += 1

        self.drones = new_positions
        return self.drones, self.jammed_positions
    
    def save_summary(self):
        with open("simulation_summary.txt", "w") as f:
            f.write(f"Simulation completed in {self.step_count} steps.\n\n")
            for i in range(self.num_drones):
                success_rate = self.success_attempts[i] / self.total_attempts[i] if self.total_attempts[i] != 0 else 0
                f.write(f"Drone {i} Success Rate: {success_rate:.2f} ({self.success_attempts[i]}/{self.total_attempts[i]})\n")
        
            f.write("\nJamming Log:\n")
            for step, drone_id, pos in self.jam_log:
                f.write(f"Step {step}: Drone {drone_id} jammed at {pos}\n")


    def get_scores(self):
        return [s / t if t != 0 else 0 for s, t in zip(self.success_attempts, self.total_attempts)]

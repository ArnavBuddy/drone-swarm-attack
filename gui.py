import tkinter as tk
import math
from collections import defaultdict
from simulation import DroneSimulation  # Make sure this file exists
import cv2
import numpy as np
from PIL import ImageGrab
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

class DroneSwarmGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Drone Swarm Visualization")

        # Grid and animation settings
        self.cell_size = 80
        self.grid_size = 6
        self.animation_steps = 10
        self.animation_delay = 50

        # Video and simulation settings
        self.video_fps = 2
        self.export_video_after_sim = True
        self.is_running = False
        self.jamming_zones = [(2, 3), (4, 1), (1, 4)]

        # Initialize simulation
        self.simulation = DroneSimulation()
        self.targets = self.simulation.targets
        self.current_positions = self.simulation.drones.copy()
        self.jamming_zones = self.simulation.jamming_zones
        self.jammed_positions = []
        self.drones = []
        self.video_frames = []

        # === GUI Layout ===
        main_frame = tk.Frame(master)
        main_frame.pack(padx=10, pady=10)

        # Canvas for grid
        self.canvas = tk.Canvas(main_frame, width=self.grid_size * self.cell_size,
                                height=self.grid_size * self.cell_size, bg="lightgray")
        self.canvas.grid(row=0, column=0, rowspan=2)

        # Control panel
        control_frame = tk.Frame(main_frame)
        control_frame.grid(row=0, column=1, padx=10)

        # Simulation parameters
        param_frame = tk.LabelFrame(control_frame, text="Simulation Parameters", padx=10, pady=5)
        param_frame.pack(fill="x", pady=5)

        tk.Label(param_frame, text="Max Steps:").grid(row=0, column=0, sticky="w")
        self.max_steps_entry = tk.Entry(param_frame, width=5)
        self.max_steps_entry.insert(0, "20")
        self.max_steps_entry.grid(row=0, column=1, padx=5)

        tk.Label(param_frame, text="Jamming Radius:").grid(row=1, column=0, sticky="w")
        self.jamming_radius_entry = tk.Entry(param_frame, width=5)
        self.jamming_radius_entry.insert(0, "1")
        self.jamming_radius_entry.grid(row=1, column=1, padx=5)

        # Buttons
        button_frame = tk.LabelFrame(control_frame, text="Controls", padx=10, pady=5)
        button_frame.pack(fill="x", pady=5)

        self.step_button = tk.Button(button_frame, text="Next Step", width=12, command=self.step)
        self.step_button.grid(row=0, column=0, pady=2)

        self.play_button = tk.Button(button_frame, text="\u25B6 Play", width=12, command=self.toggle_play)
        self.play_button.grid(row=1, column=0, pady=2)

        self.pause_button = tk.Button(button_frame, text="\u23F8 Pause", width=12, command=self.toggle_pause, state=tk.DISABLED)
        self.pause_button.grid(row=2, column=0, pady=2)

        self.restart_button = tk.Button(button_frame, text="\u21BB Restart", width=12, command=self.restart_simulation)
        self.restart_button.grid(row=3, column=0, pady=2)

        # Matplotlib chart for score tracking
        self.fig = Figure(figsize=(5, 3), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Drone Success Score Over Time")
        self.ax.set_xlabel("Step")
        self.ax.set_ylabel("Success Rate")
        self.score_lines = []
        self.score_data = [[] for _ in range(len(self.simulation.drones))]
        self.x_data = []

        self.canvas_fig = FigureCanvasTkAgg(self.fig, master=main_frame)
        self.canvas_fig.draw()
        self.canvas_fig.get_tk_widget().grid(row=1, column=1, sticky="nsew")

        # Initialize visuals
        self.draw_grid()
        self.draw_targets()
        self.draw_target_lines()
        self.init_drones()
        self.capture_frame()

    def draw_grid(self):
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                x0, y0 = j * self.cell_size, i * self.cell_size
                x1, y1 = x0 + self.cell_size, y0 + self.cell_size
                fill = "#ffcccc" if (i, j) in self.jamming_zones else ""
                self.canvas.create_rectangle(x0, y0, x1, y1, outline="gray", fill=fill)
                if (i, j) in self.jamming_zones:
                    self.canvas.create_text(x0 + 40, y0 + 40, text="J", fill="red", font=("Helvetica", 12, "bold"))

    def draw_targets(self):
        for i, (x, y) in enumerate(self.targets):
            cx, cy = y * self.cell_size + 40, x * self.cell_size + 40
            self.canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill="green")
            self.canvas.create_text(cx, cy - 14, text=f"T{i}", fill="black")

    def draw_target_lines(self):
        for i, (dx, dy) in enumerate(self.current_positions):
            tx, ty = self.targets[i]
            sx, sy = dy * self.cell_size + 40, dx * self.cell_size + 40
            ex, ey = ty * self.cell_size + 40, tx * self.cell_size + 40
            self.canvas.create_line(sx, sy, ex, ey, fill="gray", dash=(4, 2))

    def init_drones(self):
        for i, (x, y) in enumerate(self.current_positions):
            oval, label = self.draw_drone_at(x, y, i, "blue", 0, 1)
            self.drones.append((oval, label))

    def draw_drone_at(self, x, y, drone_id, color, offset_index, total):
        angle = (2 * math.pi / total) * offset_index
        offset_x = 10 * math.cos(angle)
        offset_y = 10 * math.sin(angle)
        cx, cy = y * self.cell_size + 40 + offset_x, x * self.cell_size + 40 + offset_y
        oval = self.canvas.create_oval(cx - 15, cy - 15, cx + 15, cy + 15, fill=color)
        label = self.canvas.create_text(cx, cy, text=f"D{drone_id}", fill="white")
        return oval, label

    def get_jammed_positions(self, positions):
        return [(x, y) for x, y in positions if (x, y) in self.jamming_zones]

    def animate_step(self, count, new_positions):
        if count >= self.animation_steps:
            self.current_positions = new_positions
            self.update_positions_and_colors()
            self.capture_frame()
            if self.is_running:
                self.master.after(500, self.step)
            return
        for (oval, label), (ox, oy), (nx, ny) in zip(self.drones, self.current_positions, new_positions):
            dx = ((ny - oy) * self.cell_size) / self.animation_steps
            dy = ((nx - ox) * self.cell_size) / self.animation_steps
            self.canvas.move(oval, dx, dy)
            self.canvas.move(label, dx, dy)
        self.master.after(self.animation_delay, lambda: self.animate_step(count + 1, new_positions))

    def update_positions_and_colors(self):
        for oval, label in self.drones:
            self.canvas.delete(oval)
            self.canvas.delete(label)
        self.drones.clear()

        self.jammed_positions = self.get_jammed_positions(self.current_positions)
        mapping = defaultdict(list)
        for i, pos in enumerate(self.current_positions):
            mapping[pos].append(i)

        for (x, y), ids in mapping.items():
            for idx, drone_id in enumerate(ids):
                color = "red" if (x, y) in self.jammed_positions else "blue"
                self.drones.append(self.draw_drone_at(x, y, drone_id, color, idx, len(ids)))

    def step(self):
        result = self.simulation.next_step()
        if result is None:
            self.is_running = False
            self.step_button.config(state=tk.DISABLED)
            self.pause_button.config(state=tk.DISABLED)
            self.play_button.config(state=tk.NORMAL)
            if self.export_video_after_sim:
                self.export_video()
            return

        new_positions, _ = result
        scores = self.simulation.get_scores()
        self.x_data.append(self.simulation.step_count)

        if not self.score_lines:
            for i in range(len(scores)):
                line, = self.ax.plot([], [], label=f"Drone {i}")
                self.score_lines.append(line)
            self.ax.legend()

        for i, score in enumerate(scores):
            self.score_data[i].append(score)
            self.score_lines[i].set_data(self.x_data, self.score_data[i])

        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas_fig.draw()

        self.animate_step(0, new_positions)

    def toggle_play(self):
        self.is_running = True
        self.play_button.config(state=tk.DISABLED)
        self.pause_button.config(state=tk.NORMAL)
        self.step()

    def toggle_pause(self):
        self.is_running = False
        self.play_button.config(state=tk.NORMAL)
        self.pause_button.config(state=tk.DISABLED)

    def restart_simulation(self):
        try:
            max_steps = int(self.max_steps_entry.get())
            radius = int(self.jamming_radius_entry.get())
        except ValueError:
            max_steps = 20
            radius = 1

        self.simulation = DroneSimulation(max_steps=max_steps, jamming_radius=radius)
        self.current_positions = self.simulation.drones.copy()
        self.targets = self.simulation.targets
        self.jamming_zones = self.simulation.jamming_zones

        self.x_data.clear()
        self.video_frames.clear()
        for i in range(len(self.score_data)):
            self.score_data[i].clear()
        self.ax.clear()
        self.ax.set_title("Drone Success Score Over Time")
        self.ax.set_xlabel("Step")
        self.ax.set_ylabel("Success Rate")
        self.score_lines.clear()
        self.canvas_fig.draw()

        self.step_button.config(state=tk.NORMAL)
        self.play_button.config(state=tk.NORMAL)
        self.pause_button.config(state=tk.DISABLED)
        self.is_running = False
        self.canvas.delete("all")
        self.draw_grid()
        self.draw_targets()
        self.draw_target_lines()
        self.drones.clear()
        self.init_drones()
        self.capture_frame()

    def capture_frame(self):
        self.master.update()
        x = self.master.winfo_rootx() + self.canvas.winfo_x()
        y = self.master.winfo_rooty() + self.canvas.winfo_y()
        x1 = x + self.canvas.winfo_width()
        y1 = y + self.canvas.winfo_height()
        img = ImageGrab.grab().crop((x, y, x1, y1)).convert("RGB")
        frame = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        self.video_frames.append(frame)

    def export_video(self):
        if not self.video_frames:
            return
        h, w, _ = self.video_frames[0].shape
        out = cv2.VideoWriter("drone_simulation.mp4", cv2.VideoWriter_fourcc(*'mp4v'), self.video_fps, (w, h))
        for frame in self.video_frames:
            out.write(frame)
        out.release()
        print("Video saved as drone_simulation.mp4")


if __name__ == "__main__":
    root = tk.Tk()
    app = DroneSwarmGUI(root)
    root.mainloop()
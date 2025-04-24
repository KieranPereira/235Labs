import tkinter as tk
import random
import math
import time
import bisect

# global threshold values in degrees
ROLL_THRESHOLD = 10
PITCH_THRESHOLD = 10
MAX_POINTS = 1000

class IMULineViewer:
    def __init__(self, master):
        self.master = master
        master.title("IMU Roll & Pitch History")

        # canvas parameters
        self.size   = 300
        self.center = self.size // 2
        self.length = 200
        self.half   = self.length / 2

        # buffers for time, roll, pitch
        self.time_data  = []
        self.roll_data  = []
        self.pitch_data = []

        # follow-live flag
        self.follow = True
        self.start_time = time.time()

        # --- Roll canvas ---
        self.roll_canvas = tk.Canvas(master, width=self.size, height=self.size, bg="#f0f0f0")
        self.roll_canvas.grid(row=0, column=0, padx=10, pady=10)
        self.roll_canvas.create_text(self.center, 20, text="Roll", font=("Arial", 16))
        x1, y1 = self.center - self.half, self.center
        x2, y2 = self.center + self.half, self.center
        self.roll_line  = self.roll_canvas.create_line(x1, y1, x2, y2, width=4, fill="blue")
        self.roll_label = self.roll_canvas.create_text(
            self.center, self.size - 20, text="0.0°", font=("Arial", 12), fill="black"
        )

        # --- Pitch canvas ---
        self.pitch_canvas = tk.Canvas(master, width=self.size, height=self.size, bg="#f0f0f0")
        self.pitch_canvas.grid(row=0, column=1, padx=10, pady=10)
        self.pitch_canvas.create_text(self.center, 20, text="Pitch", font=("Arial", 16))
        self.pitch_line  = self.pitch_canvas.create_line(x1, y1, x2, y2, width=4, fill="green")
        self.pitch_label = self.pitch_canvas.create_text(
            self.center, self.size - 20, text="0.0°", font=("Arial", 12), fill="black"
        )

        # --- History scrollbar showing timestamps (seconds since start) ---
        self.slider = tk.Scale(
            master,
            from_=0.0,
            to=0.0,
            resolution=0.1,
            tickinterval=1.0,
            orient=tk.HORIZONTAL,
            length=self.size * 2,
            label="Time (s)",
            showvalue=True
        )
        self.slider.grid(row=1, column=0, columnspan=2, padx=10, pady=(0,5))
        # Bind any click on the slider to pause live-follow
        self.slider.bind("<Button-1>", lambda e: setattr(self, 'follow', False))

        # --- Sync button ---
        self.sync_button = tk.Button(master, text="Sync to Live", command=self.sync_to_live)
        self.sync_button.grid(row=2, column=0, columnspan=2, pady=(0,10))

        # start the update loop
        self.master.after(0, self.update_loop)

    def read_sensor_data(self):
        # <-- replace this stub with your BLE/ESP32 read logic -->
        return random.uniform(-30, 30), random.uniform(-30, 30)

    def sync_to_live(self):
        """Jump slider to the newest timestamp and resume live-follow."""
        if not self.time_data:
            return
        latest = self.time_data[-1]
        self.follow = True
        self.slider.set(latest)

    def update_loop(self):
        # read and store new sample
        now = time.time() - self.start_time
        roll, pitch = self.read_sensor_data()
        self.time_data.append(now)
        self.roll_data.append(roll)
        self.pitch_data.append(pitch)

        # trim history
        if len(self.time_data) > MAX_POINTS:
            self.time_data.pop(0)
            self.roll_data.pop(0)
            self.pitch_data.pop(0)

        # update slider range
        new_max = self.time_data[-1]
        self.slider.config(to=new_max)
        if self.follow:
            # keep slider pinned to live end
            self.slider.set(new_max)

        # redraw at the selected time
        self.redraw()

        # schedule next update
        self.master.after(100, self.update_loop)

    def redraw(self):
        t = self.slider.get()
        # find closest index by timestamp
        idx = bisect.bisect_left(self.time_data, t)
        if idx == 0:
            nearest = 0
        elif idx >= len(self.time_data):
            nearest = len(self.time_data) - 1
        else:
            before, after = self.time_data[idx-1], self.time_data[idx]
            nearest = idx if abs(after - t) < abs(t - before) else idx-1

        roll  = self.roll_data[nearest]
        pitch = self.pitch_data[nearest]

        # draw roll needle
        r_rad = math.radians(roll)
        dx, dy = self.half * math.cos(r_rad), self.half * math.sin(r_rad)
        x1, y1 = self.center - dx, self.center + dy
        x2, y2 = self.center + dx, self.center - dy
        self.roll_canvas.coords(self.roll_line, x1, y1, x2, y2)
        color = "red" if abs(roll) > ROLL_THRESHOLD else "black"
        self.roll_canvas.itemconfig(self.roll_label, text=f"{roll:.1f}°", fill=color)

        # draw pitch needle
        p_rad = math.radians(pitch)
        dx, dy = self.half * math.cos(p_rad), self.half * math.sin(p_rad)
        x1, y1 = self.center - dx, self.center + dy
        x2, y2 = self.center + dx, self.center - dy
        self.pitch_canvas.coords(self.pitch_line, x1, y1, x2, y2)
        color = "red" if abs(pitch) > PITCH_THRESHOLD else "black"
        self.pitch_canvas.itemconfig(self.pitch_label, text=f"{pitch:.1f}°", fill=color)

if __name__ == "__main__":
    root = tk.Tk()
    IMULineViewer(root)
    root.mainloop()

import numpy as np
import matplotlib.pyplot as plt

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtCore import QTimer
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D

def euler_to_rotation_matrix(roll, pitch, yaw):
    # [Unchanged helper function]
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    return Rz @ Ry @ Rx

class Visualizer3dTab(QWidget):
    def __init__(
        self,
        data_lock,
        upper_arm_euler_ref,
        forearm_euler_ref,
        upper_arm_length=0.4,
        forearm_length=0.35,
        update_interval_ms=50
    ):
        super().__init__()
        self.data_lock = data_lock
        self.upper_arm_euler_ref = upper_arm_euler_ref
        self.forearm_euler_ref   = forearm_euler_ref
        self.upper_arm_length = upper_arm_length
        self.forearm_length   = forearm_length
        self.update_interval_ms = update_interval_ms
        
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()
        self.fig = plt.figure(figsize=(6, 5))

        # Dark figure background
        self.fig.patch.set_facecolor('#202020')
        self.canvas = FigureCanvas(self.fig)

        self.ax = self.fig.add_subplot(111, projection='3d')
        # Dark axes background
        self.ax.set_facecolor('#202020')

        # -- Remove or tone down tick labels
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])
        self.ax.set_zticklabels([])

        # If desired, modify the color of the 3D panes:
        self.ax.xaxis.pane.set_edgecolor('#444444')
        self.ax.xaxis.pane.set_facecolor('#202020')
        self.ax.yaxis.pane.set_edgecolor('#444444')
        self.ax.yaxis.pane.set_facecolor('#202020')
        self.ax.zaxis.pane.set_edgecolor('#444444')
        self.ax.zaxis.pane.set_facecolor('#202020')

        # Optional: change axis lines and label colors
        # self.ax.xaxis.line.set_color('#ffffff')
        # self.ax.yaxis.line.set_color('#ffffff')
        # self.ax.zaxis.line.set_color('#ffffff')

        # Turn off the default axis grid for a cleaner look
        self.ax.grid(False)

        self.ax.set_title("3D Euler Tracking", color="white")

        self.toolbar = NavigationToolbar(self.canvas, self)
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        # Create the 3D lines for upper arm and forearm
        self.shoulder_pos = np.array([0.0, 0.0, 0.0])
        self.upper_arm_line, = self.ax.plot([], [], [], 'o-', lw=6, markersize=10,
                                            color='#4cc9f0', alpha=0.8)
        self.forearm_line, = self.ax.plot([], [], [], 'o-', lw=6, markersize=10,
                                        color='#4895ef', alpha=0.8)

        self.ax.set_xlim3d(-1, 1)
        self.ax.set_ylim3d(-1, 1)
        self.ax.set_zlim3d(-1, 1)

        self.ax.view_init(elev=20, azim=45)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(self.update_interval_ms)

    def update_plot(self):
        with self.data_lock:
            upper_euler_deg = self.upper_arm_euler_ref[:]
            forearm_euler_deg = self.forearm_euler_ref[:]

        upper_euler_rad = np.radians(upper_euler_deg)
        forearm_euler_rad = np.radians(forearm_euler_deg)

        R_upper = euler_to_rotation_matrix(*upper_euler_rad)
        R_forearm = euler_to_rotation_matrix(*forearm_euler_rad)

        # Upper arm endpoint
        upper_arm_end = R_upper @ np.array([self.upper_arm_length, 0, 0], dtype=float)
        # Forearm endpoint relative to upper arm
        forearm_end = upper_arm_end + R_forearm @ np.array([self.forearm_length, 0, 0], dtype=float)

        # Update the lines
        self.upper_arm_line.set_data_3d(
            [self.shoulder_pos[0], upper_arm_end[0]],
            [self.shoulder_pos[1], upper_arm_end[1]],
            [self.shoulder_pos[2], upper_arm_end[2]]
        )
        self.forearm_line.set_data_3d(
            [upper_arm_end[0], forearm_end[0]],
            [upper_arm_end[1], forearm_end[1]],
            [upper_arm_end[2], forearm_end[2]]
        )

        # Dynamically adjust the axes for the furthest endpoint
        max_val = max(np.abs(forearm_end).max(), 1.0)
        self.ax.set_xlim3d(-max_val, max_val)
        self.ax.set_ylim3d(-max_val, max_val)
        self.ax.set_zlim3d(-max_val, max_val)

        self.canvas.draw_idle()

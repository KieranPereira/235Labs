import sys
import serial
import threading
import numpy as np
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, 
                            QVBoxLayout)
from PyQt6.QtCore import QTimer
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from mpl_toolkits.mplot3d import Axes3D

# ======== CONFIGURATION ========
ESP32_UPPER_ARM = "/dev/cu.usbserial-1440"  # Upper Arm IMU
ESP32_FOREARM = "/dev/cu.usbserial-1430"    # Forearm IMU
BAUD_RATE = 115200
UPDATE_INTERVAL_MS = 50
# ===============================

# Shared variables with thread lock
upper_arm_quat = np.array([1.0, 0.0, 0.0, 0.0])  # (w, x, y, z)
forearm_quat = np.array([1.0, 0.0, 0.0, 0.0])
angle_lock = threading.Lock()

def read_imu(port, label):
    """ Read IMU quaternion data from ESP32 with error handling """
    global upper_arm_quat, forearm_quat
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"✅ Connected to {label} ({port})")
        
        while True:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    
                    if "Quat" in line:
                        parts = line.split(":")[1].split(",")
                        quat = np.array([float(p) for p in parts], dtype=np.float32)
                        
                        with angle_lock:
                            if label == "Upper Arm":
                                upper_arm_quat = quat
                            elif label == "Forearm":
                                forearm_quat = quat

                except Exception as e:
                    print(f"❌ Parse error: {str(e)} in line: {line}")

    except Exception as e:
        print(f"❌ Connection error ({label}): {str(e)}")

# Start serial threads
threading.Thread(target=read_imu, args=(ESP32_UPPER_ARM, "Upper Arm"), daemon=True).start()
threading.Thread(target=read_imu, args=(ESP32_FOREARM, "Forearm"), daemon=True).start()

class BiometricVisualizer3D(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Window configuration
        self.setWindowTitle("3D Lateral Raise Analyzer")
        self.setGeometry(100, 100, 1200, 800)
        
        # 3D Figure setup
        self.fig = plt.figure(figsize=(10, 8))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.toolbar = NavigationToolbar(self.canvas, self)
        
        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Arm parameters
        self.upper_arm_length = 0.4
        self.forearm_length = 0.35
        self.shoulder_pos = np.array([0, 0, 0])
        
        # Initialize arm segments with distinct colors
        self.upper_arm_line, = self.ax.plot([], [], [], 'o-', 
                                           lw=6, markersize=12, 
                                           color='#1a759f', alpha=0.8)
        self.forearm_line, = self.ax.plot([], [], [], 'o-', 
                                        lw=6, markersize=12, 
                                        color='#76c893', alpha=0.8)
        
        # Configure 3D plot limits and view
        self.ax.set_xlim3d(-1, 1)
        self.ax.set_ylim3d(-1, 1)
        self.ax.set_zlim3d(-1, 1)
        self.ax.view_init(elev=20, azim=45)
        
        # Timer setup with blitting optimization
        self.timer = QTimer()
        self.timer.setInterval(UPDATE_INTERVAL_MS)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

        # Store line references for blitting
        self._blit_artists = [self.upper_arm_line, self.forearm_line]

    def quaternion_rotation_matrix(self, q):
        """ Convert quaternion to rotation matrix """
        w, x, y, z = q
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])

    def update_plot(self):
        """ Update 3D arm positions with blitting optimization """
        global upper_arm_quat, forearm_quat
        
        with angle_lock:
            q_upper = upper_arm_quat.copy()
            q_forearm = forearm_quat.copy()

        # Calculate upper arm orientation
        R_upper = self.quaternion_rotation_matrix(q_upper)
        upper_arm_end = R_upper @ np.array([0, self.upper_arm_length, 0])
        
        # Calculate forearm orientation relative to upper arm
        R_forearm = self.quaternion_rotation_matrix(q_forearm)
        forearm_end = upper_arm_end + R_forearm @ np.array([0, self.forearm_length, 0])

        # Update line data
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

        # Dynamic axis scaling
        max_val = max(np.abs(forearm_end).max(), 1.0)
        self.ax.set_xlim3d(-max_val, max_val)
        self.ax.set_ylim3d(-max_val, max_val)
        self.ax.set_zlim3d(-max_val, max_val)

        # Force redraw with blitting
        self.canvas.draw_idle()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = BiometricVisualizer3D()
    window.show()
    sys.exit(app.exec())

import sys
import json
import serial
import threading
import time
import numpy as np

from PyQt6.QtWidgets import QApplication, QMainWindow, QTabWidget
from PyQt6.QtGui import QPalette, QColor

# Import your custom 2D and 3D tabs (unchanged except for any color updates)
from lateralraise2d import LateralRaise2DTab
from visualizer_3d import Visualizer3dTab

from smoothing import MovingAverageFilter

data_lock = threading.Lock()

shoulder_angle = [0.0]
elbow_angle    = [0.0]

upper_arm_euler = [0.0, 0.0, 0.0]
forearm_euler   = [0.0, 0.0, 0.0]

ESP32_UPPER_ARM = "COM7"
ESP32_FOREARM   = "COM10"
BAUD_RATE = 115200

upper_arm_roll_filter  = MovingAverageFilter(window_size=5)
upper_arm_pitch_filter = MovingAverageFilter(window_size=5)
upper_arm_yaw_filter   = MovingAverageFilter(window_size=5)

forearm_roll_filter  = MovingAverageFilter(window_size=5)
forearm_pitch_filter = MovingAverageFilter(window_size=5)
forearm_yaw_filter   = MovingAverageFilter(window_size=5)

def read_imu(port, label):
    #  [ Code is unchanged, handles reading and smoothing sensor data. ]
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"✅ Connected to {label} ({port})")
    except Exception as e:
        print(f"❌ Connection error ({label}): {e}")
        return

    while True:
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                try:
                    data = json.loads(line)
                    roll_value  = data.get("roll", 0.0)
                    pitch_value = data.get("pitch", 0.0)
                    yaw_value   = data.get("yaw", 0.0)

                    if label == "Upper Arm":
                        smoothed_roll  = upper_arm_roll_filter.update(roll_value)
                        smoothed_pitch = upper_arm_pitch_filter.update(pitch_value)
                        smoothed_yaw   = upper_arm_yaw_filter.update(yaw_value)
                    else:  # Forearm
                        smoothed_roll  = forearm_roll_filter.update(roll_value)
                        smoothed_pitch = forearm_pitch_filter.update(pitch_value)
                        smoothed_yaw   = forearm_yaw_filter.update(yaw_value)

                    with data_lock:
                        if label == "Upper Arm":
                            upper_arm_euler[0] = smoothed_roll
                            upper_arm_euler[1] = smoothed_pitch
                            upper_arm_euler[2] = smoothed_yaw
                            shoulder_angle[0]   = smoothed_pitch
                        else:
                            forearm_euler[0] = smoothed_roll
                            forearm_euler[1] = smoothed_pitch
                            forearm_euler[2] = smoothed_yaw
                            elbow_angle[0]   = smoothed_pitch
                except json.JSONDecodeError:
                    pass
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"❌ Error reading from {label}: {e}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motion Tracking - Demo")
        self.setGeometry(100, 100, 1200, 800)

        # Create the QTabWidget
        self.tabs = QTabWidget()

        # OPTIONAL: Style the QTabWidget to have a flat, dark look
        self.tabs.setStyleSheet("""
            QTabBar::tab {
                background: #2f2f2f; 
                color: #ffffff; 
                padding: 10px; 
                margin: 3px; 
                border-top-left-radius: 4px; 
                border-top-right-radius: 4px;
            }
            QTabBar::tab:hover {
                background: #444444;
            }
            QTabBar::tab:selected {
                background: #555555;
                font-weight: bold;
            }
            QTabWidget::pane {
                border: 1px solid #555;
                background: #1e1e1e;
            }
        """)

        # Create the 2D and 3D tabs
        self.tab2d = LateralRaise2DTab(
            angle_lock=data_lock,
            shoulder_angle_ref=shoulder_angle,
            elbow_angle_ref=elbow_angle,
            upper_arm_length=0.5,
            forearm_length=0.5,
            update_interval_ms=50
        )
        self.tab3d = Visualizer3dTab(
            data_lock=data_lock,
            upper_arm_euler_ref=upper_arm_euler,
            forearm_euler_ref=forearm_euler,
            upper_arm_length=0.4,
            forearm_length=0.35,
            update_interval_ms=50
        )

        # Add the tabs (you can rename these to more descriptive or simpler labels if you like)
        self.tabs.addTab(self.tab2d, "2D Visualization")
        self.tabs.addTab(self.tab3d, "3D Visualization")

        self.setCentralWidget(self.tabs)

def main():
    # Start threads for reading sensor data
    threading.Thread(target=read_imu, args=(ESP32_UPPER_ARM, "Upper Arm"), daemon=True).start()
    threading.Thread(target=read_imu, args=(ESP32_FOREARM, "Forearm"), daemon=True).start()

    # Create the PyQt application
    app = QApplication(sys.argv)
    
    # OPTIONAL: Use a dark palette for the entire app
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor("#121212"))
    palette.setColor(QPalette.ColorRole.Base, QColor("#1e1e1e"))
    palette.setColor(QPalette.ColorRole.Text, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.WindowText, QColor("#ffffff"))
    app.setPalette(palette)

    # Or load a custom stylesheet for the entire app
    app.setStyleSheet("""
        QMainWindow {
            background-color: #121212;
        }
        QLabel, QCheckBox, QRadioButton {
            color: #ffffff;
        }
    """)

    # Launch main window
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()

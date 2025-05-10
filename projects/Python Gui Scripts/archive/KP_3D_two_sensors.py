# main.py
import sys
import time
import json
import serial
import threading
import numpy as np

from PyQt6.QtWidgets import QApplication, QMainWindow, QTabWidget

# Import the 2D and 3D classes from separate files
from lateralraise2d import LateralRaise2DTab

from KP_biometric_visualizer_3d import BiometricVisualizer3DTab

# ===================== GLOBALS / CONFIG =====================
data_lock = threading.Lock()

# We store angles as single-element lists for easy reference passing:
shoulder_angle = [0.0]
elbow_angle    = [0.0]

# Quaternions for 3D (w, x, y, z)
upper_arm_quat = [1.0, 0.0, 0.0, 0.0]
forearm_quat   = [1.0, 0.0, 0.0, 0.0]

ESP32_UPPER_ARM = "COM10"
ESP32_FOREARM   = "COM11"
BAUD_RATE       = 115200

def read_imu(port, label):
    """
    Reads JSON data (angle) or "Quat:" lines from the ESP32 over serial.
    """
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
                # print(f"{label} Raw: {line}")

                with data_lock:
                    # If it's JSON with "angle"
                    if line.startswith("{"):
                        try:
                            data = json.loads(line)
                            angle_value = data.get("angle", None)
                            if angle_value is not None:
                                # Validate angle
                                if not (-180 <= angle_value <= 180):
                                    continue
                                # Update 2D angles
                                if label == "Upper Arm":
                                    shoulder_angle[0] = angle_value
                                elif label == "Forearm":
                                    elbow_angle[0] = angle_value
                        except json.JSONDecodeError:
                            pass
                    # If line has "Quat:"
                    elif "Quat" in line:
                        try:
                            parts = line.split(":")[1].split(",")
                            q = [float(p) for p in parts]
                            if len(q) == 4:
                                if label == "Upper Arm":
                                    upper_arm_quat[:] = q  # update in place
                                elif label == "Forearm":
                                    forearm_quat[:] = q
                        except Exception:
                            pass
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"❌ Error reading from {label}: {e}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Integrated 2D + 3D Motion Tracking")
        self.setGeometry(100, 100, 1200, 800)

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Create the 2D tab
        self.tab2d = LateralRaise2DTab(
            angle_lock=data_lock,
            shoulder_angle_ref=shoulder_angle,
            elbow_angle_ref=elbow_angle,
            upper_arm_length=0.5,
            forearm_length=0.5,
            update_interval_ms=50
        )

        # Create the 3D tab
        self.tab3d = BiometricVisualizer3DTab(
            data_lock=data_lock,
            upper_arm_quat_ref=upper_arm_quat,
            forearm_quat_ref=forearm_quat,
            upper_arm_length=0.4,
            forearm_length=0.35,
            update_interval_ms=50
        )

        # Add both tabs
        self.tabs.addTab(self.tab2d, "2D Elliptical Arm")
        self.tabs.addTab(self.tab3d, "3D Motion Tracking")

def main():
    # Start sensor threads
    threading.Thread(target=read_imu, args=(ESP32_UPPER_ARM, "Upper Arm"), daemon=True).start()
    threading.Thread(target=read_imu, args=(ESP32_FOREARM, "Forearm"), daemon=True).start()

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()

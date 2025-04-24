import sys
import serial
import threading
import numpy as np
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas

# ======== CONFIGURATION ========
ESP32_UPPER_ARM = "/dev/cu.usbserial-1440"  # Upper Arm ESP32
ESP32_FOREARM = "/dev/cu.usbserial-1430"    # Forearm ESP32
BAUD_RATE = 115200
UPDATE_INTERVAL_MS = 50  # GUI refresh rate
# ===============================

# Shared variables with thread lock
shoulder_angle = 0.0
elbow_angle = 0.0
angle_lock = threading.Lock()

def read_imu(port, label):
    """ Read IMU data from ESP32 over USB Serial with error handling """
    global shoulder_angle, elbow_angle
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"✅ Connected to {label} ({port})")
        
        while True:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    print(f"{label} Raw: {line}")  # Debug raw input

                    # Improved parsing with fallbacks
                    if "Roll" in line and "Pitch" in line:
                        roll_value = float(line.split("Roll:")[1].split()[0].strip())
                        
                        # Validate angle range
                        if not (-90 <= roll_value <= 90):
                            print(f"⚠️ Invalid angle {roll_value} from {label}")
                            continue
                            
                        with angle_lock:
                            if label == "Upper Arm":
                                shoulder_angle = roll_value
                            elif label == "Forearm":
                                elbow_angle = roll_value
                                
                            print(f"DEBUG: {label} = {roll_value}°")  # Verbose debug

                except Exception as e:
                    print(f"❌ Parse error: {str(e)} in line: {line}")

    except Exception as e:
        print(f"❌ Connection error ({label}): {str(e)}")

# Start serial threads
threading.Thread(target=read_imu, args=(ESP32_UPPER_ARM, "Upper Arm"), daemon=True).start()
threading.Thread(target=read_imu, args=(ESP32_FOREARM, "Forearm"), daemon=True).start()

class LateralRaiseGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Window configuration
        self.setWindowTitle("Lateral Raise Trainer")
        self.setGeometry(100, 100, 800, 600)
        
        # Figure setup
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)
        self.setCentralWidget(self.canvas)
        
        # Plot configuration
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-0.2, 1.2)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_title("Real-time Arm Position Tracking")
        
        # Arm segments initialization
        self.shoulder = np.array([0, 0])  # Fixed origin
        self.elbow = np.array([0, 0.5])
        self.hand = np.array([0, 1.0])
        
        # Create arm plots
        self.upper_arm, = self.ax.plot([], [], 'o-', lw=6, markersize=10, color='#2c7da0')
        self.forearm, = self.ax.plot([], [], 'o-', lw=6, markersize=10, color='#468faf')
        
        # QTimer-based updates (critical fix)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(UPDATE_INTERVAL_MS)

    def update_plot(self):
        """ Update arm positions with thread-safe angle access """
        global shoulder_angle, elbow_angle
        
        with angle_lock:
            shoulder_deg = shoulder_angle
            elbow_deg = elbow_angle

        # Convert to radians
        shoulder_rad = np.radians(shoulder_deg)
        elbow_rad = np.radians(elbow_deg)
        
        # Calculate positions (improved coordinate system)
        # Upper arm (shoulder to elbow)
        self.elbow = np.array([
            np.sin(shoulder_rad) * 0.5,
            0.5 - np.abs(np.cos(shoulder_rad) * 0.5)  # Prevent negative Y
        ])
        
        # Forearm (elbow to hand)
        self.hand = np.array([
            self.elbow[0] + np.sin(elbow_rad) * 0.5,
            self.elbow[1] + np.cos(elbow_rad) * 0.5
        ])
        
        # Update plot data
        self.upper_arm.set_data(
            [self.shoulder[0], self.elbow[0]],
            [self.shoulder[1], self.elbow[1]]
        )
        self.forearm.set_data(
            [self.elbow[0], self.hand[0]],
            [self.elbow[1], self.hand[1]]
        )
        
        self.canvas.draw_idle()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LateralRaiseGUI()
    window.show()
    sys.exit(app.exec())

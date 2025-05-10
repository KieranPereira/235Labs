import sys
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas

ESP32_PORT = "/dev/cu.usbserial-1430"  # Update with your ESP32 USB port
BAUD_RATE = 115200

class LateralRaiseGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Lateral Raise Visualization")
        self.setGeometry(100, 100, 600, 600)

        # Setup serial connection
        try:
            self.serial_port = serial.Serial(ESP32_PORT, BAUD_RATE, timeout=1)
            print(f"✅ Connected to {ESP32_PORT}")
        except Exception as e:
            print(f"❌ Error: {e}")
            sys.exit(1)

        # Setup figure
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)
        self.setCentralWidget(self.canvas)

        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-0.2, 1.2)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_title("Lateral Raise Motion")

        # Initialize arm points
        self.shoulder = np.array([0, 0])  # Fixed shoulder position
        self.elbow = np.array([0, 0.5])   # Default elbow position
        self.hand = np.array([0, 1])      # Default hand position

        # Plot initial arm lines
        self.upper_arm, = self.ax.plot([], [], "o-", lw=5, markersize=8)
        self.forearm, = self.ax.plot([], [], "o-", lw=5, markersize=8)

        # Start animation
        self.anim = animation.FuncAnimation(self.figure, self.update_plot, interval=50)

    def update_plot(self, frame):
        """ Read IMU data and update the stick figure """
        if self.serial_port.in_waiting:
            try:
                line = self.serial_port.readline().decode("utf-8").strip()
                if "Shoulder Angle" in line and "Elbow Angle" in line:
                    parts = line.split("|")
                    shoulder_angle = float(parts[0].split(":")[1].strip())
                    elbow_angle = float(parts[1].split(":")[1].strip())

                    # Convert angles to radians
                    shoulder_rad = np.radians(shoulder_angle)
                    elbow_rad = np.radians(elbow_angle)

                    # Calculate new elbow position (shoulder is fixed)
                    self.elbow[0] = np.sin(shoulder_rad) * 0.5
                    self.elbow[1] = np.cos(shoulder_rad) * 0.5

                    # Calculate new hand position based on elbow angle
                    self.hand[0] = self.elbow[0] + np.sin(elbow_rad) * 0.5
                    self.hand[1] = self.elbow[1] + np.cos(elbow_rad) * 0.5

                    # Update arm lines
                    self.upper_arm.set_data([self.shoulder[0], self.elbow[0]], [self.shoulder[1], self.elbow[1]])
                    self.forearm.set_data([self.elbow[0], self.hand[0]], [self.elbow[1], self.hand[1]])

                    # Change color based on correct form
                    if 85 <= shoulder_angle <= 95 and elbow_angle < 10:
                        self.upper_arm.set_color("green")
                        self.forearm.set_color("green")
                    else:
                        self.upper_arm.set_color("red")
                        self.forearm.set_color("red")

                    self.canvas.draw()

            except Exception as e:
                print(f"❌ Data Error: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LateralRaiseGUI()
    window.show()
    sys.exit(app.exec())

import serial
import json
import tkinter as tk
from tkinter import ttk
import math

# ========================
# Serial Port Configuration
# ========================
SERIAL_PORT = 'COM11'
BAUD_RATE = 115200

# ========================
# Initialize Serial Communication
# ========================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

# ========================
# Setup the GUI
# ========================
root = tk.Tk()
root.title("ESP32 MPU6050 Arm Orientation - Smoothed")
root.geometry("500x600")

# Labels for Displaying Sensor Data
labels = {
    "accel_x": ttk.Label(root, text="Accel X: "),
    "accel_y": ttk.Label(root, text="Accel Y: "),
    "accel_z": ttk.Label(root, text="Accel Z: "),
    "gyro_x": ttk.Label(root, text="Gyro X: "),
    "gyro_y": ttk.Label(root, text="Gyro Y: "),
    "gyro_z": ttk.Label(root, text="Gyro Z: "),
    "temp":    ttk.Label(root, text="Temp: "),
    "angle":   ttk.Label(root, text="Angle: "),
    "status":  ttk.Label(root, text="Status: ")
}

# Place the labels in a grid
for i, (key, label) in enumerate(labels.items()):
    label.grid(row=i, column=0, sticky="W", padx=10, pady=5)

# Make the status label larger and bold
labels["status"].config(font=("Arial", 16, "bold"))

# Create a canvas for displaying the arm line
canvas = tk.Canvas(root, width=300, height=300, bg='white')
canvas.grid(row=11, column=0, columnspan=2, pady=20)
# Draw an initial line (centered at (150,150))
line = canvas.create_line(150, 150, 150, 50, width=5, fill='blue')

# ========================
# Filtering Variables
# ========================
# We'll use an exponential smoothing filter for the angle value.
smoothed_angle = 0
first_angle = True  # To initialize the filter on the first valid measurement

# ========================
# Function to Read, Filter, and Update Data
# ========================
def update_data():
    global smoothed_angle, first_angle
    try:
        # Read a line from the serial port, decode it, and strip whitespace
        line_data = ser.readline().decode('utf-8').strip()
        print("Raw received data:", repr(line_data))  # Debug print
        
        if line_data:
            try:
                data = json.loads(line_data)
                
                # Update sensor value labels
                labels["accel_x"].config(text=f"Accel X: {data.get('accel_x', 0):.2f} m/s²")
                labels["accel_y"].config(text=f"Accel Y: {data.get('accel_y', 0):.2f} m/s²")
                labels["accel_z"].config(text=f"Accel Z: {data.get('accel_z', 0):.2f} m/s²")
                labels["gyro_x"].config(text=f"Gyro X: {data.get('gyro_x', 0):.2f} rad/s")
                labels["gyro_y"].config(text=f"Gyro Y: {data.get('gyro_y', 0):.2f} rad/s")
                labels["gyro_z"].config(text=f"Gyro Z: {data.get('gyro_z', 0):.2f} rad/s")
                labels["temp"].config(text=f"Temp: {data.get('temp', 0):.2f} °C")
                labels["angle"].config(text=f"Angle: {data.get('angle', 0):.2f}°")
                
                # Update status label with color coding
                status = data.get("status", "Unknown")
                status_color = "green" if status == "Done" else "red"
                labels["status"].config(text=f"Status: {status}", foreground=status_color)
                
                # Get the new angle value from the sensor data
                new_angle = data.get('angle', 0)
                # On the very first reading, initialize the smoothed angle to the current value.
                if first_angle:
                    smoothed_angle = new_angle
                    first_angle = False
                # Apply exponential smoothing filter:
                # alpha controls the amount of smoothing (lower alpha means smoother, but slower to react)
                alpha = 0.1
                smoothed_angle = alpha * new_angle + (1 - alpha) * smoothed_angle
                
                # Update the arm line on the canvas based on the smoothed angle.
                radians = math.radians(-smoothed_angle)  # Adjust for coordinate system
                end_x = 150 + 100 * math.sin(radians)
                end_y = 150 - 100 * math.cos(radians)
                canvas.coords(line, 150, 150, end_x, end_y)
            
            except json.JSONDecodeError as e:
                print("JSONDecodeError:", e, "Raw data:", line_data)
        else:
            print("No data received (empty line).")
    
    except (UnicodeDecodeError, serial.SerialException) as e:
        print("Error reading from serial port:", e)
    
    # Schedule the next update after 50 milliseconds
    root.after(50, update_data)

# Start the update loop
update_data()
root.mainloop()

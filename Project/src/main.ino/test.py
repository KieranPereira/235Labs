import serial
import tkinter as tk
from tkinter import scrolledtext

# ========================
# Serial Port Configuration
# ========================
SERIAL_PORT = 'COM8'  # Update this to match your device's COM port
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
root.title("COM Port Data Viewer")
root.geometry("600x400")

# Create a scrolled text widget for displaying output
output_text = scrolledtext.ScrolledText(root, wrap=tk.WORD, width=70, height=20)
output_text.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

# ========================
# Function to Update the GUI with Serial Data
# ========================
def update_output():
    try:
        # Read one line from the serial port
        line_data = ser.readline().decode('utf-8').strip()
        if line_data:
            # Append the new line to the text widget
            output_text.insert(tk.END, line_data + '\n')
            # Automatically scroll to the end
            output_text.see(tk.END)
    except (UnicodeDecodeError, serial.SerialException) as e:
        output_text.insert(tk.END, f"Error: {e}\n")
    
    # Schedule the next call to update_output after 50 milliseconds
    root.after(50, update_output)

# Start the update loop
update_output()

# Run the GUI event loop
root.mainloop()

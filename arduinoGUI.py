# import tkinter as tk
# from tkinter import ttk
# import serial
# import time

# initial_joint_angles = {
#     "servo1": 0,  # Adjust these values as needed
#     "servo2": 0,
#     "servo3": 0,
#     "servo4": 0
# }

# # Connect to the Arduino
# try:
#     arduino = serial.Serial(port='COM4', baudrate=9600, timeout=1)
#     time.sleep(3)  # Wait for Arduino to initialize
# except Exception as e:
#     print(f"Error connecting to Arduino: {e}")
#     arduino = None

# def send_angles():
#     if arduino is not None:
#         # Get the values from the sliders
#         servo1_angle = servo1_scale.get()
#         servo2_angle = servo2_scale.get()
#         servo3_angle = servo3_scale.get()
#         servo4_angle = servo4_scale.get()
        
#         # Create the command string
#         command = f"{servo1_angle},{servo2_angle},{servo3_angle},{servo4_angle}\n"
        
#         # Send the command via serial
#         arduino.write(command.encode())
#         print(f"Sent: {command}")
#     else:
#         print("Arduino is not connected!")

# def reset_angles():
#     servo1_scale.set(initial_joint_angles["servo1"])
#     servo2_scale.set(initial_joint_angles["servo2"])
#     servo3_scale.set(initial_joint_angles["servo3"])
#     servo4_scale.set(initial_joint_angles["servo4"])

# def update_label(slider, label, text_variable):
#     """Update the label text dynamically when the slider is moved"""
#     angle = slider.get()
#     text_variable.set(f"{label}: {angle:.1f}°")

# # Create the main Tkinter window
# root = tk.Tk()
# root.title("Servo Control GUI")

# # Create variables to hold the servo angle text for each label
# servo1_text = tk.StringVar(value=f"Servo 1 Angle: {initial_joint_angles['servo1']}")
# servo2_text = tk.StringVar(value=f"Servo 2 Angle: {initial_joint_angles['servo2']}")
# servo3_text = tk.StringVar(value=f"Servo 3 Angle: {initial_joint_angles['servo3']}")
# servo4_text = tk.StringVar(value=f"Servo 4 Angle: {initial_joint_angles['servo4']}")

# # Servo 1 Control
# servo1_label = tk.Label(root, textvariable=servo1_text)
# servo1_label.pack()
# servo1_scale = ttk.Scale(root, from_=-107, to=83, orient="horizontal")
# servo1_scale.set(initial_joint_angles["servo1"])  # Set initial angle
# servo1_scale.pack()
# servo1_scale.config(command=lambda val: update_label(servo1_scale, "Servo 1 Angle", servo1_text))

# # Servo 2 Control
# servo2_label = tk.Label(root, textvariable=servo2_text)
# servo2_label.pack()
# servo2_scale = ttk.Scale(root, from_=-22, to=158, orient="horizontal")
# servo2_scale.set(initial_joint_angles["servo2"])  # Set initial angle
# servo2_scale.pack()
# servo2_scale.config(command=lambda val: update_label(servo2_scale, "Servo 2 Angle", servo2_text))

# # Servo 3 Control
# servo3_label = tk.Label(root, textvariable=servo3_text)
# servo3_label.pack()
# servo3_scale = ttk.Scale(root, from_=-40, to=140, orient="horizontal")
# servo3_scale.set(initial_joint_angles["servo3"])  # Set initial angle
# servo3_scale.pack()
# servo3_scale.config(command=lambda val: update_label(servo3_scale, "Servo 3 Angle", servo3_text))

# # Servo 4 Control
# servo4_label = tk.Label(root, textvariable=servo4_text)
# servo4_label.pack()
# servo4_scale = ttk.Scale(root, from_=-100, to=80, orient="horizontal")
# servo4_scale.set(initial_joint_angles["servo4"])  # Set initial angle
# servo4_scale.pack()
# servo4_scale.config(command=lambda val: update_label(servo4_scale, "Servo 4 Angle", servo4_text))

# # Send Button
# send_button = tk.Button(root, text="Send Angles", command=send_angles)
# send_button.pack()

# # Reset Button
# reset_button = tk.Button(root, text="Reset Angles", command=reset_angles)
# reset_button.pack()

# # Run the Tkinter main loop
# root.mainloop()

import tkinter as tk
from tkinter import ttk
import serial
import time

initial_joint_angles = {
    "servo1": 0,  # Adjust these values as needed
    "servo2": 0,
    "servo3": 0,
    "servo4": 0
}

default_angles = [0, 15, 105, 60]

# Connect to the Arduino
try:
    arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)
    time.sleep(3)  # Wait for Arduino to initialize
except Exception as e:
    print(f"Error connecting to Arduino: {e}")
    arduino = None

def send_angles():
    if arduino is not None:
        try:
            # Get values from entry boxes
            servo1_angle = int(servo1_entry.get())
            servo2_angle = int(servo2_entry.get())
            servo3_angle = int(servo3_entry.get())
            servo4_angle = int(servo4_entry.get())
            
            # Create the command string
            command = f"{servo1_angle},{servo2_angle},{servo3_angle},{servo4_angle}\n"
            
            # Send the command via serial
            arduino.write(command.encode())
            print(f"Sent: {command}")
        except ValueError:
            print("Error: Please enter valid integer angles!")
    else:
        print("Arduino is not connected!")

def reset_angles():
    # Reset both the sliders and the entry boxes
    servo1_scale.set(initial_joint_angles["servo1"])
    servo1_entry.delete(0, tk.END)
    servo1_entry.insert(0, initial_joint_angles["servo1"])

    servo2_scale.set(initial_joint_angles["servo2"])
    servo2_entry.delete(0, tk.END)
    servo2_entry.insert(0, initial_joint_angles["servo2"])

    servo3_scale.set(initial_joint_angles["servo3"])
    servo3_entry.delete(0, tk.END)
    servo3_entry.insert(0, initial_joint_angles["servo3"])

    servo4_scale.set(initial_joint_angles["servo4"])
    servo4_entry.delete(0, tk.END)
    servo4_entry.insert(0, initial_joint_angles["servo4"])

    command = f"{initial_joint_angles['servo1']},{initial_joint_angles['servo2']},{initial_joint_angles['servo3']},{initial_joint_angles['servo4']}\n"
    arduino.write(command.encode())
    print(f"Sent default angles: {command}")

def set_default_angles():
    if arduino is not None:
        # Set the sliders to default values
        servo1_scale.set(default_angles[0])
        servo2_scale.set(default_angles[1])
        servo3_scale.set(default_angles[2])
        servo4_scale.set(default_angles[3])

        # Update the entry boxes
        servo1_entry.delete(0, tk.END)
        servo1_entry.insert(0, default_angles[0])
        servo2_entry.delete(0, tk.END)
        servo2_entry.insert(0, default_angles[1])
        servo3_entry.delete(0, tk.END)
        servo3_entry.insert(0, default_angles[2])
        servo4_entry.delete(0, tk.END)
        servo4_entry.insert(0, default_angles[3])

        # Send default values to Arduino
        command = f"{default_angles[0]},{default_angles[1]},{default_angles[2]},{default_angles[3]}\n"
        arduino.write(command.encode())
        print(f"Sent default angles: {command}")
    else:
        print("Arduino is not connected!")

def swing():
    if arduino is not None:
        new_value = servo1_scale.get() + 20
        servo1_scale.set(new_value)
        servo1_entry.delete(0, tk.END)
        servo1_entry.insert(0, f"{int(new_value)}")

        if arduino is not None:
            command = f"{int(new_value)},{servo2_scale.get()},{servo3_scale.get()},{servo4_scale.get()}\n"
            arduino.write(command.encode())
        else:
            print("Arduino is not connected!")



# Create the main Tkinter window
root = tk.Tk()
root.title("Servo Control GUI")

# Servo controls with sliders and text entry fields
def create_servo_control(parent, servo_name, min_val, max_val):
    # Frame to organize each servo's controls
    frame = tk.Frame(parent)
    frame.pack(pady=5)

    # Label
    label = tk.Label(frame, text=f"{servo_name} Angle:")
    label.grid(row=0, column=0, padx=5)

    # Scale (slider)
    scale = ttk.Scale(frame, from_=min_val, to=max_val, orient="horizontal")
    scale.set(initial_joint_angles[servo_name.lower()])
    scale.grid(row=0, column=1, padx=5)

    # Entry box
    entry = tk.Entry(frame, width=5)
    entry.insert(0, initial_joint_angles[servo_name.lower()])
    entry.grid(row=0, column=2, padx=5)

    # Sync slider and entry box
    def update_entry(val):
        entry.delete(0, tk.END)
        entry.insert(0, f"{int(float(val))}")

    def update_slider(event):
        try:
            scale.set(float(entry.get()))
        except ValueError:
            pass  # Ignore invalid inputs in the entry box

    scale.config(command=update_entry)
    entry.bind("<Return>", update_slider)  # Update slider when pressing Enter

    return scale, entry

# Create servo controls
servo1_scale, servo1_entry = create_servo_control(root, "Servo1", -107, 83)
servo2_scale, servo2_entry = create_servo_control(root, "Servo2", -22, 158)
servo3_scale, servo3_entry = create_servo_control(root, "Servo3", -40, 140)
servo4_scale, servo4_entry = create_servo_control(root, "Servo4", -100, 80)

# Send Button
send_button = tk.Button(root, text="Send Angles", command=send_angles)
send_button.pack(pady=10)

# Reset Button
reset_button = tk.Button(root, text="Reset Angles", command=reset_angles)
reset_button.pack(pady=10)

# Default Button
default_button = tk.Button(root, text = "Set Default Angles", command=set_default_angles)
default_button.pack(pady=5)

# Default Button
swing_frame = tk.Frame(root)
swing_frame.pack(pady=5)
swing_button = tk.Button(swing_frame, text="Swing", command=swing)
swing_button.pack(side=tk.LEFT, padx=5)

# Run the Tkinter main loop
root.mainloop()

#good angles: 0, 15, 105, 60 then change to 20 to hit the ball
#good position: CD, GH

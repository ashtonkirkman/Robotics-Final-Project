import serial
import time
import keyboard


def send_angles(angles):
    command = f"{angles[0]},{angles[1]},{angles[2]},{angles[3]}\n"
    arduino.write(command.encode())
    # time.sleep(3)

try:
    arduino = serial.Serial(port='COM3 ', baudrate=9600, timeout=1)
    time.sleep(3)  # Wait for Arduino to initialize
except Exception as e:
    print(f"Error connecting to Arduino: {e}")
    arduino = None

# all_angles = [
#     [83, 16, 125, 0],
#     [83, 16, 95, 0],
#     [53, 16, 125, 0],
#     [53, 16, 95, 0],
#     [23, 16, 125, 0],
#     [23, 16, 95, 0],
#     [3, 16, 125, 0],
#     [3, 16, 95, 0]
# ]

all_angles = [
    [0, 90, 0, 0],
    [0, 15, 105, 60],
    [20, 15, 105, 60]
    ]


# Send each set of angles to Arduino
if arduino:
    while True:
        if keyboard.is_pressed('Esc'):
            print('Exiting...')
            break

        if keyboard.is_pressed('space'):
            send_angles(all_angles[0])
        
        if keyboard.is_pressed('ctrl'):
            send_angles(all_angles[1])          
            time.sleep(1)
            send_angles(all_angles[2])                      
              
else:
    print("Arduino is not connected!")
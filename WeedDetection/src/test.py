import serial
import time

try:
    ser = serial.Serial('COM10', 9600, timeout=1)  # Replace COM7 with the port you found
    time.sleep(2)  # wait for Arduino to initialize
    print("Serial port opened successfully!")
except serial.SerialException as e:
    print(f"There was an error opening serial port: {e}")
    exit()
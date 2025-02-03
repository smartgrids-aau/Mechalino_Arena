import serial
import time

# Configure the serial connection
ser = serial.Serial('COM3', 115200, timeout=1)  # Replace 'COM3' with your port if different
time.sleep(2)  # Wait for the connection to initialize

try:
    while True:
        # Send data to ESP8266 every 150ms
        ser.write(b'STATE:2;CURRENT_INDEX:4;MOTOR_L:800;MOTOR_R:2000;ANGLE_ERROR:20.3421;CALCULATED_PID:2.7523;TARGET_DISTANCE:2.3423\r\n')  # Send data, make sure to encode as bytes
        #print("Data sent to ESP8266")

        # Read data from ESP8266
        if ser.in_waiting > 0:  # Check if there is data to read
            data = ser.readline().decode('utf-8').rstrip()  # Read the incoming data and decode it
            print(f"Received from ESP8266: {data}")
            if data.__contains__("DEBUG"):
                print("Received DEBUG message")

        # Wait for 20 milliseconds before sending the next data
        time.sleep(0.02)

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    ser.close()  # Close the serial connection when done



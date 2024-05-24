import speech_recognition as sr
import requests
import time

server_ip = "192.168.137.233"
server_port = 80

# P (kp=50.0, correction_speed=30, correction_time=160)

url = f"http://{server_ip}:{server_port}/?data=P 30 30 240\r"
response = requests.get(url)
print(response.status_code)
time.sleep(1)

# X (r_ccw=-69, r_cw=76.5, Rdelay=10)

url = f"http://{server_ip}:{server_port}/?data=X -79.5 79.5 10\r"
response = requests.get(url)
print(response.status_code)
time.sleep(1)

# move forward
def move_forward():
    url = f"http://{server_ip}:{server_port}/?data=M 100 2000\r"
    response = requests.get(url)
    time.sleep(2)

# turn cw
def turn_cw():
    url = f"http://{server_ip}:{server_port}/?data=R 90\r"
    response = requests.get(url)
    time.sleep(3)

# turn ccw
def turn_ccw():
    url = f"http://{server_ip}:{server_port}/?data=R -90\r"
    response = requests.get(url)
    time.sleep(3)

def listen_microphone():
    # Initialize recognizer
    recognizer = sr.Recognizer()
    
    with sr.Microphone() as source:
        recognizer.adjust_for_ambient_noise(source, duration=2)  # Adjust for ambient noise
        
        while True:
            try:
                print("Listening...")
                audio = recognizer.listen(source, timeout=5)  # Listen for 5 seconds
                
                # Recognize speech
                command = recognizer.recognize_google(audio).lower()
                print("You said:", command)
                
                # Process recognized command
                process_command(command)
                
            except sr.UnknownValueError:
                print("Sorry, I couldn't understand what you said.")
            except sr.RequestError:
                print("Sorry, I couldn't request results. Please check your internet connection.")
            except sr.WaitTimeoutError:
                print("Timeout. Please try again.")

def process_command(command):
    # Check for specific commands
    if "go" in command:
        # Do something for moving forward
        print("Moving forward...")
        move_forward()
    elif "stop" in command:
        # Do something for stopping
        print("Stopping...")
    elif "left" in command:
        # Do something for turning left
        print("Turning left...")
        turn_ccw()
    elif "right" in command:
        # Do something for turning right
        print("Turning right...")
        turn_cw()
    else:
        print("Command not recognized.")

# Main function
if __name__ == "__main__":
    listen_microphone()

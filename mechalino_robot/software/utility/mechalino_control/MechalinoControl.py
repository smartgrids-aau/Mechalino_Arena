import requests
import time
import os

class MechalinoControl:
    def make_get_request(self,url):
        try:
            response = requests.get(url)
            response.raise_for_status()  # Raises an exception if the request was not successful (status code >= 400)
            return response.text
        except requests.exceptions.RequestException as e:
            print(f"Error making GET request: {e}")
            return None

    def __init__(self,ip = None):
        if ip is not None:
            self.ip = ip
        else:
            mac_to_detect = "EC-FA-BC-31"

            devices = []

            # Run the arp -a command to list devices on the network
            arp_output = os.popen("arp -a").read()

            # Regular expression to extract IP and MAC addresses from arp output
            lines = arp_output.split("\n")
            mechalinos = []
            for line in lines:
                if mac_to_detect in line.upper():
                    print(str(len(mechalinos)+1) + ". Mechalino fount at:",line.replace('static',''))
                    mechalinos.append(line.replace('static','').strip().split(' ')[0])
            if len(mechalinos) == 0:
                print("No robot was found on the network . Press any key to exit ...")
                input()
                exit()
            while(True):
                try:
                    selection = input("select to continue:")  # Waits for user input
                    self.ip = mechalinos[int(selection)-1]
                    break
                except:
                    print("Invalid selection. Please try again...")
    def forward(self,duration=1.0):
        url = f'http://{self.ip}/Robot1/?F={duration*1000:.0f}'
        self.make_get_request(url)
        time.sleep(duration+1)
    def backward(self,duration=1.0):
        url = f'http://{self.ip}/Robot1/?B={duration*1000:.0f}'
        self.make_get_request(url)
        time.sleep(duration+1)
    def rotate_cw(self,degrees=90):
        url = f'http://{self.ip}/Robot1/?R={degrees}'
        self.make_get_request(url)
        time.sleep(degrees/360*8)
    def rotate_ccw(self,degrees=90):
        url = f'http://{self.ip}/Robot1/?L={degrees}'
        self.make_get_request(url)
        time.sleep(degrees/360*8)

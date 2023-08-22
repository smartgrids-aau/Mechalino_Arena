import requests
import time
import os
import subprocess

def check_ping(ip_address):
    try:
        subprocess.check_output(["ping", "-c", "4", ip_address], stderr=subprocess.STDOUT, universal_newlines=True)
        return True
    except:
        return False
    
class MechalinoControl:
    def make_get_request(self,url):
        try:
            response = requests.get(url)
            response.raise_for_status()  # Raises an exception if the request was not successful (status code >= 400)
            return response.text
        except Exception as e:
            print(f"Error making GET request: {e}")
            return None

    def __init__(self,ip = None):
        if ip is not None:
            self.ip = ip
            if not check_ping(self.ip):
                return None
        else:
            self.ip = input('please enter the robot\'s IP address:')
            if not check_ping(self.ip):
                return None
            
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

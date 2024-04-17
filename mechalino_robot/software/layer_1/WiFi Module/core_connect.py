import requests
import time

server_ip = "192.168.137.70"
server_port = 80

#url = f"http://{server_ip}:{server_port}/?data=P 100 4000 1000 0 0\r"
url = f"http://{server_ip}:{server_port}/?data=P 1200 0 0\r"
response = requests.get(url)
time.sleep(2.5)
url = f"http://{server_ip}:{server_port}/?data=X 1 0 0 0.0 18.5 10\r"
response = requests.get(url)
time.sleep(2.5)
for i in range(4):
    url = f"http://{server_ip}:{server_port}/?data=M 100 4000\r"
    response = requests.get(url)
    time.sleep(5)
    url = f"http://{server_ip}:{server_port}/?data=R -90\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=M 100 1000\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=R -90\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=M 100 4000\r"
    response = requests.get(url)
    time.sleep(5)
    url = f"http://{server_ip}:{server_port}/?data=R 90\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=M 100 1000\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=R 90\r"
    response = requests.get(url)
    time.sleep(2)
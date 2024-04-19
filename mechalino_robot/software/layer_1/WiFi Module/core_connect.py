import requests
import time

server_ip = "192.168.137.35"
server_port = 80

url = f"http://{server_ip}:{server_port}/?data=P 500 0 0\r"
response = requests.get(url)
print(response.status_code)
time.sleep(1)

for i in range(2):
    url = f"http://{server_ip}:{server_port}/?data=M 100 4000\r"
    response = requests.get(url)
    time.sleep(7)
    url = f"http://{server_ip}:{server_port}/?data=R -69\r"
    response = requests.get(url)
    time.sleep(7)
    url = f"http://{server_ip}:{server_port}/?data=M 100 2000\r"
    response = requests.get(url)
    time.sleep(7)
    url = f"http://{server_ip}:{server_port}/?data=R -69\r"
    response = requests.get(url)
    time.sleep(7)
    url = f"http://{server_ip}:{server_port}/?data=M 100 4000\r"
    response = requests.get(url)
    time.sleep(7)
    url = f"http://{server_ip}:{server_port}/?data=R 76.5\r"
    response = requests.get(url)
    time.sleep(7)
    url = f"http://{server_ip}:{server_port}/?data=M 100 2000\r"
    response = requests.get(url)
    time.sleep(7)
    url = f"http://{server_ip}:{server_port}/?data=R 76.5\r"
    response = requests.get(url)
    time.sleep(7)
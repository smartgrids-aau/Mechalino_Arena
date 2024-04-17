import requests
import time

server_ip = "192.168.137.185"
server_port = 80

#url = f"http://{server_ip}:{server_port}/?data=P 100 4000 1000 0 0\r"
url = f"http://{server_ip}:{server_port}/?data=P 500 0 0\r"
response = requests.get(url)
print(response.status_code)
time.sleep(1)
url = f"http://{server_ip}:{server_port}/?data=X 1 0 0 0.0 16.75 10\r"
response = requests.get(url)
print(response.status_code)


for i in range(2):
    url = f"http://{server_ip}:{server_port}/?data=M 100 4000\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=R -90\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=M 100 2000\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=R -90\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=M 100 4000\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=R 90\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=M 100 2000\r"
    response = requests.get(url)
    time.sleep(2)
    url = f"http://{server_ip}:{server_port}/?data=R 90\r"
    response = requests.get(url)
    time.sleep(2)
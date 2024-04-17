import requests
import time

server_ip = "192.168.137.70"
server_port = 80

#url = f"http://{server_ip}:{server_port}/?data=P 100 4000 1000 0 0\r"
url = f"http://{server_ip}:{server_port}/?data=X 1 0 0 0.0 22.185 10\r"

response = requests.get(url)

if response.status_code == 200:
    print("GET request sent successfully.")
else:
    print(f"Failed to send GET request. Status code: {response.status_code}")

time.sleep(2.5)

url = f"http://{server_ip}:{server_port}/?data=R 270\r"

response = requests.get(url)

if response.status_code == 200:
    print("GET request sent successfully.")
else:
    print(f"Failed to send GET request. Status code: {response.status_code}")

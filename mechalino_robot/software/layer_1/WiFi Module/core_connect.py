import requests
import time

server_ip = "192.168.137.248"
server_port = 80

# url = f"http://{server_ip}:{server_port}/?data=P -9 0 0\r"
# response = requests.get(url)
# print(response.status_code)
# time.sleep(1)
url = f"http://{server_ip}:{server_port}/?data=P 50 0 0\r"
response = requests.get(url)
time.sleep(1)

url = f"http://{server_ip}:{server_port}/?data=X 170 0 0 0 0 0 0\r"
response = requests.get(url)
time.sleep(1)
# url = f"http://{server_ip}:{server_port}/?data=M 70 4000\r"
# response = requests.get(url)
# time.sleep(1)
for i in range(1):
    url = f"http://{server_ip}:{server_port}/?data=M 70 8000\r"
    response = requests.get(url)
    time.sleep(1)
exit()

# for i in range(3):
#     url = f"http://{server_ip}:{server_port}/?data=M 100 550\r"
#     response = requests.get(url)
#     time.sleep(7)
#     url = f"http://{server_ip}:{server_port}/?data=R -69.5\r"
#     response = requests.get(url)
#     time.sleep(7)
#     url = f"http://{server_ip}:{server_port}/?data=M 100 550\r"
#     response = requests.get(url)
#     time.sleep(7)
#     url = f"http://{server_ip}:{server_port}/?data=R -69.5\r"
#     response = requests.get(url)
#     time.sleep(7)
#     url = f"http://{server_ip}:{server_port}/?data=M 100 550\r"
#     response = requests.get(url)
#     time.sleep(7)
#     url = f"http://{server_ip}:{server_port}/?data=R 76.5\r"
#     response = requests.get(url)
#     time.sleep(7)
#     url = f"http://{server_ip}:{server_port}/?data=M 100 550\r"
#     response = requests.get(url)
#     time.sleep(7)
#     url = f"http://{server_ip}:{server_port}/?data=R 76.5\r"
#     response = requests.get(url)
#     time.sleep(7)
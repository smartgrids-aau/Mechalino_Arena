import requests

server_ip = "192.168.137.60"
server_port = 80

url = f"http://{server_ip}:{server_port}/?data=M 0\r"

response = requests.get(url)

if response.status_code == 200:
    print("GET request sent successfully.")
else:
    print(f"Failed to send GET request. Status code: {response.status_code}")

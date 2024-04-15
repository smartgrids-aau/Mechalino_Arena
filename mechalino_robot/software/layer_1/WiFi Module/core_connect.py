import requests

server_ip = "192.168.137.230"
server_port = 80

url = f"http://{server_ip}:{server_port}/?data=M 100 4000\r"

response = requests.get(url)

if response.status_code == 200:
    print("GET request sent successfully.")
else:
    print(f"Failed to send GET request. Status code: {response.status_code}")

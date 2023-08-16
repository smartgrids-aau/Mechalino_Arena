import os
import sys

mac_to_detect = "EC-FA-BC-31"

devices = []

# Run the arp -a command to list devices on the network
arp_output = os.popen("arp -a").read()

# Regular expression to extract IP and MAC addresses from arp output
lines = arp_output.split("\n")
for line in lines:
    if mac_to_detect in line.upper():
        print("Mechalino fount at:",line.replace('static',''))

print("Press Enter to exit...")
input()  # Waits for user input
sys.exit()  # Exits the script
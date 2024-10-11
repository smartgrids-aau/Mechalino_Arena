import socket

UDP_IP = "127.0.0.1"  # Replace with your machine's IP if different
UDP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Example message format:
# robotID;current_x;current_y;current_yaw;STATE:state;TARGET_X:tx;TARGET_Y:ty;MOTOR_L:ml;MOTOR_R:mr;ANGLE_ERROR:ae;CALCULATED_PID:pid
message = "2;1.5;-0.78;20.0;STATE:2;CURRENT_INDEX:4;MOTOR_L:800;MOTOR_R:2000;ANGLE_ERROR:20.3421;CALCULATED_PID:2.7523;TARGET_DISTANCE:2.3423\n"
sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))

message = "0.07:0.6:1.2:1.7:1.7:1.2:0.6:0.07;-0.07:-0.07:-0.07:-0.07:-0.15:-0.15:-0.15:-0.15;40\n"
sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))

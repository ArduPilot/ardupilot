import cv2
import socket
import pickle
import struct

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.0.187', 8888))  # Replace 'server_ip_address' with the actual server IP

data = b""
payload_size = struct.calcsize("Q")
while True:
    while len(data) < payload_size:
        packet = client_socket.recv(4 * 1024)  # 4K buffer size
        if not packet:
            break
        data += packet
    if not data:
        break    
    
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("Q", packed_msg_size)[0]   
    
    while len(data) < msg_size:
        data += client_socket.recv(4 * 1024)  # 4K buffer size
        
    frame_data = data[:msg_size]
    data = data[msg_size:]
    frame = pickle.loads(frame_data)
    cv2.imshow('Client', frame)
    
    if cv2.waitKey(1) == 13:
        break
        
cv2.destroyAllWindows()

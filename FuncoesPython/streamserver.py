import cv2
import socket
import pickle
import struct
import argparse

parser = argparse.ArgumentParser(description='Descricao dos parametros de configuracao')

parser.add_argument('--addr_server_socket', type=str, help='endereco servidor', default='127.0.0.1')
parser.add_argument('--addr_server_port', type=int, help='porta', default=8888)
parser.add_argument('--show_image', type=bool, help='flag para mostrar imagem', default=True)

args = parser.parse_args()

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((args.addr_server_socket, args.addr_server_port))
server_socket.listen(5)
print("Server is listening...")
client_socket, client_address = server_socket.accept()
print(f"Connection from {client_address} accepted")
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    frame_data = pickle.dumps(frame)
    client_socket.sendall(struct.pack("Q", len(frame_data)))
    client_socket.sendall(frame_data)    
    if args.show_image:
        cv2.imshow('Server', frame)
    if cv2.waitKey(1) == 13:
        breakcap.release()
cv2.destroyAllWindows()

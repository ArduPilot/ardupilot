import socket
import time
import struct
from pymavlink import mavutil

# Configurações do ArduPilot (USB)
serial_port = '/dev/ttyUSB0'  # Substitua pelo seu dispositivo serial
baud_rate = 57600
#mav_connection = mavutil.mavlink_connection(serial_port, baud=baud_rate)

# Configurações do Gazebo (UDP)
gazebo_addr = "127.0.0.1"
fdm_port_in = 9003  # Porta para receber dados do Gazebo
fdm_port_out = 9002  # Porta para enviar dados para o Gazebo
sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_in.bind((gazebo_addr, fdm_port_in))
sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# def read_mavlink():
#     while True:
#         msg = mav_connection.recv_match(blocking=True)
#         if msg:
#             process_mavlink_message(msg)

def process_mavlink_message(msg):
    return
    # Processar mensagens MAVLink e enviar para o Gazebo se necessário
    # ...

def send_to_gazebo(data):
    return
    # Enviar dados para o Gazebo
    sock_out.sendto(data, (gazebo_addr, fdm_port_out))

def main():
    # Iniciar uma thread ou processo para ler dados do MAVLink
    #start_new_thread(read_mavlink)

    while True:
        # Receber dados do Gazebo
        data, addr = sock_in.recvfrom(1024)  # buffer size is 1024 bytes
        process_gazebo_data(data)

        # Pausa para evitar sobrecarga
        time.sleep(0.1)

def process_gazebo_data(data):
    # Decodificar os dados recebidos
    # O formato exato depende de como o plugin envia os dados.
    # Este é um exemplo genérico que você precisará ajustar.
    try:
        decoded_data = struct.unpack('<16f', data)  # Exemplo: 16 floats
        position = decoded_data[0:3]  # Posição x, y, z
        orientation = decoded_data[3:6]  # Orientação roll, pitch, yaw

        print(f"Position: {position}, Orientation: {orientation}")

        # Aqui você pode adicionar lógica para processar os dados
        # E, se necessário, convertê-los em mensagens MAVLink para enviar ao ArduPilot

    except struct.error:
        print("Erro ao decodificar os dados recebidos do Gazebo.")

if __name__ == "__main__":
    main()
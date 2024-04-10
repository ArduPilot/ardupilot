import time
from pymavlink import mavutil

# Endereço e porta do MAVProxy
mavlink_address = '127.0.0.1'
mavlink_port = 14550

# Crie uma conexão com o MAVProxy
connection_string = f"udp:{mavlink_address}:{mavlink_port}"
master = mavutil.mavlink_connection(connection_string)

print("iniciando conexao!")

# Aguarde a inicialização do MAVProxy
while not master.wait_heartbeat():
    print("Aguardando o coração do MAVProxy...")
    time.sleep(1)

print("Conexão com o MAVProxy estabelecida!")

# Envie um comando para o drone (por exemplo, comandos de modo)
mode = 'GUIDED'  # Modo desejado, como 'GUIDED', 'LOITER', etc.
custom_mode = 0  # Deixe como 0 para usar o modo de voo padrão

# # Crie a mensagem de comando de modo
# msg = master.mav.set_mode_encode(
#     master.target_system,
#     custom_mode,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     0,
#     mode.encode('utf-8')
# )
#
#
#
# # Envie o comando de modo
# master.mav.send(msg)

# Exemplo: leia informações do drone
while True:
    try:
        # Leia os dados do drone
        msg = master.recv_match()

        #print(msg.get_type())

        # Exemplo: imprimir informações de atitude do drone
        if msg is not None:
            #print(msg.get_type())
            if msg.get_type() == 'ATTITUDE':
                print(f"Roll: {msg.roll}, Pitch: {msg.pitch}, Yaw: {msg.yaw}")

    except KeyboardInterrupt:
        break

# Feche a conexão quando terminar
master.close()

# import subprocess
# import time
#
# command_to_send = "param set SCR_USER6 2\n"      # MAVProxy command to list the available logs (running this is important before you use any other log module command)
#
# mavproxy_process = subprocess.Popen(["mavproxy.py", "--console"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
#
# # Send the command to mavproxy
# try:
#     # Send the command to mavproxy
#     mavproxy_process.stdin.write(command_to_send)
#     mavproxy_process.stdin.flush()
#     time.sleep(15)
#     print("Command sent")
#
#     command_to_send = "param get SCR_USER6\n"    # MAVProxy command to download the log number 300
#
#     mavproxy_process.stdin.write(command_to_send)
#     mavproxy_process.stdin.flush()
#     #xtime.sleep(60)
#     print("Command sent")
#
#     # Read the response
#     response = mavproxy_process.stdout.read()
#     print(response)
# finally:
#     # Close the process and cleanup
#     mavproxy_process.stdin.close()
#     mavproxy_process.stdout.close()
#     mavproxy_process.stderr.close()
#     mavproxy_process.terminate()  # Terminate the process
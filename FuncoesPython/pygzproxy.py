from pymavlink import mavutil
import time

# Criar uma conexão MAVLink com o Gazebo
# Aqui, estamos usando a porta 9002 para ouvir o Gazebo
connection_string = 'udp:127.0.0.1:9002'
mav = mavutil.mavlink_connection(connection_string)

# Esperar pelo heartbeat para confirmar a conexão
mav.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (mav.target_system, mav.target_component))

# Loop principal
while True:
    # Enviar heartbeat
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,  # type
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # autopilot
        mavutil.mavlink.MAV_MODE_MANUAL_ARMED,  # base mode
        0,  # custom mode
        mavutil.mavlink.MAV_STATE_ACTIVE  # system status
    )

    # Ler mensagens do Gazebo
    msg = mav.recv_match(blocking=False)
    if msg is not None:
        # Aqui você pode lidar com as mensagens recebidas
        pass

    time.sleep(1)
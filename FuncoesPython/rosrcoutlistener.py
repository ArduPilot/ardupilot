import rospy
from std_msgs.msg import String
from mavros_msgs.msg import RCOut
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import argparse
import atexit


def exit_handler():
    print('My application is ending!')
    try:
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED,0)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_MOVING_SPEED,0)
    except:
        print('No motors connected!')


atexit.register(exit_handler)

parser = argparse.ArgumentParser(description='Descricao dos parametros de configuracao')

parser.add_argument('--addr_mx_torque_enable', type=int, help='endereco torque', default=24)
parser.add_argument('--addr_mx_goal_position', type=int, help='endereco torque', default=30)
parser.add_argument('--addr_mx_present_position', type=int, help='endereco torque', default=36)
parser.add_argument('--addr_mx_moving_speed', type=int, help='endereco torque', default=32)
parser.add_argument('--moving_speed', type=int,
                    help='Torque maximo - Control table address is different in Dynamixel model', default=24)
parser.add_argument('--dxl_id', type=int, help='endereco do motor 1', default=1)
parser.add_argument('--dxl_id2', type=int, help='endereco do motor 2', default=2)
parser.add_argument('--baundrate', type=int, help='endereco do motor 2', default=57600)
parser.add_argument('--canal_motor_1', type=int, help='canal de escuta rcout motor 1', default=0)
parser.add_argument('--canal_motor_2', type=int, help='canal de escuta rcout motor 2', default=2)
parser.add_argument('--pwm_trim', type=int, help='valor zero do pwm', default=1500)
parser.add_argument('--devicename', type=str, help='endereco de conexao', default='/dev/ttyUSB0')

args = parser.parse_args()

# Control table address
ADDR_MX_TORQUE_ENABLE = args.addr_mx_torque_enable  # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION = args.addr_mx_goal_position
ADDR_MX_PRESENT_POSITION = args.addr_mx_present_position
ADDR_MX_MOVING_SPEED = args.addr_mx_moving_speed

# Protocol version
PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID = args.dxl_id  # Dynamixel ID : 1
DXL_ID2 = args.dxl_id2  # Dynamixel ID : 1
BAUDRATE = args.baundrate  # Dynamixel default baudrate : 57600
DEVICENAME = args.devicename  # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque

PWM_TRIM = args.pwm_trim


def outputmapping(valor):
    if valor < 0:
        saida = int((-valor / (2000-PWM_TRIM)) * 1023)
    else:
        saida = int((1024 + (valor / (2000-PWM_TRIM)) * 1023))
    return saida


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard  %d and %d", data.channels[args.canal_motor_1],
                  data.channels[args.canal_motor_2])
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED,
                                                              outputmapping(data.channels[args.canal_motor_1] - PWM_TRIM))
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_MOVING_SPEED,
                                                              outputmapping(data.channels[args.canal_motor_2] - PWM_TRIM))


def roslistener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('roslistener', anonymous=True)

    rospy.Subscriber("/mavros/rc/out", RCOut, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':

    # Set the port path
    portHandler = PortHandler(DEVICENAME)

    # Set the protocol version
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    try:
        if portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()
    except:
        print("Failed to open the port - port is busy")
        quit()

    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        quit()

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 1 has been successfully connected")

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_MX_TORQUE_ENABLE,
                                                              TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel 2 has been successfully connected")

    roslistener()

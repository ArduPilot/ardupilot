import sys, os
import time
import serial
from pymavlink import mavutil

import logging, coloredlogs

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../', 'Tools/scripts')))

# if "BALENA_SERVICE_NAME" in os.environ:
#     sys.path.append("/app/ardupilot/Tools/scripts")
#     sys.path.append("/app/ardupilot/dv/scripts")
#     # sys.path.append("/data")
# else:
#     os.path.dirname(os.path.realpath(__file__))
#     sys.path.append("/home/slovak/ardupilot/Tools/scripts")
#     sys.path.append("/home/slovak/ardupilot/dv/scripts")

import uploader

coloredlogs.install(
    level='DEBUG',
    # fmt="%(asctime)s %(hostname)s %(name)s[%(process)d] %(levelname)s %(message)s"
    fmt="%(name)s[%(process)d] %(levelname)s %(message)s",
    level_styles={
        'info': {
            'color': 'green',
            'bold': True,
        },
        'debug': {
            'color': 'white'
        },
        'warn': {
            'color': 'yellow',
        },
        'error': {
            'color': 'red',
            'bold': True,
        },
    },
)
logger = logging.getLogger(__name__)


def get_tcp_master(connection='tcp:127.0.0.1:5760'):
    logger.info(f"Probing autopilot at {connection}")
    try:
        some_master = mavutil.mavlink_connection(connection)
    except Exception as e:
        logger.error(type(e))
        logger.error(e)
        logger.error(e.args)
        return None

    if some_master.wait_heartbeat(timeout=10) is None:
        return None

    return some_master


def get_serial_master(lport, bauds=(921600, 115200, 57600, 1500000)):
    for baud_flightstack in bauds:
        logger.info(f"Probing autopilot at {lport}:{baud_flightstack}")
        some_master = mavutil.mavlink_connection(lport, baud=baud_flightstack)

        res = None

        for serial_tries in range(10):
            try:
                res = some_master.wait_heartbeat(timeout=4)
                break
            except serial.serialutil.SerialException:
                logger.warning(
                    f"The serial might be occupied by the mavlink-router, waiting untill it is free: {serial_tries}")
                time.sleep(3)

        if res is not None:
            return some_master

    return None


def get_git_revision(some_master):
    some_master.mav.command_long_send(
        some_master.target_system, some_master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 1,
        *[mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0]
    )

    msg = some_master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=10)

    if msg is not None:
        return "".join([chr(c) for c in msg.to_dict()["flight_custom_version"]])
    else:
        return None


def flash_bootloader(some_master):
    print("Sending MAV_CMD_FLASH_BOOTLOADER ...")

    now = time.time()

    some_master.mav.command_long_send(
        some_master.target_system, some_master.target_component,
        mavutil.mavlink.MAV_CMD_FLASH_BOOTLOADER, 1,
        *[0, 0, 0, 0, 290876, 0, 0]
    )

    print("Listening to ack and status text ...")
    while True:
        if time.time() - now > 10:
            print("Timeout expired, exiting ...")
            return False

        msg = some_master.recv_match(blocking=True, timeout=1)
        if msg is not None:
            if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT:
                print(msg.to_dict()["text"])
            if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_ACK:
                msgd = msg.to_dict()
                print(msgd)
                # TODO, OLSLO, handle automatic exit from here.
                # mavutil.mavlink.MAV_RESULT_ACCEPTED
                print("OK")
                break

    print("Listening to status text few more seconds...")
    now = time.time()
    while True:
        if time.time() - now > 5:
            print("Timeout expired, exiting ...")
            break

        msg = some_master.recv_match(blocking=True, timeout=1)
        if msg is not None:
            if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT:
                print(msg.to_dict()["text"])

    return True


def get_firmware_revision(apj_path="./build/CubeOrangePlus-dv/bin/arducopter.apj"):
    fw = uploader.firmware(apj_path)
    return fw.desc["git_identity"]

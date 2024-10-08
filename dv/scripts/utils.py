import sys, os
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


def get_serial_master(lport):
    for baud_flightstack in [921600, 500000, 115200, 57600]:
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

    AUTOPILOT_VERSION = some_master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=10).to_dict()

    if AUTOPILOT_VERSION is not None:
        return "".join([chr(c) for c in AUTOPILOT_VERSION["flight_custom_version"]])
    else:
        return None

def get_firmware_revision(apj_path="./build/CubeOrangePlus-dv/bin/arducopter.apj"):
    fw = uploader.firmware(apj_path)
    return fw.desc["git_identity"]

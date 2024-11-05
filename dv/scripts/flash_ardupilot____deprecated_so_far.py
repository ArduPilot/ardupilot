#!/usr/bin/env python3

import time
import serial
import sys, os
from pprint import pprint
from pymavlink import mavutil
from balena_device import Device

if "BALENA_SERVICE_NAME" in os.environ:
    sys.path.append("/app/ardupilot/Tools/scripts")
    sys.path.append("/app/ardupilot/dv/scripts")
    # sys.path.append("/data")
else:
    sys.path.append("/home/slovak/ardupilot/Tools/scripts")
    sys.path.append("/home/slovak/ardupilot/dv/scripts")

import uploader
import param_manip as pm

import logging, coloredlogs

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


# Not used for now
def force_reboot_if_in_bootloader(port="/dev/ttyTHS1", baud_rate_bootloader=115200, baud_rate_flightstack=(921600)):
    up = uploader.uploader(port, baud_rate_bootloader, baud_rate_flightstack)
    logger.info("opening")

    up.open()

    logger.info("ident")
    up.identify()

    logger.info("dump")
    up.dump_board_info()

    time.sleep(1)

    logger.info("reboot")
    logger.info(up._uploader__reboot())

    logger.info("close")
    up.close()


def check_autopilot_accessible(some_master):
    out = some_master.wait_heartbeat(timeout=10)
    if out is not None:
        logger.info("HEARTBEAT_DONE")
        return True
    else:
        return False


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


def try_upgrade_firmware(apj_path, force_flash=True, preserve_calibrations=False, preserve_params_list="",
                         port="/dev/ttyACM0",
                         baud_bootloader=115200):
    calib_file_path = os.path.join(os.getcwd(), "calibration_parameters.csv")

    current_ardupilot_git_revision = "unknown"
    res = None

    device = Device()
    logger.info(device.authenticate())

    # TODO, OLSLO, stop mavlink-router here only.
    # Or even try to get the version over mavlink-router if it is running and if cube is healthy.

    # Wait for supervisor state to applied and success
    device.wait_for_supervisor_state(targets_super_state="success", target_app_state="applied")
    mavlink_router_present = device.wait_for_service_status("mavlink-router", "Running")

    if mavlink_router_present:
        # Now stop mavlink router
        device.try_stop_service("mavlink-router")
        logger.warning(f"Services stopped: mavlink-router")

    some_master = get_serial_master(port)

    if some_master is None:
        logger.warning("autopilot is inaccessible, check if it is already in bootloader")
        preserve_calibrations = False
    else:
        logger.info("OK")

    fw = uploader.firmware(apj_path)
    candidate_ardupilot_git_revision = fw.desc["git_identity"]

    if preserve_calibrations:
        pm.save_calibs(some_master, calib_file_path=calib_file_path, calib_param_ids=preserve_params_list)
        some_master.close()

    sys.argv = [sys.argv[0]] + [
        "--port", port,
        "--baud-flightstack", "921600,500000,115200,57600",
        "--baud-bootloader", str(baud_bootloader),
        apj_path
    ]

    try:
        uploader.main()
    except SystemExit:
        logger.info("ignoring SystemExit")

    time.sleep(30)

    some_master = get_serial_master(port)
    if some_master is None:
        logger.fatal("autopilot is inaccessible, something went wrong during the update.")
    else:
        just_uploaded_version = get_git_revision(some_master)

        if just_uploaded_version == candidate_ardupilot_git_revision:
            logger.info(
                f"Succesfully updated ardupilot from {current_ardupilot_git_revision} to {just_uploaded_version}")
        else:
            logger.fatal(f"Failed to update ardupilot from {current_ardupilot_git_revision} to {just_uploaded_version}")

        if preserve_calibrations:
            if pm.restore_calibs(some_master, calib_file_path=calib_file_path):
                logger.info("Restore calibrations success")
            else:
                logger.fatal("Restore calibrations failed")

    if mavlink_router_present:
        logger.warning(f"Restarting services: mavlink-router")
        device.try_start_service("mavlink-router")


def need_update(apj_path, connection_serial="/dev/ttyTHS1", connection_tcp='tcp:127.0.0.1:5760'):
    device = Device()
    logger.info(device.authenticate())
    device.wait_for_supervisor_state(targets_super_state="success", target_app_state="applied")
    mavlink_router_present = device.wait_for_service_status("mavlink-router", "Running", timeout=3.0)

    some_master = None

    if mavlink_router_present:
        some_master = get_tcp_master(connection=connection_tcp)

    if some_master is None:
        some_master = get_serial_master(connection_serial)

    if some_master is None:
        return True

    current_ardupilot_git_revision = get_git_revision(some_master)
    logger.info(f"current_ardupilot_git_revision: {current_ardupilot_git_revision}")
    some_master.close()

    fw = uploader.firmware(apj_path)
    candidate_ardupilot_git_revision = fw.desc["git_identity"]

    logger.info(f"candidate_ardupilot_git_revision: {candidate_ardupilot_git_revision}")

    if candidate_ardupilot_git_revision == current_ardupilot_git_revision:
        logger.info(
            f"candidate_ardupilot_git_revision == current_ardupilot_git_revision ({candidate_ardupilot_git_revision} == {current_ardupilot_git_revision})")
        logger.info(f"Current and candidate git revisions are the same")
        return False
    else:
        logger.info(
            f"candidate_ardupilot_git_revision != current_ardupilot_git_revision ({candidate_ardupilot_git_revision} == {current_ardupilot_git_revision})")

    return True


if __name__ == '__main__':

    if "BALENA_SERVICE_NAME" in os.environ:
        gapj_path = "./build/CubeOrangePlus-dv/bin/arducopter.apj"
        gport = "/dev/ttyTHS1"
    else:
        gapj_path = "/home/slovak/remote-id/dv-kd-sw-rid/ardupilot/build/CubeOrangePlus-dv/bin/arducopter.apj"
        gport = "/dev/ttyACM0"

    force_flash = False
    if "FORCE_FLASH" in os.environ:
        if os.environ["FORCE_FLASH"] == "1":
            force_flash = True
        else:
            force_flash = False

    preserve_calibs = True
    if "PRESERVE_CALIBRATIONS" in os.environ:
        if os.environ["PRESERVE_CALIBRATIONS"] == "1":
            preserve_calibs = True
        else:
            preserve_calibs = False

    preserve_params_list = pm.calib_param_ids_default
    if "PRESERVE_PARAMETERS" in os.environ:
        preserve_params_list = os.environ["PRESERVE_PARAMETERS"].split(",")

    if not force_flash:
        if not need_update(gapj_path, connection_serial=gport):
            logger.info("No update needed, sleeping...")
            sys.exit(0)

    try_upgrade_firmware(
        gapj_path,
        force_flash=force_flash,
        preserve_calibrations=preserve_calibs,
        preserve_params_list=preserve_params_list,
        port=gport,
        baud_bootloader=115200
    )


# %%
#
# ser = serial.Serial(port='/dev/ttyTHS1', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)


# python3 ./Tools/scripts/uploader.py --port /dev/ttyTHS1 --baud-flightstack 57600  --baud-bootloader 115200 --identify
# python3 ./Tools/scripts/uploader.py --port /dev/ttyTHS1 --baud-flightstack 57600  --baud-bootloader 115200 ./build/CubeOrangePlus-dv/bin/arducopter.apj
# python3 ./Tools/scripts/uploader.py --port /dev/ttyTHS1 --baud-flightstack 57600  --baud-bootloader 115200 --identify

# "--baud-flightstack", "921600,500000,115200,57600",
# "--baud-bootloader", str(baud_bootloader),

#!/usr/bin/env python3

import time
import sys, os
from pprint import pprint
from pymavlink import mavutil

if "BALENA_SERVICE_NAME" in os.environ:
    sys.path.append("/app/ardupilot/Tools/scripts")
    sys.path.append("/app/ardupilot/dv/scripts")
    # sys.path.append("/slovak-shared")
else:
    sys.path.append("/home/slovak/remote-id/dv-kd-sw-rid/ardupilot/Tools/scripts")
    sys.path.append("/home/slovak/remote-id/dv-kd-sw-rid/ardupilot/dv/scripts")

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


def try_upgrade_firmware(apj_path, force_flash=True, preserve_calibrations=False, preserve_params_list="",
                         port="/dev/ttyACM0",
                         baud_bootloader=115200):
    bauds_flightstack = [921600, 500000, 115200, 57600]

    calib_file_path = os.path.join(os.getcwd(), "calibration_parameters.csv")

    res = None

    baud_flightstack = None

    for baud_flightstack in bauds_flightstack:
        logger.info(f"Probing autopilot at {port}:{baud_flightstack}")
        some_master = mavutil.mavlink_connection(port, baud=baud_flightstack)
        res = some_master.wait_heartbeat(timeout=4)
        if res is not None:
            break

    autopilot_online = False

    if res is None:
        logger.warning("autopilot is inaccessible, check if it is already in bootloader")
        preserve_calibrations = False
    else:
        logger.info("OK")

    current_ardupilot_git_revision = "unknown"
    if autopilot_online:
        current_ardupilot_git_revision = get_git_revision(some_master)
        logger.info(f"current_ardupilot_git_revision: {current_ardupilot_git_revision}")

    fw = uploader.firmware(apj_path)
    candidate_ardupilot_git_revision = fw.desc["git_identity"]

    logger.info(f"candidate_ardupilot_git_revision: {candidate_ardupilot_git_revision}")

    if candidate_ardupilot_git_revision == current_ardupilot_git_revision:
        logger.info(
            f"candidate_ardupilot_git_revision == current_ardupilot_git_revision ({candidate_ardupilot_git_revision} == {current_ardupilot_git_revision})")
        if not force_flash:
            logger.info(f"Current and candidate git revisions are the same, exiting ...")
            sys.exit(0)
        else:
            logger.warning(f"Current and candidate git revisions are the same, but forced flash is enabled")
    else:
        logger.info(
            f"candidate_ardupilot_git_revision != current_ardupilot_git_revision ({candidate_ardupilot_git_revision} == {current_ardupilot_git_revision})")

    logger.info(
        f"Autopilot will be updated from {current_ardupilot_git_revision} to {candidate_ardupilot_git_revision}")

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

    logger.info("Probing autopilot ...")
    some_master = mavutil.mavlink_connection(port, baud=baud_flightstack)
    res = some_master.wait_heartbeat(timeout=30)
    if res is None:
        logger.fatal("autopilot is inaccessible, something went wrong during the update.")
        sys.exit(1)

    just_uploaded_version = get_git_revision(some_master)

    if just_uploaded_version == candidate_ardupilot_git_revision:
        logger.info(f"Succesfully updated ardupilot from {current_ardupilot_git_revision} to {just_uploaded_version}")
    else:
        logger.fatal(f"Failed to update ardupilot from {current_ardupilot_git_revision} to {just_uploaded_version}")
        sys.exit(1)

    if preserve_calibrations:
        if pm.restore_calibs(some_master, calib_file_path=calib_file_path):
            logger.info("Restore calibrations success")
        else:
            logger.fatal("Restore calibrations failed")
            sys.exit(1)

    logger.info("Upgrade autopilot success")
    sys.exit(0)


# %%


if __name__ == '__main__':

    if "BALENA_SERVICE_NAME" in os.environ:
        gapj_path = "./build/CubeOrangePlus-dv/bin/arducopter.apj"
        port = "/dev/ttyTHS1"
    else:
        gapj_path = "/home/slovak/remote-id/dv-kd-sw-rid/ardupilot/build/CubeOrangePlus-dv/bin/arducopter.apj"
        port = "/dev/ttyACM0"

    force_flash = False
    if "FORCE_FLASH" in os.environ:
        if os.environ["FORCE_FLASH"] == "1":
            force_flash = True
        else:
            force_flash = False

    preserve_calibrations = True
    if "PRESERVE_CALIBRATIONS" in os.environ:
        if os.environ["PRESERVE_CALIBRATIONS"] == "1":
            preserve_calibrations = True
        else:
            preserve_calibrations = False

    preserve_params_list = pm.calib_param_ids_default
    if "PRESERVE_PARAMETERS" in os.environ:
        preserve_params_list = os.environ["PRESERVE_PARAMETERS"].split(",")

    try_upgrade_firmware(
        gapj_path,
        force_flash=force_flash,
        preserve_calibrations=preserve_calibrations,
        preserve_params_list=preserve_params_list,
        port=port,
        baud_bootloader=115200
    )

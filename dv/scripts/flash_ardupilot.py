#!/usr/bin/env python3

import time
import sys, os
from pprint import pprint
from pymavlink import mavutil

if "BALENA_SERVICE_NAME" in os.environ:
    sys.path.append("/app/ardupilot/Tools/scripts")
    sys.path.append("/app/ardupilot/dv/scripts")
else:
    sys.path.append("/home/slovak/remote-id/dv-kd-sw-rid/ardupilot/Tools/scripts")
    sys.path.append("/home/slovak/remote-id/dv-kd-sw-rid/ardupilot/dv/scripts")

import uploader
import param_manip as pm


# Not used for now
def force_reboot_if_in_bootloader(port="/dev/ttyTHS1", baud_rate_bootloader=115200, baud_rate_flightstack=(921600)):
    up = uploader.uploader(port, baud_rate_bootloader, baud_rate_flightstack)
    print("opening")

    up.open()

    print("ident")
    up.identify()

    print("dump")
    up.dump_board_info()

    time.sleep(1)

    print("reboot")
    print(up._uploader__reboot())

    print("close")
    up.close()


def check_autopilot_accessible(some_master):
    out = some_master.wait_heartbeat(timeout=10)
    if out is not None:
        print("HEARTBEAT_DONE")
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


def try_upgrade_firmware(apj_path, preserve_calibrations=True, port="/dev/ttyACM0", baud_flightstack=921600,
                         baud_bootloader=115200):
    print("Probing autopilot ...")
    some_master = mavutil.mavlink_connection(port, baud=baud_flightstack)
    res = some_master.wait_heartbeat(timeout=10)
    autopilot_online = True
    if res is None:
        print("autopilot is inaccessible, check if it is already in bootloader")
        preserve_calibrations = False
        autopilot_online = False
    else:
        print("OK")

    current_ardupilot_git_revision = "unknown"
    if autopilot_online:
        current_ardupilot_git_revision = get_git_revision(some_master)
        print("current_ardupilot_git_revision: ", current_ardupilot_git_revision)

    fw = uploader.firmware("/home/slovak/remote-id/dv-kd-sw-rid/ardupilot/build/CubeOrangePlus-dv/bin/arducopter.apj")
    candidate_ardupilot_git_revision = fw.desc["git_identity"]

    print("candidate_ardupilot_git_revision: ", candidate_ardupilot_git_revision)

    if preserve_calibrations:
        pm.save_calibs(some_master)

    some_master.close()

    sys.argv = [sys.argv[0]] + [
        "--port", port,
        "--baud-flightstack", str(baud_flightstack),
        "--baud-bootloader", str(baud_bootloader),
        apj_path
    ]

    try:
        uploader.main()
    except SystemExit:
        print("ignoring SystemExit")

    time.sleep(3)

    print("Probing autopilot ...")
    some_master = mavutil.mavlink_connection(port, baud=baud_flightstack)
    res = some_master.wait_heartbeat(timeout=10)
    if res is None:
        print("autopilot is inaccessible, something went wrong during the update.")
        sys.exit(1)

    just_uploaded_version = get_git_revision(some_master)

    if just_uploaded_version == candidate_ardupilot_git_revision:
        print(f"Succesfully updated ardupilot from {current_ardupilot_git_revision} to {just_uploaded_version}")
    else:
        print(f"Failed to update ardupilot from {current_ardupilot_git_revision} to {just_uploaded_version}")
        sys.exit(1)

    if preserve_calibrations:
        pm.restore_calibs(some_master)

    sys.exit(0)


# %%


if __name__ == '__main__':

    if "BALENA_SERVICE_NAME" in os.environ:
        gapj_path = "./build/CubeOrangePlus-dv/bin/arducopter.apj"
        port = "/dev/ttyTHS1"
    else:
        gapj_path = "/home/slovak/remote-id/dv-kd-sw-rid/ardupilot/build/CubeOrangePlus-dv/bin/arducopter.apj"
        port = "/dev/ttyACM0"

    try_upgrade_firmware(
        gapj_path,
        preserve_calibrations=True,
        port=port,
        baud_flightstack=921600,
        baud_bootloader=115200
    )

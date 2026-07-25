#!/usr/bin/env python3

'''
MAVLink File Transfer Protocol support example

SPDX-FileCopyrightText: 2024 Amilcar Lucas

SPDX-License-Identifier: GPL-3.0-or-later
'''

from argparse import ArgumentParser

from logging import basicConfig as logging_basicConfig
from logging import getLevelName as logging_getLevelName

from logging import debug as logging_debug
from logging import info as logging_info
from logging import error as logging_error

import os
import sys
#import time

import requests


from pymavlink import mavutil
from pymavlink import mavftp

old_mavftp_member_variable_values = {}


# pylint: disable=duplicate-code
def argument_parser():
    """
    Parses command-line arguments for the script.
    """
    parser = ArgumentParser(description='This main is just an example, adapt it to your needs')
    parser.add_argument("--baudrate", type=int, default=115200,
                        help="master port baud rate. Defaults to %(default)s")
    parser.add_argument("--device", type=str, default='',
                        help="serial device. For windows use COMx where x is the port number. "
                                "For Unix use /dev/ttyUSBx where x is the port number. Defaults to autodetection")
    parser.add_argument("--source-system", type=int, default=250,
                        help='MAVLink source system for this GCS. Defaults to %(default)s')
    parser.add_argument("--loglevel", default="INFO",
                        help="log level. Defaults to %(default)s")

    # MAVFTP settings
    parser.add_argument("--debug", type=int, default=0, choices=[0, 1, 2],
                        help="Debug level 0 for none, 2 for max verbosity. Defaults to %(default)s")

    return parser.parse_args()

def auto_detect_serial():
    preferred_ports = [
        '*FTDI*',
        "*3D*",
        "*USB_to_UART*",
        '*Ardu*',
        '*PX4*',
        '*Hex_*',
        '*Holybro_*',
        '*mRo*',
        '*FMU*',
        '*Swift-Flyer*',
        '*Serial*',
        '*CubePilot*',
        '*Qiotek*',
    ]
    serial_list = mavutil.auto_detect_serial(preferred_list=preferred_ports)
    serial_list.sort(key=lambda x: x.device)

    # remove OTG2 ports for dual CDC
    if len(serial_list) == 2 and serial_list[0].device.startswith("/dev/serial/by-id"):
        if serial_list[0].device[:-1] == serial_list[1].device[0:-1]:
            serial_list.pop(1)

    return serial_list


def auto_connect(device):
    comport = None
    if device:
        comport = mavutil.SerialPort(device=device, description=device)
    else:
        autodetect_serial = auto_detect_serial()
        if autodetect_serial:
            # Resolve the soft link if it's a Linux system
            if os.name == 'posix':
                try:
                    dev = autodetect_serial[0].device
                    logging_debug("Auto-detected device %s", dev)
                    # Get the directory part of the soft link
                    softlink_dir = os.path.dirname(dev)
                    # Resolve the soft link and join it with the directory part
                    resolved_path = os.path.abspath(os.path.join(softlink_dir, os.readlink(dev)))
                    autodetect_serial[0].device = resolved_path
                    logging_debug("Resolved soft link %s to %s", dev, resolved_path)
                except OSError:
                    pass # Not a soft link, proceed with the original device path
            comport = autodetect_serial[0]
        else:
            logging_error("No serial ports found. Please connect a flight controller and try again.")
            sys.exit(1)
    return comport


def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    logging_info("Waiting for flight controller heartbeat")
    m.wait_heartbeat()
    logging_info("Got heartbeat from system %u, component %u", m.target_system, m.target_system)
# pylint: enable=duplicate-code


def delete_local_file_if_exists(filename):
    if os.path.exists(filename):
        os.remove(filename)


def get_list_dir(mav_ftp, directory):
    ret = mav_ftp.cmd_list([directory])
    ret.display_message()
    debug_class_member_variable_changes(mav_ftp)


def get_file(mav_ftp, remote_filename, local_filename, timeout=5):
    #session = mav_ftp.session # save the session to restore it after the file transfer
    mav_ftp.cmd_get([remote_filename, local_filename])
    ret = mav_ftp.process_ftp_reply('OpenFileRO', timeout=timeout)
    ret.display_message()
    #mav_ftp.session = session # FIXME: this is a huge workaround hack # pylint: disable=fixme
    debug_class_member_variable_changes(mav_ftp)
    #time.sleep(0.2)


def get_last_log(mav_ftp):
    try:
        with open('LASTLOG.TXT', 'r', encoding='UTF-8') as file:
            file_contents = file.readline()
            remote_filenumber = int(file_contents.strip())
    except FileNotFoundError:
        logging_error("File LASTLOG.TXT not found.")
        return
    except ValueError:
        logging_error("Could not extract last log file number from LASTLOG.TXT contents %s", file_contents)
        return
    remote_filenumber = remote_filenumber - 1 # we do not want the very last log
    remote_filename = f'/APM/LOGS/{remote_filenumber:08}.BIN'
    get_file(mav_ftp, remote_filename, 'LASTLOG.BIN', 0)


def download_script(url, local_filename):
    # Download the script from the internet to the PC
    response = requests.get(url, timeout=5)

    if response.status_code == 200:
        with open(local_filename, "wb") as file:
            file.write(response.content)
    else:
        logging_error("Failed to download the file")


def create_directory(mav_ftp, remote_directory):
    ret = mav_ftp.cmd_mkdir([remote_directory])
    ret.display_message()
    debug_class_member_variable_changes(mav_ftp)


def remove_directory(mav_ftp, remote_directory):
    ret = mav_ftp.cmd_rmdir([remote_directory])
    ret.display_message()
    debug_class_member_variable_changes(mav_ftp)


def upload_script(mav_ftp, remote_directory, local_filename, timeout):
    # Upload it from the PC to the flight controller
    mav_ftp.cmd_put([local_filename, remote_directory + '/' + local_filename])
    ret = mav_ftp.process_ftp_reply('CreateFile', timeout=timeout)
    ret.display_message()
    debug_class_member_variable_changes(mav_ftp)


def debug_class_member_variable_changes(instance):
    return
    global old_mavftp_member_variable_values  # pylint: disable=global-statement, unreachable
    new_mavftp_member_variable_values = instance.__dict__
    if old_mavftp_member_variable_values and instance.ftp_settings.debug > 1:  # pylint: disable=too-many-nested-blocks
        logging_info(f"{instance.__class__.__name__} member variable changes:")
        for key, value in new_mavftp_member_variable_values.items():
            if old_mavftp_member_variable_values[key] != value:
                old_value = old_mavftp_member_variable_values[key]
                if old_value and isinstance(value, mavftp.FTP_OP):
                    # Convert both new and old FTP_OP instances to dictionaries for comparison
                    new_op_dict = dict(value.items())
                    old_op_dict = dict(old_value.items()) if isinstance(old_value, mavftp.FTP_OP) else {}
                    for op_key, op_value in new_op_dict.items():
                        old_op_value = old_op_dict.get(op_key)
                        if old_op_value != op_value:
                            logging_info(f"CHANGED {key}.{op_key}: {old_op_value} -> {op_value}")
                else:
                    logging_info(f"CHANGED {key}: {old_mavftp_member_variable_values[key]} -> {value}")
    old_mavftp_member_variable_values = new_mavftp_member_variable_values.copy()

def main():
    '''for testing/example purposes only'''
    args = argument_parser()

    logging_basicConfig(level=logging_getLevelName(args.loglevel), format='%(levelname)s - %(message)s')

    # create a mavlink serial instance
    comport = auto_connect(args.device)
    master = mavutil.mavlink_connection(comport.device, baud=args.baudrate, source_system=args.source_system)

    # wait for the heartbeat msg to find the system ID
    wait_heartbeat(master)

    mav_ftp = mavftp.MAVFTP(master,
                            target_system=master.target_system,
                            target_component=master.target_component)

    mav_ftp.ftp_settings.debug = args.debug

    if args.loglevel == 'DEBUG':
        mav_ftp.ftp_settings.debug = 2

    debug_class_member_variable_changes(mav_ftp)

    get_list_dir(mav_ftp, '/APM/LOGS')

    delete_local_file_if_exists("params.param")
    delete_local_file_if_exists("defaults.param")
    mav_ftp.cmd_getparams(["params.param", "defaults.param"])
    ret = mav_ftp.process_ftp_reply('OpenFileRO', timeout=500)
    ret.display_message()

    get_list_dir(mav_ftp, '/APM/LOGS')

    #delete_local_file_if_exists("LASTLOG.TXT")
    delete_local_file_if_exists("LASTLOG.BIN")

    #get_file(mav_ftp, '/APM/LOGS/LASTLOG.TXT', 'LASTLOG.TXT')

    get_list_dir(mav_ftp, '/APM/LOGS')

    #get_file(mav_ftp, '/APM/LOGS/LASTLOG.TXT', 'LASTLOG2.TXT')

    get_last_log(mav_ftp)

    remove_directory(mav_ftp, "test_dir")
    create_directory(mav_ftp, "test_dir")
    remove_directory(mav_ftp, "test_dir")
    create_directory(mav_ftp, "test_dir2")

    remote_directory = '/APM/Scripts'
    #create_directory(mav_ftp, remote_directory)

    url = "https://discuss.ardupilot.org/uploads/short-url/4pyrl7PcfqiMEaRItUhljuAqLSs.lua"
    local_filename = "copter-magfit-helper.lua"

    if not os.path.exists(local_filename):
        download_script(url, local_filename)

    upload_script(mav_ftp, remote_directory, local_filename, 5)

    url = "https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5/libraries/AP_Scripting/applets/" \
            "VTOL-quicktune.lua"
    local_filename = "VTOL-quicktune.lua"

    if not os.path.exists(local_filename):
        download_script(url, local_filename)

    upload_script(mav_ftp, remote_directory, local_filename, 5)

    master.close()


if __name__ == "__main__":
    main()

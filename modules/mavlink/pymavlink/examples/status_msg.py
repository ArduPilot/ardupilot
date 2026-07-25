#!/usr/bin/env python3

'''
Print flight controller banner statustext messages and AUTOPILOT_VERSION message information.

SPDX-FileCopyrightText: 2024 Amilcar do Carmo Lucas <amilcar.lucas@iav.de>

SPDX-License-Identifier: GPL-3.0-or-later
'''

import time

from argparse import ArgumentParser

from typing import List

from pymavlink import mavutil

#FLAKE_CLEAN


class FlightcontrollerInfo:  # pylint: disable=too-many-instance-attributes
    """
    Handle flight controller information.

    It includes methods for setting various attributes such as system ID, component ID,
    autopilot type, vehicle type, and capabilities among others.
    """
    def __init__(self):
        self.system_id = None
        self.component_id = None
        self.autopilot = None
        self.vehicle_type = None
        self.mav_type = None
        self.flight_sw_version = None
        self.flight_sw_version_and_type = None
        self.board_version = None
        self.flight_custom_version = None
        self.os_custom_version = None
        self.vendor = None
        self.vendor_id = None
        self.vendor_and_vendor_id = None
        self.product = None
        self.product_id = None
        self.product_and_product_id = None
        self.capabilities = None

        self.is_supported = False
        self.is_mavftp_supported = False

    def get_info(self):
        return {
            "Vendor": self.vendor_and_vendor_id,
            "Product": self.product_and_product_id,
            "Hardware Version": self.board_version,
            "Autopilot Type": self.autopilot,
            "ArduPilot FW Type": self.vehicle_type,
            "MAV Type": self.mav_type,
            "Firmware Version": self.flight_sw_version_and_type,
            "Git Hash": self.flight_custom_version,
            "OS Git Hash": self.os_custom_version,
            "Capabilities": self.capabilities,
            "System ID": self.system_id,
            "Component ID": self.component_id
        }

    def set_system_id_and_component_id(self, system_id, component_id):
        self.system_id = system_id
        self.component_id = component_id

    def set_autopilot(self, autopilot):
        self.autopilot = self.__decode_mav_autopilot(autopilot)
        self.is_supported = autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA

    def set_type(self, mav_type):
        self.vehicle_type = self.__classify_vehicle_type(mav_type)
        self.mav_type = self.__decode_mav_type(mav_type)

    def set_flight_sw_version(self, version):
        v_major, v_minor, v_patch, v_fw_type = self.__decode_flight_sw_version(version)
        self.flight_sw_version = f"{v_major}.{v_minor}.{v_patch}"
        self.flight_sw_version_and_type = self.flight_sw_version + " " + v_fw_type

    def set_board_version(self, board_version):
        self.board_version = board_version

    def set_flight_custom_version(self, flight_custom_version):
        self.flight_custom_version = ''.join(chr(c) for c in flight_custom_version)

    def set_os_custom_version(self, os_custom_version):
        self.os_custom_version = ''.join(chr(c) for c in os_custom_version)

    def set_vendor_id_and_product_id(self, vendor_id, product_id):
        pid_vid_dict = self.__list_ardupilot_supported_usb_pid_vid()

        self.vendor_id = f"0x{vendor_id:04X}" if vendor_id else "Unknown"
        if vendor_id and vendor_id in pid_vid_dict:
            self.vendor = f"{pid_vid_dict[vendor_id]['vendor']}"
        elif vendor_id:
            self.vendor = "Unknown"
        self.vendor_and_vendor_id = f"{self.vendor} ({self.vendor_id})"

        self.product_id = f"0x{product_id:04X}" if product_id else "Unknown"
        if vendor_id and product_id and product_id in pid_vid_dict[vendor_id]['PID']:
            self.product = f"{pid_vid_dict[vendor_id]['PID'][product_id]}"
        elif product_id:
            self.product = "Unknown"
        self.product_and_product_id = f"{self.product} ({self.product_id})"

    def set_capabilities(self, capabilities):
        self.capabilities = self.__decode_flight_capabilities(capabilities)
        self.is_mavftp_supported = capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FTP

    @staticmethod
    def __decode_flight_sw_version(flight_sw_version):
        '''decode 32 bit flight_sw_version mavlink parameter
        corresponds to ArduPilot encoding in  GCS_MAVLINK::send_autopilot_version'''
        fw_type_id = (flight_sw_version >>  0) % 256  # noqa E221, E222
        patch      = (flight_sw_version >>  8) % 256  # noqa E221, E222
        minor      = (flight_sw_version >> 16) % 256  # noqa E221
        major      = (flight_sw_version >> 24) % 256  # noqa E221
        if fw_type_id == 0:
            fw_type = "dev"
        elif fw_type_id == 64:
            fw_type = "alpha"
        elif fw_type_id == 128:
            fw_type = "beta"
        elif fw_type_id == 192:
            fw_type = "rc"
        elif fw_type_id == 255:
            fw_type = "official"
        else:
            fw_type = "undefined"
        return major, minor, patch, fw_type


    @staticmethod
    def __decode_flight_capabilities(capabilities):
        '''Decode 32 bit flight controller capabilities bitmask mavlink parameter.
        Returns a dict of concise English descriptions of each active capability.
        '''
        capabilities_dict = {}

        # Iterate through each bit in the capabilities bitmask
        for bit in range(32):
            # Check if the bit is set
            if capabilities & (1 << bit):
                # Use the bit value to get the corresponding capability enum
                capability = mavutil.mavlink.enums["MAV_PROTOCOL_CAPABILITY"].get(1 << bit, "Unknown capability")

                if hasattr(capability, 'description'):
                    # Append the abbreviated name and description of the capability dictionary
                    capabilities_dict[capability.name.replace("MAV_PROTOCOL_CAPABILITY_", "")] = capability.description
                else:
                    capabilities_dict[f'BIT{bit}'] = capability

        return capabilities_dict


    # see for more info:
    # import pymavlink.dialects.v20.ardupilotmega
    # pymavlink.dialects.v20.ardupilotmega.enums["MAV_TYPE"]
    @staticmethod
    def __decode_mav_type(mav_type):
        return mavutil.mavlink.enums["MAV_TYPE"].get(mav_type,
                                                    mavutil.mavlink.EnumEntry("None", "Unknown type")).description


    @staticmethod
    def __decode_mav_autopilot(mav_autopilot):
        return mavutil.mavlink.enums["MAV_AUTOPILOT"].get(mav_autopilot,
                                                        mavutil.mavlink.EnumEntry("None", "Unknown type")).description


    @staticmethod
    def __classify_vehicle_type(mav_type_int):
        """
        Classify the vehicle type based on the MAV_TYPE enum.

        Parameters:
        mav_type_int (int): The MAV_TYPE enum value.

        Returns:
        str: The classified vehicle type.
        """
        # Define the mapping from MAV_TYPE_* integer to vehicle type category
        mav_type_to_vehicle_type = {
            mavutil.mavlink.MAV_TYPE_FIXED_WING: 'ArduPlane',
            mavutil.mavlink.MAV_TYPE_QUADROTOR: 'ArduCopter',
            mavutil.mavlink.MAV_TYPE_COAXIAL: 'Heli',
            mavutil.mavlink.MAV_TYPE_HELICOPTER: 'Heli',
            mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER: 'AntennaTracker',
            mavutil.mavlink.MAV_TYPE_GCS: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_AIRSHIP: 'ArduBlimp',
            mavutil.mavlink.MAV_TYPE_FREE_BALLOON: 'ArduBlimp',
            mavutil.mavlink.MAV_TYPE_ROCKET: 'ArduCopter',
            mavutil.mavlink.MAV_TYPE_GROUND_ROVER: 'Rover',
            mavutil.mavlink.MAV_TYPE_SURFACE_BOAT: 'Rover',
            mavutil.mavlink.MAV_TYPE_SUBMARINE: 'ArduSub',
            mavutil.mavlink.MAV_TYPE_HEXAROTOR: 'ArduCopter',
            mavutil.mavlink.MAV_TYPE_OCTOROTOR: 'ArduCopter',
            mavutil.mavlink.MAV_TYPE_TRICOPTER: 'ArduCopter',
            mavutil.mavlink.MAV_TYPE_FLAPPING_WING: 'ArduPlane',
            mavutil.mavlink.MAV_TYPE_KITE: 'ArduPlane',
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR: 'ArduPlane',
            mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR: 'ArduPlane',
            mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR: 'ArduPlane',
            mavutil.mavlink.MAV_TYPE_VTOL_RESERVED2: 'ArduPlane',
            mavutil.mavlink.MAV_TYPE_VTOL_RESERVED3: 'ArduPlane',
            mavutil.mavlink.MAV_TYPE_VTOL_RESERVED4: 'ArduPlane',
            mavutil.mavlink.MAV_TYPE_VTOL_RESERVED5: 'ArduPlane',
            mavutil.mavlink.MAV_TYPE_GIMBAL: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_ADSB: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_PARAFOIL: 'ArduPlane',
            mavutil.mavlink.MAV_TYPE_DODECAROTOR: 'ArduCopter',
            mavutil.mavlink.MAV_TYPE_CAMERA: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_CHARGING_STATION: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_FLARM: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_SERVO: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_ODID: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_DECAROTOR: 'ArduCopter',
            mavutil.mavlink.MAV_TYPE_BATTERY: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_PARACHUTE: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_LOG: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_OSD: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_IMU: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_GPS: 'AP_Periph',
            mavutil.mavlink.MAV_TYPE_WINCH: 'AP_Periph',
            # Add more mappings as needed
        }

        # Return the classified vehicle type based on the MAV_TYPE enum
        return mav_type_to_vehicle_type.get(mav_type_int, None)

    @staticmethod
    def __list_ardupilot_supported_usb_pid_vid():
        """
        List all ArduPilot supported USB vendor ID (VID) and product ID (PID).

        source: https://ardupilot.org/dev/docs/USB-IDs.html
        """
        return {
            0x0483: {'vendor': 'ST Microelectronics', 'PID': {0x5740: 'ChibiOS'}},
            0x1209: {'vendor': 'ArduPilot', 'PID': {0x5740: 'MAVLink',
                                                    0x5741: 'Bootloader',
                                                    }
                     },
            0x16D0: {'vendor': 'ArduPilot', 'PID': {0x0E65: 'MAVLink'}},
            0x26AC: {'vendor': '3D Robotics', 'PID': {}},
            0x2DAE: {'vendor': 'CubePilot', 'PID': {0x1001: 'CubeBlack bootloader',
                                                    0x1011: 'CubeBlack',
                                                    0x1101: 'CubeBlack+',
                                                    0x1002: 'CubeYellow bootloader',
                                                    0x1012: 'CubeYellow',
                                                    0x1005: 'CubePurple bootloader',
                                                    0x1015: 'CubePurple',
                                                    0x1016: 'CubeOrange',
                                                    0x1058: 'CubeOrange+',
                                                    0x1059: 'CubeRed'
                                                    }
                     },
            0x3162: {'vendor': 'Holybro', 'PID': {0x004B: 'Durandal'}},
            0x27AC: {'vendor': 'Laser Navigation', 'PID': {0x1151: 'VRBrain-v51',
                                                           0x1152: 'VRBrain-v52',
                                                           0x1154: 'VRBrain-v54',
                                                           0x1910: 'VRCore-v10',
                                                           0x1351: 'VRUBrain-v51',
                                                           }
                     },
        }

def collect_banner_messages(master) -> List[str]:
    '''Collect banner information from the flight controller'''
    start_time = time.time()
    banner_msgs = []
    while True:
        msg = master.recv_match(blocking=False)
        if msg is not None:
            if msg.get_type() == 'STATUSTEXT':
                if banner_msgs:
                    banner_msgs.append(msg.text)
                else:
                    banner_msgs = [msg.text]
        if time.time() - start_time > 2:  # Check if 2 seconds have passed since the start of the loop
            break  # Exit the loop if 2 seconds have elapsed
    return banner_msgs

def request_message(master, message_id: int):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0, # confirmation
        message_id, 0, 0, 0, 0, 0, 0)

def request_banner(master):
    '''Request banner information from the flight controller'''
    # https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_SEND_BANNER
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SEND_BANNER,
        0,
        0,  # param1: 0
        0,  # param2: 0
        0,  # param3: 0
        0,  # param4: 0
        0,  # param5: 0
        0,  # param6: 0
        0   # param7: 0
    )

def process_autopilot_version(info, m, banner_msgs) -> str:
    if m is None:
        return "No AUTOPILOT_VERSION MAVLink message received, connection failed.\n" \
            "Only ArduPilot versions newer than 4.0.0 are supported.\n" \
            "Make sure parameter SERIAL0_PROTOCOL is set to 2"

    info.set_capabilities(m.capabilities)
    info.set_flight_sw_version(m.flight_sw_version)
    info.set_board_version(m.board_version)
    info.set_flight_custom_version(m.flight_custom_version)
    info.set_os_custom_version(m.os_custom_version)
    info.set_vendor_id_and_product_id(m.vendor_id, m.product_id)

    os_custom_version = ''
    os_custom_version_index = None
    for i, msg in enumerate(banner_msgs):
        if 'ChibiOS:' in msg:
            os_custom_version = msg.split(' ')[1].strip()
            if os_custom_version != info.os_custom_version:
                print("ChibiOS version mismatch: %s (BANNER)!= % s(AUTOPILOT_VERSION)", os_custom_version,
                                info.os_custom_version)
            os_custom_version_index = i
            continue
        print(f"FC banner {msg}")

    # the banner message after the ChibiOS one contains the FC type
    fc_product = ''
    if os_custom_version_index is not None:
        fc_product_banner_substrings = banner_msgs[os_custom_version_index+1].split(' ')
        if len(fc_product_banner_substrings) >= 3:
            fc_product = fc_product_banner_substrings[0]
    if fc_product != info.product:
        print("FC product mismatch: %s (BANNER)!= %s(AUTOPILOT_VERSION)", fc_product, info.product)
        info.product = fc_product  # force the one from the banner because it is more reliable
    return ""

def main():
    '''Print banner information from the flight controller'''
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--baudrate", type=int,
                    help="master port baud rate", default=115200)
    parser.add_argument("--device", required=True, help="serial device")
    parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                    default=255, help='MAVLink source system for this GCS')
    args = parser.parse_args()

    timeout = 5
    # Connect to the flight controller
    connection = mavutil.mavlink_connection(args.device, baud=args.baudrate, source_system=args.SOURCE_SYSTEM)

    try:
        m = connection.wait_heartbeat(timeout=timeout)  # Wait for heartbeat to confirm connection
        info = FlightcontrollerInfo()
        info.set_system_id_and_component_id(m.get_srcSystem(), m.get_srcComponent())
        info.set_autopilot(m.autopilot)
        info.set_type(m.type)

        request_banner(connection)
        banner_msgs = collect_banner_messages(connection)

        request_message(connection, mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
        m = connection.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=timeout)
        print(process_autopilot_version(info, m, banner_msgs))

        info_dict = info.get_info()
        for key, value in info_dict.items():
            if key == 'Capabilities':
                print(f"{key}:")
                for ckey, cvalue in value.items():
                    print(f"{ckey} ({cvalue})")
                print()
            else:
                print(f"{key}: {value}")

    except KeyboardInterrupt:
        print("\nDisconnecting...")
    finally:
        connection.close()

if __name__ == "__main__":
    main()

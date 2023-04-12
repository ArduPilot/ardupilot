#!/usr/bin/env python3

import enum
import io
import sys
import struct
from argparse import ArgumentParser
from dataclasses import dataclass
from elftools.elf.elffile import ELFFile
from typing import Any


class FirmwareVersionType(enum.Enum):
    Dev = 0
    Alpha = 64
    Beta = 128
    RC = 192
    Official = 255
    EnumEnd = 256


class VehicleType(enum.Enum):
    Rover = 1
    ArduCopter = 2
    ArduPlane = 3
    AntennaTracker = 4
    UNKNOWN = 5
    Replay = 6
    ArduSub = 7
    iofirmware = 8
    AP_Periph = 9
    NONE = 256


class BoardType(enum.Enum):
    SITL = 3
    SMACCM = 4
    PX4 = 5
    LINUX = 7
    VRBRAIN = 8
    CHIBIOS = 10
    F4LIGHT = 11
    EMPTY = 99


class BoardSubType(enum.Enum):
    NONE = 65535

    LINUX_NONE = 1000
    LINUX_ERLEBOARD = 1001
    LINUX_PXF = 1002
    LINUX_NAVIO = 1003
    LINUX_ZYNQ = 1004
    LINUX_BBBMINI = 1005
    LINUX_BEBOP = 1006
    LINUX_ERLEBRAIN2 = 1009
    LINUX_BH = 1010
    LINUX_PXFMINI = 1012
    LINUX_NAVIO2 = 1013
    LINUX_DISCO = 1014
    LINUX_AERO = 1015
    LINUX_DARK = 1016
    LINUX_BLUE = 1018
    LINUX_OCPOC_ZYNQ = 1019
    LINUX_EDGE = 1020
    LINUX_RST_ZYNQ = 1021
    LINUX_POCKET = 1022
    LINUX_NAVIGATOR = 1023
    LINUX_VNAV = 1024
    LINUX_OBAL = 1025
    CHIBIOS_SKYVIPER_F412 = 5000
    CHIBIOS_FMUV3 = 5001
    CHIBIOS_FMUV4 = 5002
    CHIBIOS_GENERIC = 5009
    CHIBIOS_FMUV5 = 5013
    CHIBIOS_VRBRAIN_V51 = 5016
    CHIBIOS_VRBRAIN_V52 = 5017
    CHIBIOS_VRUBRAIN_V51 = 5018
    CHIBIOS_VRCORE_V10 = 5019
    CHIBIOS_VRBRAIN_V54 = 5020


@dataclass
class FWVersion:
    header: int = 0x61706677766572FB
    header_version: bytes = bytes([0, 0])
    pointer_size: int = 0
    vehicle_type: VehicleType = VehicleType.NONE
    board_type: BoardType = BoardType.EMPTY
    board_subtype: BoardSubType = BoardSubType.NONE
    major: int = 0
    minor: int = 0
    patch: int = 0
    firmware_type: FirmwareVersionType = FirmwareVersionType.EnumEnd
    os_software_version: int = 0
    firmware_string: str = ""
    firmware_hash_string: str = ""
    firmware_hash: int = 0
    middleware_name: str = ""
    middleware_hash_string: str = ""
    os_name: str = ""
    os_hash_string: str = ""

    def __str__(self):
        header = self.header.to_bytes(8, byteorder="big")
        header_version = self.header_version.to_bytes(2, byteorder="big")
        firmware_day = self.os_software_version % 100
        firmware_month = self.os_software_version % 10000 - firmware_day
        firmware_year = self.os_software_version - firmware_month - firmware_day
        firmware_month = int(firmware_month / 100)
        firmware_year = int(firmware_year / 10000)
        return f"""
{self.__class__.__name__}:
    header:
        magic: {header[0:7].decode("utf-8")}
        checksum: {hex(header[-1])}
        version: {header_version[0]}.{header_version[1]}
        pointer_size: {self.pointer_size}
    firmware:
        string: {self.firmware_string}
        vehicle: {self.vehicle_type.name}
        board: {self.board_type.name}
        board subtype: {self.board_subtype.name}
        hash: {self.firmware_hash_string}
        hash integer: 0x{self.firmware_hash:02x}
        version: {self.major}.{self.minor}.{self.patch}
        type: {self.firmware_type.name}
    os:
        name: {self.os_name}
        hash: {self.os_hash_string}
        software_version: {firmware_day}/{firmware_month}/{firmware_year}
    middleware:
        name: {self.middleware_name}
        hash: {self.middleware_hash_string}
"""


class Decoder:
    def __init__(self) -> None:
        self.bytesio = io.BytesIO()
        self.fwversion = FWVersion()
        self.byteorder = ""
        self.pointer_size = 0
        self.elffile = None

    def unpack(self, struct_format: str) -> Any:
        struct_format = f"{self.byteorder}{struct_format}"
        size = struct.calcsize(struct_format)
        return struct.unpack(struct_format, self.bytesio.read(size))[0]

    def unpack_string_from_pointer(self) -> str:
        pointer_format = "Q" if self.pointer_size == 8 else "I"
        address = self.unpack(pointer_format)

        # nullptr, return empty string
        if address == 0:
            return ""

        # Calculate address offset for PIE (Position Independent Executables) binaries
        address = next(self.elffile.address_offsets(address))

        current_address = self.bytesio.seek(0, io.SEEK_CUR)
        self.bytesio.seek(address)
        string = []
        while True:
            string += self.bytesio.read(1)
            if string[-1] == 0:
                string = string[0 : len(string) - 1]
                break
        self.bytesio.seek(current_address)
        return bytes(string).decode("UTF-8")

    @staticmethod
    def locate_header(data: bytes, byteorder: str) -> int:
        return data.find(struct.pack(f"{byteorder}Q", FWVersion.header))

    def unpack_fwversion(self) -> None:
        assert self.bytesio.read(8) == struct.pack(
            f"{self.byteorder}Q", FWVersion.header
        )

        self.fwversion.header_version = self.unpack("H")
        major_version = self.fwversion.header_version >> 8

        self.pointer_size = self.unpack("B")
        self.fwversion.pointer_size = self.pointer_size
        self.unpack("B")  # reserved
        self.fwversion.vehicle_type = VehicleType(self.unpack("B"))
        self.fwversion.board_type = BoardType(self.unpack("B"))
        self.fwversion.board_subtype = BoardSubType(self.unpack("H"))

        self.fwversion.major = self.unpack("B")
        self.fwversion.minor = self.unpack("B")
        self.fwversion.patch = self.unpack("B")
        self.fwversion.firmware_type = FirmwareVersionType(self.unpack("B"))
        self.fwversion.os_software_version = self.unpack("I")

        self.fwversion.firmware_string = self.unpack_string_from_pointer()
        self.fwversion.firmware_hash_string = self.unpack_string_from_pointer()
        if major_version >= 2:
            self.fwversion.firmware_hash = self.unpack("I")

        self.fwversion.middleware_name = self.unpack_string_from_pointer()
        self.fwversion.middleware_hash_string = self.unpack_string_from_pointer()
        self.fwversion.os_name = self.unpack_string_from_pointer()
        self.fwversion.os_hash_string = self.unpack_string_from_pointer()

    def process(self, filename) -> FWVersion:
        # We need the file open for ELFFile
        file = open(filename, "rb")
        data = file.read()
        self.elffile = ELFFile(file)

        if not data:
            raise RuntimeError("Failed to find FWVersion.")

        # Detect endianness
        for order in [">", "<"]:
            position = Decoder.locate_header(data, order)
            if position != -1:
                self.byteorder = order
                self.bytesio = io.BytesIO(data)
                self.bytesio.seek(position)
                break
        else:
            raise RuntimeError("Failed to find FWVersion.")

        # Unpack struct and print it
        self.unpack_fwversion()
        return self.fwversion


if __name__ == "__main__":
    assert (
        sys.version_info.major >= 3 and sys.version_info.minor >= 7
    ), "Python version should be at least 3.7"

    # Parse arguments
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "-f",
        dest="file",
        required=True,
        help="File that contains a valid ardupilot firmware in ELF format.",
    )
    parser.add_argument(
        "--expected-hash",
        dest="expected_hash",
        help="Expected git hash. The script fails if this doesn't match the git hash in the binary file. Used in CI",
    )
    args = parser.parse_args()

    decoder = Decoder()
    try:
        firmware_data = decoder.process(args.file)
    except Exception as e:
        print(f"Error decoding FWVersion: {type(e)}")
        exit(-1)

    print(firmware_data)
    if args.expected_hash and args.expected_hash != firmware_data.firmware_hash_string:
        print(f"Git hashes don't match! expected: {args.expected_hash}, got {firmware_data.firmware_hash_string}")
        exit(-1)

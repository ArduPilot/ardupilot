#!/usr/bin/env python3

"""
Decode a device ID such as used for COMPASS_DEV_ID, INS_ACC_ID etc

Bus type and devtype names are parsed live from the C++ enums in the
sensor backend headers (such as libraries/AP_Compass/AP_Compass_Backend.h)
using Tools/autotest/logger_metadata/enum_parse.py.  --dump-json and
--dump-json5 write those tables out for use by tools which do not have an
ArduPilot source tree; --json reads such a file back instead of parsing
the headers.  A copy of this script outside the ArduPilot source tree reads
devid.json from the directory it is in.

AP_FLAKE8_CLEAN

SPDX-FileCopyrightText: 2017-2026 ArduPilot developers

SPDX-License-Identifier: GPL-3.0-or-later

"""

import argparse
import functools
import hashlib
import importlib
import json
import os
import sys

from typing import Any
from typing import Dict
from typing import List
from typing import Literal
from typing import NamedTuple
from typing import Optional
from typing import TextIO
from typing import Tuple
from typing import TypedDict
from typing import get_args

__all__ = [  # noqa: F822 (the *_TYPES tables are provided by __getattr__)
    "AIRSPEED_TYPES",
    "BARO_TYPES",
    "BUSTYPES",
    "COMPASS_TYPES",
    "DEVICE_CATEGORIES",
    "DEVID_JSON_FORMAT_VERSION",
    "DeviceCategory",
    "DeviceInfo",
    "IMU_TYPES",
    "MAVLINK_TYPES",
    "decode_device_id",
    "devid_data_version",
    "dump_devid_json",
    "dump_devid_json5",
    "format_device_info",
    "get_bus_type_name",
    "get_device_type_description",
    "get_device_type_name",
    "get_devid_tables",
    "load_devid_json",
    "parse_device_id",
    "parse_devid_sources",
    "use_devid_json",
]

# realpath so that running through a symlink still finds the source tree
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.realpath(os.path.join(SCRIPT_DIR, "..", ".."))
ENUM_PARSE_DIR = os.path.join(REPO_ROOT, "Tools", "autotest", "logger_metadata")

# used when this script has been copied out of the ArduPilot source tree
DEVID_JSON_FALLBACK_PATH = os.path.join(SCRIPT_DIR, "devid.json")

# version of the structure of the files written by --dump-json/--dump-json5
DEVID_JSON_FORMAT_VERSION = 1

DeviceCategory = Literal["compass", "imu", "baro", "airspeed", "mavlink"]
DEVICE_CATEGORIES: Tuple[DeviceCategory, ...] = get_args(DeviceCategory)


class EnumSource(NamedTuple):
    header: str  # relative to REPO_ROOT
    enum_name: str  # fully-qualified name as reported by enum_parse
    strip_prefix: str = ""
    add_prefix: str = ""


BUS_TYPE_SOURCE = EnumSource("libraries/AP_HAL/Device.h", "AP_HAL::Device::BusType", strip_prefix="BUS_TYPE_")

DEVICE_TYPE_SOURCES: Dict[str, EnumSource] = {
    "compass": EnumSource("libraries/AP_Compass/AP_Compass_Backend.h", "AP_Compass_Backend::DevTypes"),
    "imu": EnumSource("libraries/AP_InertialSensor/AP_InertialSensor_Backend.h", "AP_InertialSensor_Backend::DevTypes"),
    "baro": EnumSource("libraries/AP_Baro/AP_Baro_Backend.h", "AP_Baro_Backend::DevTypes"),
    "airspeed": EnumSource(
        "libraries/AP_Airspeed/AP_Airspeed_Backend.h", "AP_Airspeed_Backend::DevType", add_prefix="DEVTYPE_AIRSPEED_"
    ),
    "mavlink": EnumSource(
        "libraries/AP_SerialManager/AP_SerialManager.h",
        "AP_SerialManager::DeviceType",
        add_prefix="DEVTYPE_MAVLINK_",
    ),
}

# Names we present differently from the C++ enumerator (after prefix
# handling).  Keys are "bus" or a device category.  Every rename must
# match an enumerator, so stale entries are caught.
RENAMES: Dict[str, Dict[str, str]] = {
    "bus": {"UAVCAN": "DRONECAN"},
    "compass": {
        # the driver supports several similarly-named AK0991x compasses
        "DEVTYPE_AK09916": "DEVTYPE_AK0991x",
    },
    "baro": {"DEVTYPE_BARO_UAVCAN": "DEVTYPE_BARO_DRONECAN"},
    "airspeed": {"DEVTYPE_AIRSPEED_UAVCAN": "DEVTYPE_AIRSPEED_DRONECAN"},
    "mavlink": {"DEVTYPE_MAVLINK_CANBUS": "DEVTYPE_MAVLINK_CAN"},
}


class DevTypeEntry(TypedDict, total=False):
    value: int
    name: str
    description: str


# Entries which are no longer in any C++ enum but may still be found in
# parameters and logs.  Values must not collide with the enums.
EXTRA_ENTRIES: Dict[str, List[DevTypeEntry]] = {
    "compass": [
        {
            "value": 0x19,
            "name": "DEVTYPE_LIS2MDL",
            "description": "retired; same sensor as IIS2MDC, only used on pre-release firmware",
        },
    ],
}


class DevIdTables(TypedDict):
    bus_types: List[DevTypeEntry]
    device_types: Dict[str, List[DevTypeEntry]]


def _entries_from_enum(enum_parse: Any, source: EnumSource, table: str) -> List[DevTypeEntry]:
    """Extract the entries of one C++ enum, applying prefix changes and RENAMES for table."""
    path = os.path.join(REPO_ROOT, source.header)
    enums = [e for e in enum_parse.EnumDocco(None).enumerations_from_file(path) if e.name == source.enum_name]
    if len(enums) != 1 or len(enums[0].entries) == 0:
        # enum_parse silently skips enums containing entries it cannot evaluate
        raise ValueError(f"{source.header}: could not parse enum {source.enum_name}")

    renames = RENAMES.get(table, {})
    unused_renames = set(renames)
    entries: List[DevTypeEntry] = []
    for enum_entry in enums[0].entries:
        name = enum_entry.name
        if source.strip_prefix:
            if not name.startswith(source.strip_prefix):
                raise ValueError(f"{source.header}: {name} lacks prefix {source.strip_prefix}")
            name = name[len(source.strip_prefix):]
        name = source.add_prefix + name
        if name in renames:
            unused_renames.discard(name)
            name = renames[name]
        entry: DevTypeEntry = {"value": enum_entry.value, "name": name}
        if enum_entry.comment:
            entry["description"] = enum_entry.comment.strip()
        entries.append(entry)
    if unused_renames:
        raise ValueError(f"{source.header}: renames for unknown enumerators {sorted(unused_renames)}")
    return entries


def _validate_entries(entries: List[DevTypeEntry], table_name: str, max_value: int) -> None:
    seen = set()
    for entry in entries:
        value = entry.get("value")
        if not isinstance(value, int) or isinstance(value, bool) or not isinstance(entry.get("name"), str):
            raise ValueError(f"{table_name}: malformed entry {entry!r}")
        if not 0 <= value <= max_value:
            raise ValueError(f"{table_name}: value {value} out of range 0..{max_value}")
        if value in seen:
            raise ValueError(f"{table_name}: duplicate value {value}")
        seen.add(value)


def parse_devid_sources() -> DevIdTables:
    """Build the device ID tables by parsing the C++ headers in this source tree."""
    sys.path.insert(0, ENUM_PARSE_DIR)
    try:
        enum_parse = importlib.import_module("enum_parse")
    finally:
        sys.path.pop(0)

    bus_types = _entries_from_enum(enum_parse, BUS_TYPE_SOURCE, "bus")
    _validate_entries(bus_types, "bus_types", 0x07)

    device_types: Dict[str, List[DevTypeEntry]] = {}
    for category in DEVICE_CATEGORIES:
        entries = _entries_from_enum(enum_parse, DEVICE_TYPE_SOURCES[category], category)
        entries += EXTRA_ENTRIES.get(category, [])
        _validate_entries(entries, f"device_types.{category}", 0xFF)
        device_types[category] = sorted(entries, key=lambda e: e["value"])

    return {"bus_types": sorted(bus_types, key=lambda e: e["value"]), "device_types": device_types}


def load_devid_json(path: str) -> DevIdTables:
    """Load tables from a file written by --dump-json, checking its format_version."""
    with open(path, encoding="utf-8") as f:
        raw = json.load(f)

    if not isinstance(raw, dict):
        raise ValueError(f"{path}: expected a JSON object")

    format_version = raw.get("format_version")
    if format_version != DEVID_JSON_FORMAT_VERSION:
        raise ValueError(
            f"{path}: unsupported format_version {format_version!r} (expected {DEVID_JSON_FORMAT_VERSION})"
        )
    if not isinstance(raw.get("data_version"), str):
        raise ValueError(f"{path}: missing or invalid data_version")

    bus_types = raw.get("bus_types")
    device_types = raw.get("device_types")
    if not isinstance(bus_types, list) or not isinstance(device_types, dict):
        raise ValueError(f"{path}: missing bus_types or device_types")
    _validate_entries(bus_types, "bus_types", 0x07)
    for category in DEVICE_CATEGORIES:
        if not isinstance(device_types.get(category), list):
            raise ValueError(f"{path}: device_types missing category {category}")
        _validate_entries(device_types[category], f"device_types.{category}", 0xFF)
    return {"bus_types": bus_types, "device_types": device_types}


def devid_data_version(tables: DevIdTables) -> str:
    """Content hash of the tables; changes whenever any entry changes."""
    canonical = json.dumps(tables, sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _write_devid_file(tables: DevIdTables, f: TextIO, json5: bool) -> None:
    """
    Write tables as JSON or JSON5, one entry per line.

    Both formats carry identical data; JSON5 uses hexadecimal values and
    unquoted keys.
    """
    def key(k: str) -> str:
        return k if json5 else json.dumps(k)

    def entry_line(entry: DevTypeEntry) -> str:
        value = f"0x{entry['value']:02X}" if json5 else str(entry["value"])
        fields = [f"{key('value')}: {value}", f"{key('name')}: {json.dumps(entry['name'])}"]
        if "description" in entry:
            fields.append(f"{key('description')}: {json.dumps(entry['description'])}")
        return "{" + ", ".join(fields) + "}"

    def table(entries: List[DevTypeEntry], indent: str) -> str:
        rows = [indent + "    " + entry_line(e) for e in entries]
        return "[\n" + ",\n".join(rows) + "\n" + indent + "]"

    if json5:
        f.write("// ArduPilot device ID bus types and device types.\n")
        f.write("// Generated by Tools/scripts/decode_devid.py --dump-json5 from the C++ headers; do not edit.\n")
    f.write("{\n")
    f.write(f"    {key('format_version')}: {DEVID_JSON_FORMAT_VERSION},\n")
    f.write(f"    {key('data_version')}: {json.dumps(devid_data_version(tables))},\n")
    f.write(f"    {key('bus_types')}: {table(tables['bus_types'], '    ')},\n")
    f.write(f"    {key('device_types')}: {{\n")
    categories = [
        f"        {key(category)}: {table(entries, '        ')}"
        for category, entries in tables["device_types"].items()
    ]
    f.write(",\n".join(categories) + "\n")
    f.write("    }\n")
    f.write("}\n")


def dump_devid_json(tables: DevIdTables, path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        _write_devid_file(tables, f, json5=False)


def dump_devid_json5(tables: DevIdTables, path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        _write_devid_file(tables, f, json5=True)


@functools.lru_cache(maxsize=None)
def get_devid_tables(json_path: Optional[str] = None) -> DevIdTables:
    """
    Tables from json_path if given, otherwise parsed from the C++ headers.

    If this script is not in an ArduPilot source tree, DEVID_JSON_FALLBACK_PATH
    (devid.json beside the script) is read instead.
    """
    if json_path is not None:
        return load_devid_json(json_path)
    in_tree = os.path.exists(os.path.join(REPO_ROOT, BUS_TYPE_SOURCE.header)) and os.path.isdir(ENUM_PARSE_DIR)
    if in_tree:
        return parse_devid_sources()
    if os.path.exists(DEVID_JSON_FALLBACK_PATH):
        return load_devid_json(DEVID_JSON_FALLBACK_PATH)
    raise FileNotFoundError(
        f"ArduPilot C++ headers not found under {REPO_ROOT} and no {DEVID_JSON_FALLBACK_PATH}; "
        "use --json with a file written by --dump-json"
    )


def _lookup(entries: List[DevTypeEntry]) -> Dict[int, str]:
    return {e["value"]: e["name"] for e in entries}


_json_path: Optional[str] = None


def use_devid_json(path: Optional[str]) -> None:
    """Make lookups read path (from --dump-json) rather than parsing the C++ headers."""
    global _json_path
    _json_path = path


def get_bus_type_name(bus_type: int) -> str:
    return _lookup(get_devid_tables(_json_path)["bus_types"]).get(bus_type, "UNKNOWN")


_LEGACY_TABLES = {
    "COMPASS_TYPES": "compass",
    "IMU_TYPES": "imu",
    "BARO_TYPES": "baro",
    "AIRSPEED_TYPES": "airspeed",
    "MAVLINK_TYPES": "mavlink",
}


def __getattr__(name: str) -> Dict[int, str]:
    """Lazily provide the lookup dicts this module used to define statically."""
    # check the name before loading: the import machinery probes for
    # attributes such as __path__ and must get AttributeError
    if name == "BUSTYPES":
        return _lookup(get_devid_tables(_json_path)["bus_types"])
    if name in _LEGACY_TABLES:
        return _lookup(get_devid_tables(_json_path)["device_types"][_LEGACY_TABLES[name]])
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


class DeviceInfo(TypedDict):
    bus_type_value: int
    bus_type_name: str
    bus: int
    address: int
    devtype: int
    is_dronecan: bool


def parse_device_id(device_id_str: str) -> Tuple[Optional[int], Optional[str]]:
    """Parse a device ID and return a diagnostic message, if applicable."""
    # backwards compatibility:
    #  - assume all digit strings are decimal,
    #  - assume strings without "0x" prefix but with characters A-F are hexadecimal
    try:
        device_id = int(device_id_str)
    except ValueError:
        try:
            device_id = int(device_id_str, 16)
        except ValueError:
            return None, f"Invalid device ID '{device_id_str}'"

    if device_id < 0:
        return None, "device ID must not be negative"

    # Warn that ambiguous digits-only input is interpreted as decimal.
    if device_id_str.isdigit():
        return (
            device_id,
            f"Warning: device ID '{device_id_str}' is interpreted as decimal; prefix hexadecimal IDs with 0x.",
        )

    return device_id, None


def decode_device_id(device_id: int) -> Tuple[Optional[DeviceInfo], Optional[str]]:
    """
    Decode an ArduPilot device ID into its components.

    Args:
        device_id: Device ID as integer

    Returns:
        A tuple containing the decoded dictionary and an error message. If
        device_id does not fit in the 24-bit device ID format, the decoded
        dictionary is None. Otherwise, it contains these keys:
            - bus_type_value: Bus type numeric value
            - bus_type_name: Bus type name (I2C, SPI, etc.)
            - bus: Bus number
            - address: Device address
            - devtype: Device type numeric value
            - is_dronecan: Whether this is DRONECAN bus type

    """
    if device_id < 0 or device_id > 0x00FFFFFF:
        return None, f"Error: device ID 0x{device_id:x} is outside the 24-bit range."

    bus_type = device_id & 0x07
    bus = (device_id >> 3) & 0x1F
    address = (device_id >> 8) & 0xFF
    devtype = (device_id >> 16) & 0xFF

    decoded: DeviceInfo = {
        "bus_type_value": bus_type,
        "bus_type_name": get_bus_type_name(bus_type),
        "bus": bus,
        "address": address,
        "devtype": devtype,
        "is_dronecan": bus_type == 3,
    }
    return decoded, None


def _device_type_entry(devtype: int, device_category: DeviceCategory) -> Optional[DevTypeEntry]:
    try:
        entries = get_devid_tables(_json_path)["device_types"][device_category.lower()]
    except KeyError:
        raise ValueError(f"Unknown device category: {device_category}") from None
    return next((e for e in entries if e["value"] == devtype), None)


def get_device_type_description(devtype: int, device_category: DeviceCategory) -> Optional[str]:
    """Return the device type's comment from the C++ enum (or EXTRA_ENTRIES), if any."""
    entry = _device_type_entry(devtype, device_category)
    return None if entry is None else entry.get("description")


def get_device_type_name(devtype: int, device_category: DeviceCategory) -> str:
    """
    Look up device type name based on device category.

    Args:
        devtype: Device type numeric value
        device_category: One of 'compass', 'imu', 'baro', 'airspeed', 'mavlink'

    Returns:
        Device type name string, or 'UNKNOWN' if not found

    """
    entry = _device_type_entry(devtype, device_category)
    return "UNKNOWN" if entry is None else entry["name"]


def format_device_info(decode_info: DeviceInfo, device_type_name: str = "") -> str:
    """
    Format decoded device info as a readable string.

    Args:
        decode_info: Dictionary returned by decode_device_id()
        device_type_name: Device type name (optional; not applicable to DroneCAN IDs)

    Returns:
        Formatted string with device information

    """
    bus_type = decode_info["bus_type_value"]
    bus_type_name = decode_info["bus_type_name"]
    bus = decode_info["bus"]
    address = decode_info["address"]
    devtype = decode_info["devtype"]
    is_dronecan = decode_info["is_dronecan"]

    if is_dronecan:
        # Compass DroneCAN IDs store sensor_id + 1; other DroneCAN IDs use 0.
        sensor_id = devtype - 1 if devtype > 0 else None
        sensor_id_info = (
            f"sensor_id:{sensor_id}(0x{sensor_id:x})"
            if sensor_id is not None
            else "sensor_id:not encoded"
        )
        return (
            f"bus_type:{bus_type_name}({bus_type})  bus:{bus} "
            f"address:{address}(0x{address:x}) {sensor_id_info}"
        )
    device_type_suffix = f" {device_type_name}" if device_type_name else ""
    return (
        f"bus_type:{bus_type_name}({bus_type})  bus:{bus} "
        f"address:{address}(0x{address:x}) devtype:{devtype}(0x{devtype:x})"
        f"{device_type_suffix}"
    )


def main() -> None:
    """Command-line interface."""
    parser = argparse.ArgumentParser(description="DEVICE_ID must be decimal or hexadecimal with a 0x prefix")
    category_group = parser.add_mutually_exclusive_group()
    category_group.add_argument("-C", "--compass", action="store_true", help="decode compass IDs")
    category_group.add_argument("-I", "--imu", action="store_true", help="decode IMU IDs")
    category_group.add_argument("-B", "--baro", action="store_true", help="decode barometer IDs")
    category_group.add_argument("-A", "--airspeed", action="store_true", help="decode airspeed IDs")
    category_group.add_argument("-M", "--mavlink", action="store_true", help="decode MAVLink channel IDs")
    parser.add_argument("--json", metavar="FILE", help="read tables from a file written by --dump-json")
    parser.add_argument("--dump-json", metavar="FILE", help="write the device ID tables as JSON")
    parser.add_argument("--dump-json5", metavar="FILE", help="write the device ID tables as JSON5")
    parser.add_argument("device_id", nargs="?", help="decimal or hexadecimal device ID")

    opts = parser.parse_args()

    dumping = opts.dump_json is not None or opts.dump_json5 is not None
    if opts.device_id is None and not dumping:
        parser.error("a device ID is required unless --dump-json or --dump-json5 is given")

    use_devid_json(opts.json)
    try:
        tables = get_devid_tables(opts.json)
    except (OSError, ValueError) as ex:
        print(f"Error: {ex}", file=sys.stderr)
        sys.exit(1)

    try:
        if opts.dump_json is not None:
            dump_devid_json(tables, opts.dump_json)
        if opts.dump_json5 is not None:
            dump_devid_json5(tables, opts.dump_json5)
    except OSError as ex:
        print(f"Error: {ex}", file=sys.stderr)
        sys.exit(1)
    if opts.device_id is None:
        return

    device_id, err_msg = parse_device_id(opts.device_id)
    if err_msg is not None:
        print(err_msg, file=sys.stderr)
    if device_id is None:
        sys.exit(1)

    decoded, err_msg = decode_device_id(device_id)
    if err_msg is not None:
        print(err_msg, file=sys.stderr)
        sys.exit(1)

    if decoded is None:
        sys.exit(1)

    device_type_name = ""
    if not decoded["is_dronecan"]:
        for category in DEVICE_CATEGORIES:
            if getattr(opts, category):
                device_type_name = get_device_type_name(decoded["devtype"], category)
                # enum comments flag IDs needing care, e.g. retired or mistaken IDs
                description = get_device_type_description(decoded["devtype"], category)
                if description is not None:
                    print(
                        f"Warning: devtype 0x{decoded['devtype']:x} ({device_type_name}): {description}",
                        file=sys.stderr,
                    )

    print(format_device_info(decoded, device_type_name))


if __name__ == "__main__":
    main()

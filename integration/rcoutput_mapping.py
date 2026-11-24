"""Validate native RCOutput observations and map motor channels to ESP32 IDs.

This module has no transport or actuator side effects. The geometry and wiring
remain provisional until measured on the vehicle.
"""

import json

from check_layout import CONFIG, MOTOR_DEFAULTS, MOTORS, check_motor_defaults, mapping, upstream_factors


def load_motor_order(config_path=CONFIG, allow_enabled=False):
    config = json.loads(config_path.read_text())
    resolved = mapping(config, upstream_factors(MOTORS.read_text()),
                       require_disabled=not allow_enabled)
    check_motor_defaults(resolved, config["actuators"], MOTOR_DEFAULTS.read_text())
    return tuple(resolved[number]["esp32_id"] - 1 for number in range(1, 9))


def parse_observation(packet):
    if len(packet) > 128:
        raise ValueError("RCOutput observation too long")
    try:
        fields = packet.decode("ascii").split(",")
        if len(fields) != 19 or fields[:2] != ["ATAL_RCOUT", "1"] or fields[2] not in ("0", "1"):
            raise ValueError("Invalid RCOutput observation header")
        if any(not field.isdecimal() for field in fields[3:]):
            raise ValueError("Invalid RCOutput channel")
        channels = tuple(int(field) for field in fields[3:])
    except UnicodeDecodeError as exc:
        raise ValueError("Non-ASCII RCOutput observation") from exc
    if any(value != 0 and not 900 <= value <= 2100 for value in channels):
        raise ValueError("RCOutput channel outside supported range")
    armed = fields[2] == "1"
    if armed and any(not 1300 <= value <= 1700 for value in channels[:8]):
        raise ValueError("Armed motor channel outside provisional range")
    return armed, channels


def motors_in_esp_order(channels, order):
    if len(channels) < 8 or sorted(order) != list(range(8)):
        raise ValueError("Expected eight motor channels and a complete ESP32 permutation")
    output = [0] * 8
    for ap_index, esp_index in enumerate(order):
        output[esp_index] = channels[ap_index]
    return tuple(output)

#!/usr/bin/env python3
"""Validate provisional reference layout against the pinned ArduPilot 6DOF frame.

No physical output is opened. Source factors are read from AP_Motors6DOF.cpp,
so upstream changes cannot silently invalidate the mapping.
"""

import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = Path(__file__).with_name("vehicle_defaults.json")
MOTORS = ROOT / "libraries/AP_Motors/AP_Motors6DOF.cpp"
MOTOR_DEFAULTS = Path(__file__).with_name("atal_motor_defaults.parm")


def upstream_factors(source):
    section = source.split("case SUB_FRAME_VECTORED_6DOF:", 1)[1].split("break;", 1)[0]
    rows = {}
    for match in re.finditer(r"add_motor_raw_6dof\(AP_MOTORS_MOT_(\d+),\s*([^;]+)\);", section):
        number = int(match.group(1))
        values = [float(value.strip().rstrip("f")) for value in match.group(2).split(",")]
        if len(values) != 7 or int(values[6]) != number:
            raise ValueError("Unexpected upstream motor factor line")
        rows[number] = values[:6]
    if set(rows) != set(range(1, 9)):
        raise ValueError("Expected eight upstream Vectored-6DOF motors")
    return rows


def rank(rows, tolerance=1e-9):
    matrix = [[float(value) for value in row] for row in rows]
    pivots = 0
    for column in range(len(matrix[0])):
        candidate = max(range(pivots, len(matrix)), key=lambda i: abs(matrix[i][column]))
        if abs(matrix[candidate][column]) < tolerance:
            continue
        matrix[pivots], matrix[candidate] = matrix[candidate], matrix[pivots]
        scale = matrix[pivots][column]
        matrix[pivots] = [value / scale for value in matrix[pivots]]
        for row in range(pivots + 1, len(matrix)):
            scale = matrix[row][column]
            matrix[row] = [a - scale * b for a, b in zip(matrix[row], matrix[pivots])]
        pivots += 1
        if pivots == len(matrix):
            break
    return pivots


def mapping(config, upstream, require_disabled=True):
    actuators = config["actuators"]
    if require_disabled and actuators["enabled"] is not False:
        raise ValueError("Development configuration must keep actuators disabled")
    if type(actuators["enabled"]) is not bool:
        raise ValueError("actuators.enabled must be a boolean")
    motors = actuators["motors"]
    pins = actuators["firmware_esp32_id_to_gpio"]
    if len(motors) != 8 or len(pins) != 8 or len(set(pins)) != 8:
        raise ValueError("Expected eight motors and eight distinct ESP32 pins")
    if {motor["logical_id"] for motor in motors} != set(range(1, 9)):
        raise ValueError("Logical motor IDs must be 1..8")
    if {motor["esp32_gpio"] for motor in motors} != set(pins):
        raise ValueError("Each motor must map to one firmware GPIO")
    effective = {}
    for motor in motors:
        factors = motor["factors_rpyvfl"]
        direction = motor["direction"]
        if len(factors) != 6 or direction not in (-1, 1) or not all(math.isfinite(x) for x in factors):
            raise ValueError("Invalid factor row or direction")
        effective[motor["logical_id"]] = [direction * factor for factor in factors]
    if rank(list(effective.values())) != 6:
        raise ValueError("Allocation matrix does not span six axes")
    resolved = {}
    used = set()
    for number, row in upstream.items():
        choices = [(motor, sign) for motor in motors for sign in (-1, 1)
                   if all(abs(sign * axis - ref) < 1e-9 for axis, ref in
                          zip(row, effective[motor["logical_id"]]))]
        if len(choices) != 1 or choices[0][0]["logical_id"] in used:
            raise ValueError(f"No unique reference mapping for upstream motor {number}")
        motor, sign = choices[0]
        used.add(motor["logical_id"])
        resolved[number] = {"logical_id": motor["logical_id"],
                            "esp32_id": pins.index(motor["esp32_gpio"]) + 1,
                            "MOT_DIRECTION": sign}
    return resolved


def check_motor_defaults(resolved, actuators, source):
    """Catch drift between the editable geometry and provisional AP parameters."""
    params = {}
    for line in source.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        name, value = line.split()
        if name in params:
            raise ValueError(f"Duplicate motor parameter {name}")
        params[name] = value
    if params.get("FRAME_CONFIG") != "2":
        raise ValueError("Motor defaults must select Vectored-6DOF")
    pwm_min = actuators["pwm_min_us"]
    pwm_neutral = actuators["pwm_neutral_us"]
    pwm_max = actuators["pwm_max_us"]
    if not (1000 <= pwm_min < pwm_neutral == 1500 < pwm_max <= 2000):
        raise ValueError("Invalid provisional PWM range")
    if (params.get("MOT_PWM_MIN") != str(pwm_min) or
            params.get("MOT_PWM_MAX") != str(pwm_max)):
        raise ValueError("Motor PWM defaults do not match vehicle_defaults.json")
    for number, item in resolved.items():
        name = f"MOT_{number}_DIRECTION"
        if params.get(name) != str(item["MOT_DIRECTION"]):
            raise ValueError(f"{name} does not match vehicle_defaults.json")
        function = f"SERVO{number}_FUNCTION"
        if params.get(function) != str(32 + number):
            raise ValueError(f"{function} must assign ArduSub motor {number}")


def main():
    config = json.loads(CONFIG.read_text())
    result = mapping(config, upstream_factors(MOTORS.read_text()))
    check_motor_defaults(result, config["actuators"], MOTOR_DEFAULTS.read_text())
    for number, item in result.items():
        print(f"AP motor {number}: logical M{item['logical_id']} -> ESP32 ID {item['esp32_id']}; "
              f"MOT_{number}_DIRECTION={item['MOT_DIRECTION']}")
    print("Allocation rank: 6; actuator interface: disabled")


if __name__ == "__main__":
    main()

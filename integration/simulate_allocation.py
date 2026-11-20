#!/usr/bin/env python3
"""Compare source-derived ROS and ArduSub mixes without opening any output device.

Assumptions: all eight motors enabled, unit weights, no battery/current limit,
full spool, zero feed-forward, and provisional ROS mixer defaults. These small
models cover allocation and PWM shaping; they do not execute either controller.
"""

import json
import math

from check_layout import CONFIG, MOTORS, MOTOR_DEFAULTS, check_motor_defaults, mapping, upstream_factors


def clamp(value):
    return max(-1.0, min(1.0, value))


def ros_group(motors, commands, attitude_axes, linear_axes):
    """Reference priority/headroom logic for one group of four motors."""
    attitude = [sum(commands[axis] * motor["factors_rpyvfl"][axis]
                    for axis in attitude_axes) for motor in motors]
    linear = [sum(commands[axis] * motor["factors_rpyvfl"][axis]
                  for axis in linear_axes) for motor in motors]
    att_max = max(abs(value) for value in attitude)
    lin_max = max(abs(value) for value in linear)
    if max(abs(a + b) for a, b in zip(attitude, linear)) > 1.0:
        if lin_max > 0.01:
            if att_max > 0.65:
                attitude = [value * 0.65 / att_max for value in attitude]
            scale = min(1.0, *(max(0.0, 1.0 - abs(a)) / abs(b)
                               for a, b in zip(attitude, linear) if abs(b) > 1e-6))
            linear = [value * scale for value in linear]
        elif att_max > 1.0:
            attitude = [value / att_max for value in attitude]
    return {motor["logical_id"]: motor["direction"] * clamp(a + b)
            for motor, a, b in zip(motors, attitude, linear)}


def ros_mix(config, commands):
    motors = config["actuators"]["motors"]
    vertical = [motor for motor in motors if motor["factors_rpyvfl"][3]]
    horizontal = [motor for motor in motors if not motor["factors_rpyvfl"][3]]
    return (ros_group(vertical, commands, (0, 1), (3,)) |
            ros_group(horizontal, commands, (2,), (4, 5)))


def ap_mix(upstream, resolved, commands):
    """Vectored-6DOF normalization with output limit fixed at one."""
    rpt = {n: sum(commands[axis] * row[axis] for axis in (0, 1, 3))
           for n, row in upstream.items()}
    yfl = {n: sum(commands[axis] * row[axis] for axis in (2, 4, 5))
           for n, row in upstream.items()}
    rpt_max = max(1.0, *(abs(value) for value in rpt.values()))
    yfl_max = max(1.0, *(abs(value) for value in yfl.values()))
    return {resolved[n]["logical_id"]: clamp(resolved[n]["MOT_DIRECTION"] *
                                           (rpt[n] / rpt_max + yfl[n] / yfl_max))
            for n in upstream}


def ros_pwm(thrust, actuators):
    if abs(thrust) < 0.001:
        return actuators["pwm_neutral_us"]
    magnitude = min(1.0, abs(thrust) / (0.8 if thrust < 0 else 1.0))
    linearized = (-0.4 + math.sqrt(0.16 + 2.4 * magnitude)) / 1.2
    neutral = actuators["pwm_neutral_us"]
    span = (actuators["pwm_max_us"] - neutral if thrust > 0 else
            neutral - actuators["pwm_min_us"])
    return neutral + math.copysign(30.0 + linearized * max(0.0, span - 30.0), thrust)


def ap_pwm(thrust, actuators):
    neutral = actuators["pwm_neutral_us"]
    span = (actuators["pwm_max_us"] - neutral if thrust > 0 else
            neutral - actuators["pwm_min_us"])
    return neutral + thrust * span


def main():
    config = json.loads(CONFIG.read_text())
    upstream = upstream_factors(MOTORS.read_text())
    resolved = mapping(config, upstream)
    check_motor_defaults(resolved, config["actuators"], MOTOR_DEFAULTS.read_text())
    cases = {f"{axis}{sign:+d}": tuple(sign * 0.5 if i == axis else 0.0
                                        for i in range(6))
             for axis in range(6) for sign in (-1, 1)}
    cases.update({"mixed_unsaturated": (0.1, -0.1, 0.1, 0.1, 0.1, -0.1),
                  "roll_plus_heave": (1, 0, 0, 1, 0, 0),
                  "yaw_plus_surge": (0, 0, 1, 0, 1, 0)})
    for name, commands in cases.items():
        ros = ros_mix(config, commands)
        ap = ap_mix(upstream, resolved, commands)
        delta = max(abs(ros[n] - ap[n]) for n in range(1, 9))
        if name[0].isdigit() or name == "mixed_unsaturated":
            assert delta < 1e-9, (name, delta)
        else:
            assert delta > 0.1, (name, delta)
        print(f"{name}: max thrust delta={delta:.3f}")
    actuators = config["actuators"]
    for thrust in (-0.5, 0.5):
        print(f"PWM at thrust {thrust:+.1f}: ROS={ros_pwm(thrust, actuators):.1f}, "
              f"ArduSub={ap_pwm(thrust, actuators):.1f} us")
    print("Source-derived simulation passed; no actuator interface opened")


if __name__ == "__main__":
    main()

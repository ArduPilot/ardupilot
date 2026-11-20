# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

@tool
@icon("res://icons/heli.svg")
class_name ArduPilotHeliRotor
extends "res://scripts/ap_component.gd" # main rotor, H3-120 swash mixing

@export var swash_servo_1: int = 1 ## H3-120 swash servo (1-based)
@export var swash_servo_2: int = 2
@export var swash_servo_3: int = 3
@export var rsc_servo: int = 8 ## rotor speed control servo
@export var pwm_min: float = 1000.0
@export var pwm_max: float = 2000.0
@export var collective_min: float = -25.0 ## blade pitch at pwm_min (deg)
@export var collective_max: float = 25.0 ## blade pitch at pwm_max (deg)
@export var cyclic_scale: float = 10.0 ## blade pitch, full cyclic (deg)
@export var nominal_rpm: float = 1500.0 ## full throttle rotor speed (rpm)
@export var spool_time: float = 5.0 ## stopped to nominal rpm (s)
@export var thrust_scale: float = 9.0: ## thrust per collective degree (N/deg)
    set(v):
        thrust_scale = v
        update_gizmos()
@export var torque_ratio: float = 0.035 ## reaction torque per thrust (N.m/N)
@export var hub_stiffness: float = 15.0 ## hub moment per tilt (N.m/rad)
@export var flap_time_constant: float = 0.04 ## tip path plane lag (s)
@export_enum("ccw", "cw") var direction: String = "cw": ## spin seen from above (+Y)
    set(v):
        direction = v
        update_gizmos()
@export var diameter: float = 1.4: ## rotor diameter (m)
    set(v):
        diameter = v
        update_gizmos()

var rpm: float = 0.0
var thrust: float = 0.0 # (N)
var tpp: Vector2 = Vector2.ZERO # (rad): x right, y aft

func gltf_key() -> String:
    return "heliRotor"

func gltf_fields() -> Dictionary:
    return {
        "swashServo1": "swash_servo_1", "swashServo2": "swash_servo_2",
        "swashServo3": "swash_servo_3", "rscServo": "rsc_servo",
        "pwmMin": "pwm_min", "pwmMax": "pwm_max",
        "collectiveMin": "collective_min", "collectiveMax": "collective_max",
        "cyclicScale": "cyclic_scale", "nominalRpm": "nominal_rpm",
        "spoolTime": "spool_time", "thrustScale": "thrust_scale",
        "torqueRatio": "torque_ratio", "hubStiffness": "hub_stiffness",
        "flapTimeConstant": "flap_time_constant", "direction": "direction",
        "diameter": "diameter",
    }

func physics_update(vehicle: RigidBody3D, pwm: PackedInt32Array, dt: float) -> void:
    var rsc := maxf(servo_fraction(pwm, rsc_servo, pwm_min, pwm_max), 0.0)
    var target := nominal_rpm * clampf(rsc / 0.66, 0.0, 1.0) # full speed at 66% throttle
    var slew := nominal_rpm * dt / maxf(spool_time, 0.01)
    rpm = clampf(target, rpm - slew, rpm + slew)
    var rpm_frac := rpm / nominal_rpm if nominal_rpm > 0.0 else 0.0

    var s1 := servo_fraction(pwm, swash_servo_1, pwm_min, pwm_max)
    var s2 := servo_fraction(pwm, swash_servo_2, pwm_min, pwm_max)
    var s3 := servo_fraction(pwm, swash_servo_3, pwm_min, pwm_max)
    if s1 < 0.0 or s2 < 0.0 or s3 < 0.0 or rpm_frac < 0.05:
        thrust = 0.0
        tpp = Vector2.ZERO
        _spin_visuals(dt)
        return
    var collective := lerpf(collective_min, collective_max, (s1 + s2 + s3) / 3.0)
    var roll_cyclic := deg_to_rad(cyclic_scale) * (s1 - s2)
    var pitch_cyclic := deg_to_rad(cyclic_scale) * 1.15 * ((s1 + s2) * 0.5 - s3)

    var xform := global_transform
    var b := xform.basis.orthonormalized()

    var w_local: Vector3 = b.transposed() * vehicle.angular_velocity
    var tau := maxf(flap_time_constant, 0.005)
    tpp.x += ((roll_cyclic - tpp.x) / tau - w_local.z) * dt # flapping lags the body rate
    tpp.y += ((pitch_cyclic - tpp.y) / tau + w_local.x) * dt
    tpp = tpp.clampf(-0.3, 0.3)

    var axis := b.y
    var v_axial: float = vehicle.air_velocity_at(xform.origin).dot(axis)
    var v_blade := 0.75 * (rpm * TAU / 60.0) * (0.5 * diameter)
    if v_blade > 1.0:
        collective -= rad_to_deg(atan(v_axial / v_blade)) # climb inflow reduces effective pitch

    thrust = thrust_scale * collective * rpm_frac * rpm_frac
    var dir_local := Vector3(-tpp.x, 1.0, -tpp.y).normalized()
    vehicle.apply_force(b * (dir_local * thrust), xform.origin - vehicle.global_position)

    var spin := -1.0 if direction == "ccw" else 1.0
    var reaction := torque_ratio * (absf(thrust) + 1.5 * thrust_scale * rpm_frac * rpm_frac)
    vehicle.apply_torque(b * Vector3(
            -hub_stiffness * tpp.y, spin * reaction, hub_stiffness * tpp.x))
    _spin_visuals(dt)

func _spin_visuals(dt: float) -> void:
    var spin := 1.0 if direction == "ccw" else -1.0
    _spin_children(spin * minf(rpm * TAU / 60.0, 60.0) * dt)

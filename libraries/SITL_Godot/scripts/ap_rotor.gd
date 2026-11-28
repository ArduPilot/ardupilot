# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

@tool
@icon("res://icons/rotor.svg")
class_name ArduPilotRotor
extends "res://scripts/ap_component.gd" # propeller: thrust along local +Y

@export_range(1, 32) var servo: int = 1 ## SERVOn output (1-based)
@export var pwm_min: float = 1000.0
@export var pwm_max: float = 2000.0
@export var max_thrust: float = 10.0: ## static full-throttle thrust (N)
    set(v):
        max_thrust = v
        update_gizmos()
@export_range(0.0, 1.0) var thrust_expo: float = 0.65 ## 0 linear, 1 quadratic (MOT_THST_EXPO)
@export var bidirectional: bool = false ## mid PWM zero, below reverses
@export var torque_ratio: float = 0.016 ## reaction torque per thrust (N.m/N)
@export_enum("ccw", "cw") var direction: String = "ccw": ## spin seen from thrust tip
    set(v):
        direction = v
        update_gizmos()
@export var time_constant: float = 0.03 ## motor spin-up time constant (s)
@export var diameter: float = 0.25: ## propeller diameter (m)
    set(v):
        diameter = v
        update_gizmos()
@export var pitch_speed: float = 0.0 ## zero-thrust inflow (m/s), 0 disables
@export var inplane_drag: float = 0.0 ## edgewise-flow drag coefficient (s/m)

var static_thrust: float = 0.0 # after motor lag (N)
var thrust: float = 0.0 # after inflow effects (N)
var slipstream: float = 0.0 # excess over free stream (m/s)

func gltf_key() -> String:
    return "rotor"

func gltf_fields() -> Dictionary:
    return {
        "servo": "servo", "pwmMin": "pwm_min", "pwmMax": "pwm_max",
        "maxThrust": "max_thrust", "thrustExpo": "thrust_expo",
        "bidirectional": "bidirectional",
        "torqueRatio": "torque_ratio", "direction": "direction",
        "timeConstant": "time_constant", "diameter": "diameter",
        "pitchSpeed": "pitch_speed", "inplaneDrag": "inplane_drag",
    }

func physics_update(vehicle: RigidBody3D, pwm: PackedInt32Array, dt: float) -> void:
    var f := servo_fraction(pwm, servo, pwm_min, pwm_max)
    var t := maxf(f, 0.0)
    var forward := 1.0
    if bidirectional and f >= 0.0:
        forward = signf(2.0 * f - 1.0)
        t = absf(2.0 * f - 1.0)
    var demand := forward * max_thrust * ((1.0 - thrust_expo) * t + thrust_expo * t * t)
    if time_constant > 0.0:
        static_thrust += (demand - static_thrust) * clampf(dt / time_constant, 0.0, 1.0)
    else:
        static_thrust = demand

    var xform := global_transform
    var axis := xform.basis.y.normalized()
    var v_air: Vector3 = vehicle.air_velocity_at(xform.origin)
    var inflow := v_air.dot(axis)
    thrust = static_thrust
    if pitch_speed > 0.0 and max_thrust > 0.0 and static_thrust > 0.0:
        var v_pitch := pitch_speed * sqrt(static_thrust / max_thrust) # pitch speed scales with rpm
        if v_pitch > 0.01:
            thrust *= clampf(1.0 - inflow / v_pitch, 0.0, 1.5)

    var area := PI * 0.25 * diameter * diameter
    if thrust > 0.0 and area > 0.0:
        var v_axial := maxf(inflow, 0.0)
        var rho: float = vehicle.air_density
        var v_induced := -0.5 * v_axial + sqrt(0.25 * v_axial * v_axial + thrust / (2.0 * rho * area))
        slipstream = 2.0 * v_induced # induced velocity doubles far behind
    else:
        slipstream = 0.0

    var force := axis * thrust
    if inplane_drag > 0.0:
        force -= inplane_drag * absf(thrust) * (v_air - axis * inflow)
    var offset := xform.origin - vehicle.global_position
    vehicle.apply_force(force, offset)
    var spin := 1.0 if direction == "ccw" else -1.0 # ccw reaction torque is clockwise
    vehicle.apply_torque(-axis * (spin * torque_ratio * thrust))

    if max_thrust > 0.0:
        _spin_children(spin * 200.0 * sqrt(absf(static_thrust) / max_thrust) * dt)

func reset() -> void:
    static_thrust = 0.0
    thrust = 0.0

func propwash_at(point: Vector3, v_air: Vector3, span_dir := Vector3.ZERO, span := 0.0) -> Vector3: # slipstream velocity added at point
    if slipstream <= 0.0:
        return Vector3.ZERO
    var xform := global_transform
    var axis := xform.basis.y.normalized()
    var rel := point - xform.origin
    var behind := -rel.dot(axis)
    if behind <= 0.0:
        return Vector3.ZERO
    var r := 0.5 * diameter / sqrt(2.0) # slipstream has half disc area
    var offset := rel + axis * behind
    var covered := 0.0
    var u := span_dir - axis * span_dir.dot(axis)
    var a := u.length_squared()
    if span <= 0.0 or a < 1e-6:
        covered = 1.0 if offset.length() <= r else 0.0
    else:
        var b := 2.0 * offset.dot(u)
        var disc := b * b - 4.0 * a * (offset.length_squared() - r * r)
        if disc <= 0.0:
            return Vector3.ZERO
        var s1 := maxf((-b - sqrt(disc)) / (2.0 * a), -0.5 * span)
        var s2 := minf((-b + sqrt(disc)) / (2.0 * a), 0.5 * span)
        covered = maxf(s2 - s1, 0.0) / span # span fraction inside slipstream
    if covered <= 0.0:
        return Vector3.ZERO
    var v := maxf(v_air.dot(axis), 0.0)
    var w := slipstream
    return axis * (sqrt((1.0 - covered) * v * v + covered * (v + w) * (v + w)) - v)

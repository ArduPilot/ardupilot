# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

@tool
extends Node3D # base class for vehicle parts

const G := 9.80665

func gltf_key() -> String: # ARDUPILOT_sitl extension member name
    return ""

func gltf_fields() -> Dictionary: # JSON key -> property name
    return {}

func physics_update(_vehicle: RigidBody3D, _pwm: PackedInt32Array, _dt: float) -> void:
    pass

static func servo_fraction(pwm: PackedInt32Array, servo: int, pwm_min: float, pwm_max: float) -> float: # 0..1, -1 without output
    if servo < 1 or servo > pwm.size() or pwm_max <= pwm_min:
        return -1.0
    var v := float(pwm[servo - 1])
    if v < 500.0: # disabled channel outputs 0
        return -1.0
    return clampf((v - pwm_min) / (pwm_max - pwm_min), 0.0, 1.0)

func _spin_children(angle: float) -> void:
    for child in get_children():
        if child is Node3D:
            child.rotate_object_local(Vector3.UP, angle)

func to_gltf() -> Dictionary:
    var d := {}
    for key in gltf_fields():
        var v = get(gltf_fields()[key])
        d[key] = [v.x, v.y, v.z] if v is Vector3 else v
    return d

func from_gltf(d: Dictionary) -> void:
    var fields := gltf_fields()
    for key in d:
        if not fields.has(key):
            push_warning("ARDUPILOT_sitl.%s: unknown field '%s'" % [gltf_key(), key])
            continue
        var prop: String = fields[key]
        var cur = get(prop)
        var v = d[key]
        if cur is Vector3 and v is Array and v.size() == 3:
            set(prop, Vector3(v[0], v[1], v[2]))
        elif cur is int:
            set(prop, int(v))
        elif cur is float:
            set(prop, float(v))
        else:
            set(prop, v)

# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

@tool
@icon("res://icons/wheel.svg")
class_name ArduPilotWheel
extends "res://scripts/ap_component.gd" # compresses +Y, rolls along +Z

@export var radius: float = 0.05:
    set(v):
        radius = v
        update_gizmos()
@export var travel: float = 0.05: ## suspension travel (m)
    set(v):
        travel = v
        update_gizmos()
@export var spring: float = 500.0 ## spring rate (N/m)
@export var damping: float = 20.0 ## suspension damping (N.s/m)
@export var rolling_resistance: float = 0.02
@export var grip: float = 0.9 ## sideways friction coefficient
@export_range(0, 32) var drive_servo: int = 0 ## 0 none; pwm_min full reverse
@export var drive_pwm_min: float = 1000.0
@export var drive_pwm_max: float = 2000.0
@export var max_drive_force: float = 0.0 ## at full throttle (N)
@export var free_speed: float = 0.0 ## zero-force speed (m/s), 0 constant

var compression: float = 0.0 # (m), 0 off the ground
var _spin: float = 0.0

func reset() -> void: # vehicle was placed somewhere new
    compression = 0.0

func gltf_key() -> String:
    return "wheel"

func gltf_fields() -> Dictionary:
    return {
        "radius": "radius", "travel": "travel", "spring": "spring",
        "damping": "damping", "rollingResistance": "rolling_resistance",
        "grip": "grip", "driveServo": "drive_servo", "drivePwmMin": "drive_pwm_min",
        "drivePwmMax": "drive_pwm_max", "maxDriveForce": "max_drive_force",
        "freeSpeed": "free_speed",
    }

func physics_update(vehicle: RigidBody3D, pwm: PackedInt32Array, dt: float) -> void:
    var xform := global_transform
    var up := xform.basis.y.normalized()
    var top := xform.origin + up * travel
    var length := travel + radius
    var query := PhysicsRayQueryParameters3D.create(top, top - up * length)
    query.exclude = [vehicle.get_rid()]
    var hit := vehicle.get_world_3d().direct_space_state.intersect_ray(query)
    var last := compression
    if hit.is_empty():
        compression = 0.0
        _place_visuals(dt, 0.0)
        return
    compression = length - top.distance_to(hit.position)
    var normal: Vector3 = hit.normal
    var rate := clampf((compression - last) / dt if dt > 0.0 else 0.0, -10.0, 10.0) # clamp: teleport would spike damping
    var load := maxf(spring * compression + damping * rate, 0.0)

    var contact: Vector3 = hit.position
    var v: Vector3 = vehicle.velocity_at(contact)
    var fwd := xform.basis.z
    fwd = (fwd - normal * fwd.dot(normal)).normalized()
    var side := normal.cross(fwd)
    var v_fwd := v.dot(fwd)
    var v_side := v.dot(side)

    var force := normal * load # along normal, not the strut
    force -= side * grip * load * clampf(v_side / 0.2, -1.0, 1.0) # smoothed near zero for stability
    force -= fwd * rolling_resistance * load * clampf(v_fwd / 0.2, -1.0, 1.0)
    if drive_servo > 0 and max_drive_force > 0.0:
        var f := servo_fraction(pwm, drive_servo, drive_pwm_min, drive_pwm_max)
        if f >= 0.0:
            var demand := 2.0 * f - 1.0
            if free_speed > 0.0:
                demand -= v_fwd / free_speed
            var drive := clampf(demand * max_drive_force, -grip * load, grip * load)
            force += fwd * drive
    vehicle.apply_force(force, contact - vehicle.global_position)
    _place_visuals(dt, v_fwd)

func _place_visuals(dt: float, v_fwd: float) -> void:
    if radius > 0.0:
        _spin = fmod(_spin + v_fwd / radius * dt, TAU)
    var t := Transform3D(Basis(Vector3.RIGHT, _spin), Vector3.UP * compression)
    for child in get_children():
        if child is Node3D:
            if not child.has_meta("ap_rest"):
                child.set_meta("ap_rest", child.transform)
            child.transform = t * (child.get_meta("ap_rest") as Transform3D)

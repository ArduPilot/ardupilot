# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

@tool
@icon("res://icons/hinge.svg")
class_name ArduPilotHinge
extends "res://scripts/ap_component.gd" # servo pivot about local X

@export_range(1, 32) var servo: int = 1 ## SERVOn output (1-based)
@export var pwm_min: float = 1000.0
@export var pwm_max: float = 2000.0
@export var angle_min: float = -30.0: ## angle at pwm_min (deg)
    set(v):
        angle_min = v
        update_gizmos()
@export var angle_max: float = 30.0: ## angle at pwm_max (deg)
    set(v):
        angle_max = v
        update_gizmos()
@export var max_rate: float = 300.0 ## slew rate (deg/s), 0 instant
@export var max_torque: float = 2.0 ## joint servo torque limit (N.m)
@export var continuous: bool = false ## continuous-rotation servo, joints only
@export var built_angle: float = 0.0 ## joint angle as built (deg)
@export var sheet: bool = false ## sail sheet: sign follows airflow
@export_range(-180.0, 180.0) var preview_angle: float = 0.0: ## editor-only preview deflection (deg)
    set(v):
        preview_angle = v
        if Engine.is_editor_hint() and is_inside_tree():
            _set_angle(v)

var angle: float = 0.0 # (deg)
var rest: Transform3D # transform at zero angle
var _applying := false
var _joint: HingeJoint3D = null # driven joint, if any
var _bound := false
var _body_a: RigidBody3D = null
var _body_b: RigidBody3D = null
var _ref_a := Vector3.RIGHT
var _ref_b := Vector3.RIGHT
var _axis_b := Vector3.BACK
var _turned := 0.0 # unwrapped joint angle (rad)
var _last_raw := 0.0
var _held := 0.0 # last target, held without signal
const JOINT_GAIN := 30.0 # (deg/s) per degree error
var _lee := 1.0 # last lee side, +1/-1
var _initialised := false

func gltf_key() -> String:
    return "hinge"

func gltf_fields() -> Dictionary:
    return {
        "servo": "servo", "pwmMin": "pwm_min", "pwmMax": "pwm_max",
        "angleMin": "angle_min", "angleMax": "angle_max", "maxRate": "max_rate",
        "sheet": "sheet", "maxTorque": "max_torque", "continuous": "continuous",
        "builtAngle": "built_angle",
    }

func _enter_tree() -> void:
    set_notify_local_transform(true)
    var first := not _initialised
    if first: # saved transform includes preview deflection
        _initialised = true
        angle = preview_angle
    rest = transform * _rotation(-angle)
    if first and not Engine.is_editor_hint():
        _set_angle(0.0)

func _notification(what: int) -> void:
    if what == NOTIFICATION_LOCAL_TRANSFORM_CHANGED and not _applying: # moved in editor: keep deflection
        rest = transform * _rotation(-angle)

func rest_transform() -> Transform3D: # valid before entering the tree
    if not _initialised:
        return transform * _rotation(-preview_angle)
    return rest

static func _rotation(deg: float) -> Transform3D:
    return Transform3D(Basis(Vector3.RIGHT, deg_to_rad(deg)), Vector3.ZERO)

func _set_angle(deg: float) -> void:
    angle = deg
    _applying = true
    transform = rest * _rotation(deg)
    _applying = false

func _bind_joint() -> void: # deferred: joint may enter later
    _bound = true
    if not has_meta(&"ap_joint"):
        return
    _joint = get_node_or_null(get_meta(&"ap_joint")) as HingeJoint3D
    if _joint == null:
        return
    _body_a = _joint.get_node(_joint.node_a) as RigidBody3D
    _body_b = _joint.get_node(_joint.node_b) as RigidBody3D
    _joint.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
    var jb := _joint.global_transform.basis.orthonormalized()
    _ref_a = _body_a.global_transform.basis.orthonormalized().inverse() * jb.x
    _ref_b = _body_b.global_transform.basis.orthonormalized().inverse() * jb.x
    _axis_b = _body_b.global_transform.basis.orthonormalized().inverse() * jb.z
    _held = built_angle

func joint_angle() -> float: # body A's turn (rad)
    var ba := _body_a.global_transform.basis.orthonormalized()
    var bb := _body_b.global_transform.basis.orthonormalized()
    var va := ba * _ref_a
    var vb := bb * _ref_b
    var raw := atan2((bb * _axis_b).dot(vb.cross(va)), vb.dot(va))
    _turned += wrapf(raw - _last_raw, -PI, PI) # counts past half a turn
    _last_raw = raw
    return _turned

func physics_update(_vehicle: RigidBody3D, pwm: PackedInt32Array, dt: float) -> void:
    var f := servo_fraction(pwm, servo, pwm_min, pwm_max)
    if not _bound:
        _bind_joint()
    if f < 0.0 and _joint == null:
        return
    var target := lerpf(angle_min, angle_max, f) if f >= 0.0 else _held
    _held = target
    if _joint != null:
        angle = rad_to_deg(joint_angle()) + built_angle
        var rate := clampf(JOINT_GAIN * (target - angle), -max_rate, max_rate)
        if continuous:
            var way := 1.0 if angle_max >= angle_min else -1.0
            rate = way * (2.0 * f - 1.0) * max_rate if f >= 0.0 else 0.0
        _joint.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, deg_to_rad(rate))
        _joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, max_torque * dt)
        return
    if sheet:
        target = absf(target) * _lee_sign(_vehicle)
    if max_rate > 0.0:
        var step := max_rate * dt
        target = clampf(target, angle - step, angle + step)
    if target != angle:
        _set_angle(target)

func _lee_sign(vehicle: RigidBody3D) -> float: # +1/-1; held head to wind
    var parent := get_parent() as Node3D
    if vehicle == null or parent == null:
        return _lee
    var b := (parent.global_transform.basis * rest.basis).orthonormalized() # X mast, Y across boom
    var air: Vector3 = vehicle.wind - vehicle.linear_velocity
    var side := air.dot(b.y)
    if absf(side) > 0.15: # positive turn carries boom +Y
        _lee = 1.0 if side > 0.0 else -1.0
    return _lee

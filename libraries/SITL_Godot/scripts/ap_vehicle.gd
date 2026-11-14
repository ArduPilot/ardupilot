# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

@tool
@icon("res://icons/vehicle.svg")
class_name ArduPilotVehicle
extends RigidBody3D # +Z forward, -X right

const Component := preload("res://scripts/ap_component.gd")
const Rotor := preload("res://scripts/ap_rotor.gd")
const Hinge := preload("res://scripts/ap_hinge.gd")
const Surface := preload("res://scripts/ap_surface.gd")

const TO_NED := Basis(Vector3(0, -1, 0), Vector3(0, 0, -1), Vector3(1, 0, 0)) # NED = (z, -x, -y)

@export var drag_area: Vector3 = Vector3.ZERO ## CdA per body axis (m^2)
@export var air_density: float = 1.225
@export_group("Water")
@export var water_enabled: bool = false ## floats; --water= option overrides
@export var water_level: float = 0.0 ## water surface, world Y (m)
@export var water_density: float = 1000.0
@export_group("")

var wind: Vector3 = Vector3.ZERO # world frame, Godot axes (m/s)
var alpha: float = 0.0 # angle of attack (rad)
var last_gyro := Vector3.ZERO # FRD p, q, r (rad/s)

signal impact(other: Node, speed: float) # hard contact, speed (m/s)

const IMPACT_SPEED := 2.0 # sink rate threshold (m/s)

var _components: Array[Component] = []
var _limbs: Array = [] # [jointed body, start transform]
var _initial_transform: Transform3D
var _initial_velocity: Vector3
var _last_velocity: Vector3
var _have_last_velocity := false
var _pre_step_vel := Vector3.ZERO # contacts report post-impact velocity

func _ready() -> void:
    if Engine.is_editor_hint():
        return
    can_sleep = false
    contact_monitor = true
    max_contacts_reported = 4
    body_entered.connect(_on_body_entered)
    linear_damp_mode = DAMP_MODE_REPLACE
    angular_damp_mode = DAMP_MODE_REPLACE
    linear_damp = 0.0
    angular_damp = 0.0
    _initial_transform = global_transform
    _initial_velocity = linear_velocity
    _components.clear()
    _find_components(self)
    _limbs.clear()
    if get_parent() != null:
        for sib in get_parent().get_children():
            if sib != self and sib is RigidBody3D: # sibling bodies are limbs
                (sib as RigidBody3D).can_sleep = false
                _limbs.append([sib, (sib as RigidBody3D).global_transform])
                _find_components(sib)
    _components.sort_custom(func(a, b): return (a is Hinge) and not (b is Hinge)) # hinges first: children need pose

func _on_body_entered(other: Node) -> void:
    var sink := maxf(-linear_velocity.y, -_pre_step_vel.y) # signed: climbing is no crash
    if sink >= IMPACT_SPEED:
        var speed := maxf(linear_velocity.length(), _pre_step_vel.length())
        impact.emit(other, speed)

func _find_components(n: Node) -> void:
    for c in n.get_children():
        if c is Component:
            _components.append(c)
        if not (c is RigidBody3D): # nested body owns its parts
            _find_components(c)

func get_components() -> Array[Component]:
    return _components

func velocity_at(point: Vector3) -> Vector3: # world frame, body-fixed point
    var st := PhysicsServer3D.body_get_direct_state(get_rid())
    var com := st.transform.origin + st.center_of_mass
    return st.linear_velocity + st.angular_velocity.cross(point - com)

func air_velocity_at(point: Vector3) -> Vector3: # relative to the air
    return velocity_at(point) - wind

func propwash_at(point: Vector3, v_air: Vector3, span_dir := Vector3.ZERO, span := 0.0) -> Vector3: # see ArduPilotRotor.propwash_at
    var wash := Vector3.ZERO
    for c in _components:
        if c is Rotor:
            wash += c.propwash_at(point, v_air, span_dir, span)
    return wash

func apply_actuators(pwm: PackedInt32Array, dt: float) -> void:
    _pre_step_vel = linear_velocity
    var b := global_basis.orthonormalized()
    var v_body := b.transposed() * (linear_velocity - wind)
    alpha = atan2(-v_body.y, v_body.z) if absf(v_body.z) + absf(v_body.y) > 0.1 else 0.0
    for c in _components:
        c.physics_update(self, pwm, dt)

    if drag_area != Vector3.ZERO:
        var f_local := -0.5 * air_density * drag_area * v_body * v_body.abs()
        apply_central_force(b * f_local)

func set_initial_velocity(v: Vector3) -> void:
    _initial_velocity = v
    linear_velocity = v

func reset() -> void:
    var rid := get_rid()
    PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_TRANSFORM, _initial_transform)
    PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY, _initial_velocity)
    PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY, Vector3.ZERO)
    for limb in _limbs:
        var lr: RID = (limb[0] as RigidBody3D).get_rid()
        PhysicsServer3D.body_set_state(lr, PhysicsServer3D.BODY_STATE_TRANSFORM, limb[1])
        PhysicsServer3D.body_set_state(lr, PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY, Vector3.ZERO)
        PhysicsServer3D.body_set_state(lr, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY, Vector3.ZERO)
    for c in _components:
        if c.has_method("reset"): # forget state, e.g. suspension compression
            c.reset()
    _have_last_velocity = false

func sitl_state(dt: float) -> Dictionary: # ArduPilot frames, after step dt
    var st := PhysicsServer3D.body_get_direct_state(get_rid())
    var b := st.transform.basis.orthonormalized()
    var w := st.angular_velocity
    var v := st.linear_velocity + w.cross(-st.center_of_mass) # velocity at origin, the IMU

    var accel_world := Vector3.ZERO
    if _have_last_velocity and dt > 0.0:
        accel_world = (v - _last_velocity) / dt
    _last_velocity = v
    _have_last_velocity = true
    var specific_force := accel_world - st.total_gravity

    var dcm := TO_NED * b * TO_NED.transposed()
    last_gyro = TO_NED * (b.transposed() * w)
    var s := {
        "gyro": last_gyro,
        "accel_body": TO_NED * (b.transposed() * specific_force),
        "position": TO_NED * st.transform.origin,
        "velocity": TO_NED * v,
        "quaternion": dcm.get_rotation_quaternion(),
        "wind": TO_NED * wind,
    }
    if _components.any(func(c): return c is Surface):
        s["airspeed"] = maxf((b.transposed() * (v - wind)).z, 0.0) # pitot along forward axis
    if wind != Vector3.ZERO: # SITL cannot model this wind
        var air_body: Vector3 = b.transposed() * (wind - v) # apparent wind, body frame
        s["windvane"] = {
            "direction": wrapf(atan2(-air_body.x, air_body.z) + PI, -PI, PI), # where it comes from
            "speed": Vector2(air_body.x, air_body.z).length(),
        }
    return s

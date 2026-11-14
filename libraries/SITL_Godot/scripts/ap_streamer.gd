# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

@tool
@icon("res://icons/surface.svg")
class_name ArduPilotStreamer
extends "res://scripts/ap_component.gd" # telltale ribbon, display only

@export var length: float = 0.6: ## ribbon length (m)
    set(v):
        length = v
        _pts.clear()
@export var width: float = 0.04 ## root width (m), tapers
@export var colour: Color = Color(0.95, 0.25, 0.15)
@export var whip: float = 0.0 ## whip height above origin (m)
@export var drag: float = 2.6 ## acceleration per airflow speed (1/s)
@export_range(0.0, 1.0) var damping: float = 0.80 ## speed kept per 1/60 s
@export var flutter: float = 0.22 ## shake as fraction of airflow

const LINKS := 10
const DRAW_MS := 33

var _pts: PackedVector3Array = PackedVector3Array()
var _vel: PackedVector3Array = PackedVector3Array()
var _ribbon: MeshInstance3D = null
var _whip_mesh: MeshInstance3D = null
var _drawn_ms := 0
var _age := 0.0 # gust phase clock

func gltf_key() -> String:
    return "streamer"

func gltf_fields() -> Dictionary:
    return {"length": "length", "width": "width", "drag": "drag",
            "flutter": "flutter", "whip": "whip"}

func _ready() -> void:
    if _ribbon == null:
        _ribbon = MeshInstance3D.new()
        _ribbon.name = "ribbon"
        _ribbon.top_level = true # cloth lives in world space
        add_child(_ribbon)
        var mat := StandardMaterial3D.new()
        mat.albedo_color = colour
        mat.cull_mode = BaseMaterial3D.CULL_DISABLED
        mat.roughness = 0.9
        _ribbon.material_override = mat
    if whip > 0.0 and _whip_mesh == null:
        var rod := CylinderMesh.new()
        rod.top_radius = 0.004
        rod.bottom_radius = 0.006
        rod.height = whip
        rod.radial_segments = 6
        _whip_mesh = MeshInstance3D.new()
        _whip_mesh.name = "whip"
        _whip_mesh.mesh = rod
        _whip_mesh.position = Vector3(0, 0.5 * whip, 0)
        var rodmat := StandardMaterial3D.new()
        rodmat.albedo_color = Color(0.15, 0.15, 0.17)
        rodmat.metallic = 0.4
        _whip_mesh.material_override = rodmat
        add_child(_whip_mesh)

func _seed() -> void:
    _pts.resize(LINKS + 1)
    _vel.resize(LINKS + 1)
    var seg := length / LINKS
    for i in _pts.size():
        _pts[i] = _anchor() + Vector3.DOWN * (seg * i)
        _vel[i] = Vector3.ZERO

func physics_update(vehicle: RigidBody3D, _pwm: PackedInt32Array, dt: float) -> void:
    if _ribbon == null or dt <= 0.0:
        return
    if _pts.size() != LINKS + 1:
        _seed()
    var seg := length / LINKS
    var tip := _anchor()
    var air: Vector3 = vehicle.wind - vehicle.velocity_at(tip) # apparent wind at the masthead

    _pts[0] = tip # root fixed to whip tip
    _vel[0] = Vector3.ZERO
    var keep := pow(damping, dt * 60.0)
    var speed := air.length()
    _age += dt * (0.6 + speed)
    var across := air.cross(Vector3.UP) # shake axes across the flow
    if across.length() < 0.01:
        across = air.cross(Vector3.RIGHT)
    across = across.normalized() if across.length() > 0.0001 else Vector3.RIGHT
    var lift := across.cross(air.normalized()) if speed > 0.01 else Vector3.UP
    for i in range(1, _pts.size()):
        var v: Vector3 = _vel[i]
        v += (air * drag + Vector3.DOWN * G) * dt
        var phase := _age * 1.9 - float(i) * 0.7 # ripple runs root to tip
        var shake := speed * flutter * (1.0 + 0.4 * float(i) / LINKS)
        v += across * sin(phase) * shake * dt
        v += lift * cos(phase * 0.7 + 1.1) * shake * 0.6 * dt
        v *= keep
        var p: Vector3 = _pts[i] + v * dt
        var to_prev: Vector3 = p - _pts[i - 1]
        if to_prev.length() > 0.0001:
            p = _pts[i - 1] + to_prev.normalized() * seg # keep link length fixed
        _vel[i] = (p - _pts[i]) / dt
        _pts[i] = p

    var now := Time.get_ticks_msec()
    if now - _drawn_ms < DRAW_MS:
        return
    _drawn_ms = now
    _rebuild()

func _rebuild() -> void: # tapered strip through the chain
    if _ribbon == null or _pts.size() < 2:
        return
    var st := SurfaceTool.new()
    st.begin(Mesh.PRIMITIVE_TRIANGLES)
    for i in range(_pts.size() - 1):
        var t0 := float(i) / (_pts.size() - 1)
        var t1 := float(i + 1) / (_pts.size() - 1)
        var a: Vector3 = _pts[i]
        var b: Vector3 = _pts[i + 1]
        var along := (b - a)
        if along.length() < 0.0001:
            continue
        var side := along.normalized().cross(Vector3.UP)
        if side.length() < 0.01:
            side = along.normalized().cross(Vector3.RIGHT)
        side = side.normalized()
        var w0 := 0.5 * width * (1.0 - t0)
        var w1 := 0.5 * width * (1.0 - t1)
        st.add_vertex(a - side * w0)
        st.add_vertex(a + side * w0)
        st.add_vertex(b + side * w1)
        st.add_vertex(a - side * w0)
        st.add_vertex(b + side * w1)
        st.add_vertex(b - side * w1)
    st.generate_normals()
    _ribbon.mesh = st.commit()
    _ribbon.global_transform = Transform3D.IDENTITY

func _anchor() -> Vector3: # whip top, or the node
    return global_position + global_transform.basis.y.normalized() * whip

# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

@tool
@icon("res://icons/floater.svg")
class_name ArduPilotFloater
extends "res://scripts/ap_component.gd" # buoyancy: hull or lighter-than-air envelope

@export_enum("water", "air") var fluid: String = "water" ## fluid displaced
@export var volume: float = 0.001: ## fully immersed displaced volume (m^3)
    set(v):
        volume = v
        update_gizmos()
@export var height: float = 0.1: ## vertical extent (m), water ramp
    set(v):
        height = v
        update_gizmos()
@export var drag_area: Vector3 = Vector3.ZERO ## CdA per local axis (m^2)
@export var damping: float = 0.0 ## linear, per local axis (N.s/m)

func gltf_key() -> String:
    return "floater"

func gltf_fields() -> Dictionary:
    return {
        "fluid": "fluid", "volume": "volume", "height": "height",
        "dragArea": "drag_area", "damping": "damping",
    }

func physics_update(vehicle: RigidBody3D, _pwm: PackedInt32Array, _dt: float) -> void:
    var density: float = vehicle.air_density
    var submerged := 1.0
    if fluid != "air":
        if not vehicle.water_enabled:
            return
        density = vehicle.water_density
        var depth: float = vehicle.water_level - global_position.y
        submerged = clampf(depth / maxf(height, 0.001) + 0.5, 0.0, 1.0)
        if submerged <= 0.0:
            return

    var xform := global_transform
    var force := Vector3.UP * (density * G * volume * submerged)
    if drag_area != Vector3.ZERO or damping > 0.0:
        var b := xform.basis.orthonormalized()
        var v_local: Vector3 = b.transposed() * vehicle.velocity_at(xform.origin)
        var f_local: Vector3 = -0.5 * density * drag_area * v_local * v_local.abs() - damping * v_local
        force += b * (f_local * submerged)
    vehicle.apply_force(force, xform.origin - vehicle.global_position)

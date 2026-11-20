# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

@tool
@icon("res://icons/surface.svg")
class_name ArduPilotSurface
extends "res://scripts/ap_component.gd" # lift +Y, origin quarter chord

@export_enum("air", "water") var fluid: String = "air" ## water for keel or rudder

@export var span: float = 1.0: ## span (m), along local X
    set(v):
        span = v
        update_gizmos()
@export var chord: float = 0.2: ## chord (m), leading edge +Z
    set(v):
        chord = v
        update_gizmos()
@export var aspect_ratio: float = 0.0 ## whole wing's; 0 uses span/chord
@export var cl0: float = 0.0 ## lift coefficient at zero alpha
@export var cd0: float = 0.015 ## zero-lift drag coefficient
@export var cm0: float = 0.0 ## pitching moment about aerodynamic centre
@export var oswald: float = 0.85 ## Oswald efficiency, induced drag
@export var stall_angle: float = 15.0 ## where lift stops rising (deg)
@export var downwash: float = 0.0 ## d(epsilon)/d(alpha): tailplane 0.4-0.5, wing 0
@export_group("Flap")
@export_range(0, 32) var flap_servo: int = 0 ## trailing-edge flap servo, 0 none
@export var flap_pwm_min: float = 1000.0
@export var flap_pwm_max: float = 2000.0
@export var flap_angle_min: float = -20.0 ## at pwm_min (deg), down positive
@export var flap_angle_max: float = 20.0 ## at pwm_max (deg), down positive
@export_range(0.05, 0.95) var flap_chord_fraction: float = 0.25: ## fraction of panel chord
    set(v):
        flap_chord_fraction = v
        update_gizmos()
@export_group("")

const FLAP_VISUAL := "flap_visual" # child turned to show flap

var flap: float = 0.0 # deflection (rad)

func gltf_key() -> String:
    return "surface"

func gltf_fields() -> Dictionary:
    return {
        "span": "span", "chord": "chord", "aspectRatio": "aspect_ratio",
        "cl0": "cl0", "cd0": "cd0", "cm0": "cm0", "oswald": "oswald",
        "stallAngle": "stall_angle", "fluid": "fluid", "downwash": "downwash", "flapServo": "flap_servo",
        "flapPwmMin": "flap_pwm_min", "flapPwmMax": "flap_pwm_max",
        "flapAngleMin": "flap_angle_min", "flapAngleMax": "flap_angle_max",
        "flapChordFraction": "flap_chord_fraction",
    }

func effective_aspect_ratio() -> float:
    if aspect_ratio > 0.0:
        return aspect_ratio
    return span / chord if chord > 0.0 else 1.0

func lift_slope() -> float: # per radian, finite wing (Helmbold)
    var ar := effective_aspect_ratio()
    return TAU * ar / (2.0 + sqrt(ar * ar + 4.0))

func flap_effectiveness() -> float: # d(alpha)/d(flap), thin aerofoil theory
    var theta := acos(2.0 * flap_chord_fraction - 1.0)
    return 1.0 - (theta - sin(theta)) / PI

func coefficients(a: float, d: float) -> Vector2: # returns (CL, CD)
    var a_eff := a + flap_effectiveness() * d
    var a0 := deg_to_rad(stall_angle)
    var m := 50.0
    var e1 := exp(-m * (a_eff - a0))
    var e2 := exp(m * (a_eff + a0))
    var sigma := (1.0 + e1 + e2) / ((1.0 + e1) * (1.0 + e2)) # Beard & McLain 2012, 4.2
    var cl_linear := cl0 + lift_slope() * a_eff
    var s := sin(a_eff)
    var c := cos(a_eff)
    var cl_plate := 2.0 * signf(a_eff) * s * s * c
    var cl := (1.0 - sigma) * cl_linear + sigma * cl_plate
    var cd_induced := cl_linear * cl_linear / (PI * oswald * effective_aspect_ratio())
    var cd := cd0 + (1.0 - sigma) * cd_induced + sigma * 2.0 * s * s
    var sd := sin(d)
    cd += 0.9 * pow(flap_chord_fraction, 1.38) * sd * sd
    return Vector2(cl, cd)

func physics_update(vehicle: RigidBody3D, pwm: PackedInt32Array, _dt: float) -> void:
    if flap_servo > 0:
        var f := servo_fraction(pwm, flap_servo, flap_pwm_min, flap_pwm_max)
        if f >= 0.0:
            flap = deg_to_rad(lerpf(flap_angle_min, flap_angle_max, f))
            var pivot := get_node_or_null(FLAP_VISUAL) as Node3D
            if pivot:
                pivot.basis = Basis(Vector3.RIGHT, -flap) # trailing edge down is -X

    var xform := global_transform
    var b := xform.basis.orthonormalized()
    var v_air: Vector3
    var rho: float = vehicle.air_density
    if fluid == "water": # wind does not move water
        var reach := maxf(maxf(absf(b.x.y) * span, absf(b.z.y) * chord), 0.02)
        var wet := 0.0
        if vehicle.water_enabled:
            wet = clampf((vehicle.water_level - xform.origin.y) / reach + 0.5, 0.0, 1.0)
        if wet <= 0.0:
            return
        rho = vehicle.water_density * wet # only the wetted part
        v_air = vehicle.velocity_at(xform.origin)
    else:
        v_air = vehicle.air_velocity_at(xform.origin)
        v_air += vehicle.propwash_at(xform.origin, v_air, b.x, span)
    var v_local: Vector3 = b.transposed() * v_air
    v_local.x = 0.0 # strip theory: ignore spanwise flow
    var speed := v_local.length()
    if speed < 0.1:
        return
    var alpha := atan2(-v_local.y, v_local.z)
    if downwash != 0.0:
        alpha -= downwash * clampf(vehicle.alpha, -0.3, 0.3) # downwash collapses at wing stall
    var k := coefficients(alpha, flap)
    var q := 0.5 * rho * speed * speed
    var area := span * chord
    var wind_dir := -v_local / speed
    var lift_dir := Vector3.RIGHT.cross(wind_dir)
    var force := lift_dir * (q * area * k.x) + wind_dir * (q * area * k.y)
    vehicle.apply_force(b * force, xform.origin - vehicle.global_position)
    if cm0 != 0.0:
        vehicle.apply_torque(b * Vector3(-q * area * chord * cm0, 0.0, 0.0)) # nose-up moment is about -X

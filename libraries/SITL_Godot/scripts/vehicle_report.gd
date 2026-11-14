# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

extends RefCounted # design figures from simulation models

const Component := preload("res://scripts/ap_component.gd")
const Vehicle := preload("res://scripts/ap_vehicle.gd")
const Rotor := preload("res://scripts/ap_rotor.gd")
const Hinge := preload("res://scripts/ap_hinge.gd")
const Surface := preload("res://scripts/ap_surface.gd")
const Wheel := preload("res://scripts/ap_wheel.gd")
const Floater := preload("res://scripts/ap_floater.gd")
const HeliRotor := preload("res://scripts/ap_heli_rotor.gd")

static func _parts(n: Node, out: Array) -> void:
    for c in n.get_children():
        if c is Component:
            out.append(c)
        if not (c is RigidBody3D):
            _parts(c, out)

static func _relative(n: Node, vehicle: Node) -> Transform3D: # valid outside the SceneTree
    var t := Transform3D.IDENTITY
    while n != null and n != vehicle:
        if n is Node3D:
            t = n.transform * t
        n = n.get_parent()
    return t

static func report(vehicle: RigidBody3D) -> String:
    var lines := PackedStringArray()
    var mass := vehicle.mass
    var com := vehicle.center_of_mass if vehicle.center_of_mass_mode == RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM else Vector3.ZERO
    lines.append("vehicle '%s': mass %.3f kg, inertia %s kg.m^2 (glTF X, Y, Z)" % [vehicle.name, mass, vehicle.inertia])
    var parts: Array = []
    _parts(vehicle, parts)

    var lift_thrust := 0.0
    var fwd_thrust := 0.0
    var area := 0.0
    var cl_area := 0.0
    var slope_area := 0.0
    var slope_area_z := 0.0
    var chord_area := 0.0
    var water_displacement := 0.0
    var air_displacement := 0.0
    for p in parts:
        var t := _relative(p, vehicle)
        var pos := Vehicle.TO_NED * t.origin
        var where := "FRD (%.3f, %.3f, %.3f)" % [pos.x, pos.y, pos.z]
        if p is HeliRotor:
            lift_thrust += p.thrust_scale * p.collective_max * maxf(t.basis.y.normalized().y, 0.0)
            lines.append("  rotor   %-14s swash %d/%d/%d rsc %d at %s %s, %.1f N/deg" % [
                    p.name, p.swash_servo_1, p.swash_servo_2, p.swash_servo_3,
                    p.rsc_servo, where, p.direction, p.thrust_scale])
        elif p is Floater:
            if p.fluid == "air":
                air_displacement += p.volume
            else:
                water_displacement += p.volume
            lines.append("  floater %-14s at %s, %.4f m^3 of %s" % [p.name, where, p.volume, p.fluid])
        elif p is Rotor:
            var axis := t.basis.y.normalized()
            lift_thrust += p.max_thrust * maxf(axis.y, 0.0)
            fwd_thrust += p.max_thrust * maxf(axis.z, 0.0)
            lines.append("  rotor   %-14s servo %2d at %s %s, %.1f N" % [p.name, p.servo, where, p.direction, p.max_thrust])
        elif p is Hinge:
            lines.append("  hinge   %-14s servo %2d at %s, %.0f..%.0f deg" % [p.name, p.servo, where, p.angle_min, p.angle_max])
        elif p is Wheel:
            lines.append("  wheel   %-14s at %s, radius %.3f m" % [p.name, where, p.radius])
        elif p is Surface:
            var s: float = p.span * p.chord
            var flap := "flap servo %d" % p.flap_servo if p.flap_servo > 0 else "no flap"
            lines.append("  surface %-14s at %s, %.3f m^2, %s" % [p.name, where, s, flap])
            var up := t.basis.y.normalized().y
            if up > 0.7: # upward lifters set wing loading
                var a: float = p.lift_slope() * up * up * (1.0 - p.downwash)
                area += s
                cl_area += s * (p.cl0 + p.lift_slope() * deg_to_rad(p.stall_angle))
                slope_area += a * s
                slope_area_z += a * s * t.origin.z
                chord_area += s * p.chord

    var weight := mass * Component.G
    if lift_thrust > 0.0:
        lines.append("hover thrust/weight %.2f" % (lift_thrust / weight))
    if fwd_thrust > 0.0:
        lines.append("forward thrust/weight %.2f" % (fwd_thrust / weight))
    if air_displacement > 0.0:
        var lift: float = air_displacement * vehicle.air_density
        lines.append("buoyant lift %.3f kg for %.3f kg of mass (%.0f%%)" % [
                lift, mass, 100.0 * lift / mass])
    if water_displacement > 0.0:
        var supports: float = water_displacement * vehicle.water_density
        lines.append("hull displaces %.1f kg fully immersed, %.1f%% draught at rest" % [
                supports, 100.0 * mass / supports])
    if area > 0.0:
        lines.append("lifting area %.3f m^2, wing loading %.1f kg/m^2" % [area, mass / area])
        lines.append("stall speed about %.1f m/s" % sqrt(2.0 * weight / (1.225 * cl_area)))
        var neutral := slope_area_z / slope_area
        var mean_chord := chord_area / area
        lines.append("neutral point %.3f m ahead of the origin, centre of mass %.3f m ahead" % [neutral, com.z])
        lines.append("static margin %.0f%% of mean chord %.3f m (positive is stable)" % [
                100.0 * (com.z - neutral) / mean_chord, mean_chord])
    return "\n".join(lines)

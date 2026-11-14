# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

class_name ArduPilotHUD
extends Control # top-left HUD, true vehicle state

const Vehicle := preload("res://scripts/ap_vehicle.gd")

const SKY := Color(0.25, 0.5, 0.9, 0.4)
const GROUND := Color(0.5, 0.33, 0.16, 0.4)
const LINE := Color(1, 1, 1, 0.6)
const WIND := Color(0.95, 0.3, 0.2) # wind arrow, matches telltale red
const PITCH_RANGE := 45.0 # pitch across ball radius (deg)

var vehicle: Vehicle = null

func _process(_delta: float) -> void:
    var vs := get_viewport_rect().size
    position = Vector2(8, 8)
    size = Vector2(0.1 * vs.x, 0.1 * vs.y)
    queue_redraw()

func _draw() -> void:
    if vehicle == null or not vehicle.is_inside_tree():
        return
    var b := Vehicle.TO_NED * vehicle.global_basis.orthonormalized() * Vehicle.TO_NED.transposed() # attitude via FRD/NED mapping
    var fwd := b.x
    var heading := fposmod(rad_to_deg(atan2(fwd.y, fwd.x)), 360.0)
    var pitch := atan2(-fwd.z, Vector2(fwd.x, fwd.y).length())
    var roll := atan2(b.y.z, b.z.z)
    var spd: float = vehicle.linear_velocity.length()
    var alt: float = vehicle.global_position.y

    var r := 0.5 * size.y - 2.0
    var centre := Vector2(r + 2.0, 0.5 * size.y)
    draw_circle(centre, r, SKY)

    var d := clampf(rad_to_deg(pitch) / PITCH_RANGE * r, -r + 1.5, r - 1.5)
    var x_int := sqrt(maxf(r * r - d * d, 0.0))
    draw_set_transform(centre, -roll, Vector2.ONE)
    if x_int > 1.0 and r - absf(d) > 1.0 and is_finite(d) and is_finite(roll): # sub-pixel sliver fails to triangulate
        var a0 := atan2(d, x_int)
        var a1 := PI - a0
        var ground := PackedVector2Array()
        for i in 17: # arc already includes chord ends
            ground.append(Vector2(cos(lerpf(a0, a1, i / 16.0)), sin(lerpf(a0, a1, i / 16.0))) * r)
        draw_colored_polygon(ground, GROUND)
    draw_line(Vector2(-x_int, d), Vector2(x_int, d), LINE, 1.2)
    for deg: float in [-30.0, -15.0, 15.0, 30.0]: # pitch ladder
        var y := d - deg / PITCH_RANGE * r
        if absf(y) < 0.85 * r:
            var w := 0.3 * r if absf(deg) == 15.0 else 0.45 * r
            draw_line(Vector2(-w, y), Vector2(w, y), LINE, 1.0)
    draw_set_transform(Vector2.ZERO, 0.0, Vector2.ONE)

    draw_line(centre + Vector2(-0.6 * r, 0), centre + Vector2(-0.2 * r, 0), LINE, 2.0)
    draw_line(centre + Vector2(0.2 * r, 0), centre + Vector2(0.6 * r, 0), LINE, 2.0)
    draw_circle(centre, 1.5, LINE)
    draw_arc(centre, r, 0, TAU, 40, LINE, 1.0)

    var font := ThemeDB.fallback_font
    var fs := clampi(int(0.22 * size.y), 8, 14)
    var x := centre.x + r + 6.0
    var y := centre.y - r + fs
    var widest := 0.0
    var top := y
    for line in ["GS %5.1f" % spd, "ALT %5.0f" % alt, "HDG  %03d" % int(heading)]:
        draw_string(font, Vector2(x, y), line, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, LINE)
        widest = maxf(widest, font.get_string_size(line, HORIZONTAL_ALIGNMENT_LEFT, -1, fs).x)
        y += fs + 2

    var wind: Vector3 = vehicle.wind
    var blow := Vector2(wind.z, -wind.x) # x north, y east
    var from := "---" # a calm comes from nowhere
    if blow.length() > 0.05:
        from = "%03d" % (int(round(fposmod(rad_to_deg(atan2(-blow.y, -blow.x)), 360.0))) % 360)
    x += widest + 14.0
    y = top
    var wind_wide := 0.0
    for line in ["WINDSPEED %4.1f" % blow.length(), "WINDDIRECTION %s" % from]:
        draw_string(font, Vector2(x, y), line, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, LINE)
        wind_wide = maxf(wind_wide, font.get_string_size(line, HORIZONTAL_ALIGNMENT_LEFT, -1, fs).x)
        y += fs + 2

    var cr := 0.82 * r # wind compass, north up
    var cc := Vector2(x + wind_wide + 14.0 + cr, centre.y)
    draw_arc(cc, cr, 0, TAU, 40, LINE, 1.0)
    draw_string(font, cc + Vector2(-0.3 * fs, -cr + fs), "N",
            HORIZONTAL_ALIGNMENT_LEFT, -1, fs, LINE)
    var hd := deg_to_rad(heading)
    var bow := Vector2(sin(hd), -cos(hd))
    draw_line(cc + bow * cr, cc + bow * (cr + 5.0), LINE, 3.0) # heading notch outside the rim
    if blow.length() > 0.05:
        var to := Vector2(blow.y, -blow.x).normalized() # screen: east right, north up
        var tail := cc - to * (0.6 * cr)
        var head := cc + to * (0.6 * cr)
        var across := Vector2(-to.y, to.x)
        draw_line(tail, head, WIND, 2.0) # arrow points downwind
        draw_line(head, head - to * (0.34 * cr) + across * (0.2 * cr), WIND, 2.0)
        draw_line(head, head - to * (0.34 * cr) - across * (0.2 * cr), WIND, 2.0)

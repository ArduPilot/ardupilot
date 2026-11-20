# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

class_name ArduPilotGyroGraph
extends Control # strip chart of SITL gyro

const Vehicle := preload("res://scripts/ap_vehicle.gd")

const SAMPLES := 480
const AXIS_COLORS: Array[Color] = [
    Color(1.0, 0.4, 0.4, 0.9), # p, roll rate
    Color(0.4, 1.0, 0.4, 0.9), # q, pitch rate
    Color(0.5, 0.7, 1.0, 0.9), # r, yaw rate
]
const AXIS_NAMES := ["p", "q", "r"]
const BG := Color(0, 0, 0, 0.35)
const LINE := Color(1, 1, 1, 0.5)

var vehicle: Vehicle = null
var _buf: Array[Vector3] = []

func _process(_delta: float) -> void:
    if vehicle == null:
        return
    _buf.append(vehicle.last_gyro) # per rendered frame, not physics
    while _buf.size() > SAMPLES:
        _buf.pop_front()
    var vs := get_viewport_rect().size
    position = Vector2(8, 0.1 * vs.y + 112)
    size = Vector2(maxf(0.2 * vs.x, 220), maxf(0.12 * vs.y, 80))
    queue_redraw()

func _draw() -> void:
    draw_rect(Rect2(Vector2.ZERO, size), BG)
    if _buf.is_empty():
        return
    var peak := 0.5
    for v in _buf:
        peak = maxf(peak, maxf(absf(v.x), maxf(absf(v.y), absf(v.z))))
    var mid := 0.5 * size.y
    draw_line(Vector2(0, mid), Vector2(size.x, mid), LINE, 1.0)
    var n := _buf.size()
    for axis in 3:
        var pts := PackedVector2Array()
        pts.resize(n)
        for i in n:
            var v: float = _buf[i][axis]
            pts[i] = Vector2(size.x * (1.0 - float(n - 1 - i) / (SAMPLES - 1)),
                    mid - v / peak * (mid - 2.0))
        if n >= 2:
            draw_polyline(pts, AXIS_COLORS[axis], 1.2)
    var font := ThemeDB.fallback_font
    var last := _buf[n - 1]
    var text := "gyro ±%.1f rad/s   p %+0.2f  q %+0.2f  r %+0.2f" % [
        peak, last.x, last.y, last.z]
    draw_string(font, Vector2(6, 14), text, HORIZONTAL_ALIGNMENT_LEFT, -1, 12, LINE)

# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.

class_name ArduPilotSITLBridge
extends RefCounted # see libraries/SITL/examples/JSON/readme.md

const MAGIC_16 := 18458
const MAGIC_32 := 29569

var udp := PacketPeerUDP.new()
var frame_rate := 0
var frame_count := -1
var pwm := PackedInt32Array()
var sitl_ip := ""
var sitl_port := 0
var last_reply := PackedByteArray()

func listen(port: int) -> Error:
    return udp.bind(port, "*")

func close() -> void: # frees port for next simulation
    udp.close()

func wait_for_servos(timeout_us: int) -> bool: # true on a new frame
    var deadline := Time.get_ticks_usec() + timeout_us
    while true:
        while udp.get_available_packet_count() > 0:
            if not _decode(udp.get_packet()):
                continue
            sitl_ip = udp.get_packet_ip()
            sitl_port = udp.get_packet_port()
            udp.set_dest_address(sitl_ip, sitl_port)
            return true
        if Time.get_ticks_usec() >= deadline:
            return false
        OS.delay_usec(20)
    return false

func _decode(pkt: PackedByteArray) -> bool:
    if pkt.size() < 8:
        return false
    var magic := pkt.decode_u16(0)
    var channels := 16 if magic == MAGIC_16 else (32 if magic == MAGIC_32 else 0)
    if channels == 0 or pkt.size() != 8 + 2 * channels:
        push_warning("ignoring bad SITL packet: magic %d, %d bytes" % [magic, pkt.size()])
        return false
    var count := pkt.decode_u32(4)
    if count == frame_count and not last_reply.is_empty():
        udp.put_packet(last_reply)
        return false
    frame_rate = pkt.decode_u16(2)
    frame_count = count
    pwm.resize(channels)
    for i in channels:
        pwm[i] = pkt.decode_u16(8 + 2 * i)
    return true

func send_state(timestamp: float, s: Dictionary) -> void:
    var g: Vector3 = s["gyro"]
    var a: Vector3 = s["accel_body"]
    var p: Vector3 = s["position"]
    var v: Vector3 = s["velocity"]
    var q: Quaternion = s["quaternion"]
    var wind: Vector3 = s.get("wind", Vector3.ZERO)
    var line := ("\n{\"timestamp\":%.6f,\"imu\":{\"gyro\":[%.6f,%.6f,%.6f],\"accel_body\":[%.6f,%.6f,%.6f]}," # SITL substring-searches: keep key order
            + "\"position\":[%.4f,%.4f,%.4f],\"quaternion\":[%.8f,%.8f,%.8f,%.8f],"
            + "\"velocity\":[%.5f,%.5f,%.5f],\"velocity_wind\":[%.5f,%.5f,%.5f]}\n") % [
            timestamp, g.x, g.y, g.z, a.x, a.y, a.z, p.x, p.y, p.z,
            q.w, q.x, q.y, q.z, v.x, v.y, v.z, wind.x, wind.y, wind.z]
    if s.has("airspeed"):
        line = line.trim_suffix("}\n") + ",\"airspeed\":%.4f}\n" % s["airspeed"]
    if s.has("windvane"): # must be last: "speed" clash
        var vane: Dictionary = s["windvane"]
        line = line.trim_suffix("}\n") + (",\"windvane\":{\"direction\":%.5f,\"speed\":%.4f}}\n"
                % [float(vane["direction"]), float(vane["speed"])])
    last_reply = line.to_ascii_buffer()
    udp.put_packet(last_reply)

#!/usr/bin/env python3
# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Convert a RealFlight vehicle export (.RFX) into a glTF vehicle for the Godot
backend, from the physics tree in its .rfvehicle. Motor thrust is estimated,
not read: the printed report says what was assumed. The .KEX mesh is ignored.

  ./rfx2gltf.py vehicle.rfx [output.gltf]
  ./rfx2gltf.py --dump vehicle.rfx      # print the parsed physics tree

AP_FLAKE8_CLEAN
'''

import io
import math
import os
import re
import sys
import zipfile

from gltf_model import X, Z, qaxis
from json_pretify import pretty

# Fly the result with the model's .parm from ArduPilot/SITL_Models, stripped
# of STAT_*, FORMAT_VERSION and SYSID_SW_MREV: a defaults file reapplying an
# old STAT_BOOTCNT on every boot stops the autotests detecting reboots.
GRAVITY = 9.81
AIR_DENSITY = 1.225


class Group(object):
    '''a SUBGROUP node: typed values plus ordered child groups'''

    def __init__(self, name):
        self.name = name
        self.values = {}
        self.children = []

    def child(self, name):
        for c in self.children:
            if c.name == name:
                return c
        return None

    def get(self, key, default=None):
        return self.values.get(key, default)

    def walk(self):
        yield self
        for c in self.children:
            for g in c.walk():
                yield g


def _decode(typed):
    '''a "TYPE:value" string to a python value'''
    kind, _, raw = typed.partition(":")
    if kind == "FLOAT":
        return float(raw) if raw else 0.0
    if kind in ("INT", "UINT64"):
        return int(raw) if raw else 0
    if kind == "BOOL":
        return raw == "Yes"
    if kind == "INTARRAY":
        return [int(v) for v in raw.split("~") if v != ""]
    if kind == "FLOATARRAY":
        return [float(v) for v in raw.split("~") if v != ""]
    if kind in ("VECTOR3", "VECTOR4", "BYTECOLOR"):
        return [float(v) for v in raw.split(",") if v != ""]
    return raw


def parse_rfvehicle(text):
    '''the .rfvehicle INI tree as {section name: Group}'''
    sections = {}
    stack = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            root = Group(line[1:-1])
            sections[root.name] = root
            stack = [root]
            continue
        if not stack:
            continue
        if line.startswith("SUBGROUP["):
            child = Group(line[len("SUBGROUP["):-1])
            stack[-1].children.append(child)
            stack.append(child)
        elif line.startswith("ENDGROUP["):
            if len(stack) > 1:
                stack.pop()
        elif "=" in line:
            key, _, typed = line.partition("=")
            stack[-1].values[key] = _decode(typed)
    return sections


def load_rfx(path):
    '''the parsed .rfvehicle sections from an .RFX archive, or from a
    bare .rfvehicle file'''
    if path.lower().endswith(".rfvehicle"):
        with open(path, "r", errors="replace") as f:
            return parse_rfvehicle(f.read())
    with zipfile.ZipFile(path) as z:
        names = [n for n in z.namelist() if n.lower().endswith(".rfvehicle")]
        if not names:
            raise ValueError("no .rfvehicle inside %s" % path)
        with z.open(names[0]) as f:
            return parse_rfvehicle(io.TextIOWrapper(f, errors="replace").read())


def dump(group, indent=0):
    pad = "  " * indent
    print("%s%s" % (pad, group.name))
    for k in sorted(group.values):
        print("%s  %s = %r" % (pad, k, group.values[k]))
    for c in group.children:
        dump(c, indent + 1)


def to_gltf(v):
    '''RealFlight (x right, y forward, z up) to glTF (x left, y up, z forward)'''
    return [-v[0], v[2], v[1]]


class Component(object):
    '''one RealFlight component with its absolute glTF position'''

    def __init__(self, group, parent):
        self.group = group
        self.parent = parent
        self.kind = group.get("ComponentType", "")
        self.name = group.get("ComponentNameTSTRING", "")
        loc = group.get("LocationInParentMTR", [0, 0, 0])
        base = parent.pos if parent else [0, 0, 0]
        off = to_gltf(loc)
        # a rolled parent (a V-tail panel) carries its children around
        # the forward axis with it
        proll = parent.roll if parent else 0.0
        a = math.radians(proll)
        off = [off[0] * math.cos(a) - off[1] * math.sin(a),
               off[0] * math.sin(a) + off[1] * math.cos(a), off[2]]
        self.pos = [base[i] + off[i] for i in range(3)]
        self.roll = proll + group.get("RotationAboutYDEG", 0.0)
        # pod/pivot tilt, composed down the chain (degrees, RF axes)
        rot = group.get("RotationCenterDEG", [0, 0, 0])
        prot = parent.tilt if parent else [0, 0, 0]
        self.tilt = [prot[i] + rot[i] for i in range(3)]

    def weight(self):
        return self.group.get("WeightKG", 0.0) or self.group.get("Mass", 0.0)

    def mirrored(self):
        '''the component doubles itself across the symmetry plane'''
        slave = self.group.get("FrameNameSlave", "<None>")
        return slave not in ("", "<None>") or \
            self.group.get("SymetricalAboutAxis", 0) == 2

    def mirror_chain(self):
        '''mirrored itself, or carried along by a mirrored ancestor'''
        n = self
        while n is not None:
            if n.mirrored():
                return True
            n = n.parent
        return False

    def ancestor_tilt_servo(self):
        '''the pod tilt servo IDs (master, slave) and range, if a pod
        up the chain is turned by a servo'''
        n = self
        while n is not None:
            g = n.group
            if g.get("xServoRotationAboutMaster", 0):
                lo = g.get("RotationMinDEG", [0, 0, 0])[0]
                hi = g.get("RotationMaxDEG", [0, 0, 0])[0]
                if abs(hi - lo) > 5:
                    return (g.get("xServoRotationAboutMaster"),
                            g.get("xServoRotationAboutSlave"), lo, hi)
            n = n.parent
        return None


def build_components(root):
    '''flatten the nested component tree, keeping parent links'''
    out = []

    def rec(group, parent):
        comp = Component(group, parent)
        out.append(comp)
        for c in group.children:
            if c.get("ComponentType") is not None:
                rec(c, comp)
    rec(root, None)
    return out


class Electronics(object):
    '''signal generator IDs to ArduPilot servo channels.

    A part names a signal generator (ServoThrottle=111); the live
    generator connects to a software radio output channel, and that
    output's first input feed names the receiver channel the autopilot
    drives (InputChannel 100 is receiver channel 1).'''

    def __init__(self, section, radio):
        self.outputs = {}
        live = set(section.get("SignalGenerators", []))
        for c in section.children:
            if not c.name.startswith("#"):
                continue
            sid = int(c.name[1:])
            ch = c.get("ConnectTo_InternalName")
            if sid in live and c.get("Type") == "PhysicalServo" and ch:
                self.outputs[sid] = int(ch)
        self.feeds = {}
        array = radio.child("OutputChannelsArray") if radio else None
        for out in (array.children if array else []):
            if not out.name.startswith("#"):
                continue
            for feeds in out.children:
                if feeds.name != "InputFeedsArray":
                    continue
                for feed in feeds.children:
                    ch = feed.get("InputChannel")
                    if ch and ch >= 100:
                        self.feeds[int(out.name[1:])] = ch - 99
                        break

    def channel(self, sid):
        '''ArduPilot servo channel for a signal generator, or None'''
        out = self.outputs.get(sid)
        if out is None:
            return None
        return self.feeds.get(out, out)


def naca_cl0(name):
    '''camber lift from a NACA 4-digit name; 0 when unknown'''
    m = re.search(r"NACA\s*(\d)(\d)\d\d", name or "")
    if not m:
        return 0.0
    return round(0.115 * int(m.group(1)), 3)


class Converter(object):
    def __init__(self, sections, report, cell_weight=0.05, motors=None, reversals=None):
        self.sections = sections
        self.report = report
        self.cell_weight = cell_weight
        # named motors from the archive's .electricTorque files
        self.motors = motors or {}
        # channels the model's ArduPilot .parm reverses
        self.reversals = reversals or set()
        self.electronics = Electronics(sections["VehicleElectronics"],
                                       sections.get("AirplaneSoftwareRadio"))
        self.components = build_components(sections["RootComponent"])
        self.nodes = [None]
        self.shapes = []
        self.parts = []
        self.masses = []       # (mass, pos, own inertia diagonal)
        self.inactive = set()
        self.main_chord = 0.0
        self.heli_thrust_scale = 9.0
        self.body_nose = -1e9
        self._find_inactive_engines()

    def parm_reversed(self, servo):
        return servo in self.reversals

    def say(self, msg):
        self.report.append(msg)

    def chassis(self):
        """how the backend should draw the vehicle's structure: the
        body's own dimensions, and which kind of airframe it is"""
        size = [0.2, 0.1, 0.3]
        # where the body actually sits: the centre of mass is not the
        # middle of the fuselage, and a chassis drawn about the origin
        # would stand ahead of the vehicle it belongs to
        centre = [0.0, 0.0, 0.0]
        for n in self.parts:
            kh = n.get("extensions", {}).get("KHR_physics_rigid_bodies", {})
            if "collider" in kh and n["name"].startswith("fuselage"):
                size = self.shapes[kh["collider"]["geometry"]["shape"]]["box"]["size"]
                centre = n["translation"]
                break
        lifters = 0
        for c in self.active(("PropellerComponent",)):
            if abs(c.tilt[0]) > 40:
                lifters += len(self.prop_sides(c))
        wings = any("surface" in n.get("extensions", {}).get("ARDUPILOT_sitl", {})
                    for n in self.parts)
        rotary = any("heliRotor" in n.get("extensions", {}).get("ARDUPILOT_sitl", {})
                     for n in self.parts)
        if rotary:
            # a cabin under the head, with a boom out to the tail rotor
            style = "helicopter"
        elif lifters >= 3:
            # a wing means the rotors hang on booms, not on arms from
            # the centre of a bare multirotor
            style = "quadplane" if wings else "multirotor"
        else:
            style = "plane"
        return {"style": style, "size": [round(v, 4) for v in size],
                "centre": [round(v, 4) for v in centre]}

    def neutral_point(self):
        '''the lift-weighted centre of the horizontal surfaces (forward
        axis), the same way the backend's vehicle report estimates it'''
        total = 0.0
        moment = 0.0
        for n in self.parts:
            ap = n.get("extensions", {}).get("ARDUPILOT_sitl", {})
            s = ap.get("surface")
            if s is None or abs(s.get("downwash", 0.0)) > 0.9:
                continue
            rot = n.get("rotation")
            if rot is not None and abs(rot[2]) > 0.5:
                continue  # a fin lifts sideways
            ar = s.get("aspectRatio") or (s["span"] / s["chord"])
            slope = ar / (2.0 + math.sqrt(ar * ar + 4.0))
            w = s["span"] * s["chord"] * slope * (1.0 - s.get("downwash", 0.0))
            # the surface node's origin is its aerodynamic centre
            total += w
            moment += w * n["translation"][2]
        return moment / total if total > 0 else None

    def _find_inactive_engines(self):
        '''RealFlight keeps alternate powerplants in the tree: every
        engine whose throttle servo is dead is inert, and the turbine
        and pulse jet placeholders also lose to any live propeller'''
        live_prop = False
        for c in self.components:
            if c.kind in ("PropellerComponent", "TurbineComponent",
                          "PulseJetComponent", "EngineComponent"):
                sid = c.group.get("ServoThrottle")
                if self.electronics.channel(sid) is None:
                    self.inactive.add(id(c))
                elif c.kind == "PropellerComponent":
                    live_prop = True
        if live_prop:
            for c in self.components:
                if c.kind in ("TurbineComponent", "PulseJetComponent"):
                    self.inactive.add(id(c))

    def active(self, kinds):
        for c in self.components:
            if c.kind in kinds and id(c) not in self.inactive:
                yield c

    def total_mass(self):
        '''component weights plus estimates for what RealFlight keeps
        in its own database: motors, battery cells and small gear'''
        m = 0.05  # servos, wiring, linkages
        for c in self.components:
            if id(c) in self.inactive:
                continue
            m += c.weight()
            if c.kind == "PropellerComponent":
                m += self.motor_mass(c) * len(self.prop_sides(c))
            if c.kind == "BatteryStoredEnergy":
                cells = (c.group.get("NumCellsInSeries", 3) *
                         max(c.group.get("NumCellsInParallel", 1), 1))
                m += cells * self.cell_weight
        return m

    def battery_cells(self):
        for c in self.components:
            if c.kind == "BatteryStoredEnergy" and id(c) not in self.inactive:
                return c.group.get("NumCellsInSeries", 3)
        return 3

    def motor_for(self, c):
        '''the named motor's data, when its .electricTorque came along'''
        return self.motors.get(c.group.get("TorqueGeneratorElectric", ""))

    def motor_mass(self, c):
        motor = self.motor_for(c)
        if motor is not None:
            # the named motor's own weight, plus a speed controller
            return 1.3 * motor["weight"]
        # otherwise motor and speed controller scale with the prop
        return 0.45 * c.group.get("PropDiameterMTR", 0.25)

    def motor_thrust(self, c, motor):
        '''static thrust and pitch speed from the motor constants and the
        propeller, by absorbed power and momentum theory'''
        g = c.group
        d = g.get("PropDiameterMTR", 0.25)
        volts = 3.7 * self.battery_cells()
        rpm = 0.8 * motor["kv"] * volts / max(g.get("GearRatio", 1.0), 0.1)
        n = rpm / 60.0
        power = 0.045 * AIR_DENSITY * n ** 3 * d ** 5
        area = math.pi * d * d / 4
        thrust = (2 * AIR_DENSITY * area) ** (1.0 / 3.0) * (0.8 * power) ** (2.0 / 3.0)
        return thrust, n * g.get("PropPitchMTR", 0.15)

    def add_mass(self, mass, pos, own=(0.0, 0.0, 0.0)):
        self.masses.append((mass, pos, own))

    # ---- glTF assembly -------------------------------------------------

    def shape(self, d):
        self.shapes.append(d)
        return len(self.shapes) - 1

    def node(self, n):
        self.nodes.append(n)
        return len(self.nodes) - 1

    def collider(self, name, size, pos):
        s = self.shape({"type": "box", "box": {"size": size}})
        self.parts.append({"name": name, "translation": pos, "extensions": {
            "KHR_physics_rigid_bodies":
            {"collider": {"geometry": {"shape": s}, "physicsMaterial": 0}}}})

    def part(self, name, kind, params, pos, rotation=None):
        n = {"name": name, "translation": [round(v, 5) for v in pos],
             "extensions": {"ARDUPILOT_sitl": {kind: params}}}
        if rotation:
            n["rotation"] = rotation
        self.parts.append(n)

    # ---- component mapping ---------------------------------------------

    def convert(self):
        for c in self.components:
            if id(c) in self.inactive:
                continue
            if c.kind == "Wing":
                self.map_wing(c)
            elif c.kind == "PropellerComponent":
                self.map_prop(c)
            elif c.kind == "TurbineComponent":
                self.map_turbine(c)
            elif c.kind == "Wheel":
                self.map_wheel(c)
            elif c.kind in ("Fuselage", "HeliMechanicsComponent"):
                self.map_fuselage(c)
            elif c.kind == "MainRotorComponent":
                self.map_main_rotor(c)
            elif c.kind == "TailRotorComponent":
                self.map_tail_rotor(c)
            elif c.kind == "HeliSkid":
                self.map_skid(c)
            w = c.weight()
            if w > 0:
                self.add_mass(w, c.pos)
        self._apply_downwash()

    def _apply_downwash(self):
        '''a horizontal tail flies in the main wing's downwash'''
        wings = []
        for n in self.parts:
            s = n.get("extensions", {}).get("ARDUPILOT_sitl", {}).get("surface")
            rot = n.get("rotation")
            if s is not None and (rot is None or abs(rot[2]) < 0.5):
                wings.append((s["span"] * s["chord"], n["translation"][2], s))
        if not wings:
            return
        main_z = max(wings, key=lambda w: w[0])[1]
        for area, z, s in wings:
            if z < main_z - 0.25:
                s["downwash"] = 0.45
                # tail surfaces stall late, and a full control throw
                # must stay effective (the plane3d lesson)
                s["stallAngle"] = max(s["stallAngle"], 35.0)

    def wing_sides(self, c):
        '''(suffix, root x, outward direction) per built side of a wing.
        RealFlight authors the master half on the right (CS_R frames),
        so the left half mirrors both the root and the panel direction.'''
        if c.mirror_chain():
            return [("right", c.pos[0], -1), ("left", -c.pos[0], 1)]
        return [("", c.pos[0], 1)]

    def map_wing(self, c):
        g = c.group
        span = g.get("SpanLengthMTR", 0.0) or g.get("LengthMTR", 0.0)
        if span <= 0.01:
            return
        c_root = g.get("ChordAtRootMTR", g.get("LengthMTR", 0.2))
        c_tip = g.get("ChordAtTipMTR", c_root)
        mirrored = c.mirrored()
        full_span = span * (2 if mirrored else 1)
        area = full_span * (c_root + c_tip) / 2
        ar = full_span * full_span / area if area > 0 else 6.0
        camber = (naca_cl0(g.get("AirfoilAtRoot")) + naca_cl0(g.get("AirfoilAtTip"))) / 2
        # thin-aerofoil camber understates a real wing's lift at working
        # Reynolds numbers, and flying showed these wings short of lift
        cl0 = min(1.5 * camber, 0.45)
        # the nose down moment that camber comes with: without it a
        # cambered wing trims nose up until it stalls
        cm0 = round(-0.25 * camber, 4)
        self.main_chord = max(self.main_chord, (c_root + c_tip) / 2)
        incidence = g.get("WingIncedenceDEG", 0.0)
        # the composed roll about the forward axis: 90 stands a fin
        # upright, and a V-tail panel inherits its boom's roll
        roll = c.roll
        flap = None
        for cs in g.children:
            if cs.get("ControlSurfaceID") is not None:
                flap = cs
                break
        # a cambered wing works to a higher angle than a plain plate; a
        # fin is a low-aspect surface whose full rudder throw must stay
        # effective, so it stalls late like the hand-made models' tails
        stall = (18.0 if cl0 > 0.05 else 16.0) + 6.0 * g.get("PostStallLiftFactor", 1.0)
        if abs(roll) > 45:
            stall = max(stall, 35.0)

        for suffix, root_x, sign in self.wing_sides(c):
            panels = [(0.0, span, False)]
            servo = None
            if flap is not None:
                flen = min(flap.get("LengthMTR", span), span)
                tip_gap = min(flap.get("DistanceToTipMTR", 0.0), span - flen)
                inner = span - flen - tip_gap
                panels = [p for p in [(0.0, inner, False),
                                      (inner, inner + flen, True),
                                      (inner + flen, span, False)] if p[1] - p[0] > 0.01]
                key = "ServoMaster" if sign < 0 or suffix == "" else "ServoSlave"
                servo = self.electronics.channel(flap.get(key))
                reverse = flap.get(key + "Rev", False)
            for lo, hi, flapped in panels:
                mid = (lo + hi) / 2
                frac = mid / span
                chord = c_root + (c_tip - c_root) * frac
                params = {"span": round(hi - lo, 4), "chord": round(chord, 4),
                          "aspectRatio": round(ar, 2), "cl0": cl0, "cm0": cm0, "cd0": 0.015,
                          "stallAngle": round(stall, 1), "oswald": g.get("WingEffec", 0.85)}
                if flapped and servo:
                    # RealFlight's rig absorbs an ArduPilot channel reversal,
                    # so a SERVOn_REVERSED in the model's .parm is cancelled
                    # here; measured both ways on the Griffin's elevator
                    down = flap.get("DeflectionDownDEG", -20.0)
                    up = flap.get("DeflectionUpDEG", 20.0)
                    # SITL has already applied SERVOn_REVERSED to the PWM, and
                    # RealFlight rigged this surface against those same
                    # parameters, so the file's own sense is what is needed
                    lo_a, hi_a = (up, down) if reverse else (down, up)
                    chord_frac = (flap.get("PercentOfChordTowardsRoot", 0.25) +
                                  flap.get("PercentOfChordTowardsTip", 0.25)) / 2
                    params.update({"flapServo": servo, "flapAngleMin": round(lo_a, 2),
                                   "flapAngleMax": round(hi_a, 2),
                                   "flapChordFraction": round(chord_frac, 3)})
                # lateral offset from the side's root, rolled about the
                # forward axis for a fin
                a = math.radians(roll)
                off = [sign * mid * math.cos(a), mid * math.sin(a), 0.0]
                pos = [root_x + off[0], c.pos[1] + off[1], c.pos[2] + off[2]]
                rot = None
                if roll or incidence:
                    rot = _compose(qaxis(Z, roll * (1 if sign > 0 else -1)),
                                   qaxis(X, -incidence))
                nm = ("%s_%s" % (c.name, suffix)).strip("_").replace(" ", "_").lower()
                self.part("%s_%d" % (nm, int(mid * 100)), "surface", params, pos, rot)
            # a thin box so the wing can touch things
            a = math.radians(roll)
            centre = [root_x + sign * span / 2 * math.cos(a),
                      c.pos[1] + span / 2 * math.sin(a),
                      c.pos[2]]
            size = [span * abs(math.cos(a)) + 0.02,
                    span * abs(math.sin(a)) + 0.02, (c_root + c_tip) / 2]
            nm_box = ("%s_box%s" % (c.name, "_" + suffix if suffix else "")).replace(" ", "_").lower()
            self.collider(nm_box, size, centre)
        # the wing carries through the fuselage: a mirrored horizontal
        # wing rooted off-centre still lifts across the gap
        if mirrored and abs(roll) < 10 and abs(c.pos[0]) > 0.02:
            params = {"span": round(2 * abs(c.pos[0]), 4), "chord": round(c_root, 4),
                      "aspectRatio": round(ar, 2), "cl0": cl0, "cm0": cm0, "cd0": 0.015,
                      "stallAngle": round(stall, 1), "oswald": g.get("WingEffec", 0.85)}
            nm = ("%s_centre" % c.name).replace(" ", "_").lower()
            self.part(nm, "surface", params, [0.0, c.pos[1], c.pos[2]],
                      qaxis(X, -incidence) if incidence else None)
        self.say("wing %-16s span %.2fm chord %.2f/%.2f servo %s" %
                 (c.name, full_span, c_root, c_tip,
                  servo if flap is not None else "none"))

    def prop_sides(self, c):
        '''(suffix, lateral sign) per built instance of a prop'''
        if c.mirrored() or (c.parent is not None and c.parent.mirrored()):
            return [("", 1), ("_r", -1)]
        return [("", 1)]

    def prop_thrust(self, c, total_mass):
        '''maximum thrust estimate: lifting props carry the vehicle at
        half throttle; a forward prop pulls 1.5 of the weight (these are
        aerobatic models), split between the live engines'''
        lifters = pullers = 0
        for p in self.active(("PropellerComponent",)):
            n = len(self.prop_sides(p))
            if abs(p.tilt[0]) > 40:
                lifters += n
            else:
                pullers += n
        if abs(c.tilt[0]) > 40:
            return 2.0 * total_mass * GRAVITY / max(1, lifters)
        # these are aerobatic models: thrust around twice the weight,
        # or the takeoff roll plateaus just below rotation speed
        return 2.2 * total_mass * GRAVITY / max(1, pullers)

    def map_prop(self, c):
        g = c.group
        d = g.get("PropDiameterMTR", 0.25)
        motor = self.motor_for(c)
        area = math.pi * d * d / 4
        if motor is not None:
            thrust, pitch_speed = self.motor_thrust(c, motor)
            pitch_speed = round(pitch_speed, 1)
            v_exit = math.sqrt(2 * thrust / (AIR_DENSITY * area))
        else:
            thrust = self.prop_thrust(c, self.total_mass())
            v_exit = math.sqrt(2 * thrust / (AIR_DENSITY * area))
            # a forward prop is pitched for speed; a lifter for hover
            pitch_speed = round((1.8 if abs(c.tilt[0]) <= 40 else 1.3) * v_exit, 1)
        # torque from the induced power at the estimated thrust
        omega = pitch_speed / max(g.get("PropPitchMTR", 0.15), 0.05) * 2 * math.pi
        torque_ratio = round(v_exit / 2 / omega, 4)
        tilt = c.ancestor_tilt_servo()
        for suffix, sign in self.prop_sides(c):
            # "clockwise from the rear" is counter-clockwise seen facing
            # the thrust, which is the backend's convention
            key = "ClockSpinsClockwiseFromRear" + ("Slave" if sign < 0 else "")
            cw = not g.get(key, False)
            skey = "ServoThrottle" + ("Slave" if sign < 0 else "")
            servo = self.electronics.channel(g.get(skey) or g.get("ServoThrottle"))
            params = {"servo": servo, "maxThrust": round(thrust, 1), "thrustExpo": 0.5,
                      "torqueRatio": torque_ratio, "direction": "cw" if cw else "ccw",
                      "timeConstant": 0.05, "diameter": round(d, 3),
                      "pitchSpeed": pitch_speed}
            pos = [sign * c.pos[0], c.pos[1], c.pos[2]]
            # the motor mass sits at the prop
            self.add_mass(self.motor_mass(c), pos)
            nm = (c.name + suffix).replace(" ", "_").lower()
            if tilt is not None:
                # the pod turns the motor between forward and up: ArduPilot's
                # tilt output spans vertical to horizontal, whatever the range
                # in RealFlight, and the model's .parm has the servo direction
                m_sid, s_sid, lo, hi = tilt
                tservo = self.electronics.channel(s_sid if sign < 0 else m_sid)
                rotor = self.node({"name": nm + "_disc", "translation": [0, 0, 0],
                                   "rotation": qaxis(X, 90),
                                   "extensions": {"ARDUPILOT_sitl": {"rotor": params}}})
                # the servo throw spans the pod's full mechanical travel (the
                # Griffin's 110 degrees), measured against the hover value
                # ArduPilot outputs; the rig absorbs a .parm reversal: cancel it
                t_lo, t_hi = -hi, -lo
                if self.parm_reversed(tservo):
                    t_lo, t_hi = t_hi, t_lo
                self.part(nm + "_tilt", "hinge",
                          {"servo": tservo, "angleMin": t_lo, "angleMax": t_hi,
                           "maxRate": 90}, pos)
                self.parts[-1]["children"] = [rotor]
                self.say("prop %-16s servo %s tilt servo %s %g..%g deg d=%.2fm "
                         "thrust~%.1fN %s" % (c.name + suffix, servo, tservo,
                                              lo, hi, d, thrust, "cw" if cw else "ccw"))
                continue
            if abs(c.tilt[0]) > 40:
                rot = None  # the rotor part's default thrust is up
            else:
                rot = qaxis(X, 90 - g.get("DownThrustDEG", 0.0))
            self.part(nm, "rotor", params, pos, rot)
            self.say("prop %-16s servo %s d=%.2fm thrust~%.1fN pitchSpeed~%.0fm/s %s" %
                     (c.name + suffix, servo, d, thrust, pitch_speed,
                      "cw" if cw else "ccw"))

    def map_turbine(self, c):
        g = c.group
        servo = self.electronics.channel(g.get("ServoThrottle"))
        params = {"servo": servo, "maxThrust": round(g.get("MaxThrustN", 20.0), 1),
                  "thrustExpo": 0.0, "torqueRatio": 0.0, "direction": "cw",
                  "timeConstant": 0.3, "diameter": 0.1, "pitchSpeed": 0.0}
        self.part("turbine", "rotor", params, c.pos, qaxis(X, 90))
        self.say("turbine servo %s thrust %.1fN" % (servo, g.get("MaxThrustN", 20.0)))

    def map_wheel(self, c):
        gear = c.parent.group if c.parent else c.group
        length = gear.get("GearLengthMTR", 0.1)
        radius = min(max(0.25 * length, 0.02), 0.09)
        m = self.total_mass()
        spring = m * GRAVITY / 0.02 / 3
        params = {"radius": round(radius, 3), "travel": 0.04,
                  # near-critical damping: an underdamped settle can rock
                  # a taildragger over its nose
                  "spring": round(spring, 0), "damping": round(1.0 * math.sqrt(spring * m), 1),
                  "grip": 0.8, "rollingResistance": 0.03}
        steer = self.electronics.channel(gear.get("ServoSteer")) if gear else None
        nm = c.name.replace(" ", "_").lower()
        # a mirrored gear leg carries a wheel on each side; the legs
        # splay outwards by the gear-down roll, which sets the track
        splay = math.radians(abs(gear.get("RotationAboutYGearDown", 0.0)))
        lat = math.sin(splay) * length
        lift_y = (1.0 - math.cos(splay)) * length
        sides = [1, -1] if gear.get("SymetricalAboutAxis", 0) == 2 else [1]
        for sign in sides:
            pos = [sign * (abs(c.pos[0]) + lat), c.pos[1] + lift_y, c.pos[2]]
            suffix = {1: "", -1: "_r"}[sign] if len(sides) > 1 else ""
            if steer:
                hinge = self.node({"name": nm + suffix + "_tyre", "translation": [0, 0, 0],
                                   "extensions": {"ARDUPILOT_sitl": {"wheel": params}}})
                self.part(nm + suffix + "_steer", "hinge",
                          {"servo": steer, "angleMin": 30, "angleMax": -30, "maxRate": 200}, pos)
                self.parts[-1]["children"] = [hinge]
                self.parts[-1]["rotation"] = qaxis(Z, 90)
            else:
                self.part(nm + suffix, "wheel", params, pos)
            self.say("wheel %-16s r=%.3fm at %s%s" %
                     (c.name + suffix, radius, [round(v, 2) for v in pos],
                      " steering" if steer else ""))

    def map_fuselage(self, c):
        g = c.group
        dim = g.get("Dimension", g.get("FuseDimensionMTR", [0.1, 0.8, 0.12]))
        size = [abs(dim[0]), abs(dim[2]), abs(dim[1])]
        nm = c.name.replace(" ", "_").lower()
        self.collider(nm + "_box", size, c.pos)
        if nm.startswith("fuselage"):
            self.body_nose = max(self.body_nose, c.pos[2] + 0.5 * size[2])

    def map_skid(self, c):
        m = self.total_mass()
        spring = m * GRAVITY / 0.02 / 2
        params = {"radius": 0.03, "travel": 0.03, "spring": round(spring, 0),
                  "damping": round(0.4 * math.sqrt(spring * m), 1), "grip": 0.7,
                  "rollingResistance": 0.4}
        for i, dz in enumerate((0.12, -0.12)):
            self.part("skid_%d" % i, "wheel", params,
                      [c.pos[0], c.pos[1], c.pos[2] + dz])
        self.say("skids at %s" % [round(v, 2) for v in c.pos])

    def map_main_rotor(self, c):
        g = c.group
        blade = re.search(r"(\d{3,4})\s*mm", g.get("RotorBladeType", "") or "")
        radius = (float(blade.group(1)) / 1000 if blade else 0.7) + \
            g.get("HubRadiusToLeadLagMTR", 0.07)
        mech = c.parent.group if c.parent else g
        rpm = mech.get("FullyEngagedRPM", 9000.0) / max(g.get("EngineToRotorRatio", 9.0), 1.0)
        m = self.total_mass()
        self.heli_thrust_scale = round(2.2 * m * GRAVITY / 9.0, 2)
        # RealFlight counts a clockwise head, seen from above, as zero
        spin = "cw" if g.get("RotorDirection", 0) == 0 else "ccw"
        params = {
            "swashServo1": 1, "swashServo2": 2, "swashServo3": 3,
            "direction": spin,
            "rscServo": self.electronics.channel(mech.get("ServoThrottle")) or 8,
            "collectiveMin": g.get("MinCollectiveDEG", -12.0),
            "collectiveMax": g.get("MaxCollectiveDEG", 12.0),
            "cyclicScale": g.get("PitchDeflectionDEG", 10.0),
            "nominalRpm": round(rpm, 0),
            "thrustScale": self.heli_thrust_scale,
            "torqueRatio": 0.035, "diameter": round(2 * radius, 3),
        }
        self.part("main_rotor", "heliRotor", params, c.pos)
        self.say("main rotor d=%.2fm rpm~%.0f collective %+.0f..%+.0f" %
                 (2 * radius, rpm, params["collectiveMin"], params["collectiveMax"]))

    def map_tail_rotor(self, c):
        g = c.group
        d = 2 * (g.get("RotorRadiusMTR", 0.14) or 0.14)
        servo = self.electronics.channel(g.get("PitchConnection")) or 4
        m = self.total_mass()
        # the tail holds the head's reaction torque, which at a hover is
        # the torque ratio against thrust and profile drag; it needs
        # several times that to steer the yaw as the collective moves
        arm = max(abs(c.pos[2]), 0.3)
        reaction = 0.035 * (m * GRAVITY + 1.5 * self.heli_thrust_scale)
        thrust = 3.5 * reaction / arm
        params = {"servo": servo, "maxThrust": round(thrust, 1), "thrustExpo": 0.0,
                  "torqueRatio": 0.0, "direction": "cw", "timeConstant": 0.03,
                  "diameter": round(d, 3), "pitchSpeed": 0.0,
                  "bidirectional": True}
        # the disc faces across the hull, pushing against the head's turn
        self.part("tail_rotor", "rotor", params, c.pos, qaxis(Z, -90))
        self.say("tail rotor servo %s %.1f N at %s" % (servo, thrust, [round(v, 2) for v in c.pos]))

    # ---- output --------------------------------------------------------

    def finish(self, out_path, name):
        mass = self.total_mass()
        com = [0.0, 0.0, 0.0]
        for m, pos, _ in self.masses:
            for i in range(3):
                com[i] += m * pos[i] / mass
        # the component weights leave out what RealFlight keeps in its
        # database, so the summed centre of mass drifts; a vehicle with three
        # or more lifting rotors balances on their centroid by construction
        lift = [n["translation"] for n in self.parts
                if "rotor" in n.get("extensions", {}).get("ARDUPILOT_sitl", {})
                and n.get("rotation") is None]
        lift += [n["translation"] for n in self.parts
                 if "hinge" in n.get("extensions", {}).get("ARDUPILOT_sitl", {})
                 and n["name"].endswith("_tilt")]
        np_z = self.neutral_point()
        if len(lift) >= 3:
            com[0] = sum(p[0] for p in lift) / len(lift)
            com[2] = sum(p[2] for p in lift) / len(lift)
            if np_z is not None:
                # far enough ahead of the neutral point to be stable, close
                # enough to the lifters that tilting the front motors forward
                # does not drop the nose faster than the elevator can catch
                com[2] = np_z + 0.04 * self.main_chord
        elif np_z is not None and self.main_chord > 0:
            # a plane's summed weights leave out too much to balance on,
            # and a nose motor estimate tips it onto the propeller; put the
            # centre of mass where the aerodynamics say it belongs
            com[2] = np_z + 0.10 * self.main_chord
        # move every node so the centre of mass is the origin
        for n in self.parts + [n for n in self.nodes[1:] if n]:
            if "translation" in n:
                n["translation"] = [round(n["translation"][i] - com[i], 5) for i in range(3)]
        inertia = [0.0, 0.0, 0.0]
        for m, pos, own in self.masses:
            d = [pos[i] - com[i] for i in range(3)]
            inertia[0] += m * (d[1] * d[1] + d[2] * d[2]) + own[0]
            inertia[1] += m * (d[0] * d[0] + d[2] * d[2]) + own[1]
            inertia[2] += m * (d[0] * d[0] + d[1] * d[1]) + own[2]
        inertia = [round(max(v, 0.01) * 1.15, 4) for v in inertia]

        # straddle the structure: the fuselage's anchor in the file is
        # not the middle of its box.  Propellers are left out, since a
        # tractor propeller is then placed from this very box
        zs = [n["translation"][2] for n in self.parts
              if "translation" in n
              and "rotor" not in n.get("extensions", {}).get("ARDUPILOT_sitl", {})]
        body = None
        for n in self.parts:
            if n["name"].startswith("fuselage_box"):
                if zs:
                    n["translation"][2] = round((max(zs) + min(zs)) / 2, 4)
                body = n
                break

        # a tractor propeller turns ahead of the cowling, not inside it
        if body is not None:
            kh = body["extensions"]["KHR_physics_rigid_bodies"]["collider"]
            nose = body["translation"][2] + \
                0.5 * self.shapes[kh["geometry"]["shape"]]["box"]["size"][2]
            for n in self.parts:
                ap = n.get("extensions", {}).get("ARDUPILOT_sitl", {})
                r = ap.get("rotor")
                if r is None or n.get("rotation") is None:
                    continue
                t = n["translation"]
                if abs(t[0]) < 0.05 and t[2] < nose and t[2] > -nose:
                    t[2] = round(nose + 0.5 * r["diameter"] * 0.12, 4)

        # a taildragger needs its mains well ahead of the centre of
        # mass, or a nudge at the wrong moment tips it onto the nose
        for n in self.parts:
            ap = n.get("extensions", {}).get("ARDUPILOT_sitl", {})
            if "wheel" in ap and n["translation"][2] > 0.0:
                n["translation"][2] = round(n["translation"][2] + 0.03, 5)

        # wheel ground contacts: a steering hinge carries its tyre at
        # its own origin
        contacts = []
        for n in self.parts:
            ap = n.get("extensions", {}).get("ARDUPILOT_sitl", {})
            if "wheel" in ap:
                contacts.append((n["translation"][2],
                                 n["translation"][1] - ap["wheel"]["radius"]))
            elif "hinge" in ap and n["name"].endswith("_steer"):
                for ci in n.get("children", []):
                    w = self.nodes[ci].get("extensions", {}).get("ARDUPILOT_sitl", {}).get("wheel")
                    if w:
                        contacts.append((n["translation"][2],
                                         n["translation"][1] - w["radius"]))
        # spawn in the resting stance: a taildragger pitches until its
        # tailwheel touches too, instead of flopping back and rolling
        # away from the startup location
        stance = 0.0
        if len(contacts) >= 2:
            front = max(contacts)
            back = min(contacts)
            if abs(front[0] - back[0]) > 0.05:
                stance = math.atan2(back[1] - front[1], front[0] - back[0])

        def pitched(z, y):
            return (y * math.cos(stance) + z * math.sin(stance))

        low = 0.0
        for n in self.parts:
            ap = n.get("extensions", {}).get("ARDUPILOT_sitl", {})
            kh = n.get("extensions", {}).get("KHR_physics_rigid_bodies", {})
            y = n["translation"][1]
            z = n["translation"][2]
            if "wheel" in ap:
                low = min(low, pitched(z, y - ap["wheel"]["radius"]) - 0.01)
            elif "hinge" in ap and n["name"].endswith("_steer"):
                low = min(low, pitched(z, y - 0.04) - 0.01)
            elif "collider" in kh:
                size = self.shapes[kh["collider"]["geometry"]["shape"]]["box"]["size"]
                low = min(low, pitched(z, y - size[1] / 2))
            elif "hinge" not in ap:
                low = min(low, pitched(z, y))

        children = []
        for p in self.parts:
            children.append(self.node(p))
        self.nodes[0] = {
            "name": name, "translation": [0, round(-low + 0.01, 3), 0],
            "children": children,
        }
        if abs(stance) > 0.01:
            self.nodes[0]["rotation"] = qaxis(X, -math.degrees(stance))
            self.say("resting stance %.1f deg nose up" % math.degrees(stance))
        self.nodes[0].update({
            "extensions": {
                "KHR_physics_rigid_bodies": {"motion": {
                    "mass": round(mass, 3),
                    "inertiaDiagonal": inertia,
                    "centerOfMass": [0, 0, 0],
                }},
                "ARDUPILOT_sitl": {"drag": {"area": [0.03, 0.03, 0.02]},
                                   "chassis": self.chassis()},
            },
        })
        gltf = {
            "asset": {"version": "2.0", "generator": "rfx2gltf"},
            "extensionsUsed": ["KHR_physics_rigid_bodies", "KHR_implicit_shapes", "ARDUPILOT_sitl"],
            "extensions": {
                "KHR_implicit_shapes": {"shapes": self.shapes},
                "KHR_physics_rigid_bodies": {"physicsMaterials": [
                    {"staticFriction": 0.6, "dynamicFriction": 0.5, "restitution": 0.1},
                ]},
                "ARDUPILOT_sitl": {"vehicle": 0},
            },
            "scene": 0,
            "scenes": [{"nodes": [0]}],
            "nodes": self.nodes,
        }
        with open(out_path, "w") as f:
            f.write(pretty(gltf) + "\n")
        self.say("mass %.3fkg  CoM %s  inertia %s" %
                 (mass, [round(v, 3) for v in com], inertia))
        self.say("wrote %s" % out_path)


def _compose(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [round(w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, 8),
            round(w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, 8),
            round(w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, 8),
            round(w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, 8)]


def convert(sections, src, out):
    name = os.path.splitext(os.path.basename(out or src))[0].lower().replace(" ", "_")
    if out is None:
        out = os.path.join(os.path.dirname(os.path.abspath(src)), name + ".gltf")
    cell_weight = 0.05
    motors = {}
    if src.lower().endswith(".rfx"):
        with zipfile.ZipFile(src) as z:
            for n in z.namelist():
                low = n.lower()
                if low.endswith(".battery"):
                    text = io.TextIOWrapper(z.open(n), errors="replace").read()
                    m = re.search(r"CellWeightOZ=(?:FLOAT:)?([0-9.]+)", text)
                    if m:
                        cell_weight = float(m.group(1)) * 0.02835
                elif low.endswith(".electrictorque"):
                    text = io.TextIOWrapper(z.open(n), errors="replace").read()
                    fields = dict(re.findall(r"(\w+)=(?:FLOAT|BOOL|STRING):([^\r\n]*)", text))
                    if "KV" in fields:
                        motors[os.path.splitext(os.path.basename(n))[0]] = {
                            "kv": float(fields["KV"]),
                            "weight": float(fields.get("WeightOZ", 3.5)) * 0.02835,
                        }
    reversals = set()
    parm = os.path.join(os.path.dirname(os.path.abspath(src)), "model.parm")
    if not os.path.exists(parm) and out:
        parm = os.path.splitext(out)[0] + ".parm"
    if os.path.exists(parm):
        with open(parm, errors="replace") as f:
            for mm in re.finditer(r"^SERVO(\d+)_REVERSED\s+1", f.read(), re.M):
                reversals.add(int(mm.group(1)))
    report = []
    cv = Converter(sections, report, cell_weight, motors, reversals)
    cv.say("parm %s reverses %s" % (os.path.basename(parm) if os.path.exists(parm) else "(none)",
                                    sorted(reversals) or "nothing"))
    cv.convert()
    cv.finish(out, name)
    for line in report:
        print(line)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        print(__doc__)
        sys.exit(1)
    sections = load_rfx(args[0])
    if "--dump" in sys.argv:
        for name in sections:
            dump(sections[name])
        return
    convert(sections, args[0], args[1] if len(args) > 1 else None)


if __name__ == "__main__":
    main()

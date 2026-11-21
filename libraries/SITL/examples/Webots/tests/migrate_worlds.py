#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Maintenance tool for the Webots worlds: inspect them, recalibrate their rotors,
and migrate worlds saved by old Webots releases.  It never changes a file
without showing what will change and asking first.

    migrate_worlds.py                    interactive menu
    migrate_worlds.py status             what each world is: version, mass, rotors
    migrate_worlds.py calibrate [WORLD]  recompute rotor constants for the mass
                                         the world has now -- nothing else
    migrate_worlds.py migrate [WORLD]    convert a world saved by Webots older
                                         than R2025a -- nothing else

WORLD is a file in ../worlds or a path; leave it out to be asked.  Add --check
to see what would change without writing, and --yes to write without asking
(for scripts).

calibrate
    Edit a vehicle's mass in Webots, then run this.  The vehicle's mass is
    summed from the world (body plus every rigidly attached Solid), and for the
    thrust-to-weight you choose:

        thrust at omega_max       T/W * mass * g / rotor_count
        thrustConstants[0]        thrust / omega_max^2
        torqueConstants[0]        thrustConstants[0] * torque_ratio

    which puts hover at 1/(T/W) throttle.  omega_max and the torque/thrust
    ratio default to what the world already has, so e.g. the tricopter's
    deliberately negligible propeller reaction torque is kept.  The controllers
    command omega = sqrt(u) * maxVelocity, so maxVelocity sets the thrust scale.

migrate
    Which conversions a world needs follows from its "#VRML_SIM Rxxxx" header,
    which Webots rewrites on every save -- not from comments or other marks in
    the file, which a save through the Webots GUI silently drops.  A world
    already at R2025a is left alone.  For older worlds:
      - saved before R2022a (RUB-authored, y-up NUE worlds): gravity vector to
        scalar + coordinateSystem, the RUB->FLU compensation of built-in
        geometries and devices that Webots only applies itself when loading
        such an old file, ENU-authored library protos rotated upright, and
        UnevenTerrain laid flat
      - saved before R2023b: Transform renamed to Pose
      - any older world: EXTERNPROTO declarations, explicit inertia for Solids
        that would get the identity tensor, and small R2025a proto fixes
    then the header is set to R2025a.  Migration does not touch rotor
    constants: run calibrate afterwards if the vehicle needs it.
"""

import argparse
import difflib
import functools
import math
import os
import re
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.join(HERE, '..', 'worlds')

TARGET_VERSION = (2025, 'a')
TARGET_HEADER = '#VRML_SIM R2025a utf8'
G = 9.80665
OMEGA_MAX = 400.0          # rad/s, default for worlds that do not set one
# A Propeller's shaft slews towards its commanded speed at a constant
# maxTorque rad/s^2, as if it had a unit moment of inertia (measured in Webots
# R2025a from the thrust on a free body: maxTorque 90, 1000 and 5000 give 90,
# 1000 and 5000 rad/s^2).  The helix Solids are purely graphical, so giving
# them a Physics node does not change it.  maxTorque therefore has no physical
# meaning in this model -- it only sets how fast the rotor reaches its
# commanded speed, and it has to be large if the rotor is to run at a realistic
# speed: with maxTorque 90 a quad takes about 3 s to spool up to hover, with
# 5000 about 0.05 s (0.08 s to 400 rad/s).
#
# This is what the original worlds were really working around: at omega_max 3.6
# rad/s (about 34 rpm) even maxTorque 90 reaches full speed within 0.04 s.
MAX_TORQUE = 5000.0
FAST_HELIX_THRESHOLD = 50.0

# thrust-to-weight offered by default when recalibrating: hover near 45% throttle
DEFAULT_TW = 2.2
# torqueConstants / thrustConstants, metres.  Real propellers sit around
# 0.01-0.05 m; the 2021 worlds shipped 1.1, which is offered back only if sane.
DEFAULT_TORQUE_RATIO = 0.02
MAX_TORQUE_RATIO = 0.1

# lift-rotor motors, by name.  Anything else (the tricopter's servo_tail) is
# left alone.
LIFT_MOTORS = re.compile(r'name\s+"motor\d+"')


# PROTO nodes are no longer resolved implicitly.  R2025a reports
# "Missing declaration for 'X'" and then *skips the node*, so a world without
# these declarations silently loses its scenery (and its terrain).  URLs are
# exactly the ones Webots itself suggests.
PROTO_URLS = {
    'AdvertisingBoard': 'projects/objects/advertising_board/protos/AdvertisingBoard.proto',
    'TexturedBackground': 'projects/objects/backgrounds/protos/TexturedBackground.proto',
    'TexturedBackgroundLight': 'projects/objects/backgrounds/protos/TexturedBackgroundLight.proto',
    'HouseWithGarage': 'projects/objects/buildings/protos/HouseWithGarage.proto',
    'SimpleBuilding': 'projects/objects/buildings/protos/SimpleBuilding.proto',
    'Floor': 'projects/objects/floors/protos/Floor.proto',
    'UnevenTerrain': 'projects/objects/floors/protos/UnevenTerrain.proto',
    'DogHouse': 'projects/objects/garden/protos/DogHouse.proto',
    'Crossroad': 'projects/objects/road/protos/Crossroad.proto',
    'Road': 'projects/objects/road/protos/Road.proto',
    'Car': 'projects/vehicles/protos/abstract/Car.proto',
    'BmwX5': 'projects/vehicles/protos/bmw/BmwX5.proto',
    'VehicleLights': 'projects/vehicles/protos/abstract/VehicleLights.proto',
    'VehicleWheel': 'projects/vehicles/protos/abstract/VehicleWheel.proto',
}
PROTO_BASE = 'https://raw.githubusercontent.com/cyberbotics/webots/R2025a/'

# A Solid with a Physics node but neither boundingObject nor inertiaMatrix gets
# the *identity* inertia tensor, i.e. 1 kg m^2 about every axis.  The four
# rotor nacelles in each quad world are rigidly attached to the airframe, so
# that put the vehicle's rotational inertia ~600x too high -- which is what the
# old hand-tuned ATC_RAT_P 3.5 was really compensating for.  Treat each as a
# compact body of this radius.
NACELLE_RADIUS = 0.04


class WorldError(Exception):
    """A world file this tool cannot read, e.g. with unbalanced braces."""


class SettingError(WorldError):
    """A calibration setting this tool cannot use, e.g. a zero thrust-to-weight."""


def unreadable(path, err):
    what = 'calibrate' if isinstance(err, SettingError) else 'read'
    print('==> %s\n    cannot %s this world: %s; nothing done'
          % (os.path.basename(path), what, err))


@functools.lru_cache(maxsize=16)
def code_only(text):
    """text with every comment and string blanked to spaces, keeping its length
    and newlines, so a brace or '#' there is not taken for world structure."""
    out = list(text)
    i, n = 0, len(text)
    while i < n:
        ch = text[i]
        if ch == '#':
            while i < n and text[i] != '\n':
                out[i] = ' '
                i += 1
            continue
        if ch == '"':
            out[i] = ' '
            i += 1
            while i < n and text[i] != '"':
                if text[i] == '\\' and i + 1 < n:
                    out[i] = ' '
                    i += 1
                if text[i] != '\n':
                    out[i] = ' '
                i += 1
            if i < n:
                out[i] = ' '
            i += 1
            continue
        i += 1
    return ''.join(out)


def direct_text(block):
    """The text of a node block at brace depth 1, with nested blocks elided."""
    out = []
    depth = 0
    for ch in block:
        if ch == '{':
            depth += 1
            if depth <= 1:
                out.append(ch)
            continue
        if ch == '}':
            if depth <= 1:
                out.append(ch)
            depth -= 1
            continue
        if depth <= 1:
            out.append(ch)
    return ''.join(out)


def find_blocks(text, node_re):
    """Yield (start, end) for each matching node block, outermost first; a node
    inside a comment or string, e.g. a commented-out one, is not a node."""
    code = code_only(text)
    for m in re.finditer(node_re, code):
        brace = code.index('{', m.start())
        yield m.start(), block_end(text, brace)


# R2025a's SimpleBuilding only accepts a wallColor on these wall types; any
# other type with a colour set is a hard error and the building loses its
# colour.  (The proto's own message lists no alternatives, because it iterates
# `coloredWallTypes.length` over an object -- an upstream cosmetic bug.)
COLORABLE_WALL_TYPES = {
    'old house', 'brick building', 'factory building', 'tall house',
    'office building', 'concrete building', 'transparent highrise',
}
# neutral replacement that does accept a colour, so the OSM-imported buildings
# (the Giza pyramids among them) keep the colour they were authored with
FALLBACK_WALL_TYPE = 'concrete building'


def fix_background(text, report):
    """Background.cubemap was dropped from the node; an empty one is just noise."""
    new, n = re.subn(r'\n\s*cubemap Cubemap \{\s*\n\s*\}', '', text)
    if n:
        report.append('    %d empty Background.cubemap removed' % n)
    return new


def fix_vehicle_wheels(text, report):
    """
    R2025a's VehicleWheel validates rimRadius against tireRadius/thickness and
    rimBeamOffset against thickness/rimBeamThickness.  The rover's front wheels
    (tireRadius 0.3, thickness 0.2) fail both with the default rim, so Webots
    resets them and logs an error every load.  Write the values it resets to,
    which changes nothing visually and silences the errors.
    """
    fixed = 0
    out = []
    pos = 0
    for start, end in find_blocks(text, r'\bVehicleWheel\s*\{'):
        block = text[start:end]
        if 'tireRadius' not in block or 'rimRadius' in block:
            continue
        anchor = re.search(r'^(\s*)tireRadius[^\n]*$', block, re.M)
        if not anchor:
            continue
        insert = anchor.end()
        block = (block[:insert]
                 + '\n%srimRadius 0.15\n%srimBeamOffset 0' % (anchor.group(1), anchor.group(1))
                 + block[insert:])
        out.append(text[pos:start])
        out.append(block)
        pos = end
        fixed += 1
    out.append(text[pos:])
    if fixed:
        report.append('    %d VehicleWheel rim dimensions made explicit' % fixed)
    return ''.join(out)


def fix_building_colors(text, report):
    fixed = 0
    out = []
    pos = 0
    for start, end in find_blocks(text, r'\bSimpleBuilding\s*\{'):
        block = text[start:end]
        if 'wallColor' not in block:
            continue
        wt = re.search(r'wallType\s+"([^"]*)"', block)
        current = wt.group(1) if wt else 'windowed building'
        if current in COLORABLE_WALL_TYPES:
            continue
        if wt:
            block = block[:wt.start()] + 'wallType "%s"' % FALLBACK_WALL_TYPE + block[wt.end():]
        else:
            anchor = re.search(r'^(\s*)wallColor', block, re.M)
            block = (block[:anchor.start()]
                     + '%swallType "%s"\n' % (anchor.group(1), FALLBACK_WALL_TYPE)
                     + block[anchor.start():])
        out.append(text[pos:start])
        out.append(block)
        pos = end
        fixed += 1
    out.append(text[pos:])
    if fixed:
        report.append('    %d SimpleBuilding wallType -> "%s" (kept their wallColor)'
                      % (fixed, FALLBACK_WALL_TYPE))
    return ''.join(out)


# The projects/objects protos are authored for Webots' ENU default while these
# worlds are NUE, and Webots does not convert EXTERNPROTO contents when the two
# disagree -- a z-up object in a y-up world just lies on its side.  Rotate each
# instance by the ENU->NUE frame change, composing with whatever rotation the
# instance already carried so authored yaws survive.
#   proto (E=x, N=y, U=z) -> world (N=x, U=y, E=z): -120 degrees about the
#   (1,1,1) body diagonal; quaternion (x, y, z, w) = (-0.5, -0.5, -0.5, 0.5).
# a VRML number: optional sign, decimals and exponent
NUMBER = r'([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)'

ENU_PROTO_ROTATION = (-0.5, -0.5, -0.5, 0.5)
# The composed rotation cannot be told apart from an authored one, so it is
# applied only to worlds whose #VRML_SIM header predates R2022a; migrate then
# stamps the header R2025a, which is what keeps a second pass from rotating
# them again.
ENU_PROTOS = ('AdvertisingBoard', 'Car', 'Crossroad', 'DogHouse', 'Floor',
              'HouseWithGarage', 'Road', 'SimpleBuilding')
# proto fields whose values are vectors in the proto's own frame: the shipped
# NUE numbers have to be re-expressed for the rotated proto or the geometry
# stands on edge again.  (a, b, c) -> (c, a, b) puts a flat (x, 0, z) NUE
# point at (z, x, 0), i.e. flat in the proto's XY ground plane with compass
# headings preserved; (a, b) -> (b, a) does the same for 2D fields.
ENU_VEC3_LISTS = {'Road': 'wayPoints', 'Crossroad': 'shape'}
ENU_VEC2_LISTS = {'SimpleBuilding': 'corners'}
ENU_VEC2_SCALARS = {'Floor': 'size'}


def quat_mul(a, b):
    """Hamilton product; (a * b) applies b's rotation first."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def axis_angle_to_quat(vals):
    s = math.sin(vals[3] / 2.0)
    return (vals[0] * s, vals[1] * s, vals[2] * s, math.cos(vals[3] / 2.0))


def quat_to_axis_angle(q):
    x, y, z, w = q
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    angle = 2.0 * math.acos(min(1.0, w))
    s = math.sin(angle / 2.0)
    if abs(s) < 1e-12:
        return '0 0 1 0'
    axis = [v / s for v in (x, y, z)]
    # the source angles are not exact (the shipped pi/2 is off by ~3e-7), so
    # snap dust to zero rather than print -7e-9 as an axis component
    axis = [0.0 if abs(v) < 1e-6 else v for v in axis]
    norm = math.sqrt(sum(v * v for v in axis))
    return '%.16g %.16g %.16g %.16g' % (
        axis[0] / norm, axis[1] / norm, axis[2] / norm, angle)


def _permute_vec_line(line, width):
    """Rotate each `width`-tuple on a list line right by one, preserving the
    line's indent and any trailing comma.  The tuples may be separated by
    spaces alone or by commas (`0 0 0, 2.7 0 29.9`), which is kept."""
    m = re.match(r'(\s*)(.*?)(\s*,?\s*)$', line, re.S)
    if ',' in m.group(2):
        groups = [g.split() for g in m.group(2).split(',')]
        if any(len(g) != width for g in groups):
            return line
        sep = ', '
    else:
        toks = m.group(2).split()
        if len(toks) < width or len(toks) % width:
            return line
        groups = [toks[i:i + width] for i in range(0, len(toks), width)]
        sep = ' '
    if not all(re.match(r'^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$', t)
               for g in groups for t in g):
        return line
    out = [' '.join([p[-1]] + p[:-1]) for p in groups]
    return '%s%s%s' % (m.group(1), sep.join(out), m.group(3))


def _permute_vec_list(block, field, width):
    m = re.search(re.escape(field) + r'\s*\[', block)
    if not m:
        return block, False
    close = block.index(']', m.end())
    inner = block[m.end():close]
    permuted = '\n'.join(_permute_vec_line(line, width)
                         for line in inner.split('\n'))
    return block[:m.end()] + permuted + block[close:], True


def fix_enu_protos(text, report):
    if 'coordinateSystem "NUE"' not in text:
        return text
    names = '|'.join(ENU_PROTOS)
    node_re = r'\b(?:DEF\s+\S+\s+)?(?:%s)\s*\{' % names
    out, pos, fixed = [], 0, 0
    for start, end in find_blocks(text, node_re):
        block = text[start:end]
        own_brace = block.index('{')
        nested = block.find('{', own_brace + 1)
        # the instance's own fields sit before its first nested node
        header = block if nested < 0 else block[:nested]
        rot = re.search(r'^(\s*)rotation\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s*$',
                        header, re.M)
        if rot:
            q = quat_mul(axis_angle_to_quat(
                [float(rot.group(i)) for i in (2, 3, 4, 5)]), ENU_PROTO_ROTATION)
            block = (block[:rot.start()]
                     + '%srotation %s'
                     % (rot.group(1), quat_to_axis_angle(q))
                     + block[rot.end():])
        else:
            anchor = re.search(r'^(\s*)translation\s+[^\n]*$', header, re.M)
            if anchor:
                at, indent = anchor.end(), anchor.group(1)
            else:
                indent = ' ' * (start - text.rindex('\n', 0, start) + 1)
                at = own_brace + 1
            block = (block[:at]
                     + '\n%srotation %s'
                     % (indent, quat_to_axis_angle(ENU_PROTO_ROTATION))
                     + block[at:])

        name = re.match(r'(?:DEF\s+\S+\s+)?(\w+)', block).group(1)
        if name in ENU_VEC3_LISTS:
            block, _ = _permute_vec_list(block, ENU_VEC3_LISTS[name], 3)
        elif name in ENU_VEC2_LISTS:
            block, _ = _permute_vec_list(block, ENU_VEC2_LISTS[name], 2)
        elif name in ENU_VEC2_SCALARS:
            block = re.sub(
                r'^(\s*)%s\s+(\S+)\s+(\S+)\s*$' % ENU_VEC2_SCALARS[name],
                r'\1%s \3 \2' % ENU_VEC2_SCALARS[name], block, count=1,
                flags=re.M)

        out.append(text[pos:start])
        out.append(block)
        pos = end
        fixed += 1
    out.append(text[pos:])
    if fixed:
        report.append('    %d ENU protos rotated upright for NUE' % fixed)
    return ''.join(out)


def fix_terrain(text, report):
    """
    UnevenTerrain is authored for Webots' ENU default: `size` is (x, y, height)
    and the grid lies in the XY plane.  These worlds are NUE, so the shipped
    `size 500 1 500` gave a terrain 1 m wide and 500 m tall, standing on edge --
    the vehicle simply fell past it and never touched ground.  Lay it flat and
    give the spawn area a flat, deterministic centre so a takeoff does not start
    on the side of a random dune.

    Only what the frame change needs is edited: the node is rotated (composed
    with any rotation it had), `size` goes from the old NUE (x, height, z) to
    (x, z, height), and the old `zDimension` becomes `yDimension`.  Its
    translation, name, appearance and any dimensions, seed or flatCenter it
    already has are kept; the grid dimensions, seed and flat centre are only
    added when the world does not set them.  Every UnevenTerrain is converted;
    this runs only on worlds older than R2022a, which is what keeps a second
    pass from converting them again.
    """
    blocks = list(find_blocks(text, r'\b(?:DEF\s+\S+\s+)?UnevenTerrain\s*\{'))
    # last first, so an edit never moves a block still to come
    for start, end in reversed(blocks):
        block = text[start:end]
        if '\n' not in block.strip():
            raise WorldError('an UnevenTerrain written on one line cannot be converted; '
                             'put its fields on their own lines')
        line_start = text.rindex('\n', 0, start) + 1 if '\n' in text[:start] else 0
        indent = re.match(r'[ \t]*', text[line_start:start]).group(0)
        field_indent = indent + '  '
        notes = []

        size = own_field(block, r'(?m)^(\s*)size\s+%s\s+%s\s+%s([ \t]*(?:#[^\n]*)?)$'
                         % ((NUMBER,) * 3))
        if size:
            x, height, z = size.group(2), size.group(3), size.group(4)
            block = (block[:size.start()]
                     + '%ssize %s %s %s%s' % (size.group(1), x, z, height, size.group(5))
                     + block[size.end():])
            notes.append('%sx%s, height %s' % (x, z, height))
        elif own_field(block, r'(?m)^\s*size\s'):
            raise WorldError('UnevenTerrain size is not three numbers on one line')

        zdim = own_field(block, r'(?m)^(\s*)zDimension(\s)')
        if zdim:
            block = (block[:zdim.start()] + zdim.group(1) + 'yDimension' + zdim.group(2)
                     + block[zdim.end():])

        rot = own_field(block, r'(?m)^(\s*)rotation\s+%s\s+%s\s+%s\s+%s([ \t]*(?:#[^\n]*)?)$'
                        % ((NUMBER,) * 4))
        if rot:
            q = quat_mul(axis_angle_to_quat([float(rot.group(i)) for i in (2, 3, 4, 5)]),
                         TERRAIN_ROTATION)
            block = (block[:rot.start()]
                     + '%srotation %s%s' % (rot.group(1), quat_to_axis_angle(q), rot.group(6))
                     + block[rot.end():])
        elif own_field(block, r'(?m)^\s*rotation\s'):
            raise WorldError('UnevenTerrain rotation is not four numbers on a line of its own')
        else:
            # right after its translation if it has one, else first
            anchor = own_field(block, r'(?m)^\s*translation\s[^\n]*$')
            at = anchor.end() if anchor else block.index('\n')
            block = (block[:at] + '\n%srotation 1 0 0 -1.5707963267948966' % field_indent
                     + block[at:])

        # defaults for what the world leaves unset, before the closing brace
        added = []
        for field, value in (('xDimension', '200'), ('yDimension', '200'),
                             ('randomSeed', '52'), ('flatCenter', 'TRUE')):
            if not own_field(block, r'(?m)^\s*%s\s' % field):
                added.append('%s%s %s\n' % (field_indent, field, value))
        close = code_only(block).rindex('}')
        block = block[:close].rstrip(' \t') + ''.join(added) + indent + block[close:]

        text = text[:start] + block + text[end:]
        report.append('    UnevenTerrain laid flat for NUE (%s)'
                      % (notes[0] if notes else 'default size'))
    return text


# ----------------------------------------------------------------------
# R2022a moved every built-in geometry and device from RUB (x-Right, y-Up,
# z-Back) to FLU (x-Forward, y-Left, z-Up).  Webots inserts the compensating
# rotations itself only while parsing a world whose #VRML_SIM version
# predates R2022a; with the R2025a header stamped above, the file is taken
# to be FLU-authored and nothing is rotated.  These worlds are still
# RUB-authored, so the compensations are written into the nodes here.
# Without them every Cylinder stands on edge -- the propeller disks go
# vertical while the thrust stays correct, since shaftAxis is an explicit
# vector -- and every Camera/Viewpoint looks along +X instead of -Z.
# Cylinder & co grew a +Z axis where they used to lie along +Y: -pi/2 about X
RUB_GEOM_QUAT = (-0.7071067811865476, 0.0, 0.0, 0.7071067811865476)
# UnevenTerrain's grid lies in its XY plane: -pi/2 about X lays it in the NUE
# world's XZ ground plane
TERRAIN_ROTATION = RUB_GEOM_QUAT
# Camera & co used to look down -Z with +Y up, now +X with +Z up:
# euler (-pi/2, 0, +pi/2)
RUB_VIEW_QUAT = (-0.5, 0.5, 0.5, 0.5)
# Emitter & co: euler (-pi/2, 0, -pi/2)
RUB_EMIT_QUAT = (-0.5, -0.5, -0.5, 0.5)
RUB_GEOM_NODES = ('Cylinder', 'Capsule', 'Cone', 'Plane', 'ElevationGrid')
RUB_DEVICE_QUATS = dict.fromkeys(
    ('Camera', 'Lidar', 'Radar', 'Track'), RUB_VIEW_QUAT)
RUB_DEVICE_QUATS.update(dict.fromkeys(
    ('Emitter', 'Receiver', 'Connector', 'TouchSensor'), RUB_EMIT_QUAT))
RUB_DEVICE_QUATS['Pen'] = RUB_GEOM_QUAT


def quat_between(a, b):
    """Quaternion rotating unit vector a onto unit vector b."""
    d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    if d > 0.999999:
        return (0.0, 0.0, 0.0, 1.0)
    if d < -0.999999:
        alt = (1.0, 0.0, 0.0) if abs(a[0]) < 0.9 else (0.0, 1.0, 0.0)
        c = (a[1] * alt[2] - a[2] * alt[1],
             a[2] * alt[0] - a[0] * alt[2],
             a[0] * alt[1] - a[1] * alt[0])
        n = math.sqrt(sum(v * v for v in c))
        return (c[0] / n, c[1] / n, c[2] / n, 0.0)
    c = (a[1] * b[2] - a[2] * b[1],
         a[2] * b[0] - a[0] * b[2],
         a[0] * b[1] - a[1] * b[0])
    q = (c[0], c[1], c[2], 1.0 + d)
    n = math.sqrt(sum(v * v for v in q))
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def _block_header(block):
    """The text of a node block before its first nested node (its own fields),
    and the offset where the nested part starts (-1 when there is none)."""
    nested = block.find('{', block.index('{') + 1)
    return (block, -1) if nested < 0 else (block[:nested], nested)


def _compensate_rotation(header, quat, base_indent):
    """Post-compose quat onto the depth-1 `rotation` line of a node header,
    or insert one (after `translation`, else right after the brace at
    base_indent + 2)."""
    rot = re.search(r'^(\s*)rotation\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s*$',
                    header, re.M)
    if rot:
        q = quat_mul(axis_angle_to_quat(
            [float(rot.group(i)) for i in (2, 3, 4, 5)]), quat)
        return (header[:rot.start()]
                + '%srotation %s'
                % (rot.group(1), quat_to_axis_angle(q))
                + header[rot.end():])
    anchor = re.search(r'^(\s*)translation\s+[^\n]*$', header, re.M)
    if anchor:
        at, indent = anchor.end(), anchor.group(1)
    else:
        at, indent = header.index('{') + 1, ' ' * (base_indent + 2)
    return (header[:at]
            + '\n%srotation %s'
            % (indent, quat_to_axis_angle(quat))
            + header[at:])


def fix_helixes(text, report):
    """Re-seat each helix's disk onto its shaft.  The Propeller spins the
    helix around shaftAxis, so the Cylinder only has to be perpendicular to
    it: compose the rotation that maps the new +Z cylinder axis onto the
    shaft direction into the helix Solid's rotation."""
    fixed = 0
    out, pos = [], 0
    for start, end in find_blocks(text, r'\bPropeller\s*\{'):
        block = text[start:end]
        shaft = re.search(r'shaftAxis\s+(\S+)\s+(\S+)\s+(\S+)', block)
        if not shaft:
            continue
        axis = [float(shaft.group(i)) for i in (1, 2, 3)]
        n = math.sqrt(sum(v * v for v in axis))
        quat = quat_between((0.0, 0.0, 1.0),
                            (axis[0] / n, axis[1] / n, axis[2] / n))
        changed = False
        for helix in ('fastHelix', 'slowHelix'):
            hm = re.search(r'\b' + helix + r'\s+Solid\s*\{', block)
            if not hm:
                continue
            hend = block_end(block, block.index('{', hm.start()))
            hblock = block[hm.start():hend]
            header, cut = _block_header(hblock)
            col = (start + hm.start()) \
                - (text.rindex('\n', 0, start + hm.start()) + 1)
            hblock = (_compensate_rotation(header, quat, col)
                      + ('' if cut < 0 else hblock[cut:]))
            block = block[:hm.start()] + hblock + block[hend:]
            changed = True
        if changed:
            fixed += 1
        out.append(text[pos:start])
        out.append(block)
        pos = end
    out.append(text[pos:])
    if fixed:
        report.append('    %d propeller helixes re-seated on their shafts (rub->flu)'
                      % fixed)
    return ''.join(out)


def fix_geometry_shapes(text, report):
    """Wrap every Shape holding a RUB geometry outside a Propeller helix
    (the arm struts) in a Transform carrying the -pi/2 X compensation.
    USE references are matched against the DEF'd names of the same nodes."""
    defs = re.findall(r'\bDEF\s+(\w+)\s+(?:%s)\s*\{'
                      % '|'.join(RUB_GEOM_NODES), text)
    pat = r'(?:%s)\s*\{' % '|'.join(RUB_GEOM_NODES)
    if defs:
        pat += '|USE\\s+(?:' + '|'.join(map(re.escape, defs)) + r')\b'
    geom_re = re.compile(pat)

    protected = [(s, e) for s, e in find_blocks(text, r'\bPropeller\s*\{')]

    out, pos, fixed = [], 0, 0
    for start, end in find_blocks(text, r'\bShape\s*\{'):
        if any(s <= start < e for s, e in protected):
            continue
        block = text[start:end]
        if not geom_re.search(block):
            continue
        indent = ' ' * (start - (text.rindex('\n', 0, start) + 1))
        lines = block.split('\n')
        inner = (indent + '    ' + lines[0] + '\n'
                 + '\n'.join('    ' + ln if ln.strip() else ln
                             for ln in lines[1:]))
        block = ('Pose {\n'
                 '%s  rotation 1 0 0 -1.5707963071795863\n'
                 '%s  children [\n%s\n%s  ]\n%s}'
                 % (indent, indent, inner, indent, indent))
        out.append(text[pos:start])
        out.append(block)
        pos = end
        fixed += 1
    out.append(text[pos:])
    if fixed:
        report.append('    %d RUB geometry Shapes wrapped upright' % fixed)
    return ''.join(out)


def fix_device_axes(text, report):
    counts = {}
    out, pos = [], 0
    node_re = r'\b(?:DEF\s+\S+\s+)?(?:%s)\s*\{' % '|'.join(RUB_DEVICE_QUATS)
    for start, end in find_blocks(text, node_re):
        block = text[start:end]
        header, cut = _block_header(block)
        name = re.match(r'(?:DEF\s+\S+\s+)?(\w+)', block).group(1)
        col = start - (text.rindex('\n', 0, start) + 1)
        block = (_compensate_rotation(header, RUB_DEVICE_QUATS[name], col)
                 + ('' if cut < 0 else block[cut:]))
        counts[name] = counts.get(name, 0) + 1
        out.append(text[pos:start])
        out.append(block)
        pos = end
    out.append(text[pos:])
    if counts:
        compensated = ', '.join('%dx %s' % (n, k) for k, n in sorted(counts.items()))
        report.append('    device look directions compensated: %s' % compensated)
    return ''.join(out)


def fix_viewpoint(text, report):
    m = re.search(r'^Viewpoint\s*\{', text, re.M)
    if not m:
        return text
    end = block_end(text, text.index('{', m.start()))
    block = text[m.start():end]
    header, cut = _block_header(block)
    rot = re.search(r'^(\s*)orientation\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s*$',
                    header, re.M)
    if rot:
        q = quat_mul(axis_angle_to_quat(
            [float(rot.group(i)) for i in (2, 3, 4, 5)]), RUB_VIEW_QUAT)
        new = (header[:rot.start()]
               + '%sorientation %s'
               % (rot.group(1), quat_to_axis_angle(q))
               + header[rot.end():])
    else:
        anchor = re.search(r'^(\s*)position\s+[^\n]*$', header, re.M)
        if anchor:
            at, indent = anchor.end(), anchor.group(1)
        else:
            at, indent = header.index('{') + 1, '  '
        new = (header[:at]
               + '\n%sorientation %s'
               % (indent, quat_to_axis_angle(RUB_VIEW_QUAT))
               + header[at:])
    report.append('    Viewpoint look direction compensated (rub->flu)')
    return (text[:m.start()] + new
            + ('' if cut < 0 else block[cut:]) + text[end:])


def fix_rub_to_flu(text, report):
    text = fix_helixes(text, report)
    text = fix_geometry_shapes(text, report)
    text = fix_device_axes(text, report)
    text = fix_viewpoint(text, report)
    return text


def add_externproto(text, report):
    used = set()
    for name in PROTO_URLS:
        # may be a top-level node, a DEF, or a field value such as
        # `wheelFrontRight VehicleWheel {`
        if re.search(r'\b' + name + r'\s*\{', text):
            used.add(name)
    if not used:
        return text

    missing = [n for n in sorted(used)
               if PROTO_URLS[n] not in text]
    if not missing:
        return text

    header = HEADER_RE.search(text)
    if header is None:
        report.append('    EXTERNPROTO not added: no #VRML_SIM header line to put it under')
        return text
    lines = ['EXTERNPROTO "%s%s"' % (PROTO_BASE, PROTO_URLS[n]) for n in missing]
    # immediately after the #VRML_SIM header line, wherever that line is
    at = header.end()
    text = text[:at] + '\n\n' + '\n'.join(lines) + text[at:]
    report.append('    %d EXTERNPROTO added: %s' % (len(missing), ', '.join(missing)))
    return text


def own_field(block, pattern):
    """First match of pattern among the node's own fields (brace depth 1),
    not inside a nested node such as a child Solid, nor in a comment."""
    code = code_only(block)
    for m in re.finditer(pattern, block):
        field = m.start() + len(m.group(0)) - len(m.group(0).lstrip())
        if field < len(block) and code[field] != block[field]:
            continue
        if code[:m.start()].count('{') - code[:m.start()].count('}') == 1:
            return m
    return None


def fix_solid_inertia(text, report):
    fixed = 0
    # last-starting first, so an edit never moves a block still to come; each
    # block's end is found afresh, since a nested edit moves its parent's end
    starts = sorted((s for s, _ in find_blocks(text, r'\b(DEF\s+\S+\s+)?Solid\s*\{')),
                    reverse=True)
    for start in starts:
        end = block_end(text, text.index('{', start))
        block = text[start:end]
        top = direct_text(block)
        if 'boundingObject' in top or 'physics' not in top:
            continue

        # the Solid's own Physics, not the first one inside a child Solid
        phys = own_field(block, r'physics\s+Physics\s*\{')
        if not phys:
            continue
        pstart = phys.start()
        pend = block_end(block, block.index('{', phys.start()))
        pblock = block[pstart:pend]
        if 'inertiaMatrix' in pblock:
            continue
        mass = re.search(r'^\s*mass\s+([0-9.eE+-]+)[ \t]*(?:#[^\n]*)?$', pblock, re.M)
        if not mass:
            continue

        m = float(mass.group(1))
        i = 0.4 * m * NACELLE_RADIUS ** 2
        indent = re.match(r'^(\s*)', block[block.rindex('\n', 0, pstart) + 1:]).group(1)
        fi = indent + '  '
        # add only what is missing, so an authored centerOfMass, damping etc.
        # survive; an inertiaMatrix needs density -1 so mass is used as given
        new_phys = pblock
        inertia = ('%sinertiaMatrix [\n%s  %.6g %.6g %.6g\n%s  0 0 0\n%s]\n'
                   % (fi, fi, i, i, i, fi, fi))
        close = new_phys.rindex('}')
        new_phys = new_phys[:close].rstrip(' \t') + inertia + indent + new_phys[close:]
        if not re.search(r'^\s*centerOfMass\s', new_phys, re.M):
            at = re.search(r'^\s*mass\s[^\n]*\n', new_phys, re.M).end()
            new_phys = (new_phys[:at] + '%scenterOfMass [\n%s  0 0 0\n%s]\n' % (fi, fi, fi)
                        + new_phys[at:])
        density = re.search(r'^(\s*)density\s+[^\n]*$', new_phys, re.M)
        if density:
            new_phys = (new_phys[:density.start()] + '%sdensity -1' % density.group(1)
                        + new_phys[density.end():])
        else:
            at = new_phys.index('\n') + 1
            new_phys = new_phys[:at] + '%sdensity -1\n' % fi + new_phys[at:]
        block = block[:pstart] + new_phys + block[pend:]
        text = text[:start] + block + text[end:]
        fixed += 1

    if fixed:
        report.append('    %d Solid inertia matrices added' % fixed)
    return text


def block_end(text, open_brace):
    """Index just past the '}' matching the '{' at open_brace; braces in
    comments and strings do not count."""
    code = code_only(text)
    depth = 0
    for i in range(open_brace, len(code)):
        if code[i] == '{':
            depth += 1
        elif code[i] == '}':
            depth -= 1
            if depth == 0:
                return i + 1
    raise WorldError('unbalanced braces: a node opened at offset %d is never closed'
                     % open_brace)


def set_field(block, field, value, after=None):
    """Replace `field <rest of line>` inside block, keeping indentation.

    If the field is absent it is inserted just below the `after` field, which is
    how the quadPlus world gets a torqueConstants it never declared (and so was
    silently taking Webots' default of 1 0)."""
    pattern = re.compile(r'^(\s*)' + re.escape(field) + r'\s+[^\n]*$', re.M)
    if pattern.search(block):
        return pattern.sub(lambda m: '%s%s %s' % (m.group(1), field, value), block, count=1), True

    if after is None:
        return block, False
    anchor = re.search(r'^(\s*)' + re.escape(after) + r'\s+[^\n]*$', block, re.M)
    if not anchor:
        return block, False
    insert_at = anchor.end()
    return (block[:insert_at] + '\n%s%s %s' % (anchor.group(1), field, value)
            + block[insert_at:]), True


HEADER_RE = re.compile(r'^#VRML_SIM R(\d{4})([a-z])\b[^\n]*$', re.M)


def world_version(text):
    """(year, letter) from the "#VRML_SIM R2025a utf8" header line, or None.
    The header is looked for on any line, so a comment above it does not hide
    the version."""
    m = HEADER_RE.search(text)
    return (int(m.group(1)), m.group(2)) if m else None


def version_name(version):
    return 'R%d%s' % version if version else 'unknown'


def vehicle_blocks(text):
    """The controlled vehicles: every top-level node with its own controller --
    a Robot, or a Robot-based proto such as the rover's BmwX5."""
    return [text[s:e] for s, e in find_blocks(text, r'(?m)^(?:DEF\s+\S+\s+)?[A-Z]\w*\s*\{')
            if 'controller "' in direct_text(text[s:e])]


def vehicle_mass(text):
    """Mass of one controlled Robot: its own Physics plus every Solid rigidly
    attached to it.  A two-vehicle world holds the same airframe twice, so this
    is the total divided by the number of vehicles."""
    vehicles = vehicle_blocks(text)
    total = sum(float(m.group(1)) for block in vehicles
                for m in re.finditer(r'^\s*mass\s+([0-9.eE+-]+)[ \t]*(?:#[^\n]*)?$',
                                     block, re.M))
    return (total / len(vehicles)) if vehicles else 0.0


def lift_propellers(text):
    """(start, end) of every Propeller driven by a lift motor."""
    return [(s, e) for s, e in find_blocks(text, r'\bPropeller\s*\{')
            if LIFT_MOTORS.search(text[s:e])]


def rotor_count(text):
    """Lift rotors on one vehicle."""
    vehicles = vehicle_blocks(text)
    return (len(lift_propellers(text)) // len(vehicles)) if vehicles else 0


def rotor_constants(text):
    """(thrustConstant, torqueConstant, maxVelocity) of the first lift rotor,
    or None when the world has no lift rotors (e.g. the rover)."""
    props = lift_propellers(text)
    if not props:
        return None
    block = text[props[0][0]:props[0][1]]

    def value(field):
        m = re.search(r'\b%s\s+(-?[0-9.eE+-]+)' % field, block)
        return float(m.group(1)) if m else None
    return value('thrustConstants'), value('torqueConstants'), value('maxVelocity')


def describe(path):
    """One-line summary of a world, for the status table and the menus."""
    text = open(path).read()
    parts = [version_name(world_version(text))]
    vehicles = len(vehicle_blocks(text))
    rotors = rotor_count(text)
    mass = vehicle_mass(text)
    consts = rotor_constants(text)
    parts.append('%d vehicle%s' % (vehicles, '' if vehicles == 1 else 's'))
    if consts and rotors and mass > 0 and consts[0] and consts[2]:
        kt, _, omega = consts
        tw = abs(kt) * omega ** 2 * rotors / (mass * G)
        parts.append('%d rotors, %.3f kg, T/W %.2f (hover %.0f%%)'
                     % (rotors, mass, tw, 100.0 / tw))
    elif mass > 0:
        parts.append('%.3f kg, no lift rotors' % mass)
    return ', '.join(parts)


def recalibrate(text, tw, omega_max, torque_ratio, report):
    """Set every lift rotor's constants for the world's current mass.  Nothing
    else in the world changes."""
    rotors = rotor_count(text)
    mass = vehicle_mass(text)
    if not rotors or mass <= 0:
        report.append('    no lift rotors or no mass: nothing to calibrate')
        return text

    thrust_per_rotor = tw * mass * G / rotors
    kt = thrust_per_rotor / (omega_max ** 2)
    kq = kt * torque_ratio
    hover_omega = math.sqrt(mass * G / rotors / kt)
    report.append('    vehicle mass %.3f kg (summed from the world), %d rotors'
                  % (mass, rotors))
    # only reported once every rotor has actually taken them
    constants = [
        '    thrustConstants  %.6g   torqueConstants %.6g' % (kt, kq),
        '    omega_max %.0f rad/s (%.0f rpm), hover %.0f rad/s (%.0f rpm), '
        'hover throttle %.2f'
        % (omega_max, omega_max * 60 / (2 * math.pi),
           hover_omega, hover_omega * 60 / (2 * math.pi), 1.0 / tw)]

    out, pos = [], 0
    for n, (start, end) in enumerate(lift_propellers(text), 1):
        block = text[start:end]
        # the signs encode which way the shaft points and which way the rotor
        # turns; keep them
        thrust_sign = field_sign(block, 'thrustConstants')
        torque_sign = field_sign(block, 'torqueConstants')
        missing = []
        for field, value, after in (
                ('thrustConstants', '%s%.6g 0' % (thrust_sign, abs(kt)), 'shaftAxis'),
                ('torqueConstants', '%s%.6g 0' % (torque_sign, abs(kq)), 'thrustConstants'),
                ('fastHelixThreshold', '%g' % FAST_HELIX_THRESHOLD, 'torqueConstants'),
                ('maxVelocity', '%g' % omega_max, 'controlPID'),
                ('maxTorque', '%g' % MAX_TORQUE, 'maxVelocity')):
            block, ok = set_field(block, field, value, after=after)
            if not ok:
                missing.append('%s (no %s to put it after)' % (field, after))
        if missing:
            # a half-calibrated vehicle is worse than an untouched one
            report.append('    NOT calibrated: lift rotor %d lacks %s; the world is '
                          'left unchanged' % (n, ', '.join(missing)))
            return text
        out.append(text[pos:start])
        out.append(block)
        pos = end
    out.append(text[pos:])
    report.extend(constants)
    return ''.join(out)


def migrate_text(text, report):
    """Convert a world saved by an older Webots to R2025a.  What is converted
    follows from the world's own #VRML_SIM header, so running this on a world
    that is already R2025a changes nothing."""
    version = world_version(text)
    if version is None:
        # without the version there is no telling which conversions it needs,
        # and applying them to a world that already has them breaks it
        raise WorldError('no "#VRML_SIM Rxxxxx utf8" header, so no telling which '
                         'conversions it needs')

    def older_than(v):
        return version < v

    if not older_than(TARGET_VERSION):
        report.append('    already %s: nothing to migrate' % version_name(version))
        return text
    report.append('    saved by Webots %s' % version_name(version))

    text = HEADER_RE.sub(TARGET_HEADER, text, count=1)

    if older_than((2023, 'b')):
        # Pose is Transform without `scale`, so the rename needs no scale
        if re.search(r'^\s*scale\s', text, re.M):
            report.append('    SKIPPED Transform->Pose: world sets an explicit scale')
        else:
            text, n = re.subn(r'\b(DEF\s+\S+\s+)?Transform(\s*\{)',
                              lambda m: (m.group(1) or '') + 'Pose' + m.group(2), text)
            if n:
                report.append('    %d Transform -> Pose' % n)

    if older_than((2022, 'a')):
        # R2022a replaced WorldInfo.gravity's vector form with a scalar
        # magnitude plus WorldInfo.coordinateSystem.
        # VRML allows commas between the numbers, and a comment after them
        grav = re.search(r'^([ \t]*)gravity\s+%s[\s,]+%s[\s,]+%s([ \t]*(?:#[^\n]*)?)$'
                         % ((NUMBER,) * 3), text, re.M)
        if grav:
            indent = grav.group(1)
            vec = [float(grav.group(i)) for i in (2, 3, 4)]
            magnitude = max(abs(v) for v in vec)
            # these worlds are all y-up, i.e. gravity along -Y
            axis = 'NUE' if vec[1] < 0 else 'ENU'
            replacement = '%sgravity %g%s' % (indent, magnitude, grav.group(5))
            if 'coordinateSystem' not in text:
                replacement += '\n%scoordinateSystem "%s"' % (indent, axis)
            text = text[:grav.start()] + replacement + text[grav.end():]
            report.append('    gravity vector -> scalar + coordinateSystem "%s"' % axis)
        # stamping R2025a over a gravity vector would give Webots a field it
        # rejects, and a re-run would then take the world as already current
        # comments and strings are left out, so a note quoting the old vector
        # is not taken for it
        if re.search(r'\bgravity\s+%s[\s,]+%s' % (NUMBER, NUMBER), code_only(text)):
            raise WorldError('WorldInfo.gravity is a vector this tool cannot convert; '
                             'write it as "gravity 0 -9.81 0" on one line')

        text = fix_terrain(text, report)
        text = fix_enu_protos(text, report)
        text = fix_rub_to_flu(text, report)

    # these check the file themselves, so they are safe on any older world
    text = fix_building_colors(text, report)
    text = fix_background(text, report)
    text = fix_vehicle_wheels(text, report)
    text = add_externproto(text, report)
    text = fix_solid_inertia(text, report)
    return text


# ----------------------------------------------------------------------
# user interface


def interactive():
    return sys.stdin.isatty()


def ask(prompt, default, cast=float):
    while True:
        answer = input('%s [%s]: ' % (prompt, default)).strip()
        if not answer:
            return default
        try:
            return cast(answer)
        except ValueError:
            print('  please enter a number')


def confirm(prompt):
    return input('%s [y/N]: ' % prompt).strip().lower() in ('y', 'yes')


def all_worlds():
    return [os.path.join(WORLDS, n) for n in sorted(os.listdir(WORLDS)) if n.endswith('.wbt')]


def resolve_world(name):
    for candidate in (name, os.path.join(WORLDS, name), os.path.join(WORLDS, name + '.wbt')):
        if os.path.isfile(candidate):
            return candidate
    sys.exit('no such world: %s' % name)


def choose_worlds(prompt):
    worlds = all_worlds()
    print()
    for i, path in enumerate(worlds, 1):
        print('  %2d  %-28s %s' % (i, os.path.basename(path), describe(path)))
    while True:
        answer = input('%s (numbers separated by commas, "all", or empty to cancel): '
                       % prompt).strip().lower()
        if not answer:
            return []
        if answer == 'all':
            return worlds
        try:
            picked = [worlds[int(x) - 1] for x in answer.replace(' ', '').split(',')]
            if all(int(x) >= 1 for x in answer.replace(' ', '').split(',')):
                return picked
        except (ValueError, IndexError):
            pass
        print('  please pick from the list')


def field_sign(block, field):
    """'-' if the first number of a field is negative, else ''."""
    m = re.search(field + r'\s+(-?)[0-9.eE+-]+', block)
    return m.group(1) if m else ''


def save(path, original, text, report, args):
    """Show what changed and write it only once the user agrees."""
    print('\n'.join(report))
    if text == original:
        print('    no changes')
        return False
    if args.check:
        name = os.path.basename(path)
        sys.stdout.writelines(difflib.unified_diff(
            original.splitlines(True), text.splitlines(True),
            fromfile=name, tofile=name + ' (migrated)'))
        print('    would change (--check: not written)')
        return True
    if args.yes:
        write = True
    elif interactive():
        write = confirm('    write %s?' % os.path.basename(path))
    else:
        print('    not written: pass --yes to write without being asked')
        write = False
    if write:
        # keep the pristine original beside it (a later run must not
        # replace it with an already-migrated copy), and never leave a
        # half-written world
        backup = path + '.bak'
        if not os.path.lexists(backup):
            # 'x' creates the file or fails: it never follows a symlink
            with open(path, 'rb') as src, open(backup, 'xb') as dst:
                shutil.copyfileobj(src, dst)
            shutil.copystat(path, backup)
        elif os.path.islink(backup) or not os.path.isfile(backup):
            # only an earlier run's plain-file backup is trusted to hold the
            # original; anything else and we cannot keep that promise
            print('    not written: %s exists but is not a regular file; move it '
                  'away and run again' % os.path.basename(backup))
            return True
        # a fresh, uniquely named temporary file, created exclusively, so a
        # file or symlink already sitting beside the world is never written
        # through
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(os.path.abspath(path)),
                                   prefix=os.path.basename(path) + '.', suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                f.write(text)
            shutil.copymode(path, tmp)
            os.replace(tmp, path)
        except BaseException:
            os.unlink(tmp)
            raise
        print('    written (original kept as %s)' % os.path.basename(backup))
    else:
        print('    left unchanged')
    return True


def cmd_status(args):
    failed = False
    for path in [resolve_world(w) for w in args.worlds] or all_worlds():
        try:
            print('%-28s %s' % (os.path.basename(path), describe(path)))
        except WorldError as err:
            unreadable(path, err)
            failed = True
    return 2 if failed else 0


def worlds_for(args, prompt):
    if args.worlds:
        return [resolve_world(w) for w in args.worlds]
    if not interactive():
        sys.exit('name the world(s) to %s' % args.command)
    return choose_worlds(prompt)


def calibrate_world(path, args, omega_default=None):
    """Ask for (or take from args) the calibration settings for one world, then
    show the result and write it only if confirmed.  omega_default overrides
    the world's own maxVelocity as the offered default."""
    original = open(path).read()
    report = ['==> %s  (%s)' % (os.path.basename(path), describe(path))]
    consts = rotor_constants(original)
    if consts is None:
        print('\n'.join(report + ['    no lift rotors: nothing to calibrate']))
        return False
    kt, kq, omega = consts
    notes = []

    current_omega = omega_default or omega or OMEGA_MAX
    # the constants in the file are rounded to 6 digits; 3 recovers the design
    # ratio (0.02, 8.74e-7) so that the default reproduces the world
    # a magnitude: each rotor keeps its own sign when the constants are written
    current_ratio = float('%.3g' % (abs(kq) / abs(kt))) if (kt and kq is not None) else None
    if current_ratio is None or current_ratio > MAX_TORQUE_RATIO:
        if current_ratio is not None:
            notes.append('    the world\'s torque/thrust ratio %.3g m is not physical '
                         '(propellers are ~0.01-0.05 m); offering %g'
                         % (current_ratio, DEFAULT_TORQUE_RATIO))
        current_ratio = DEFAULT_TORQUE_RATIO

    # migrate offers calibration too, and has no calibration options of its own
    tw = getattr(args, 'tw', None)
    omega_max = getattr(args, 'omega_max', None)
    ratio = getattr(args, 'torque_ratio', None)
    if interactive() and not args.yes:
        print('\n'.join([report[0]] + notes))
        report, notes = report[1:], []
        if tw is None:
            tw = ask('    thrust-to-weight at full throttle', DEFAULT_TW)
        if omega_max is None:
            omega_max = ask('    rotor speed at full throttle, rad/s', current_omega)
        if ratio is None:
            ratio = ask('    torque/thrust constant ratio, m', current_ratio)
    tw = DEFAULT_TW if tw is None else tw
    omega_max = current_omega if omega_max is None else omega_max
    ratio = current_ratio if ratio is None else ratio
    for name, value in (('thrust-to-weight', tw), ('rotor speed', omega_max),
                        ('torque/thrust ratio', ratio)):
        if not (math.isfinite(value) and value > 0):
            raise SettingError('%s must be a positive number, not %g' % (name, value))

    report += notes
    text = recalibrate(original, tw, omega_max, ratio, report)
    return save(path, original, text, report, args)


def cmd_calibrate(args):
    changed = failed = False
    for path in worlds_for(args, 'Recalibrate which worlds'):
        try:
            changed |= calibrate_world(path, args)
        except WorldError as err:
            unreadable(path, err)
            failed = True
    if failed:
        return 2
    return 1 if (args.check and changed) else 0


def cmd_migrate(args):
    changed = failed = False
    for path in worlds_for(args, 'Migrate which worlds'):
        try:
            changed |= migrate_world(path, args)
        except WorldError as err:
            unreadable(path, err)
            failed = True
    if failed:
        return 2
    return 1 if (args.check and changed) else 0


def migrate_world(path, args):
    """Migrate one world; True if it needed changes."""
    original = open(path).read()
    report = ['==> %s' % os.path.basename(path)]
    text = migrate_text(original, report)
    rotors = rotor_constants(text)
    if text != original and rotors:
        report.append('    rotor constants were left as they were; in worlds this old '
                      'they were never physical')
    written = save(path, original, text, report, args)
    if (written and rotors and interactive() and not args.check
            and open(path).read() == text
            and confirm('    recalibrate its rotors now?')):
        # the old controllers ignored maxVelocity, so do not offer it back
        calibrate_world(path, args, omega_default=OMEGA_MAX)
    elif written and rotors and not args.check:
        print('    to size its rotors: migrate_worlds.py calibrate --omega-max %g %s'
              % (OMEGA_MAX, os.path.basename(path)))
    return written


def menu(args):
    print('Webots worlds in %s' % os.path.normpath(WORLDS))
    cmd_status(args)
    print()
    print('  1) calibrate: recompute rotor constants for a world\'s current mass')
    print('  2) migrate:   convert a world saved by an older Webots (R2021 etc.)')
    print('  q) quit')
    answer = input('What do you want to do? ').strip().lower()
    if answer == '1':
        args.command = 'calibrate'
        return cmd_calibrate(args)
    if answer == '2':
        args.command = 'migrate'
        return cmd_migrate(args)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip(),
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='command')

    def common(p):
        p.add_argument('worlds', nargs='*', metavar='WORLD',
                       help='file in ../worlds or a path; asked for when omitted')
        p.add_argument('--check', action='store_true', help='show changes, write nothing')
        p.add_argument('--yes', action='store_true', help='write without asking')

    p = sub.add_parser('status', help='summarise each world')
    p.add_argument('worlds', nargs='*', metavar='WORLD')
    p = sub.add_parser('calibrate', help='recompute rotor constants for the current mass')
    common(p)
    p.add_argument('--tw', type=float, help='thrust-to-weight (default %g)' % DEFAULT_TW)
    p.add_argument('--omega-max', type=float,
                   help='rotor speed at full throttle, rad/s (default: the world\'s)')
    p.add_argument('--torque-ratio', type=float,
                   help='torqueConstants/thrustConstants (default: the world\'s)')
    p = sub.add_parser('migrate', help='convert a world saved by an older Webots')
    common(p)

    args = ap.parse_args()
    if args.command == 'status':
        return cmd_status(args)
    if args.command == 'calibrate':
        return cmd_calibrate(args)
    if args.command == 'migrate':
        return cmd_migrate(args)

    if not interactive():
        ap.print_help()
        return 0
    args.worlds, args.check, args.yes = [], False, False
    args.tw = args.omega_max = args.torque_ratio = None
    return menu(args)


if __name__ == '__main__':
    sys.exit(main())

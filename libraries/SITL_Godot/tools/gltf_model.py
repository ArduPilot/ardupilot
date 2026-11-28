# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
What the make_*.py model generators share: quaternions, the node and
shape lists, and the glTF document they write into ../models.

AP_FLAKE8_CLEAN
'''

import math
import os

from json_pretify import pretty

MODELS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")
X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
nodes = [None]      # node 0 is the vehicle, which body() fills in
shapes = []


def _r(v):
    v = round(v, 8)
    return 0.0 if v == 0 else v     # never -0.0


def qaxis(axis, deg):
    '''the quaternion [x, y, z, w] turning deg degrees about axis'''
    h = math.radians(deg) / 2
    return [_r(a * math.sin(h)) for a in axis] + [_r(math.cos(h))]


def qmul(a, b):
    '''a then b, as a quaternion: b * a'''
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [_r(bw * ax + bx * aw + by * az - bz * ay),
            _r(bw * ay - bx * az + by * aw + bz * ax),
            _r(bw * az + bx * ay - by * ax + bz * aw),
            _r(bw * aw - bx * ax - by * ay - bz * az)]


def box_inertia(mass, size, floor=0.0):
    x, y, z = size
    return [round(max(mass / 12.0 * v, floor), 6) for v in (y * y + z * z, x * x + z * z, x * x + y * y)]


def add(node):
    nodes.append(node)
    return len(nodes) - 1


def shape(d):
    '''a shape's index, given its dict or the size of a box'''
    shapes.append(d if isinstance(d, dict) else {"type": "box", "box": {"size": d}})
    return len(shapes) - 1


def collider_ext(shape_index, material=0):
    return {"KHR_physics_rigid_bodies": {"collider": {"geometry": {"shape": shape_index}, "physicsMaterial": material}}}


def _node(name, translation, extensions, rotation, children=None):
    n = {"name": name, "translation": translation, "extensions": extensions}
    if rotation:
        n["rotation"] = rotation
    if children:
        n["children"] = children
    return add(n)


def collider(name, size, translation, rotation=None, material=0):
    '''a solid: a box of this size, or any other shape given as its dict'''
    return _node(name, translation, collider_ext(shape(size), material), rotation)


def part(name, kind, params, translation, rotation=None, children=None):
    '''one ARDUPILOT_sitl part: a rotor, surface, wheel, hinge, floater...'''
    return _node(name, translation, {"ARDUPILOT_sitl": {kind: params}}, rotation, children)


def body(name, translation, mass, inertia, sitl, com=(0, 0, 0), children=None, rotation=None, index=0):
    '''a rigid body: the vehicle, or with index=None one more. Its children
    default to every node made so far which no other node has taken.'''
    if children is None:
        taken = {c for n in nodes if n for c in n.get("children", ())}
        children = [i for i in range(1, len(nodes)) if i not in taken]
    n = {"name": name, "translation": translation, "children": children, "extensions": {
        "KHR_physics_rigid_bodies": {"motion": {"mass": mass, "inertiaDiagonal": inertia, "centerOfMass": list(com)}},
        "ARDUPILOT_sitl": sitl}}
    if rotation:
        n["rotation"] = rotation
    if index is None:
        return add(n)
    nodes[index] = n
    return index


def write(name, materials=((0.6, 0.5, 0.1),), joints=None, roots=(0,)):
    '''write models/<name>.gltf and start afresh. Each material is (static
    friction, dynamic friction, restitution); node 0 is the vehicle.'''
    rigid = {"physicsMaterials": [{"staticFriction": s, "dynamicFriction": d, "restitution": r}
                                  for s, d, r in materials]}
    if joints is not None:
        rigid["physicsJoints"] = joints
    gltf = {
        "asset": {"version": "2.0", "generator": "ArduPilot SITL Godot example"},
        "extensionsUsed": ["KHR_physics_rigid_bodies", "KHR_implicit_shapes", "ARDUPILOT_sitl"],
        "extensions": {"KHR_implicit_shapes": {"shapes": shapes}, "KHR_physics_rigid_bodies": rigid,
                       "ARDUPILOT_sitl": {"vehicle": 0}},
        "scene": 0,
        "scenes": [{"nodes": list(roots)}],
        "nodes": nodes,
    }
    with open(os.path.join(MODELS, name + ".gltf"), "w") as f:
        f.write(pretty(gltf) + "\n")
    del nodes[1:], shapes[:]
    nodes[0] = None

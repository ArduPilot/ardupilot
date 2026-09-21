#!/usr/bin/env python3
"""
Generate the schematic AC3D model for the heliquad (variable-pitch
quad helicopter, Copter FRAME_CLASS 13) FlightGear visualisation:

    heliquad/Models/heliquad.ac

Geometry is defined in the FlightGear model frame used by the
animations in the aircraft's Models/*.xml: +x aft, +y starboard,
+z up, origin at the vehicle centre.  AC3D files are y-up, and the
loader rotates them into the model frame, so vertices are converted
at write time.  Run from this directory:

    ./generate_heliquad_models.py

AP_FLAKE8_CLEAN
"""

import os

# nacelle pivot positions, indexed by engine number (== motor number - 1).
# The vehicle uses the AP_MotorsHeli_Quad X layout ordering:
# motor1 front-right, motor2 rear-left, motor3 front-left, motor4 rear-right
COPTER_NACELLE_PIVOTS = [
    (-0.30, 0.30, 0.06),
    (0.30, -0.30, 0.06),
    (-0.30, -0.30, 0.06),
    (0.30, 0.30, 0.06),
]

BLADE_LEN = 0.25
BLADE_CHORD = 0.045
DISC_RADIUS = 0.28
ROTOR_Z = 0.085   # rotor plane height above the nacelle pivot

MATERIALS = [
    # name, rgb, trans
    ("fuselage", (0.25, 0.28, 0.32), 0),
    ("wing", (0.80, 0.80, 0.82), 0),
    ("nacelle", (0.85, 0.45, 0.10), 0),
    ("blade", (0.10, 0.10, 0.10), 0),
    ("disc", (0.4, 0.4, 0.45), 0.75),
    ("port", (0.8, 0.1, 0.1), 0),
    ("starboard", (0.1, 0.6, 0.1), 0),
]

MAT_FUSELAGE, MAT_WING, MAT_NACELLE, MAT_BLADE, MAT_DISC, MAT_PORT, MAT_STARBOARD = range(7)


def material_line(name, rgb, trans):
    r, g, b = rgb
    return ('MATERIAL "%s" rgb %.2f %.2f %.2f  amb %.2f %.2f %.2f  '
            'emis 0 0 0  spec 0.3 0.3 0.3  shi 16  trans %.2f' %
            (name, r, g, b, r * 0.5, g * 0.5, b * 0.5, trans))


def poly_object(name, verts, surfs, mat):
    """one AC3D poly object; surfs is a list of vertex-index tuples"""
    lines = [
        "OBJECT poly",
        'name "%s"' % name,
        "numvert %u" % len(verts),
    ]
    for x, y, z in verts:
        # model frame to AC3D y-up frame
        lines.append("%.4f %.4f %.4f" % (x, z, -y))
    lines.append("numsurf %u" % len(surfs))
    for s in surfs:
        # 0x30: shaded polygon, two-sided, so winding order need not be exact
        lines.append("SURF 0x30")
        lines.append("mat %u" % mat)
        lines.append("refs %u" % len(s))
        for idx in s:
            lines.append("%u 0 0" % idx)
    lines.append("kids 0")
    return "\n".join(lines)


BOX_SURFS = [
    (0, 1, 3, 2),
    (4, 6, 7, 5),
    (0, 4, 5, 1),
    (2, 3, 7, 6),
    (0, 2, 6, 4),
    (1, 5, 7, 3),
]


def box(name, xr, yr, zr, mat):
    x0, x1 = xr
    y0, y1 = yr
    z0, z1 = zr
    # vertex index = 4*xi + 2*yi + zi
    verts = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    return poly_object(name, verts, BOX_SURFS, mat)


def bar(name, p0, p1, width, zr, mat):
    """a box from XY point p0 to p1, for diagonal arms"""
    from math import hypot
    (x0, y0), (x1, y1) = p0, p1
    z0, z1 = zr
    length = hypot(x1 - x0, y1 - y0)
    # unit perpendicular in the XY plane
    px = -(y1 - y0) / length * width / 2
    py = (x1 - x0) / length * width / 2
    verts = [(x, y, z)
             for x, y in ((x0 - px, y0 - py), (x0 + px, y0 + py))
             for z in (z0, z1)]
    verts += [(x, y, z)
              for x, y in ((x1 - px, y1 - py), (x1 + px, y1 + py))
              for z in (z0, z1)]
    return poly_object(name, verts, BOX_SURFS, mat)


def disc(name, centre, radius, mat, segments=16):
    from math import cos
    from math import pi
    from math import sin
    cx, cy, cz = centre
    verts = []
    for i in range(segments):
        a = 2 * pi * i / segments
        verts.append((cx + radius * cos(a), cy + radius * sin(a), cz))
    return poly_object(name, verts, [tuple(range(segments))], mat)


def group(name, members):
    out = ["OBJECT group", 'name "%s"' % name, "kids %u" % len(members)]
    out.extend(members)
    return "\n".join(out)


def nacelle(idx, pivots):
    px, py, pz = pivots[idx]
    hub_z = pz + ROTOR_Z
    pod = box("pod%u" % idx,
              (px - 0.035, px + 0.035), (py - 0.035, py + 0.035),
              (pz - 0.06, pz + 0.07), MAT_NACELLE)
    hub = box("hub%u" % idx,
              (px - 0.02, px + 0.02), (py - 0.02, py + 0.02),
              (pz + 0.07, hub_z), MAT_BLADE)
    blade_a = box("blade%ua" % idx,
                  (px - BLADE_CHORD / 2, px + BLADE_CHORD / 2),
                  (py + 0.02, py + 0.02 + BLADE_LEN),
                  (hub_z - 0.004, hub_z + 0.004), MAT_BLADE)
    blade_b = box("blade%ub" % idx,
                  (px - BLADE_CHORD / 2, px + BLADE_CHORD / 2),
                  (py - 0.02 - BLADE_LEN, py - 0.02),
                  (hub_z - 0.004, hub_z + 0.004), MAT_BLADE)
    rotor_disc = disc("disc%u" % idx, (px, py, hub_z), DISC_RADIUS, MAT_DISC)
    rotor = group("rotor%u" % idx, [hub, blade_a, blade_b, rotor_disc])
    return group("nacelle%u" % idx, [pod, rotor])


def legs(pivots):
    out = []
    for i, (px, py, pz) in enumerate(pivots):
        out.append(box("leg%u" % i, (px - 0.01, px + 0.01),
                       (py - 0.01, py + 0.01), (-0.14, pz - 0.05), MAT_FUSELAGE))
    return out


def copter_airframe():
    objects = [
        box("body", (-0.12, 0.12), (-0.12, 0.12), (-0.05, 0.07), MAT_FUSELAGE),
        # nose marker so vehicle heading is obvious
        box("nose", (-0.18, -0.12), (-0.03, 0.03), (-0.02, 0.04), MAT_PORT),
    ]
    for i, (px, py, pz) in enumerate(COPTER_NACELLE_PIVOTS):
        objects.append(bar("arm%u" % i, (0, 0), (px, py), 0.05,
                           (pz - 0.04, pz), MAT_FUSELAGE))
    return objects + legs(COPTER_NACELLE_PIVOTS)


def write_model(path, airframe, pivots):
    lines = ["AC3Db"]
    for name, rgb, trans in MATERIALS:
        lines.append(material_line(name, rgb, trans))
    members = airframe + [nacelle(i, pivots) for i in range(4)]
    lines.append(group("world", members))
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    write_model(os.path.join(here, "heliquad", "Models", "heliquad.ac"),
                copter_airframe(), COPTER_NACELLE_PIVOTS)


if __name__ == "__main__":
    main()

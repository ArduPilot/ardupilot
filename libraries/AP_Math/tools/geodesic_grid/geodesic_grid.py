#!/usr/bin/env python

# Copyright (C) 2016  Intel Corporation. All rights reserved.
#
# This file is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
from __future__ import print_function
import argparse
import numpy as np
import sys

import icosahedron as ico
import grid

def print_code_gen_notice():
    print("/* This was generated with")
    print(" * libraries/AP_Math/tools/geodesic_grid/geodesic_grid.py */")

def header_neighbor_umbrella(index):
    t = ico.triangles[0]
    a, b, c = t

    triangle, edge = (
        ( t, ( a,  b)),
        ( t, ( b,  c)),
        ( t, ( c,  a)),
        (-t, (-a, -b)),
        (-t, (-b, -c)),
        (-t, (-c, -a)),
    )[index]

    return ico.neighbor_umbrella(triangle, edge), edge

parser = argparse.ArgumentParser(
    description="""
Utility script for helping to understand concepts used by AP_GeodesicGrid as
well as for aiding its development.

When passing a vertex as argument to one of the options, the valid values for
the coordinates are 0, -1, 1, g and -g, where g is the golden ratio.
""",
)

parser.add_argument(
    '-p', '--plot',
    action='store_true',
    help="""
Plot results when applicable.
""",
)

parser.add_argument(
    '-b', '--plot-subtriangles',
    action='store_true',
    help="""
Plot subtriangles as well. This implies -p.
""",
)

parser.add_argument(
    '--icosahedron',
    action='store_true',
    help='Get the icosahedron triangles.',
)

parser.add_argument(
    '-t', '--triangle',
    action='append',
    type=int,
    nargs='+',
    metavar='INDEX',
    help="""
Get the icosahedron triangle at INDEX.
""",
)

parser.add_argument(
    '-s', '--section',
    action='append',
    type=int,
    nargs='+',
    help="""
Get the grid section SECTION. If --plot is passed, then --plot-subtriangles is
implied.
""",
)

parser.add_argument(
    '-u', '--umbrella',
    action='append',
    nargs=3,
    metavar=('X', 'Y', 'Z'),
    help="""
Get the umbrella with pivot denoted by (X, Y, Z). The pivot must be one of the
icosahedron's vertices.
""",
)

parser.add_argument(
    '-n', '--neighbor-umbrella',
    action='append',
    nargs='+',
    metavar='INDEX',
    help="""
Get the neighbor umbrella at INDEX as described by _neighbor_umbrellas in
AP_GeodesicGrid.h. The special value "all" for INDEX is also accepted, which
will make it ignore other indexes passed and get all neighbor umbrellas for
that member.
""",
)

parser.add_argument(
    '--neighbor-umbrella-gen',
    action='store_true',
    help="""
Generate C++ code for the initialization of the member _neighbor_umbrellas
described in AP_GeodesicGrid.h.
""",
)

parser.add_argument(
    '--inverses-gen',
    action='store_true',
    help="""
Generate C++ code for the initialization of members _inverses and _mid_inverses
declared in AP_GeodesicGrid.h.
""")


args = parser.parse_args()

if args.plot_subtriangles:
    args.plot = True

if args.plot:
    import plot

polygons_to_plot = []

if args.triangle:
    indexes = []
    for l in args.triangle:
        indexes += l

    for i in indexes:
        if 0 > i or i >= len(ico.triangles):
            print(
                'Triangle index must be in the range [0,%d)' % len(ico.triangles),
                file=sys.stderr,
            )
            sys.exit(1)

        print(ico.triangles[i])
        if args.plot:
            plot.polygon(ico.triangles[i])

if args.section:
    sections = []
    for l in args.section:
        sections += l

    for s in sections:
        if 0 > s or s >= 4 * len(ico.triangles):
            print(
                'Section must be in the range [0,%d)' % 4 * len(ico.triangles),
                file=sys.stderr,
            )
            sys.exit(1)
        print(grid.section_triangle(s))
    if args.plot:
        args.plot_subtriangles = True
        plot.sections(sections)

if args.umbrella:
    for pivot in args.umbrella:
        for i, x in enumerate(pivot):
            if x == 'g':
                x = ico.g
            elif x == '-g':
                x = -ico.g
            else:
                try:
                    x = int(x)
                    if x not in (0, -1, 1):
                        raise ValueError()
                except ValueError:
                    print(
                        'umbrella: invalid pivot coordinate: %s' % str(x),
                        file=sys.stderr,
                    )
                    sys.exit(1)
            pivot[i] = x

        pivot = ico.Vertex(*pivot)
        if pivot not in ico.vertices:
            print(
                'umbrella: invalid pivot:', pivot,
                file=sys.stderr,
            )
            sys.exit(1)
        u = ico.umbrella(pivot)

        print("Components of the umbrella of %s:" % str(pivot))
        for c in u.components:
            print("    %s" % str(c))

        if args.plot:
            plot.polygons(u.components)

if args.neighbor_umbrella:
    indexes = []
    for l in args.neighbor_umbrella:
        indexes += l

    if 'all' in indexes:
        indexes = range(6)
    else:
        for i, arg in enumerate(indexes):
            try:
                arg = int(arg)
                if arg not in range(6):
                    raise ValueError()
            except ValueError:
                print(
                    'neighbor_umbrella: invalid index %s' % str(arg),
                    file=sys.stderr,
                )
                sys.exit(1)
            indexes[i] = arg

    for i in indexes:
        u, order_edge = header_neighbor_umbrella(i)
        print("Header umbrella %d:" % i)
        print("    Pivot:", u.pivot)
        for i in range(5):
            print("    Vertex %d:" % i, u.vertex(i, order_edge))
        for i in range(5):
            print("    Component %d:" % i, u.component(i, order_edge))

    if args.plot:
        plot.polygons(u.components)

if args.neighbor_umbrella_gen:
    print("Header neighbor umbrellas code generation:")
    print_code_gen_notice()
    print("const struct AP_GeodesicGrid::neighbor_umbrella")
    print("AP_GeodesicGrid::_neighbor_umbrellas[3]{")
    for i in range(6):
        u, order_edge = header_neighbor_umbrella(i)

        components = tuple(
            ico.triangles.index(u.component(i, order_edge)) for i in range(5)
        )

        def vi_cj(i, j):
            v = u.vertex(i, order_edge)
            t = u.component(j, order_edge)
            return t.index(v)

        vi_cj_values = tuple(
            vi_cj(a, b) for a, b in ((0, 0), (1, 1), (2, 1), (4, 4), (0, 4))
        )

        print("    {{%s}, %s}," % (
            ", ".join("%2d" % i for i in components),
            ", ".join(str(i) for i in vi_cj_values),
        ))
    print("};")

if args.inverses_gen:
    print("Header inverses code generation:")
    print_code_gen_notice()
    print("const Matrix3f AP_GeodesicGrid::_inverses[10]{")
    for i in range(10):
        a, b, c = ico.triangles[i]
        m = np.matrix((
            (a.x, b.x, c.x),
            (a.y, b.y, c.y),
            (a.z, b.z, c.z),
        )).getI()
        print("    {{%9.6ff, %9.6ff, %9.6ff}," % (m[0,0], m[0,1], m[0,2]))
        print("     {%9.6ff, %9.6ff, %9.6ff}," % (m[1,0], m[1,1], m[1,2]))
        print("     {%9.6ff, %9.6ff, %9.6ff}}," % (m[2,0], m[2,1], m[2,2]))
    print("};")
    print()
    print_code_gen_notice()
    print("const Matrix3f AP_GeodesicGrid::_mid_inverses[10]{")
    for i in range(10):
        a, b, c = ico.triangles[i]
        ma, mb, mc = .5 * (a + b), .5 * (b + c), .5 * (c + a)
        m = np.matrix((
            (ma.x, mb.x, mc.x),
            (ma.y, mb.y, mc.y),
            (ma.z, mb.z, mc.z),
        )).getI()
        print("    {{%9.6ff, %9.6ff, %9.6ff}," % (m[0,0], m[0,1], m[0,2]))
        print("     {%9.6ff, %9.6ff, %9.6ff}," % (m[1,0], m[1,1], m[1,2]))
        print("     {%9.6ff, %9.6ff, %9.6ff}}," % (m[2,0], m[2,1], m[2,2]))
    print("};")


if args.icosahedron:
    print('Icosahedron:')
    for i, t in enumerate(ico.triangles):
        print('    %s' % str(t))
    if args.plot:
        plot.polygons(ico.triangles)

if args.plot:
    plot.show(subtriangles=args.plot_subtriangles)

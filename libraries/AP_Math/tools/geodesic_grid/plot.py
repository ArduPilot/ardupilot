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

# flake8: noqa
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import icosahedron as ico
import grid

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim3d(-2, 2)
ax.set_ylim3d(-2, 2)
ax.set_zlim3d(-2, 2)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

ax.invert_zaxis()
ax.invert_xaxis()

ax.set_aspect('equal')

added_polygons = set()
added_sections = set()

def polygons(polygons):
    for p in polygons:
        polygon(p)

def polygon(polygon):
    added_polygons.add(polygon)

def section(s):
    added_sections.add(s)

def sections(sections):
    for s in sections:
        section(s)

def show(subtriangles=False):
    polygons = []
    facecolors = []
    triangles_indexes = set()

    subtriangle_facecolors = (
        '#CCCCCC',
        '#CCE5FF',
        '#E5FFCC',
        '#FFCCCC',
    )

    if added_sections:
        subtriangles = True

    for p in added_polygons:
        try:
            i = ico.triangles.index(p)
        except ValueError:
            polygons.append(p)
            continue

        if subtriangles:
            sections(range(i * 4, i * 4 + 4))
        else:
            triangles_indexes.add(i)
            polygons.append(p)
            facecolors.append('#DDDDDD')

    for s in added_sections:
        triangles_indexes.add(int(s / 4))
        subtriangle_index = s % 4
        polygons.append(grid.section_triangle(s))
        facecolors.append(subtriangle_facecolors[subtriangle_index])

    ax.add_collection3d(Poly3DCollection(
        polygons,
        facecolors=facecolors,
        edgecolors="#777777",
    ))

    for i in triangles_indexes:
        t = ico.triangles[i]
        mx = my = mz = 0
        for x, y, z in t:
            mx += x
            my += y
            mz += z
        ax.text(mx / 2.6, my / 2.6, mz / 2.6, i, color='#444444')

    if subtriangles:
        ax.legend(
            handles=tuple(
                mpatches.Patch(color=c, label='Sub-triangle #%d' % i)
                for i, c in enumerate(subtriangle_facecolors)
            ),
        )

    plt.show()

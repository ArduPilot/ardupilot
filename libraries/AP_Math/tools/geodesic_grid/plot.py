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
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import icosahedron as ico

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

def polygons(polygons):
    for p in polygons:
        polygon(p)


def polygon(polygon):
    added_polygons.add(polygon)

def show(subtriangles=False):
    polygons = []
    facecolors = []

    for p in added_polygons:
        try:
            i = ico.triangles.index(p)
        except ValueError:
            polygons.append(p)
            continue

        if subtriangles:
            a, b, c = p

            # project the middle points to the sphere
            alpha = a.length() / (2.0 * ico.g)
            ma, mb, mc = alpha * (a + b), alpha * (b + c), alpha * (c + a)

            polygons.append(ico.Triangle(ma, mb, mc))
            facecolors.append('#CCCCCC')

            polygons.append(ico.Triangle( a, ma, mc))
            facecolors.append('#CCE5FF')

            polygons.append(ico.Triangle(ma,  b, mb))
            facecolors.append('#E5FFCC')

            polygons.append(ico.Triangle(mc, mb,  c))
            facecolors.append('#FFCCCC')
        else:
            polygons.append(p)
            facecolors.append('#DDDDDD')

        mx = my = mz = 0
        for x, y, z in p:
            mx += x
            my += y
            mz += z
        ax.text(mx / 2.6, my /2.6, mz / 2.6, i, color='#444444')

    ax.add_collection3d(Poly3DCollection(
        polygons,
        facecolors=facecolors,
        edgecolors="#777777",
    ))

    if subtriangles:
        ax.legend(
            handles=(
                mpatches.Patch(color='#CCCCCC', label='Sub-triangle #0'),
                mpatches.Patch(color='#CCE5FF', label='Sub-triangle #1'),
                mpatches.Patch(color='#E5FFCC', label='Sub-triangle #2'),
                mpatches.Patch(color='#FFCCCC', label='Sub-triangle #3'),
            ),
        )

    plt.show()

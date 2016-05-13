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
'''
This module takes libraries/AP_Math/AP_GeodesicGrid.h reference for defining
the geodesic sections.
'''
import math
from scipy.constants import golden as g

_first_half = (
    ((-g, 1, 0), (-1, 0,-g), (-g,-1, 0)),
    ((-1, 0,-g), (-g,-1, 0), ( 0,-g,-1)),
    ((-g,-1, 0), ( 0,-g,-1), ( 0,-g, 1)),
    ((-1, 0,-g), ( 0,-g,-1), ( 1, 0,-g)),
    (( 0,-g,-1), ( 0,-g, 1), ( g,-1, 0)),
    (( 0,-g,-1), ( 1, 0,-g), ( g,-1, 0)),
    (( g,-1, 0), ( 1, 0,-g), ( g, 1, 0)),
    (( 1, 0,-g), ( g, 1, 0), ( 0, g,-1)),
    (( 1, 0,-g), ( 0, g,-1), (-1, 0,-g)),
    (( 0, g,-1), (-g, 1, 0), (-1, 0,-g)),
)
_second_half = tuple(
    ((-xa, -ya, -za), (-xb, -yb, -zb), (-xc, -yc, -zc))
    for (xa, ya, za), (xb, yb, zb), (xc, yc, zc) in _first_half
)

triangles = _first_half + _second_half

def _midpoint_projection(a, b):
    xa, ya, za = a
    xb, yb, zb = b
    s = _midpoint_projection.scale
    return s * (xa + xb), s * (ya + yb), s * (za + zb)

radius = math.sqrt(1 + g**2)

# radius / (length of two vertices of an icosahedron triangle)
_midpoint_projection.scale = radius / (2 * g)

sections_triangles = ()

for a, b, c in triangles:
    ma = _midpoint_projection(a, b)
    mb = _midpoint_projection(b, c)
    mc = _midpoint_projection(c, a)

    sections_triangles += (
        (ma, mb, mc),
        ( a, ma, mc),
        (ma,  b, mb),
        (mc, mb,  c),
    )

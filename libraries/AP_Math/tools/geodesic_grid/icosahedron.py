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
import math
from scipy.constants import golden as g

class Vertex(tuple):
    def __new__(cls, x, y, z):
        instance = tuple.__new__(cls, (x, y, z))
        instance.x = x
        instance.y = y
        instance.z = z
        return instance

    def __repr__(self):
        return "(" + ",".join(Vertex._print_map.get(x, str(x)) for x in self) + ")"

    def __str__(self):
        return self.__repr__()

    def __neg__(self):
        return Vertex(-self.x, -self.y, -self.z)

    def __add__(self, other):
        return Vertex(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vertex(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, s):
        return Vertex(s * self.x, s * self.y, s * self.z)
    __rmul__ = __mul__

    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def normalized(self):
        return (1.0 / self.length()) * self

class Triangle(tuple):
    def __new__(cls, a, b, c):
        instance = tuple.__new__(cls, (a, b, c))
        instance.a = a
        instance.b = b
        instance.c = c
        return instance

    def __neg__(self):
        return Triangle(-self.a, -self.b, -self.c)

    def __str__(self):
        if self in triangles:
            i = triangles.index(self)
            return "Triangle %2d: %s" % (i, self.__repr__())
        else:
            return self.__repr__()

Vertex._print_map = {
    g: ' g', -g: '-g', 1: ' 1', -1: '-1', 0: ' 0',
}

vertices = tuple(
    Vertex(x, y, z) for x, y, z in (
        ( g, 1, 0),
        ( g,-1, 0),
        (-g, 1, 0),
        (-g,-1, 0),
        ( 1, 0, g),
        (-1, 0, g),
        ( 1, 0,-g),
        (-1, 0,-g),
        ( 0, g, 1),
        ( 0, g,-1),
        ( 0,-g, 1),
        ( 0,-g,-1),
    )
)

_first_half = (
    Triangle(Vertex(-g, 1, 0), Vertex(-1, 0,-g), Vertex(-g,-1, 0)),
    Triangle(Vertex(-1, 0,-g), Vertex(-g,-1, 0), Vertex( 0,-g,-1)),
    Triangle(Vertex(-g,-1, 0), Vertex( 0,-g,-1), Vertex( 0,-g, 1)),
    Triangle(Vertex(-1, 0,-g), Vertex( 0,-g,-1), Vertex( 1, 0,-g)),
    Triangle(Vertex( 0,-g,-1), Vertex( 0,-g, 1), Vertex( g,-1, 0)),
    Triangle(Vertex( 0,-g,-1), Vertex( 1, 0,-g), Vertex( g,-1, 0)),
    Triangle(Vertex( g,-1, 0), Vertex( 1, 0,-g), Vertex( g, 1, 0)),
    Triangle(Vertex( 1, 0,-g), Vertex( g, 1, 0), Vertex( 0, g,-1)),
    Triangle(Vertex( 1, 0,-g), Vertex( 0, g,-1), Vertex(-1, 0,-g)),
    Triangle(Vertex( 0, g,-1), Vertex(-g, 1, 0), Vertex(-1, 0,-g)),
)

_second_half = tuple(-t for t in _first_half)

triangles = _first_half + _second_half

_neighbor_triangle_data = {}
def neighbor_triangle(t, edge):
    """ Return the neighbor triangle of t with respect to edge = (a, b) """
    e = frozenset(edge)
    if (t, e) in _neighbor_triangle_data:
        return _neighbor_triangle_data[(t, e)]

    a, b = edge
    if a not in t or b not in t:
        return None

    for w in triangles:
        if a in w and b in w and w != t:
            _neighbor_triangle_data[(t, e)] = w
            return w

    return None

class _Umbrella:
    def __init__(self, pivot):
        self.pivot = pivot
        self.components = frozenset(t for t in triangles if pivot in t)

        all_vertices = set()
        for t in self.components:
            for v in t:
                if v != pivot:
                    all_vertices.add(v)
        self.all_vertices = frozenset(all_vertices)

        self._vertex_data = {}
        self._component_data = {}

    def vertex(self, i, ordered_edge):
        """ Return the i-th vertex with respect to ordered_edge = (a, b) """
        a, b = ordered_edge
        if a not in self.all_vertices:
            return None
        if b not in self.all_vertices:
            return None

        if i == 0:
            return a
        if i == 1:
            return b

        if (i, a, b) in self._vertex_data:
            return self._vertex_data[(i, a, b)]

        previous = self.vertex(i - 1, ordered_edge)
        comp = self.component(i - 2, ordered_edge)
        neighbor = neighbor_triangle(comp, (self.pivot, previous))

        for v in neighbor:
            if v not in (self.pivot, previous):
                self._vertex_data[(i, a, b)] = v
                return v
        return None

    def component(self, i, ordered_edge):
        """ Return the i-th component with respect to ordered_edge = (a, b) """
        a, b = ordered_edge
        if (i, a, b) in self._component_data:
            return self._component_data[(i, a, b)]

        vi = self.vertex(i, ordered_edge)
        vj = self.vertex(i + 1, ordered_edge)

        for t in self.components:
            if vi in t and vj in t:
                self._component_data[(i, a, b)] = t
                return t
        return None

_umbrelas = {}
def umbrella(pivot):
    if pivot not in vertices:
        return None

    if pivot not in _umbrelas:
        _umbrelas[pivot] = _Umbrella(pivot)
    return _umbrelas[pivot]

def neighbor_umbrella(t, edge):
    neighbor = neighbor_triangle(t, edge)
    if not neighbor:
        return None

    for pivot in neighbor:
        if pivot in edge:
            continue
        return umbrella(pivot)

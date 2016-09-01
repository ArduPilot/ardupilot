#!/usr/bin/env python
"""
 vector3 and rotation matrix classes
 This follows the conventions in the ArduPilot code,
 and is essentially a python version of the AP_Math library

 Andrew Tridgell, March 2012

 This library is free software; you can redistribute it and/or modify it
 under the terms of the GNU Lesser General Public License as published by the
 Free Software Foundation; either version 2.1 of the License, or (at your
 option) any later version.

 This library is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License
 for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this library; if not, write to the Free Software Foundation,
 Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

"""

from math import acos, asin, atan2, cos, pi, radians, sin, sqrt


class Vector3:
    """A vector."""

    def __init__(self, x=None, y=None, z=None):
        if x is not None and y is not None and z is not None:
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
        elif x is not None and len(x) == 3:
            self.x = float(x[0])
            self.y = float(x[1])
            self.z = float(x[2])
        elif x is not None:
            raise ValueError('bad initialiser')
        else:
            self.x = float(0)
            self.y = float(0)
            self.z = float(0)

    def __repr__(self):
        return 'Vector3(%.2f, %.2f, %.2f)' % (self.x,
                                              self.y,
                                              self.z)

    def __add__(self, v):
        return Vector3(self.x + v.x,
                       self.y + v.y,
                       self.z + v.z)

    __radd__ = __add__

    def __sub__(self, v):
        return Vector3(self.x - v.x,
                       self.y - v.y,
                       self.z - v.z)

    def __neg__(self):
        return Vector3(-self.x, -self.y, -self.z)

    def __rsub__(self, v):
        return Vector3(v.x - self.x,
                       v.y - self.y,
                       v.z - self.z)

    def __mul__(self, v):
        if isinstance(v, Vector3):
            """dot product"""
            return self.x * v.x + self.y * v.y + self.z * v.z
        return Vector3(self.x * v,
                       self.y * v,
                       self.z * v)

    __rmul__ = __mul__

    def __div__(self, v):
        return Vector3(self.x / v,
                       self.y / v,
                       self.z / v)

    def __mod__(self, v):
        """Cross product."""
        return Vector3(self.y * v.z - self.z * v.y,
                       self.z * v.x - self.x * v.z,
                       self.x * v.y - self.y * v.x)

    def __copy__(self):
        return Vector3(self.x, self.y, self.z)

    copy = __copy__

    def length(self):
        return sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def zero(self):
        self.x = self.y = self.z = 0

    def angle(self, v):
        """Return the angle between this vector and another vector."""
        return acos(self * v) / (self.length() * v.length())

    def normalized(self):
        return self / self.length()

    def normalize(self):
        v = self.normalized()
        self.x = v.x
        self.y = v.y
        self.z = v.z


class Matrix3:
    """A 3x3 matrix, intended as a rotation matrix."""

    def __init__(self, a=None, b=None, c=None):
        if a is not None and b is not None and c is not None:
            self.a = a.copy()
            self.b = b.copy()
            self.c = c.copy()
        else:
            self.identity()

    def __repr__(self):
        return 'Matrix3((%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f))' % (
            self.a.x, self.a.y, self.a.z,
            self.b.x, self.b.y, self.b.z,
            self.c.x, self.c.y, self.c.z)

    def identity(self):
        self.a = Vector3(1, 0, 0)
        self.b = Vector3(0, 1, 0)
        self.c = Vector3(0, 0, 1)

    def transposed(self):
        return Matrix3(Vector3(self.a.x, self.b.x, self.c.x),
                       Vector3(self.a.y, self.b.y, self.c.y),
                       Vector3(self.a.z, self.b.z, self.c.z))

    def from_euler(self, roll, pitch, yaw):
        """Fill the matrix from Euler angles in radians."""
        cp = cos(pitch)
        sp = sin(pitch)
        sr = sin(roll)
        cr = cos(roll)
        sy = sin(yaw)
        cy = cos(yaw)

        self.a.x = cp * cy
        self.a.y = (sr * sp * cy) - (cr * sy)
        self.a.z = (cr * sp * cy) + (sr * sy)
        self.b.x = cp * sy
        self.b.y = (sr * sp * sy) + (cr * cy)
        self.b.z = (cr * sp * sy) - (sr * cy)
        self.c.x = -sp
        self.c.y = sr * cp
        self.c.z = cr * cp

    def to_euler(self):
        """Find Euler angles (321 convention) for the matrix."""
        if self.c.x >= 1.0:
            pitch = pi
        elif self.c.x <= -1.0:
            pitch = -pi
        else:
            pitch = -asin(self.c.x)
        roll = atan2(self.c.y, self.c.z)
        yaw = atan2(self.b.x, self.a.x)
        return (roll, pitch, yaw)

    def to_euler312(self):
        """Find Euler angles (312 convention) for the matrix.
        See http://www.atacolorado.com/eulersequences.doc
        """
        T21 = self.a.y
        T22 = self.b.y
        T23 = self.c.y
        T13 = self.c.x
        T33 = self.c.z
        yaw = atan2(-T21, T22)
        roll = asin(T23)
        pitch = atan2(-T13, T33)
        return (roll, pitch, yaw)

    def from_euler312(self, roll, pitch, yaw):
        """Fill the matrix from Euler angles in radians in 312 convention."""
        c3 = cos(pitch)
        s3 = sin(pitch)
        s2 = sin(roll)
        c2 = cos(roll)
        s1 = sin(yaw)
        c1 = cos(yaw)

        self.a.x = c1 * c3 - s1 * s2 * s3
        self.b.y = c1 * c2
        self.c.z = c3 * c2
        self.a.y = -c2 * s1
        self.a.z = s3 * c1 + c3 * s2 * s1
        self.b.x = c3 * s1 + s3 * s2 * c1
        self.b.z = s1 * s3 - s2 * c1 * c3
        self.c.x = -s3 * c2
        self.c.y = s2

    def __add__(self, m):
        return Matrix3(self.a + m.a, self.b + m.b, self.c + m.c)

    __radd__ = __add__

    def __sub__(self, m):
        return Matrix3(self.a - m.a, self.b - m.b, self.c - m.c)

    def __rsub__(self, m):
        return Matrix3(m.a - self.a, m.b - self.b, m.c - self.c)

    def __mul__(self, other):
        if isinstance(other, Vector3):
            v = other
            return Vector3(self.a.x * v.x + self.a.y * v.y + self.a.z * v.z,
                           self.b.x * v.x + self.b.y * v.y + self.b.z * v.z,
                           self.c.x * v.x + self.c.y * v.y + self.c.z * v.z)
        elif isinstance(other, Matrix3):
            m = other
            return Matrix3(Vector3(self.a.x * m.a.x + self.a.y * m.b.x + self.a.z * m.c.x,
                                   self.a.x * m.a.y + self.a.y * m.b.y + self.a.z * m.c.y,
                                   self.a.x * m.a.z + self.a.y * m.b.z + self.a.z * m.c.z),
                           Vector3(self.b.x * m.a.x + self.b.y * m.b.x + self.b.z * m.c.x,
                                   self.b.x * m.a.y + self.b.y * m.b.y + self.b.z * m.c.y,
                                   self.b.x * m.a.z + self.b.y * m.b.z + self.b.z * m.c.z),
                           Vector3(self.c.x * m.a.x + self.c.y * m.b.x + self.c.z * m.c.x,
                                   self.c.x * m.a.y + self.c.y * m.b.y + self.c.z * m.c.y,
                                   self.c.x * m.a.z + self.c.y * m.b.z + self.c.z * m.c.z))
        v = other
        return Matrix3(self.a * v, self.b * v, self.c * v)

    def __div__(self, v):
        return Matrix3(self.a / v, self.b / v, self.c / v)

    def __neg__(self):
        return Matrix3(-self.a, -self.b, -self.c)

    def __copy__(self):
        return Matrix3(self.a, self.b, self.c)

    copy = __copy__

    def rotate(self, g):
        """Rotate the matrix by a given amount on 3 axes."""
        temp_matrix = Matrix3()
        a = self.a
        b = self.b
        c = self.c
        temp_matrix.a.x = a.y * g.z - a.z * g.y
        temp_matrix.a.y = a.z * g.x - a.x * g.z
        temp_matrix.a.z = a.x * g.y - a.y * g.x
        temp_matrix.b.x = b.y * g.z - b.z * g.y
        temp_matrix.b.y = b.z * g.x - b.x * g.z
        temp_matrix.b.z = b.x * g.y - b.y * g.x
        temp_matrix.c.x = c.y * g.z - c.z * g.y
        temp_matrix.c.y = c.z * g.x - c.x * g.z
        temp_matrix.c.z = c.x * g.y - c.y * g.x
        self.a += temp_matrix.a
        self.b += temp_matrix.b
        self.c += temp_matrix.c

    def normalize(self):
        """Re-normalise a rotation matrix."""
        error = self.a * self.b
        t0 = self.a - (self.b * (0.5 * error))
        t1 = self.b - (self.a * (0.5 * error))
        t2 = t0 % t1
        self.a = t0 * (1.0 / t0.length())
        self.b = t1 * (1.0 / t1.length())
        self.c = t2 * (1.0 / t2.length())

    def trace(self):
        """The trace of the matrix."""
        return self.a.x + self.b.y + self.c.z


def test_euler():
    """Check that from_euler() and to_euler() are consistent."""
    m = Matrix3()
    from math import radians, degrees
    for r in range(-179, 179, 3):
        for p in range(-89, 89, 3):
            for y in range(-179, 179, 3):
                m.from_euler(radians(r), radians(p), radians(y))
                (r2, p2, y2) = m.to_euler()
                v1 = Vector3(r, p, y)
                v2 = Vector3(degrees(r2), degrees(p2), degrees(y2))
                diff = v1 - v2
                if diff.length() > 1.0e-12:
                    print('EULER ERROR:', v1, v2, diff.length())


def test_euler312_single(r, p, y):
    """Check that from_euler312() and to_euler312() are consistent for one set of values."""
    from math import degrees, radians
    m = Matrix3()
    m.from_euler312(radians(r), radians(p), radians(y))
    (r2, p2, y2) = m.to_euler312()
    v1 = Vector3(r, p, y)
    v2 = Vector3(degrees(r2), degrees(p2), degrees(y2))
    diff = v1 - v2
    if diff.length() > 1.0e-12:
        print('EULER ERROR:', v1, v2, diff.length())


def test_one_axis(r, p, y):
    """Check that from_euler312() and from_euler() are consistent for one set of values on one axis."""
    from math import degrees, radians
    m = Matrix3()
    m.from_euler312(radians(r), radians(p), radians(y))
    (r2, p2, y2) = m.to_euler()
    v1 = Vector3(r, p, y)
    v2 = Vector3(degrees(r2), degrees(p2), degrees(y2))
    diff = v1 - v2
    if diff.length() > 1.0e-12:
        print('EULER ERROR:', v1, v2, diff.length())


def test_euler312():
    """Check that from_euler312() and to_euler312() are consistent."""
    m = Matrix3()

    for x in range(-89, 89, 3):
        test_one_axis(x, 0, 0)
        test_one_axis(0, x, 0)
        test_one_axis(0, 0, x)
    for r in range(-89, 89, 3):
        for p in range(-179, 179, 3):
            for y in range(-179, 179, 3):
                test_euler312_single(r, p, y)


if __name__ == "__main__":
    import doctest

    doctest.testmod()
    test_euler()
    test_euler312()

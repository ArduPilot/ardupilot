#!/usr/bin/env python3
#
# vector3 and rotation matrix classes
# This follows the conventions in the ArduPilot code,
# and is essentially a python version of the AP_Math library
#
# Andrew Tridgell, March 2012
#
# This library is free software; you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as published by the
# Free Software Foundation; either version 2.1 of the License, or (at your
# option) any later version.
#
# This library is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License
# for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this library; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

'''rotation matrix class
'''
from math import sin, cos, sqrt, asin, atan2, pi, acos, radians


class Vector3(object):
    '''a vector'''
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

    def __eq__(self, v):
        return self.x == v.x and self.y == v.y and self.z == v.z

    def __ne__(self, v):
        return not self == v

    def close(self, v, tol=1e-7):
        return abs(self.x - v.x) < tol and \
            abs(self.y - v.y) < tol and \
            abs(self.z - v.z) < tol

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
            '''dot product'''
            return self.x*v.x + self.y*v.y + self.z*v.z
        return Vector3(self.x * v,
                       self.y * v,
                       self.z * v)

    __rmul__ = __mul__

    def __div__(self, v):
        return Vector3(self.x / v,
                       self.y / v,
                       self.z / v)

    def __truediv__(self, v):
        return Vector3(self.x / v,
                       self.y / v,
                       self.z / v)

    def __floordiv__(self, v):
        return Vector3(self.x // v,
                       self.y // v,
                       self.z // v)

    def __mod__(self, v):
        '''cross product'''
        return Vector3(self.y*v.z - self.z*v.y,
                       self.z*v.x - self.x*v.z,
                       self.x*v.y - self.y*v.x)

    def __copy__(self):
        return Vector3(self.x, self.y, self.z)

    copy = __copy__

    def length(self):
        return sqrt(self.x**2 + self.y**2 + self.z**2)

    def zero(self):
        self.x = self.y = self.z = 0

    def angle(self, v):
        '''return the angle between this vector and another vector'''
        return acos((self * v) / (self.length() * v.length()))

    def normalized(self):
        return self.__div__(self.length())

    def normalize(self):
        v = self.normalized()
        self.x = v.x
        self.y = v.y
        self.z = v.z

    def rotate_by_id(self, rot_id):
        '''rotate a vector using a rotation enum ID'''
        if rot_id >= len(rotations):
            return None
        return rotations[rot_id].r * self

    def rotate_by_inverse_id(self, rot_id):
        '''rotate a vector using a inverse rotation enum ID'''
        if rot_id >= len(rotations):
            return None
        return rotations[rot_id].rt * self


class Matrix3(object):
    '''a 3x3 matrix, intended as a rotation matrix'''
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
        self.a = Vector3(1,0,0)
        self.b = Vector3(0,1,0)
        self.c = Vector3(0,0,1)

    def transposed(self):
        return Matrix3(Vector3(self.a.x, self.b.x, self.c.x),
                       Vector3(self.a.y, self.b.y, self.c.y),
                       Vector3(self.a.z, self.b.z, self.c.z))


    def from_euler(self, roll, pitch, yaw):
        '''fill the matrix from Euler angles in radians'''
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
        '''find Euler angles (321 convention) for the matrix'''
        if self.c.x >= 1.0:
            pitch = pi
        elif self.c.x <= -1.0:
            pitch = -pi
        else:
            pitch = -asin(self.c.x)
        roll = atan2(self.c.y, self.c.z)
        yaw  = atan2(self.b.x, self.a.x)
        return (roll, pitch, yaw)


    def to_euler312(self):
        '''find Euler angles (312 convention) for the matrix.
        See http://www.atacolorado.com/eulersequences.doc
        '''
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
        '''fill the matrix from Euler angles in radians in 312 convention'''
        c3 = cos(pitch)
        s3 = sin(pitch)
        s2 = sin(roll)
        c2 = cos(roll)
        s1 = sin(yaw)
        c1 = cos(yaw)

        self.a.x = c1 * c3 - s1 * s2 * s3
        self.b.y = c1 * c2
        self.c.z = c3 * c2
        self.a.y = -c2*s1
        self.a.z = s3*c1 + c3*s2*s1
        self.b.x = c3*s1 + s3*s2*c1
        self.b.z = s1*s3 - s2*c1*c3
        self.c.x = -s3*c2
        self.c.y = s2

    def determinant(self):
        '''return determinant'''
        ret =  self.a.x * (self.b.y * self.c.z - self.b.z * self.c.y)
        ret += self.a.y * (self.b.z * self.c.x - self.b.x * self.c.z)
        ret += self.a.z * (self.b.x * self.c.y - self.b.y * self.c.x)
        return ret

    def invert(self):
        '''invert 3x3 matrix, returning new matrix'''
        d = self.determinant()
        inv = Matrix3()
        inv.a.x = (self.b.y * self.c.z - self.c.y * self.b.z) / d
        inv.a.y = (self.a.z * self.c.y - self.a.y * self.c.z) / d
        inv.a.z = (self.a.y * self.b.z - self.a.z * self.b.y) / d
        inv.b.x = (self.b.z * self.c.x - self.b.x * self.c.z) / d
        inv.b.y = (self.a.x * self.c.z - self.a.z * self.c.x) / d
        inv.b.z = (self.b.x * self.a.z - self.a.x * self.b.z) / d
        inv.c.x = (self.b.x * self.c.y - self.c.x * self.b.y) / d
        inv.c.y = (self.c.x * self.a.y - self.a.x * self.c.y) / d
        inv.c.z = (self.a.x * self.b.y - self.b.x * self.a.y) / d
        return inv

    def __add__(self, m):
        return Matrix3(self.a + m.a, self.b + m.b, self.c + m.c)

    __radd__ = __add__

    def __sub__(self, m):
        return Matrix3(self.a - m.a, self.b - m.b, self.c - m.c)

    def __rsub__(self, m):
        return Matrix3(m.a - self.a, m.b - self.b, m.c - self.c)

    def __eq__(self, m):
        return self.a == m.a and self.b == m.b and self.c == m.c

    def __ne__(self, m):
        return not self == m
        
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

    def __truediv__(self, v):
        return Matrix3(self.a / v, self.b / v, self.c / v)

    def __neg__(self):
        return Matrix3(-self.a, -self.b, -self.c)

    def __copy__(self):
        return Matrix3(self.a, self.b, self.c)

    copy = __copy__

    def rotate(self, g):
        '''rotate the matrix by a given amount on 3 axes,
        where g is a vector of delta angles. Used
        with DCM updates in mavextra.py'''
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

    def rotate_yaw(self, yrad):
        '''rotate the matrix by a given amount in-place on yaw axis'''
        m2 = Matrix3()
        m2.from_euler(0,0,yrad)
        m2 = self * m2
        self.a = m2.a
        self.b = m2.b
        self.c = m2.c

    def rotate_pitch(self, prad):
        '''rotate the matrix by a given amount in-place on pitch axis'''
        m2 = Matrix3()
        m2.from_euler(0,prad,0)
        m2 = self * m2
        self.a = m2.a
        self.b = m2.b
        self.c = m2.c

    def rotate_roll(self, rrad):
        '''rotate the matrix by a given amount in-place on roll axis'''
        m2 = Matrix3()
        m2.from_euler(rrad,0,0)
        m2 = self * m2
        self.a = m2.a
        self.b = m2.b
        self.c = m2.c

    def rotate_321(self, ryad, prad, yrad):
        '''rotate the matrix by a given amount in-place on 3 axes with 321 ordering'''
        m2 = Matrix3()
        m2.from_euler(ryad,prad,yrad)
        m2 = self * m2
        self.a = m2.a
        self.b = m2.b
        self.c = m2.c

    def rotate_312(self, ryad, prad, yrad):
        '''rotate the matrix by a given amount in-place on 3 axes with 312 ordering'''
        m2 = Matrix3()
        m2.from_euler312(ryad,prad,yrad)
        m2 = self * m2
        self.a = m2.a
        self.b = m2.b
        self.c = m2.c

    def normalize(self):
        '''re-normalise a rotation matrix'''
        error = self.a * self.b
        t0 = self.a - (self.b * (0.5 * error))
        t1 = self.b - (self.a * (0.5 * error))
        t2 = t0 % t1
        self.a = t0 * (1.0 / t0.length())
        self.b = t1 * (1.0 / t1.length())
        self.c = t2 * (1.0 / t2.length())

    def trace(self):
        '''the trace of the matrix'''
        return self.a.x + self.b.y + self.c.z

    def from_axis_angle(self, axis, angle):
        '''create a rotation matrix from axis and angle'''
        ux = axis.x
        uy = axis.y
        uz = axis.z
        ct = cos(angle)
        st = sin(angle)
        self.a.x = ct + (1-ct) * ux**2
        self.a.y = ux*uy*(1-ct) - uz*st
        self.a.z = ux*uz*(1-ct) + uy*st
        self.b.x = uy*ux*(1-ct) + uz*st
        self.b.y = ct + (1-ct) * uy**2
        self.b.z = uy*uz*(1-ct) - ux*st
        self.c.x = uz*ux*(1-ct) - uy*st
        self.c.y = uz*uy*(1-ct) + ux*st
        self.c.z = ct + (1-ct) * uz**2


    def from_two_vectors(self, vec1, vec2):
        '''get a rotation matrix from two vectors.
           This returns a rotation matrix which when applied to vec1
           will produce a vector pointing in the same direction as vec2'''
        angle = vec1.angle(vec2)
        cross = vec1 % vec2
        if cross.length() == 0:
            # the two vectors are colinear
            return self.from_euler(0,0,angle)
        cross.normalize()
        return self.from_axis_angle(cross, angle)

    def close(self, m, tol=1e-7):
        return self.a.close(m.a, tol) and self.b.close(m.b, tol) and self.c.close(m.c, tol)

class Plane(object):
    '''a plane in 3 space, defined by a point and a vector normal'''
    def __init__(self, point=None, normal=None):
        if point is None:
            point = Vector3(0,0,0)
        if normal is None:
            normal = Vector3(0, 0, 1)
        self.point = point
        self.normal = normal

class Line(object):
    '''a line in 3 space, defined by a point and a vector'''
    def __init__(self, point=None, vector=None):
        if point is None:
            point = Vector3(0,0,0)
        if vector is None:
            vector = Vector3(0, 0, 1)
        self.point = point
        self.vector = vector

    def plane_intersection(self, plane, forward_only=False):
        '''return point where line intersects with a plane'''
        l_dot_n = self.vector * plane.normal
        if l_dot_n == 0.0:
            # line is parallel to the plane
            return None
        d = ((plane.point - self.point) * plane.normal) / l_dot_n
        if forward_only and d < 0:
            return None
        return (self.vector * d) + self.point

class Rotation(object):
    def __init__(self, name, roll, pitch, yaw):
        self.name = name
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.r = Matrix3()
        self.r.from_euler(radians(self.roll), radians(self.pitch), radians(self.yaw))
        self.rt = self.r.transposed()

# the rotations used in APM
rotations = [
    Rotation("ROTATION_NONE",                      0,   0,   0),
    Rotation("ROTATION_YAW_45",                    0,   0,  45),
    Rotation("ROTATION_YAW_90",                    0,   0,  90),
    Rotation("ROTATION_YAW_135",                   0,   0, 135),
    Rotation("ROTATION_YAW_180",                   0,   0, 180),
    Rotation("ROTATION_YAW_225",                   0,   0, 225),
    Rotation("ROTATION_YAW_270",                   0,   0, 270),
    Rotation("ROTATION_YAW_315",                   0,   0, 315),
    Rotation("ROTATION_ROLL_180",                180,   0,   0),
    Rotation("ROTATION_ROLL_180_YAW_45",         180,   0,  45),
    Rotation("ROTATION_ROLL_180_YAW_90",         180,   0,  90),
    Rotation("ROTATION_ROLL_180_YAW_135",        180,   0, 135),
    Rotation("ROTATION_PITCH_180",                 0, 180,   0),
    Rotation("ROTATION_ROLL_180_YAW_225",        180,   0, 225),
    Rotation("ROTATION_ROLL_180_YAW_270",        180,   0, 270),
    Rotation("ROTATION_ROLL_180_YAW_315",        180,   0, 315),
    Rotation("ROTATION_ROLL_90",                  90,   0,   0),
    Rotation("ROTATION_ROLL_90_YAW_45",           90,   0,  45),
    Rotation("ROTATION_ROLL_90_YAW_90",           90,   0,  90),
    Rotation("ROTATION_ROLL_90_YAW_135",          90,   0, 135),
    Rotation("ROTATION_ROLL_270",                270,   0,   0),
    Rotation("ROTATION_ROLL_270_YAW_45",         270,   0,  45),
    Rotation("ROTATION_ROLL_270_YAW_90",         270,   0,  90),
    Rotation("ROTATION_ROLL_270_YAW_135",        270,   0, 135),
    Rotation("ROTATION_PITCH_90",                  0,  90,   0),
    Rotation("ROTATION_PITCH_270",                 0, 270,   0),
    Rotation("ROTATION_PITCH_180_YAW_90",          0, 180,  90),
    Rotation("ROTATION_PITCH_180_YAW_270",         0, 180, 270),
    Rotation("ROTATION_ROLL_90_PITCH_90",         90,  90,   0),
    Rotation("ROTATION_ROLL_180_PITCH_90",       180,  90,   0),
    Rotation("ROTATION_ROLL_270_PITCH_90",       270,  90,   0),
    Rotation("ROTATION_ROLL_90_PITCH_180",        90, 180,   0),
    Rotation("ROTATION_ROLL_270_PITCH_180",      270, 180,   0),
    Rotation("ROTATION_ROLL_90_PITCH_270",        90, 270,   0),
    Rotation("ROTATION_ROLL_180_PITCH_270",      180, 270,   0),
    Rotation("ROTATION_ROLL_270_PITCH_270",      270, 270,   0),
    Rotation("ROTATION_ROLL_90_PITCH_180_YAW_90", 90, 180,  90),
    Rotation("ROTATION_ROLL_90_YAW_270",          90,   0, 270),
    Rotation("ROTATION_ROLL_90_PITCH_68_YAW_293", 90,  68, 270),
    Rotation("ROTATION_PITCH_315",                 0, 315,   0),
    Rotation("ROTATION_ROLL_90_PITCH_315",        90, 315,   0),
    Rotation("ROTATION_PITCH_7",                   0,   7,   0),
    ]


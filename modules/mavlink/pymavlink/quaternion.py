#!/usr/bin/env python3


"""
Quaternion implementation for use in pymavlink
"""


import numpy as np
from .rotmat import Vector3, Matrix3

__author__ = "Thomas Gubler"
__copyright__ = "Copyright (C) 2014 Thomas Gubler"
__license__ = "GNU Lesser General Public License v3"
__email__ = "thomasgubler@gmail.com"


class QuaternionBase(object):

    """
    Quaternion class, this is the version which supports numpy arrays
    If you need support for Matrix3 look at the Quaternion class

    Usage:
        >>> from quaternion import QuaternionBase
        >>> import numpy as np
        >>> q = QuaternionBase([np.radians(20), np.radians(20), np.radians(20)])
        >>> print(q)
        [ 0.9603483   0.13871646  0.19810763  0.13871646]
        >>> print(q.dcm)
        [[ 0.88302222 -0.21147065  0.41898917]
         [ 0.3213938   0.92303098 -0.21147065]
         [-0.34202014  0.3213938   0.88302222]]
        >>> q = QuaternionBase([1, 0, 0, 0])
        >>> print(q.euler)
        [ 0. -0.  0.]
        >>> m = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
        >>> q = QuaternionBase(m)
        >>> vector = [0, 1, 0]
        >>> vector2 = q.transform(vector)
    """

    def __init__(self, attitude=[1, 0, 0, 0]):
        """
        Construct a quaternion from an attitude

        :param attitude: another QuaternionBase,
            3 element list [roll, pitch, yaw],
            4 element list [w, x, y ,z], DCM (3x3 array)
        """
        if isinstance(attitude, QuaternionBase):
            self.q = attitude.q
        elif np.array(attitude).shape == (3, 3):
            self.dcm = attitude
        elif len(attitude) == 4:
            self.q = attitude
        elif len(attitude) == 3:
            self.euler = attitude
        else:
            raise TypeError("attitude is not valid")

    @property
    def q(self):
        """
        Get the quaternion
        :returns: array containing the quaternion elements
        """
        if self._q is None:
            if self._euler is not None:
                # get q from euler
                self._q = self._euler_to_q(self.euler)
            elif self._dcm is not None:
                # get q from DCM
                self._q = self._dcm_to_q(self.dcm)
        return self._q

    def __getitem__(self, index):
        """Returns the quaternion entry at index"""
        return self.q[index]

    @q.setter
    def q(self, q):
        """
        Set the quaternion
        :param q: list or array of quaternion values [w, x, y, z]
        """
        self._q = np.array(q)

        # mark other representations as outdated, will get generated on next
        # read
        self._euler = None
        self._dcm = None

    @property
    def euler(self):
        """
        Get the euler angles.
        The convention is Tait-Bryan (ZY'X'')

        :returns: array containing the euler angles [roll, pitch, yaw]
        """
        if self._euler is None:
            if self._q is not None:
                # try to get euler angles from q via DCM
                self._dcm = self._q_to_dcm(self.q)
                self._euler = self._dcm_to_euler(self.dcm)
            elif self._dcm is not None:
                # get euler angles from DCM
                self._euler = self._dcm_to_euler(self.dcm)
        return self._euler

    @euler.setter
    def euler(self, euler):
        """
        Set the euler angles
        :param euler: list or array of the euler angles [roll, pitch, yaw]

        """
        assert(len(euler) == 3)
        self._euler = np.array(euler)

        # mark other representations as outdated, will get generated on next
        # read
        self._q = None
        self._dcm = None

    @property
    def dcm(self):
        """
        Get the DCM

        :returns: 3x3 array
        """
        if self._dcm is None:
            if self._q is not None:
                # try to get dcm from q
                self._dcm = self._q_to_dcm(self.q)
            elif self._euler is not None:
                # try to get get dcm from euler
                self._dcm = self._euler_to_dcm(self._euler)
        return self._dcm

    @dcm.setter
    def dcm(self, dcm):
        """
        Set the DCM
        :param dcm: 3x3 array

        """
        assert(len(dcm) == 3)
        for sub in dcm:
            assert(len(sub) == 3)

        self._dcm = np.array(dcm)

        # mark other representations as outdated, will get generated on next
        # read
        self._q = None
        self._euler = None

    def transform(self, v):
        """
        Calculates the vector transformed by this quaternion
        :param v: array with len 3 to be transformed
        :returns: transformed vector
        """
        assert(len(v) == 3)
        assert(np.allclose(self.norm, 1))
        # perform transformation t = q * [0, v] * q^-1 but avoid multiplication
        # because terms cancel out
        q0 = self.q[0]
        qi = self.q[1:4]
        ui = np.array(v)
        a = q0 * ui + np.cross(qi, ui)
        t = np.dot(qi, ui) * qi + q0 * a - np.cross(a, qi)
        return t

    @property
    def norm(self):
        """
        Returns norm of quaternion

        :returns: norm (scalar)
        """
        return QuaternionBase.norm_array(self.q)

    def normalize(self):
        """Normalizes the quaternion"""
        self.q = QuaternionBase.normalize_array(self.q)

    @property
    def inversed(self):
        """
        Get inversed quaternion

        :returns: inversed quaternion
        """
        q_inv = self._q_inversed(self.q)
        return QuaternionBase(q_inv)

    def __eq__(self, other):
        """
        Equality test (same orientation, not necessarily same rotation)

        :param other: a QuaternionBase
        :returns: true if the quaternions are equal
        """
        if isinstance(other, QuaternionBase):
            return abs(self.q.dot(other.q)) > 1 - np.finfo(float).eps
        return NotImplemented

    def close(self, other):
        """
        Equality test with tolerance
        (same orientation, not necessarily same rotation)


        :param other: a QuaternionBase
        :returns: true if the quaternions are almost equal
        """
        if isinstance(other, QuaternionBase):
            return np.allclose(self.q, other.q) or np.allclose(self.q, -other.q)
        return NotImplemented

    def __mul__(self, other):
        """
        :param other: QuaternionBase
        :returns: multiplaction of this Quaternion with other
        """
        if isinstance(other, QuaternionBase):
            o = other.q
        elif len(other) == 4:
            o = other
        else:
            return NotImplemented

        return QuaternionBase(self._mul_array(self.q, o))

    def __truediv__(self, other):
        """
        :param other: QuaternionBase
        :returns: division of this Quaternion with other
        """
        if isinstance(other, QuaternionBase):
            o = other
        elif len(other) == 4:
            o = QuaternionBase(other)
        else:
            return NotImplemented
        return self * o.inversed

    @staticmethod
    def normalize_array(q):
        """
        Normalizes the list with len 4 so that it can be used as quaternion
        :param q: array of len 4
        :returns: normalized array
        """
        assert(len(q) == 4)
        q = np.array(q)
        n = QuaternionBase.norm_array(q)
        return q / n

    @staticmethod
    def norm_array(q):
        """
        Calculate quaternion norm on array q
        :param quaternion: array of len 4
        :returns: norm (scalar)
        """
        assert(len(q) == 4)
        return np.sqrt(np.dot(q, q))

    def _mul_array(self, p, q):
        """
        Performs multiplication of the 2 quaterniona arrays p and q
        :param p: array of len 4
        :param q: array of len 4
        :returns: array of len, result of p * q (with p, q quaternions)
        """
        assert(len(q) == len(p) == 4)
        p0 = p[0]
        pi = p[1:4]
        q0 = q[0]
        qi = q[1:4]

        res = np.zeros(4)
        res[0] = p0 * q0 - np.dot(pi, qi)
        res[1:4] = p0 * qi + q0 * pi + np.cross(pi, qi)

        return res

    def _euler_to_q(self, euler):
        """
        Create q array from euler angles
        :param euler: array [roll, pitch, yaw] in rad
        :returns: array q which represents a quaternion [w, x, y, z]
        """
        assert(len(euler) == 3)
        phi = euler[0]
        theta = euler[1]
        psi = euler[2]
        c_phi_2 = np.cos(phi / 2)
        s_phi_2 = np.sin(phi / 2)
        c_theta_2 = np.cos(theta / 2)
        s_theta_2 = np.sin(theta / 2)
        c_psi_2 = np.cos(psi / 2)
        s_psi_2 = np.sin(psi / 2)
        q = np.zeros(4)
        q[0] = (c_phi_2 * c_theta_2 * c_psi_2 +
                s_phi_2 * s_theta_2 * s_psi_2)
        q[1] = (s_phi_2 * c_theta_2 * c_psi_2 -
                c_phi_2 * s_theta_2 * s_psi_2)
        q[2] = (c_phi_2 * s_theta_2 * c_psi_2 +
                s_phi_2 * c_theta_2 * s_psi_2)
        q[3] = (c_phi_2 * c_theta_2 * s_psi_2 -
                s_phi_2 * s_theta_2 * c_psi_2)
        return q

    def _q_to_dcm(self, q):
        """
        Create DCM from q
        :param q: array q which represents a quaternion [w, x, y, z]
        :returns: 3x3 dcm array
        """
        assert(len(q) == 4)
        assert(np.allclose(QuaternionBase.norm_array(q), 1))
        dcm = np.zeros([3, 3])
        a = q[0]
        b = q[1]
        c = q[2]
        d = q[3]
        a_sq = a * a
        b_sq = b * b
        c_sq = c * c
        d_sq = d * d
        dcm[0][0] = a_sq + b_sq - c_sq - d_sq
        dcm[0][1] = 2 * (b * c - a * d)
        dcm[0][2] = 2 * (a * c + b * d)
        dcm[1][0] = 2 * (b * c + a * d)
        dcm[1][1] = a_sq - b_sq + c_sq - d_sq
        dcm[1][2] = 2 * (c * d - a * b)
        dcm[2][0] = 2 * (b * d - a * c)
        dcm[2][1] = 2 * (a * b + c * d)
        dcm[2][2] = a_sq - b_sq - c_sq + d_sq
        return dcm

    def _dcm_to_q(self, dcm):
        """
        Create q from dcm
        Reference:
            - Shoemake, Quaternions,
            http://www.cs.ucr.edu/~vbz/resources/quatut.pdf

        :param dcm: 3x3 dcm array
        returns: quaternion array
        """
        assert(dcm.shape == (3, 3))
        q = np.zeros(4)

        tr = np.trace(dcm)
        if tr > 0:
            s = np.sqrt(tr + 1.0)
            q[0] = s * 0.5
            s = 0.5 / s
            q[1] = (dcm[2][1] - dcm[1][2]) * s
            q[2] = (dcm[0][2] - dcm[2][0]) * s
            q[3] = (dcm[1][0] - dcm[0][1]) * s
        else:
            dcm_i = np.argmax(np.diag(dcm))
            dcm_j = (dcm_i + 1) % 3
            dcm_k = (dcm_i + 2) % 3

            s = np.sqrt((dcm[dcm_i][dcm_i] - dcm[dcm_j][dcm_j] -
                         dcm[dcm_k][dcm_k]) + 1.0)
            q[dcm_i + 1] = s * 0.5
            s = 0.5 / s
            q[dcm_j + 1] = (dcm[dcm_i][dcm_j] + dcm[dcm_j][dcm_i]) * s
            q[dcm_k + 1] = (dcm[dcm_k][dcm_i] + dcm[dcm_i][dcm_k]) * s
            q[0] = (dcm[dcm_k][dcm_j] - dcm[dcm_j][dcm_k]) * s

        return q

    def _euler_to_dcm(self, euler):
        """
        Create DCM from euler angles
        :param euler: array [roll, pitch, yaw] in rad
        :returns: 3x3 dcm array
        """
        assert(len(euler) == 3)
        phi = euler[0]
        theta = euler[1]
        psi = euler[2]
        dcm = np.zeros([3, 3])
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)

        dcm[0][0] = c_theta * c_psi
        dcm[0][1] = -c_phi * s_psi + s_phi * s_theta * c_psi
        dcm[0][2] = s_phi * s_psi + c_phi * s_theta * c_psi

        dcm[1][0] = c_theta * s_psi
        dcm[1][1] = c_phi * c_psi + s_phi * s_theta * s_psi
        dcm[1][2] = -s_phi * c_psi + c_phi * s_theta * s_psi

        dcm[2][0] = -s_theta
        dcm[2][1] = s_phi * c_theta
        dcm[2][2] = c_phi * c_theta
        return dcm

    def _dcm_to_euler(self, dcm):
        """
        Create DCM from euler angles
        :param dcm: 3x3 dcm array
        :returns: array [roll, pitch, yaw] in rad
        """
        assert(dcm.shape == (3, 3))
        theta = np.arcsin(min(1, max(-1, -dcm[2][0])))

        if abs(theta - np.pi/2) < 1.0e-3:
            phi = 0.0
            psi = (np.arctan2(dcm[1][2] - dcm[0][1],
                              dcm[0][2] + dcm[1][1]) + phi)
        elif abs(theta + np.pi/2) < 1.0e-3:
            phi = 0.0
            psi = np.arctan2(dcm[1][2] - dcm[0][1],
                             dcm[0][2] + dcm[1][1] - phi)
        else:
            phi = np.arctan2(dcm[2][1], dcm[2][2])
            psi = np.arctan2(dcm[1][0], dcm[0][0])

        return np.array([phi, theta, psi])

    def _q_inversed(self, q):
        """
        Returns inversed quaternion q
        :param q: array q which represents a quaternion [w, x, y, z]
        :returns: inversed array q which is a quaternion [w, x, y ,z]
        """
        assert(len(q) == 4)
        return np.hstack([q[0], -q[1:4]])

    def __str__(self):
        """String of quaternion values"""
        return str(self.q)


class Quaternion(QuaternionBase):

    """
    Quaternion class that supports pymavlink's Vector3 and Matrix3

    Usage:
        >>> from quaternion import Quaternion
        >>> from rotmat import Vector3, Matrix3
        >>> m = Matrix3()
        >>> m.from_euler(45, 0, 0)
        >>> print(m)
        Matrix3((1.00, 0.00, 0.00), (0.00, 0.53, -0.85), (-0.00, 0.85, 0.53))
        >>> q = Quaternion(m)
        >>> print(q)
        [ 0.87330464  0.48717451  0.          0.        ]
        >>> print(q.dcm)
        Matrix3((1.00, 0.00, 0.00), (0.00, 0.53, -0.85), (-0.00, 0.85, 0.53))
        >>> v = Vector3(0, 1, 0)
        >>> v2 = q.transform(v)
        >>> print(v2)
        Vector3(0.00, 0.53, 0.85)
    """

    def __init__(self, attitude):
        """
        Construct a quaternion from an attitude

        :param attitude: another Quaternion, QuaternionBase,
            3 element list [roll, pitch, yaw],
            4 element list [w, x, y ,z], DCM (3x3 array or Matrix3)
        """
        if isinstance(attitude, Quaternion):
            self.q = attitude.q
        if isinstance(attitude, Matrix3):
            self.dcm = attitude
        elif np.array(attitude).shape == (3, 3):
            # convert dcm array to Matrix3
            self.dcm = self._dcm_array_to_matrix3(attitude)
        elif isinstance(attitude, Vector3):
            # provided euler angles
            euler = [attitude.x, attitude.y, attitude.z]
            super(Quaternion, self).__init__(euler)
        else:
            super(Quaternion, self).__init__(attitude)

    @property
    def dcm(self):
        """
        Get the DCM

        :returns: Matrix3
        """
        if self._dcm is None:
            if self._q is not None:
                # try to get dcm from q
                self._dcm = self._q_to_dcm(self.q)
            elif self._euler is not None:
                # try to get get dcm from euler
                self._dcm = self._euler_to_dcm(self._euler)
        return self._dcm

    @dcm.setter
    def dcm(self, dcm):
        """
        Set the DCM
        :param dcm: Matrix3

        """
        assert(isinstance(dcm, Matrix3))
        self._dcm = dcm.copy()

        # mark other representations as outdated, will get generated on next
        # read
        self._q = None
        self._euler = None

    @property
    def inversed(self):
        """
        Get inversed quaternion

        :returns: inversed quaternion
        """
        return Quaternion(super(Quaternion, self).inversed)

    def transform(self, v3):
        """
        Calculates the vector transformed by this quaternion
        :param v3: Vector3 to be transformed
        :returns: transformed vector
        """
        if isinstance(v3, Vector3):
            t = super(Quaternion, self).transform([v3.x, v3.y, v3.z])
            return Vector3(t[0], t[1], t[2])
        elif len(v3) == 3:
            return super(Quaternion, self).transform(v3)
        else:
            raise TypeError("param v3 is not a vector type")

    def _dcm_array_to_matrix3(self, dcm):
        """
        Converts dcm array into Matrix3
        :param dcm: 3x3 dcm array
        :returns: Matrix3
        """
        assert(dcm.shape == (3, 3))
        a = Vector3(dcm[0][0], dcm[0][1], dcm[0][2])
        b = Vector3(dcm[1][0], dcm[1][1], dcm[1][2])
        c = Vector3(dcm[2][0], dcm[2][1], dcm[2][2])
        return Matrix3(a, b, c)

    def _matrix3_to_dcm_array(self, m):
        """
        Converts Matrix3 in an array
        :param m: Matrix3
        :returns: 3x3 array
        """
        assert(isinstance(m, Matrix3))
        return np.array([[m.a.x, m.a.y, m.a.z],
                         [m.b.x, m.b.y, m.b.z],
                         [m.c.x, m.c.y, m.c.z]])

    def _q_to_dcm(self, q):
        """
        Create DCM (Matrix3) from q
        :param q: array q which represents a quaternion [w, x, y, z]
        :returns: Matrix3
        """
        assert(len(q) == 4)
        arr = super(Quaternion, self)._q_to_dcm(q)
        return self._dcm_array_to_matrix3(arr)

    def _dcm_to_q(self, dcm):
        """
        Create q from dcm (Matrix3)
        :param dcm: Matrix3
        :returns: array q which represents a quaternion [w, x, y, z]
        """
        assert(isinstance(dcm, Matrix3))
        arr = self._matrix3_to_dcm_array(dcm)
        return super(Quaternion, self)._dcm_to_q(arr)

    def _euler_to_dcm(self, euler):
        """
        Create DCM (Matrix3) from euler angles
        :param euler: array [roll, pitch, yaw] in rad
        :returns: Matrix3
        """
        assert(len(euler) == 3)
        m = Matrix3()
        m.from_euler(*euler)
        return m

    def _dcm_to_euler(self, dcm):
        """
        Create DCM from euler angles
        :param dcm: Matrix3
        :returns: array [roll, pitch, yaw] in rad
        """
        assert(isinstance(dcm, Matrix3))
        return np.array(dcm.to_euler())
    
    def __mul__(self, other):
        """
        :param other: Quaternion
        :returns: multiplaction of this Quaternion with other
        """
        return Quaternion(super(Quaternion, self).__mul__(other))

    def __truediv__(self, other):
        """
        :param other: Quaternion
        :returns: division of this Quaternion with other
        """
        return Quaternion(super(Quaternion, self).__truediv__(other))

if __name__ == "__main__":
    import doctest
    doctest.testmod()
    import unittest
    unittest.main('quaterniontest')

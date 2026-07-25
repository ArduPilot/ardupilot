#!/usr/bin/env python3


"""
Unit tests for the quaternion library
"""

import unittest
import numpy as np
from pymavlink.quaternion import QuaternionBase, Quaternion
from pymavlink.rotmat import Vector3, Matrix3

__author__ = "Thomas Gubler"
__copyright__ = "Copyright (C) 2014 Thomas Gubler"
__license__ = "GNU Lesser General Public License v3"
__email__ = "thomasgubler@gmail.com"


class QuaternionBaseTest(unittest.TestCase):

    """
    Class to test QuaternionBase
    """

    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(QuaternionBaseTest, self).__init__(*args, **kwargs)
        self.quaternions = self._all_quaternions()

    def test_constructor(self):
        """Test the constructor functionality"""
        # Test the identity case
        q = [1, 0, 0, 0]
        euler = [0, 0, 0]
        dcm = np.eye(3)
        self._helper_test_constructor(q, euler, dcm)

        # test a case with rotations around all euler angles
        q = [0.707106781186547, 0, 0.707106781186547, 0]
        euler = [np.radians(90), np.radians(90), np.radians(90)]
        dcm = [[0, 0, 1],
               [0, 1, 0],
               [-1, 0, 0]]
        # test a case with rotations around all angles (values from matlab)
        q = [0.774519052838329, 0.158493649053890, 0.591506350946110,
             0.158493649053890]
        euler = [np.radians(60), np.radians(60), np.radians(60)]
        dcm = [[0.25, -0.058012701892219, 0.966506350946110],
               [0.433012701892219, 0.899519052838329, -0.058012701892219],
               [-0.866025403784439, 0.433012701892219, 0.25]]
        self._helper_test_constructor(q, euler, dcm)

        # test another case (values from matlab)
        q = [0.754971823897152, 0.102564313848771, -0.324261369073765,
             -0.560671625082406]
        euler = [np.radians(34), np.radians(-22), np.radians(-80)]
        dcm = [[0.161003786707723, 0.780067269138261, -0.604626195500121],
               [-0.913097848445116, 0.350255780704370, 0.208741963313735],
               [0.374606593415912, 0.518474631686401, 0.768670252102276]]
        self._helper_test_constructor(q, euler, dcm)

    def _helper_test_constructor(self, q, euler, dcm):
        """
        Helper function for constructor test

        Calls constructor for the quaternion from q euler and dcm and checks
        if the resulting conversions are equivalent to the arguments.
        The test for the euler angles is weak as the solution is not unique

        :param q: quaternion 4x1, [w, x, y, z]
        :param euler: [roll, pitch, yaw], needs to be equivalent to q
        :param q: dcm 3x3, needs to be equivalent to q
        """
        # construct q from a QuaternionBase
        quaternion_instance = QuaternionBase(q)
        q_test = QuaternionBase(quaternion_instance)
        np.testing.assert_almost_equal(q_test.q, q)
        q_test = QuaternionBase(quaternion_instance)
        np.testing.assert_almost_equal(q_test.dcm, dcm)
        q_test = QuaternionBase(quaternion_instance)
        q_euler = QuaternionBase(q_test.euler)
        assert(np.allclose(q_test.euler, euler) or
               np.allclose(q_test.q, q_euler.q))

        # construct q from a quaternion
        q_test = QuaternionBase(q)
        np.testing.assert_almost_equal(q_test.q, q)
        q_test = QuaternionBase(q)
        np.testing.assert_almost_equal(q_test.dcm, dcm)
        q_test = QuaternionBase(q)
        q_euler = QuaternionBase(q_test.euler)
        assert(np.allclose(q_test.euler, euler) or
               np.allclose(q_test.q, q_euler.q))

        # construct q from a euler angles
        q_test = QuaternionBase(euler)
        np.testing.assert_almost_equal(q_test.q, q)
        q_test = QuaternionBase(euler)
        np.testing.assert_almost_equal(q_test.dcm, dcm)
        q_test = QuaternionBase(euler)
        q_euler = QuaternionBase(q_test.euler)
        assert(np.allclose(q_test.euler, euler) or
               np.allclose(q_test.q, q_euler.q))

        # construct q from dcm
        q_test = QuaternionBase(dcm)
        np.testing.assert_almost_equal(q_test.q, q)
        q_test = QuaternionBase(dcm)
        np.testing.assert_almost_equal(q_test.dcm, dcm)
        q_test = QuaternionBase(dcm)
        q_euler = QuaternionBase(q_test.euler)
        assert(np.allclose(q_test.euler, euler) or
               np.allclose(q_test.q, q_euler.q))

    def test_norm(self):
        # """Tests the norm functions"""
        qa = [1, 2, 3, 4]
        q = QuaternionBase(qa)
        n = np.sqrt(np.dot(qa, qa))
        qan = qa / n

        self.assertAlmostEqual(n, QuaternionBase.norm_array(qa))
        np.testing.assert_almost_equal(qan, QuaternionBase.normalize_array(qa))
        np.testing.assert_almost_equal(n, q.norm)
        q.normalize()
        np.testing.assert_almost_equal(qan, q.q)
        self.assertAlmostEqual(1, q.norm)

    def _all_angles(self, step=np.radians(45)):
        """
        Creates a list of all euler angles

        :param step: stepsixe in radians
        :returns: euler angles [[phi, thea, psi], [phi, theta, psi], ...]
        """
        e = 0.5
        r_phi = np.arange(-np.pi + e, np.pi - e, step)
        r_theta = np.arange(-np.pi/2 + e, np.pi/2 - e, step)
        r_psi = np.arange(-np.pi + e, np.pi - e, step)
        return [[phi, theta, psi] for phi in r_phi for theta in r_theta
                for psi in r_psi]

    def _all_quaternions(self):
        """Generate quaternions from all euler angles"""
        return [QuaternionBase(e) for e in self._all_angles()]

    def test_conversion(self):
        """
        Tests forward and backward conversions
        """
        for q in self.quaternions:
            # quaternion -> euler -> quaternion
            q0 = q
            e = QuaternionBase(q.q).euler
            q1 = QuaternionBase(e)
            assert q0.close(q1)

            # quaternion -> dcm -> quaternion
            q0 = q
            dcm = QuaternionBase(q.q).dcm
            q1 = QuaternionBase(dcm)
            assert q0.close(q1)

    def test_inversed(self):
        """Test inverse"""
        for q in self.quaternions:
            q_inv = q.inversed
            q_inv_inv = q_inv.inversed
            assert q.close(q_inv_inv)

    def test_mul(self):
        """Test multiplication"""
        for q in self.quaternions:
            for p in self.quaternions:
                assert q.close(p * p.inversed * q)
                r = p * q
                r_dcm = np.dot(p.dcm, q.dcm)
                np.testing.assert_almost_equal(r_dcm, r.dcm)

    def test_div(self):
        """Test division"""
        for q in self.quaternions:
            for p in self.quaternions:
                mul = q * p.inversed
                div = q / p
                assert mul.close(div)

    def test_transform(self):
        """Test transform"""
        for q in self.quaternions:
            q_inv = q.inversed
            v = np.array([1, 2, 3])
            v1 = q.transform(v)
            v1_dcm = np.dot(q.dcm, v)
            np.testing.assert_almost_equal(v1, v1_dcm)
            # test versus slower solution using multiplication
            v1_mul = q * QuaternionBase(np.hstack([0, v])) * q.inversed
            np.testing.assert_almost_equal(v1, v1_mul[1:4])
            v2 = q_inv.transform(v1)
            np.testing.assert_almost_equal(v, v2)


class QuaternionTest(QuaternionBaseTest):
    """
    Class to test Quaternion
    """

    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(QuaternionTest, self).__init__(*args, **kwargs)
        self.quaternions = self._all_quaternions()

    def _all_quaternions(self):
        """Generate quaternions from all euler angles"""
        return [Quaternion(e) for e in self._all_angles()]

    def test_constructor(self):
        """Test the constructor functionality"""
        # Test the identity case
        q = [1, 0, 0, 0]
        euler = [0, 0, 0]
        dcm = Matrix3()
        self._helper_test_constructor(q, euler, dcm)

        # test a case with rotations around all angles (values from matlab)
        q = [0.774519052838329, 0.158493649053890, 0.591506350946110,
             0.158493649053890]
        euler = [np.radians(60), np.radians(60), np.radians(60)]
        dcm = Matrix3(Vector3(0.25, -0.058012701892219, 0.966506350946110),
                      Vector3(0.433012701892219, 0.899519052838329,
                              -0.058012701892219),
                      Vector3(-0.866025403784439, 0.433012701892219, 0.25))

        self._helper_test_constructor(q, euler, dcm)

    def _helper_test_constructor(self, q, euler, dcm):
        """
        Helper function for constructor test

        Calls constructor for the quaternion from q euler and dcm and checks
        if the resulting conversions are equivalent to the arguments.
        The test for the euler angles is weak as the solution is not unique

        :param q: quaternion 4x1, [w, x, y, z]
        :param euler: Vector3(roll, pitch, yaw), needs to be equivalent to q
        :param q: Matrix3, needs to be equivalent to q
        """
        # construct q from a Quaternion
        quaternion_instance = Quaternion(q)
        q_test = Quaternion(quaternion_instance)
        np.testing.assert_almost_equal(q_test.q, q)
        q_test = Quaternion(quaternion_instance)
        assert q_test.dcm.close(dcm)
        q_test = Quaternion(quaternion_instance)
        q_euler = Quaternion(q_test.euler)
        assert(np.allclose(q_test.euler, euler) or
               np.allclose(q_test.q, q_euler.q))

        # construct q from a QuaternionBase
        quaternion_instance = QuaternionBase(q)
        q_test = Quaternion(quaternion_instance)
        np.testing.assert_almost_equal(q_test.q, q)
        q_test = Quaternion(quaternion_instance)
        assert q_test.dcm.close(dcm)
        q_test = Quaternion(quaternion_instance)
        q_euler = Quaternion(q_test.euler)
        assert(np.allclose(q_test.euler, euler) or
               np.allclose(q_test.q, q_euler.q))

        # construct q from a quaternion
        q_test = Quaternion(q)
        np.testing.assert_almost_equal(q_test.q, q)
        q_test = Quaternion(q)
        assert q_test.dcm.close(dcm)
        q_test = Quaternion(q)
        q_euler = Quaternion(q_test.euler)
        assert(np.allclose(q_test.euler, euler) or
               np.allclose(q_test.q, q_euler.q))

        # # construct q from a euler angles
        q_test = Quaternion(euler)
        np.testing.assert_almost_equal(q_test.q, q)
        q_test = Quaternion(euler)
        assert q_test.dcm.close(dcm)
        q_test = Quaternion(euler)
        q_euler = Quaternion(q_test.euler)
        assert(np.allclose(q_test.euler, euler) or
               np.allclose(q_test.q, q_euler.q))

        # # construct q from dcm (Matrix3 instance)
        q_test = Quaternion(dcm)
        np.testing.assert_almost_equal(q_test.q, q)
        q_test = Quaternion(dcm)
        assert q_test.dcm.close(dcm)
        q_test = Quaternion(dcm)
        q_euler = Quaternion(q_test.euler)
        assert(np.allclose(q_test.euler, euler) or
               np.allclose(q_test.q, q_euler.q))

    def test_conversion(self):
        """
        Tests forward and backward conversions
        """
        for q in self.quaternions:
            # quaternion -> euler -> quaternion
            q0 = q
            e = Quaternion(q.q).euler
            q1 = Quaternion(e)
            assert q0.close(q1)

            # quaternion -> dcm (Matrix3) -> quaternion
            q0 = q
            dcm = Quaternion(q.q).dcm
            q1 = Quaternion(dcm)
            assert q0.close(q1)

    def test_transform(self):
        """Test transform"""
        for q in self.quaternions:
            q_inv = q.inversed
            v = Vector3(1, 2, 3)
            v1 = q.transform(v)
            v1_dcm = q.dcm * v
            assert v1.close(v1_dcm)
            v2 = q_inv.transform(v1)
            assert v.close(v2)

    def test_mul(self):
        """Test multiplication"""
        for q in self.quaternions:
            for p in self.quaternions:
                assert q.close(p * p.inversed * q)
                r = p * q
                r_dcm = p.dcm * q.dcm
                assert r_dcm.close(r.dcm)


if __name__ == '__main__':
    unittest.main()

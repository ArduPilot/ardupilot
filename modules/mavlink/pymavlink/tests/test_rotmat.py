#!/usr/bin/env python3


"""
Unit tests for the rotmat library
"""

from math import radians, degrees
import unittest
import random
import numpy as np

from pymavlink.rotmat import Vector3, Matrix3, Plane, Line

class VectorTest(unittest.TestCase):

    """
    Class to test Vector3
    """

    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(VectorTest, self).__init__(*args, **kwargs)


    def test_constructor(self):
        """Test the constructor functionality"""
        v1 = Vector3(1, 0.2, -3)
        v2 = Vector3([1, 0.2, -3])
        v3 = Vector3([1, 0.3, -3])

        assert v1 == v2
        assert v1 != v3
        assert str(v1) == "Vector3(1.00, 0.20, -3.00)"


    def test_maths(self):
        """Test simple maths"""
        v1 = Vector3(1, 2, -3)
        v2 = Vector3(1, 3, 3)

        assert v1 + v2 == Vector3(2, 5, 0)
        assert v1 - v2 == Vector3(0, -1, -6)
        assert (v1 * 3) == Vector3(3, 6, -9)
        assert v1 * v2 == -2
        assert v2 / 2.0 == Vector3(0.5, 1.5, 1.5)
        assert v2 // 2.0 == Vector3(0, 1, 1)
        assert v2 / 2.1 == Vector3(0.47619047619047616, 1.4285714285714286, 1.4285714285714286)
        assert v2 // 2.1 == Vector3(0.0, 1.0, 1.0)

        assert v1 % v2 == Vector3(15.00, -6.00, 1.00)
        np.testing.assert_almost_equal(v2.length(), 4.358898943540674)
        assert v2.normalized().close(Vector3(0.23, 0.69, 0.69), tol=1e-2)
        np.testing.assert_almost_equal(v1.angle(v2), 1.693733631245806)


class MatrixTest(unittest.TestCase):

    """
    Class to test Matrix3
    """

    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(MatrixTest, self).__init__(*args, **kwargs)

    def test_constructor(self):
        """Test the constructor functionality"""
        m1 = Matrix3(Vector3(1, 0, 0), Vector3(1, 5, 0), Vector3(1, 0, -7))
        m2 = Matrix3()

        assert str(m1) == 'Matrix3((1.00, 0.00, 0.00), (1.00, 5.00, 0.00), (1.00, 0.00, -7.00))'
        assert str(m2) == 'Matrix3((1.00, 0.00, 0.00), (0.00, 1.00, 0.00), (0.00, 0.00, 1.00))'

    def test_maths(self):
        m1 = Matrix3(Vector3(1, 0, 0), Vector3(1, 5, 0), Vector3(1, 0, -7))
        m2 = Matrix3()

        assert m1 + m2 == Matrix3(Vector3(2, 0, 0), Vector3(1, 6, 0), Vector3(1, 0, -6))
        assert m1 - m2 == Matrix3(Vector3(0, 0, 0), Vector3(1, 4, 0), Vector3(1, 0, -8))
        assert m1 * 3 == Matrix3(Vector3(3, 0, 0), Vector3(3, 15, 0), Vector3(3, 0, -21))
        assert m1 * m1 == Matrix3(Vector3(1, 0, 0), Vector3(6, 25, 0), Vector3(-6, 0, 49))
        assert m1 / 2.0 == Matrix3(Vector3(0.5, 0, 0), Vector3(0.5, 2.5, 0), Vector3(0.5, 0, -3.5))
        assert m1 / 0.5 == Matrix3(Vector3(2, 0, 0), Vector3(2, 10, 0), Vector3(2, 0, -14))
        assert m1.transposed() == Matrix3(Vector3(1, 1, 1), Vector3(0, 5, 0), Vector3(0, 0, -7))

    def test_euler(self):
        '''check that from_euler() and to_euler() are consistent'''
        m = Matrix3()
        for r in range(-179, 179, 10):
            for p in range(-89, 89, 10):
                for y in range(-179, 179, 10):
                    m.from_euler(radians(r), radians(p), radians(y))
                    (r2, p2, y2) = m.to_euler()
                    v1 = Vector3(r, p, y)
                    v2 = Vector3(degrees(r2), degrees(p2), degrees(y2))
                    diff = v1 - v2
                    assert diff.length() < 1.0e-12
                    # construct the rotation using 321 ordering
                    m2 = Matrix3()
                    m2.rotate_yaw(radians(y))
                    m2.rotate_pitch(radians(p))
                    m2.rotate_roll(radians(r))
                    (r2, p2, y2) = m2.to_euler()
                    v2 = Vector3(degrees(r2), degrees(p2), degrees(y2))
                    diff = v1 - v2
                    assert diff.length() < 1.0e-12
        # test rotate_321()
        (r1,p1,y1) = radians(17),radians(-35),radians(126)
        (r2,p2,y2) = radians(-39),radians(63),radians(-18)
        m.from_euler(r1,p1,y1)
        m2 = m.copy()
        m2.rotate_yaw(y2)
        m2.rotate_pitch(p2)
        m2.rotate_roll(r2)
        (r3,p3,y3) = m2.to_euler()
        v1 = Vector3(degrees(r3),degrees(p3),degrees(y3))
        m3 = m.copy()
        m3.rotate_321(r2,p2,y2)
        (r3,p3,y3) = m3.to_euler()
        v2 = Vector3(degrees(r3),degrees(p3),degrees(y3))
        diff = v1 - v2
        assert diff.length() < 1.0e-12


    def test_euler312(self):
        '''check that from_euler312() and to_euler312() are consistent'''
        m = Matrix3()
        for r in range(-89, 89, 10):
            for p in range(-179, 179, 10):
                for y in range(-179, 179, 10):
                    m.from_euler312(radians(r), radians(p), radians(y))
                    (r2, p2, y2) = m.to_euler312()
                    v1 = Vector3(r, p, y)
                    v2 = Vector3(degrees(r2), degrees(p2), degrees(y2))
                    diff = v1 - v2
                    assert diff.length() < 1.0e-12
                    # construct the rotation using 312 ordering
                    m2 = Matrix3()
                    m2.identity()
                    m2.rotate_yaw(radians(y))
                    m2.rotate_roll(radians(r))
                    m2.rotate_pitch(radians(p))
                    (r2, p2, y2) = m2.to_euler312()
                    v2 = Vector3(degrees(r2), degrees(p2), degrees(y2))
                    diff = v1 - v2
                    assert diff.length() < 1.0e-12
        # test rotate_312()
        (r1,p1,y1) = radians(17),radians(-35),radians(126)
        (r2,p2,y2) = radians(-39),radians(63),radians(-18)
        m.from_euler312(r1,p1,y1)
        m2 = m.copy()
        m2.rotate_yaw(y2)
        m2.rotate_roll(r2)
        m2.rotate_pitch(p2)
        (r3,p3,y3) = m2.to_euler312()
        v1 = Vector3(degrees(r3),degrees(p3),degrees(y3))
        m3 = m.copy()
        m3.rotate_312(r2,p2,y2)
        (r3,p3,y3) = m3.to_euler312()
        v2 = Vector3(degrees(r3),degrees(p3),degrees(y3))
        diff = v1 - v2
        assert diff.length() < 1.0e-12

    def test_matrixops(self):
        m1 = Matrix3(Vector3(1, 0, 0), Vector3(1, 5, 0), Vector3(1, 0, -7))

        m1.normalize()
        #print(m1)
        assert m1.close(Matrix3(Vector3(0.2, -0.98, 0), Vector3(0.1, 1, 0), Vector3(0, 0, 1)), tol=1e-2)
        np.testing.assert_almost_equal(m1.trace(), 2.19115332535)

        m1.rotate(Vector3(0.2,-0.98,0))
        assert m1.close(Matrix3(Vector3(0.2,-0.98,0), Vector3(0.1,1,-0.3), Vector3(0.98,0.2,1)), tol=1e-2)

    def test_axisangle(self):
        axis = Vector3(0, 1, 0)
        angle = radians(45)

        m1 = Matrix3()
        m1.from_axis_angle(axis, angle)
        #print(m1)
        assert m1.close(Matrix3(Vector3(0.71, 0.00, 0.71),
                                         Vector3(0.00, 1.00, 0.00),
                                         Vector3(-0.71, 0.00, 0.71)), tol=1e-2)

    def test_two_vectors(self):
        '''test the from_two_vectors() method'''
        for i in range(100):
            v1 = Vector3(1, 0.2, -3)
            v2 = Vector3(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5))
            m = Matrix3()
            m.from_two_vectors(v1, v2)
            v3 = m * v1
            diff = v3.normalized() - v2.normalized()
            (r, p, y) = m.to_euler()
            assert diff.length() < 0.001


class LinePlaneTest(unittest.TestCase):

    """
    Class to test Line and Plane classes
    """

    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(LinePlaneTest, self).__init__(*args, **kwargs)

    def test_plane(self):
        '''testing line/plane intersection'''
        plane = Plane(Vector3(0, 0, 0), Vector3(0, 0, 1))
        line = Line(Vector3(0, 0, 100), Vector3(10, 10, -90))
        p = line.plane_intersection(plane)
        assert p.close(Vector3(11.11, 11.11, 0.00), tol=1e-2)


if __name__ == '__main__':
    unittest.main()

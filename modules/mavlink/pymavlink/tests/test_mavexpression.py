#!/usr/bin/env python3


"""
Unit tests for the mavexpression library
"""
import unittest
import random

from pymavlink import mavexpression

class ExpressionTest(unittest.TestCase):

    """
    Class to test evaluate_expression
    """

    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        self.varsDict = {}
        self.varsDict['lat'] = 5.67
        self.varsDict['speed'] = 8
        super(ExpressionTest, self).__init__(*args, **kwargs)


    def test_novars(self):
        """Test the evaluate_expression functionality"""
        assert mavexpression.evaluate_expression('1+2', {}) == 3
        assert mavexpression.evaluate_expression('4/0', {}) is None
        assert mavexpression.evaluate_expression('A+4', {}) is None

    def test_vars(self):
        """Test the evaluate_expression functionality with local vars"""
        assert mavexpression.evaluate_expression('lat+10', self.varsDict) == 15.67
        assert mavexpression.evaluate_expression('4.0/speed', self.varsDict) == 0.5
        assert mavexpression.evaluate_expression('speed+lat+wrong', self.varsDict) is None
        
    def test_mavextra(self):
        """Test evaluate_expression using the functions in mavextra.py"""
        assert mavexpression.evaluate_expression('kmh(10)', {}) == 36
        assert mavexpression.evaluate_expression('angle_diff(170, -90)', {}) == -100
        
if __name__ == '__main__':
    unittest.main()

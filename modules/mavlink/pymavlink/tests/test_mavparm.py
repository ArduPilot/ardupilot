#!/usr/bin/env python3


"""
Unit tests for the mavparm library
"""

import unittest
import os

from pymavlink import mavparm

class MAVParmDictTest(unittest.TestCase):

    """
    Class to test MAVParmDict
    """

    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        self.parms = mavparm.MAVParmDict()
        self.parms['AFS_ACTION'] = 42
        self.parms['PARAM1'] = 34.45
        self.parms['PARAM2'] = 0
        self.parms['PARAM3'] = -13.4
        super(MAVParmDictTest, self).__init__(*args, **kwargs)


    def test_dict(self):
        """Test simple dict operations"""
        self.parms['AFS_ACTION'] = 34
        
        assert self.parms['AFS_ACTION'] == 34
        assert self.parms['PARAM1'] == 34.45

    def test_saveload(self):
        """Test the saving and loading to file"""
        self.parms.save('prms.txt')
        assert os.path.isfile('prms.txt')
        
        newparms = mavparm.MAVParmDict()
        newparms.load('prms.txt')
        os.remove('prms.txt')
        
        assert newparms['AFS_ACTION'] == self.parms['AFS_ACTION']
        assert newparms['PARAM3'] == self.parms['PARAM3']
        
        
        
    def test_showdiff(self):
        """Test show and diff functions"""
        self.parms.save('prms.txt')
        
        self.parms.show()
        
        self.parms.diff('prms.txt')
        
if __name__ == '__main__':
    unittest.main()

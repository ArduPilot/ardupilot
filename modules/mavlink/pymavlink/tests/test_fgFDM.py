#!/usr/bin/env python3


"""
Unit tests for the fgFDM library
"""
import unittest

from pymavlink.fgFDM import fgFDMError, fgFDMVariable, fgFDMVariableList, fgFDM


class fgFDMErrorTest(unittest.TestCase):
    """
    Class to test fgFDMError
    """
    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(fgFDMErrorTest, self).__init__(*args, **kwargs)
        
    def test_constructor(self):
        ex = fgFDMError("Test Exception {0}".format(1))
        
        assert ex.message == "fgFDMError: Test Exception 1"

class fgFDMVariableTest(unittest.TestCase):
    """
    Class to test fgFDMVariable
    """
    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(fgFDMVariableTest, self).__init__(*args, **kwargs)
        
    def test_constructor(self):
        """Test the constructor"""
        varry = fgFDMVariable(0, 3, 'radians')
        
        assert varry.index == 0
        assert varry.arraylength == 3
        assert varry.units == 'radians'

        
class fgFDMVariableListTest(unittest.TestCase):
    """
    Class to test fgFDMVariableList
    """
    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(fgFDMVariableListTest, self).__init__(*args, **kwargs)
        
    def test_constructor(self):
        """Test the constructor and adding variables"""
        mapping = fgFDMVariableList()
        mapping.add('longitude', units='radians')
        mapping.add('stall_warning')
        mapping.add('rpm', 4)
        
        assert mapping._nextidx == 6
        assert mapping.vars['longitude'].index == 0
        assert mapping.vars['longitude'].units == 'radians'
        assert mapping.vars['rpm'].index == 2
        assert mapping.vars['rpm'].units is None


class fgFDMTest(unittest.TestCase):
    """
    Class to test fgFDM
    """
    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(fgFDMTest, self).__init__(*args, **kwargs)
        
    def test_constructor(self):
        """Test the constructor"""
        fdm = fgFDM()
        
        assert fdm.FG_NET_FDM_VERSION == 24

    def test_getset(self):
        """Test the getting and setting and unit conversion of variables"""
        fdm = fgFDM()
        
        fdm.set('latitude', 67.4, units='degrees')
        fdm.set('longitude', 120.6, units='degrees')
        fdm.set('num_engines', 1)
        fdm.set('vcas', 44, units='mps')
        
        assert fdm.get('latitude', units='degrees') == 67.4
        assert round(fdm.get('vcas', units='knots'), 2) == 85.53

    def test_packparse(self):
        """Test the packing and parsing of an fgFDM packet"""
        fdm = fgFDM()     

        fdm.set('latitude', 67.4, units='degrees')
        fdm.set('longitude', 120.6, units='degrees')
        fdm.set('num_engines', 1)
        fdm.set('vcas', 44, units='mps')

        packedBytes = fdm.pack()
        
        parsedObj = fgFDM()
        parsedObj.parse(packedBytes)
        
        assert parsedObj.get('latitude', units='degrees') == 67.4
        assert round(parsedObj.get('vcas', units='knots'), 2) == 85.53
        
if __name__ == '__main__':
    unittest.main()


#!/usr/bin/env python3
# parse and construct FlightGear NET FDM packets
# Andrew Tridgell, November 2011
# released under GNU GPL version 2 or later

import struct, math

class fgFDMError(Exception):
    '''fgFDM error class'''
    def __init__(self, msg):
        Exception.__init__(self, msg)
        self.message = 'fgFDMError: ' + msg

class fgFDMVariable(object):
    '''represent a single fgFDM variable'''
    def __init__(self, index, arraylength, units):
        self.index   = index
        self.arraylength = arraylength
        self.units = units

class fgFDMVariableList(object):
    '''represent a list of fgFDM variable'''
    def __init__(self):
        self.vars = {}
        self._nextidx = 0
        
    def add(self, varname, arraylength=1, units=None):
        self.vars[varname] = fgFDMVariable(self._nextidx, arraylength, units=units)
        self._nextidx += arraylength

class fgFDM(object):
    '''a flightgear native FDM parser/generator'''
    def __init__(self):
        '''init a fgFDM object'''
        self.FG_NET_FDM_VERSION = 24
        self.pack_string = '>I 4x 3d 6f 11f 3f 2f I 4I 4f 4f 4f 4f 4f 4f 4f 4f 4f I 4f I 3I 3f 3f 3f I i f 10f'
        self.values = [0]*98

        self.FG_MAX_ENGINES = 4
        self.FG_MAX_WHEELS  = 3
        self.FG_MAX_TANKS   = 4

        # supported unit mappings
        self.unitmap = {
            ('radians', 'degrees') : math.degrees(1),
            ('rps',     'dps')     : math.degrees(1),
            ('feet',    'meters')  : 0.3048,
            ('fps',     'mps')     : 0.3048,
            ('knots',   'mps')     : 0.514444444,
            ('knots',   'fps')     : 0.514444444/0.3048,
            ('fpss',    'mpss')    : 0.3048,
            ('seconds', 'minutes') : 60,
            ('seconds', 'hours')   : 3600,
            }

        # build a mapping between variable name and index in the values array
        # note that the order of this initialisation is critical - it must
        # match the wire structure
        self.mapping = fgFDMVariableList()
        self.mapping.add('version')

        # position
        self.mapping.add('longitude', units='radians')      # geodetic (radians)
        self.mapping.add('latitude', units='radians')       # geodetic (radians)
        self.mapping.add('altitude', units='meters')        # above sea level (meters)
        self.mapping.add('agl', units='meters')             # above ground level (meters)

        # attitude
        self.mapping.add('phi', units='radians')            # roll (radians)
        self.mapping.add('theta', units='radians')          # pitch (radians)
        self.mapping.add('psi', units='radians')            # yaw or true heading (radians)
        self.mapping.add('alpha', units='radians')          # angle of attack (radians)
        self.mapping.add('beta', units='radians')           # side slip angle (radians)

        # Velocities
        self.mapping.add('phidot', units='rps')             # roll rate (radians/sec)
        self.mapping.add('thetadot', units='rps')           # pitch rate (radians/sec)
        self.mapping.add('psidot', units='rps')             # yaw rate (radians/sec)
        self.mapping.add('vcas', units='fps')               # calibrated airspeed
        self.mapping.add('climb_rate', units='fps')         # feet per second
        self.mapping.add('v_north', units='fps')            # north velocity in local/body frame, fps
        self.mapping.add('v_east', units='fps')             # east velocity in local/body frame, fps
        self.mapping.add('v_down', units='fps')             # down/vertical velocity in local/body frame, fps
        self.mapping.add('v_wind_body_north', units='fps')  # north velocity in local/body frame
        self.mapping.add('v_wind_body_east', units='fps')   # east velocity in local/body frame
        self.mapping.add('v_wind_body_down', units='fps')   # down/vertical velocity in local/body

        # Accelerations
        self.mapping.add('A_X_pilot', units='fpss')         # X accel in body frame ft/sec^2
        self.mapping.add('A_Y_pilot', units='fpss')         # Y accel in body frame ft/sec^2
        self.mapping.add('A_Z_pilot', units='fpss')         # Z accel in body frame ft/sec^2

        # Stall
        self.mapping.add('stall_warning')                   # 0.0 - 1.0 indicating the amount of stall
        self.mapping.add('slip_deg', units='degrees')       # slip ball deflection

        # Engine status
        self.mapping.add('num_engines')                     # Number of valid engines
        self.mapping.add('eng_state', self.FG_MAX_ENGINES)  # Engine state (off, cranking, running)
        self.mapping.add('rpm',       self.FG_MAX_ENGINES)  # Engine RPM rev/min
        self.mapping.add('fuel_flow', self.FG_MAX_ENGINES)  # Fuel flow gallons/hr
        self.mapping.add('fuel_px',   self.FG_MAX_ENGINES)  # Fuel pressure psi
        self.mapping.add('egt',       self.FG_MAX_ENGINES)  # Exhuast gas temp deg F
        self.mapping.add('cht',       self.FG_MAX_ENGINES)  # Cylinder head temp deg F
        self.mapping.add('mp_osi',    self.FG_MAX_ENGINES)  # Manifold pressure
        self.mapping.add('tit',       self.FG_MAX_ENGINES)  # Turbine Inlet Temperature
        self.mapping.add('oil_temp',  self.FG_MAX_ENGINES)  # Oil temp deg F
        self.mapping.add('oil_px',    self.FG_MAX_ENGINES)  # Oil pressure psi
            
        # Consumables
        self.mapping.add('num_tanks')                       # Max number of fuel tanks
        self.mapping.add('fuel_quantity', self.FG_MAX_TANKS)

        # Gear status
        self.mapping.add('num_wheels')
        self.mapping.add('wow',              self.FG_MAX_WHEELS)
        self.mapping.add('gear_pos',         self.FG_MAX_WHEELS)
        self.mapping.add('gear_steer',       self.FG_MAX_WHEELS)
        self.mapping.add('gear_compression', self.FG_MAX_WHEELS)

        # Environment
        self.mapping.add('cur_time', units='seconds')       # current unix time
        self.mapping.add('warp',     units='seconds')       # offset in seconds to unix time
        self.mapping.add('visibility', units='meters')      # visibility in meters (for env. effects)

        # Control surface positions (normalized values)
        self.mapping.add('elevator')
        self.mapping.add('elevator_trim_tab')
        self.mapping.add('left_flap')
        self.mapping.add('right_flap')
        self.mapping.add('left_aileron')
        self.mapping.add('right_aileron')
        self.mapping.add('rudder')
        self.mapping.add('nose_wheel')
        self.mapping.add('speedbrake')
        self.mapping.add('spoilers')

        self._packet_size = struct.calcsize(self.pack_string)

        self.set('version', self.FG_NET_FDM_VERSION)

        if len(self.values) != self.mapping._nextidx:
            raise fgFDMError('Invalid variable list in initialisation')

    def packet_size(self):
        '''return expected size of FG FDM packets'''
        return self._packet_size

    def convert(self, value, fromunits, tounits):
        '''convert a value from one set of units to another'''
        if fromunits == tounits:
            return value
        if (fromunits,tounits) in self.unitmap:
            return value * self.unitmap[(fromunits,tounits)]
        if (tounits,fromunits) in self.unitmap:
            return value / self.unitmap[(tounits,fromunits)]
        raise fgFDMError("unknown unit mapping (%s,%s)" % (fromunits, tounits))


    def units(self, varname):
        '''return the default units of a variable'''
        if not varname in self.mapping.vars:
            raise fgFDMError('Unknown variable %s' % varname)
        return self.mapping.vars[varname].units


    def variables(self):
        '''return a list of available variables'''
        return sorted(list(self.mapping.vars.keys()),
                      key = lambda v : self.mapping.vars[v].index)


    def get(self, varname, idx=0, units=None):
        '''get a variable value'''
        if not varname in self.mapping.vars:
            raise fgFDMError('Unknown variable %s' % varname)
        if idx >= self.mapping.vars[varname].arraylength:
            raise fgFDMError('index of %s beyond end of array idx=%u arraylength=%u' % (
                varname, idx, self.mapping.vars[varname].arraylength))
        value = self.values[self.mapping.vars[varname].index + idx]
        if units:
            value = self.convert(value, self.mapping.vars[varname].units, units)
        return value

    def set(self, varname, value, idx=0, units=None):
        '''set a variable value'''
        if not varname in self.mapping.vars:
            raise fgFDMError('Unknown variable %s' % varname)
        if idx >= self.mapping.vars[varname].arraylength:
            raise fgFDMError('index of %s beyond end of array idx=%u arraylength=%u' % (
                varname, idx, self.mapping.vars[varname].arraylength))
        if units:
            value = self.convert(value, units, self.mapping.vars[varname].units)
        # avoid range errors when packing into 4 byte floats
        if math.isinf(value) or math.isnan(value) or math.fabs(value) > 3.4e38:
            value = 0
        self.values[self.mapping.vars[varname].index + idx] = value

    def parse(self, buf):
        '''parse a FD FDM buffer'''
        try:
            t = struct.unpack(self.pack_string, buf)
        except struct.error as msg:
            raise fgFDMError('unable to parse - %s' % msg)
        self.values = list(t)

    def pack(self):
        '''pack a FD FDM buffer from current values'''
        for i in range(len(self.values)):
            if math.isnan(self.values[i]):
                self.values[i] = 0
        return struct.pack(self.pack_string, *self.values)

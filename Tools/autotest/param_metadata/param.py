
class Parameter(object):
    def __init__(self, name):
        self.name = name


class Vehicle(object):
    def __init__(self, name, path):
        self.name = name
        self.path = path
        self.params = []


class Library(object):
    def __init__(self, name):
        self.name = name
        self.params = []

known_param_fields = [
             'Description',
             'DisplayName',
             'Values',
             'Range',
             'Units',
             'Increment',
             'User',
             'RebootRequired',
             'Bitmask',
             'Volatile',
             'ReadOnly',
                      ]

# Follow SI units conventions from:
# http://physics.nist.gov/cuu/Units/units.html
# http://physics.nist.gov/cuu/Units/outside.html
# and
# http://physics.nist.gov/cuu/Units/checklist.html
# http://www.bipm.org/en/publications/si-brochure/
# http://www1.bipm.org/en/CGPM/db/3/2/   g_n unit for G-force
# one further constrain is that only printable (7bit) ASCII characters are allowed
known_units = {
#          abreviation : full-text (used in .html .rst and .wiki files)
# time
             's'       : 'seconds'               ,
             'ds'      : 'deciseconds'           ,
             'cs'      : 'centiseconds'          ,
             'ms'      : 'milliseconds'          ,
             'PWM'     : 'PWM in microseconds'   , # should be microseconds, this is NOT a SI unit, but follows https://github.com/ArduPilot/ardupilot/pull/5538#issuecomment-271943061
             'Hz'      : 'hertz'                 ,
             'kHz'     : 'kilohertz'             ,
# distance
             'km'      : 'kilometers'                , # metre is the SI unit name, meter is the american spelling of it
             'm'       : 'meters'                    , # metre is the SI unit name, meter is the american spelling of it
             'm/s'     : 'meters per second'         , # metre is the SI unit name, meter is the american spelling of it
             'm/s/s'   : 'meters per square second'  , # metre is the SI unit name, meter is the american spelling of it
             'm/s/s/s' : 'meters per cubic second'   , # metre is the SI unit name, meter is the american spelling of it
             'cm'      : 'centimeters'               , # metre is the SI unit name, meter is the american spelling of it
             'cm/s'    : 'centimeters per second'    , # metre is the SI unit name, meter is the american spelling of it
             'cm/s/s'  : 'centimeters per square second', # metre is the SI unit name, meter is the american spelling of it
             'cm/s/s/s': 'centimeters per cubic second' , # metre is the SI unit name, meter is the american spelling of it
             'mm'      : 'millimeters'               , # metre is the SI unit name, meter is the american spelling of it
# temperature
             'degC'    : 'degrees Celsius'       ,     # Not SI, but Kelvin is too cumbersome for most users
# angle
             'deg'     : 'degrees'               ,     # Not SI, but is some situations more user-friendly than radians
             'deg/s'   : 'degrees per second'    ,     # Not SI, but is some situations more user-friendly than radians
             'cdeg'    : 'centidegrees'          ,     # Not SI, but is some situations more user-friendly than radians
             'cdeg/s'  : 'centidegrees per second',    # Not SI, but is some situations more user-friendly than radians
             'cdeg/s/s': 'centidegrees per square second' , # Not SI, but is some situations more user-friendly than radians
             'rad'     : 'radians'               ,
             'rad/s'   : 'radians per second'    ,
             'rad/s/s' : 'radians per square second' ,
# electricity
             'A'       : 'ampere'                ,
             'V'       : 'volt'                  ,
             'W'       : 'watt'                  ,
# magnetism
             'Gauss'   : 'gauss'                 , # Gauss is not an SI unit, but 1 tesla = 10000 gauss so a simple replacement is not possible here
             'Gauss/s' : 'gauss per second'      , # Gauss is not an SI unit, but 1 tesla = 10000 gauss so a simple replacement is not possible here
             'mGauss'  : 'milligauss'            , # Gauss is not an SI unit, but 1 tesla = 10000 gauss so a simple replacement is not possible here
# pressure
             'Pa'      : 'pascal'                ,
             'mbar'    : 'millibar'              ,
# ratio
             '%'       : 'percent'               ,
             '%/s'     : 'percent per second'    ,
             'd%'      : 'decipercent'           , # decipercent is strange, but "per-mille" is even more exotic
# compound
             'm.m/s/s' : 'square meter per square second',
             'deg/m/s' : 'degrees per meter per second'  ,
             'm/s/m'   : 'meters per second per meter'   , # Why not use Hz here ????
             'mGauss/A': 'milligauss per ampere' ,
             'mA.h'    : 'milliampere hour'      ,
             'A/V'     : 'ampere per volt'       ,
             'm/V'     : 'meters per volt'       ,
             'gravities': 'standard acceleration due to gravity' , # g_n would be a more correct unit, but IMHO no one understands what g_n means
             }

required_param_fields = [
             'Description',
             'DisplayName',
             'User',
                      ]

known_group_fields = [
                      'Path',
                      ]

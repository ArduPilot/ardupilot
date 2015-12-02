
class Parameter(object):
    def __init__(self, name):
        self.name = name
        

class Vehicle(object):
    def __init__ (self, name, path):
        self.name = name
        self.path = path
        self.params = []
        
class Library(object):
    def __init__ (self, name):
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
             'Bitmask'
                      ]

required_param_fields = [
             'Description',
             'DisplayName',
             'User'
                      ]

known_group_fields = [
                      'Path'
                      ]

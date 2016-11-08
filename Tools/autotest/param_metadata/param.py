
class Parameter(object):
    def __init__(self, name):
        self.name = name
    def __repr__(self):
        return repr(self.name)

class Vehicle(object):
    def __init__(self, name, path):
        self.name = name
        self.path = path
        self.params = []


class ParamNode(object):
    def __init__(self, name=None):
        self.name = name
        self.params = []
        self.children = []

    def __repr__(self):
        return "ParamNode { name=%s, params=%s children=%s}" % (repr(self.name), repr(self.params), repr(self.children))

    def add_child(self, child):
        self.children.append(child)
        child.parent = self

    def validate(self):
        for param in self.params:
            self.validate_param(param)
        for child in self.children:
            child.validate()

    def is_number(self,numberString):
        try:
            float(numberString)
            return True
        except ValueError:
            return False

    def vehicle(self):
        if hasattr(self, "_vehicle"):
            return self._vehicle
        if not hasattr(self, "parent"):
            raise ValueError("No _vehicle and no _parent")
        return self.parent.vehicle()

    def validate_param(self,param):
        """
        Validates the parameter meta data.
        """
        # Validate values
        if (hasattr(param, "Range")):
            rangeValues = param.__dict__["Range"].split(" ")
            if (len(rangeValues) != 2):
                error("Invalid Range values for %s" % (param.name))
                return
            min = rangeValues[0]
            max = rangeValues[1]
            if not self.is_number(min):
                error("Min value not number: %s %s" % (param.name, min))
                return
            if not self.is_number(max):
                error("Max value not number: %s %s" % (param.name, max))
                return


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

required_param_fields = [
             'Description',
             'DisplayName',
             'User',
                      ]

known_group_fields = [
                      'Path',
                      ]

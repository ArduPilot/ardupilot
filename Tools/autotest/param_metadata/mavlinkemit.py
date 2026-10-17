'''
Emit parameter documentation in the format of the MAVLink component
metadata parameter schema (COMP_METADATA_TYPE_PARAMETER), see
https://mavlink.io/en/services/component_information.html

AP_FLAKE8_CLEAN
'''

import json
import re

from emit import Emit


def number(value):
    '''return a string as an int if it is integral, else as a float'''
    f = float(value)
    if f.is_integer():
        return int(f)
    return f


def is_true(value):
    return value.strip().lower() == 'true'


class MAVLinkEmit(Emit):
    def __init__(self, *args, **kwargs):
        Emit.__init__(self, *args, **kwargs)
        self.parameters = []
        self.names = set()
        # parameter object -> metadata type, see param_types.py
        self.param_types = {}
        self.untyped = []
        self.duplicates = []

    def output_fname(self):
        return 'parameters.json'

    def group(self, name):
        '''group parameters by the part of their name before the first
        underscore, with any instance number removed, as QGC does for
        ArduPilot's XML metadata'''
        return re.sub(r'[0-9]*$', '', name.split('_')[0])

    def close(self):
        # a group of one is no group at all
        group_count = {}
        for p in self.parameters:
            group_count[p['group']] = group_count.get(p['group'], 0) + 1
        for p in self.parameters:
            if group_count[p['group']] == 1:
                del p['group']

        content = {
            'version': 1,
            'parameters': self.parameters,
        }
        with open(self.output_fname(), mode='w') as f:
            json.dump(content, f, separators=(',', ':'))

        if self.untyped:
            print("MAVLinkEmit: omitted %u parameters whose type could not be determined: %s" %
                  (len(self.untyped), ' '.join(sorted(self.untyped))))

    def emit(self, g):
        for param in g.params:
            if not self.should_emit_param(param):
                continue

            name = param.name.split(':')[-1]
            if name in self.names:
                # e.g. documented separately for several backends
                self.duplicates.append(name)
                continue

            metadata_type = self.param_types.get(param)
            if metadata_type is None:
                # the schema requires a type, and a wrong type is worse
                # than none: the GCS would use it to validate values
                self.untyped.append(name)
                continue

            self.names.add(name)
            p = {
                'name': name,
                'type': metadata_type,
            }
            if getattr(param, 'DisplayName', None):
                p['shortDesc'] = param.DisplayName
            if getattr(param, 'Description', None):
                p['longDesc'] = param.Description
            if getattr(param, 'Units', None):
                p['units'] = param.Units
            if hasattr(param, 'Range'):
                (low, high) = param.Range.split()
                p['min'] = number(low)
                p['max'] = number(high)
            if hasattr(param, 'Increment'):
                p['increment'] = number(param.Increment)
            if hasattr(param, 'Values'):
                values = []
                for v in param.Values.split(','):
                    (value, sep, description) = v.partition(':')
                    values.append({'value': number(value), 'description': description.strip()})
                p['values'] = values
            if hasattr(param, 'Bitmask'):
                bitmask = []
                for b in param.Bitmask.split(','):
                    (index, sep, description) = b.partition(':')
                    bitmask.append({'index': int(index), 'description': description.strip()})
                p['bitmask'] = bitmask
            if is_true(getattr(param, 'RebootRequired', '')):
                p['rebootRequired'] = True
            if is_true(getattr(param, 'ReadOnly', '')):
                p['readOnly'] = True
            if is_true(getattr(param, 'Volatile', '')):
                p['volatile'] = True
            if getattr(param, 'User', None):
                p['category'] = param.User.strip()
            p['group'] = self.group(name)
            self.parameters.append(p)

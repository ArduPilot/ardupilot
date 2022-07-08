import json
import copy
from emit import Emit


# Emit ArduPilot documentation in JSON format
class JSONEmit(Emit):
    def __init__(self, *args, **kwargs):
        Emit.__init__(self, *args, **kwargs)
        json_fname = 'apm.pdef.json'
        self.f = open(json_fname, mode='w')
        self.content = {"json": {"version": 0}}

    def close(self):
        json.dump(self.content, self.f, indent=2, sort_keys=True)
        self.f.close()

    def jsonFromKeyList(self, main_key, dictionary):
        json_object = {}
        if main_key in dictionary:
            values = dictionary[main_key]
            for value in values.split(','):
                key, description = value.split(":")
                json_object[key.strip()] = description.strip()
        return json_object

    def emit(self, g):
        content = {}

        # Copy content to avoid any modification
        g = copy.deepcopy(g)

        self.content[g.name] = {}

        # Check all params available
        for param in g.params:
            param_json = {}

            # Get display name
            if hasattr(param, 'DisplayName'):
                 # i.e. ArduPlane (ArduPlane:FOOPARM)
                param_json['displayName'] = param.DisplayName

            # Get description
            if hasattr(param, 'Description'):
                param_json['description'] = param.Description

            # Get user type
            if hasattr(param, 'User'):
                 # i.e. Standard or Advanced
                param_json['user'] = param.User

            # Get param name and and remove key
            name = param.__dict__.pop('name')
            if ':' in name:
                name = name.split(':')[1]

            # Remove real_path key
            if 'real_path' in param.__dict__:
                param.__dict__.pop('real_path')

            # Get range section if available
            range_json = {}
            if 'Range' in param.__dict__:
                range = param.__dict__['Range'].split(' ')
                range_json['low'] = range[0]
                range_json['high'] = range[1]
                param.__dict__.pop('Range')

            # Get bitmask section if available
            bitmask_json = self.jsonFromKeyList('Bitmask', param.__dict__)
            if(bitmask_json):
                param.__dict__.pop('Bitmask')

            # get value section if availables
            values_json = self.jsonFromKeyList('Values', param.__dict__)
            if(values_json):
                param.__dict__.pop('Values')

            # Set actual content
            content[name] = param.__dict__

            # Set range if available
            if(range_json):
                content[name]['Range'] = range_json

            # Set bitmask if available
            if(bitmask_json):
                content[name]['Bitmask'] = bitmask_json

            # Set values if available
            if(values_json):
                content[name]['Values'] = values_json

        # Update main content with actual content
        for key in content:
            self.content[g.name][key] = content[key]

# AP_FLAKE8_CLEAN

import copy
import json

from emit import Emit


# Emit parameter metadata in MAVLink COMP_METADATA_TYPE_PARAMETER JSON format
class MAVLinkCompInfoJSONEmit(Emit):
    def __init__(self, *args, **kwargs):
        Emit.__init__(self, *args, **kwargs)
        self.output_fname = 'compinfo-parameter.json'
        self.parameters = []

    def close(self):
        content = {
            "version": 1,
            "parameters": self.parameters,
        }
        with open(self.output_fname, mode='w') as f:
            json.dump(content, f, indent=2)

    def emit(self, g):
        # Work on a deep copy to avoid modifying the original
        g = copy.deepcopy(g)

        group_name = g.name

        for param in g.params:
            if not self.should_emit_param(param):
                continue

            name = param.name
            if ':' in name:
                name = name.split(':')[1]

            p = {
                "name": name,
                "type": "Float",
            }

            if hasattr(param, 'DisplayName'):
                p["shortDesc"] = param.DisplayName

            if hasattr(param, 'Description'):
                p["longDesc"] = param.Description

            if hasattr(param, 'Units'):
                p["units"] = param.Units

            if hasattr(param, 'Range'):
                try:
                    parts = param.Range.split()
                    p["min"] = float(parts[0])
                    p["max"] = float(parts[1])
                except (ValueError, IndexError):
                    pass

            if hasattr(param, 'Increment'):
                try:
                    p["increment"] = float(param.Increment)
                except ValueError:
                    pass

            if hasattr(param, 'RebootRequired'):
                p["rebootRequired"] = param.RebootRequired.strip().lower() == 'true'

            if hasattr(param, 'ReadOnly'):
                p["readOnly"] = param.ReadOnly.strip().lower() == 'true'

            if hasattr(param, 'Volatile'):
                p["volatile"] = param.Volatile.strip().lower() == 'true'

            if hasattr(param, 'User'):
                p["category"] = param.User.strip()

            # Group is the library/vehicle prefix
            p["group"] = group_name

            if hasattr(param, 'Values'):
                values_list = []
                for entry in param.Values.split(','):
                    entry = entry.strip()
                    if ':' in entry:
                        val_str, desc = entry.split(':', 1)
                        try:
                            values_list.append({
                                "value": float(val_str.strip()),
                                "description": desc.strip(),
                            })
                        except ValueError:
                            pass
                if values_list:
                    p["values"] = values_list

            if hasattr(param, 'Bitmask'):
                bitmask_list = []
                for entry in param.Bitmask.split(','):
                    entry = entry.strip()
                    if ':' in entry:
                        idx_str, desc = entry.split(':', 1)
                        try:
                            bitmask_list.append({
                                "index": int(idx_str.strip()),
                                "description": desc.strip(),
                            })
                        except ValueError:
                            pass
                if bitmask_list:
                    p["bitmask"] = bitmask_list

            self.parameters.append(p)

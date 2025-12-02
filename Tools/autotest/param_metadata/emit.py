"""
 The standard interface emitters must implement
"""

import re


class Emit:
    def __init__(self):
        pass

    prog_values_field = re.compile(r"-?\d*\.?\d+: ?[\w ]+,?")
    emit_legacy_params = True

    def close(self):
        pass

    def start_libraries(self):
        pass

    def emit(self, g):
        pass

    def should_emit_param(self, param):
        if not self.emit_legacy_params and getattr(param, 'Legacy', False):
            return False
        return True

    def should_emit_field(self, param, field):
        return field not in ['Legacy']

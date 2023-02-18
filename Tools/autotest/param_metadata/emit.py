"""
 The standard interface emitters must implement
"""

import re


class Emit:
    def __init__(self):
        pass

    prog_values_field = re.compile(r"-?\d*\.?\d+: ?[\w ]+,?")

    def close(self):
        pass

    def start_libraries(self):
        pass

    def emit(self, g):
        pass

#!/usr/bin/env python

import re
from param import *

# The standard interface emitters must implement
class Emit:
    prog_values_field = re.compile(r"\s*(-?\w+:\w+)+,*")

    def close(self): 
        pass

    def start_libraries(self):
	pass
    
    def emit(self, g, f):
	pass



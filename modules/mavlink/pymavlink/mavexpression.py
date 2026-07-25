#!/usr/bin/env python3
'''
mavlink expression evaluation functions

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

import os

# these imports allow for mavgraph and mavlogdump to use maths expressions more easily
from math import *
from .mavextra import *

'''
Support having a $HOME/.pymavlink/mavextra.py for extra graphing functions
'''
home = os.getenv('HOME')
if home is not None:
    extra = os.path.join(home, '.pymavlink', 'mavextra.py')
    if os.path.exists(extra):
        try:
            import imp
            mavuser = imp.load_source('pymavlink.mavuser', extra)
        except ModuleNotFoundError:
            # "imp" is removed in Python 3.12.  Try to use importlib instead:
            import sys
            # from: https://docs.python.org/dev/whatsnew/3.12.html#removed
            import importlib.util
            import importlib.machinery

            def load_source(modname, filename):
                loader = importlib.machinery.SourceFileLoader(modname, filename)
                spec = importlib.util.spec_from_file_location(modname, filename, loader=loader)
                module = importlib.util.module_from_spec(spec)
                # The module is always executed and not cached in sys.modules.
                # Uncomment the following line to cache the module.
                sys.modules[module.__name__] = module
                loader.exec_module(module)
                return module

            load_source('pymavlink.mavuser', extra)

        from pymavlink.mavuser import *

def evaluate_expression(expression, vars, nocondition=False):
    '''evaluation an expression'''
    # first check for conditions which take the form EXPRESSION{CONDITION}
    if expression[-1] == '}':
        startidx = expression.rfind('{')
        if startidx == -1:
            return None
        condition=expression[startidx+1:-1]
        expression=expression[:startidx]
        try:
            v = eval(condition, globals(), vars)
        except Exception:
            return None
        if not nocondition and not v:
            return None
    try:
        v = eval(expression, globals(), vars)
    except NameError:
        return None
    except ZeroDivisionError:
        return None
    except IndexError:
        return None
    return v

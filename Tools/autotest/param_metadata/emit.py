"""
 The standard interface emitters must implement
"""

import re


def html_comment_safe(value):
    """Keep metadata valid inside an HTML comment."""
    while '--' in value:
        value = value.replace('--', '- -')
    return value


class Emit:
    def __init__(self):
        pass

    prog_values_field = re.compile(r"-?\d*\.?\d+: ?[\w ]+,?")
    emit_legacy_params = True
    git_sha = None
    git_tag = None

    def output_fname(self):
        '''Return the path of the primary output file, or None if not applicable.'''
        return None

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

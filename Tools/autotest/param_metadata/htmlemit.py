"""
Emit docs in a form acceptable to the old Ardupilot wordpress docs site
"""

from param import known_param_fields, known_units
from emit import Emit
try:
    from cgi import escape as cescape
except Exception:
    from html import escape as cescape


class HtmlEmit(Emit):

    def __init__(self, *args, **kwargs):
        Emit.__init__(self, *args, **kwargs)
        html_fname = 'Parameters.html'
        self.f = open(html_fname, mode='w')
        self.preamble = """<!-- Dynamically generated list of documented parameters
This page was generated using Tools/autotest/param_metadata/param_parse.py

DO NOT EDIT
-->


<h3 style="text-align: center">Complete Parameter List</h3>
<hr />

<p>This is a complete list of the parameters which can be set via the MAVLink protocol in the EEPROM of your autopilot to control vehicle behaviour. This list is automatically generated from the latest ardupilot source code, and so may contain parameters which are not yet in the stable released versions of the code.</p>

<!-- add auto-generated table of contents with "Table of Contents Plus" plugin -->
[toc exclude="Complete Parameter List"]

"""
        self.t = ''

    def escape(self, s):
        s = s.replace(' ', '-')
        s = s.replace(':', '-')
        s = s.replace('(', '')
        s = s.replace(')', '')
        return s

    def close(self):
        self.f.write(self.preamble)
        self.f.write(self.t)
        self.f.close()

    def start_libraries(self):
        pass

    def emit(self, g):
        tag = '%s Parameters' % g.reference
        t = '\n\n<h1>%s</h1>\n' % tag

        for param in g.params:
            if not hasattr(param, 'DisplayName') or not hasattr(param, 'Description'):
                continue
            d = param.__dict__
            tag = '%s (%s)' % (param.DisplayName, param.name)
            t += '\n\n<h2>%s</h2>' % tag
            if d.get('User', None) == 'Advanced':
                t += '<em>Note: This parameter is for advanced users</em><br>'
            t += "\n\n<p>%s</p>\n" % cescape(param.Description)
            t += "<ul>\n"

            for field in param.__dict__.keys():
                if field not in ['name', 'DisplayName', 'Description', 'User'] and field in known_param_fields:
                    if field == 'Values' and Emit.prog_values_field.match(param.__dict__[field]):
                        values = (param.__dict__[field]).split(',')
                        t += "<table><th>Value</th><th>Meaning</th>\n"
                        for value in values:
                            v = value.split(':')
                            if len(v) != 2:
                                raise ValueError("Bad value (%s)" % v)
                            t += "<tr><td>%s</td><td>%s</td></tr>\n" % (v[0], v[1])
                        t += "</table>\n"
                    elif field == 'Units':
                        abreviated_units = param.__dict__[field]
                        if abreviated_units != '':
                            units = known_units[abreviated_units]   # use the known_units dictionary to convert the abreviated unit into a full textual one
                            t += "<li>%s: %s</li>\n" % (field, cescape(units))
                    else:
                        t += "<li>%s: %s</li>\n" % (field, cescape(param.__dict__[field]))
            t += "</ul>\n"
        self.t += t

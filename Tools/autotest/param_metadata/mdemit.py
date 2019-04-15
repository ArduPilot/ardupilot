#!/usr/bin/env python
"""
Emit parameter documentation in markdown format
"""

from param import known_param_fields
from emit import Emit
import re
import os

# Parameter groups disabled at compile time (Vehicle-specific)
sub_blacklist = ['AVOID_', 'CIRCLE_', 'FLOW', 'MIS_', 'PRX', 'RALLY_', 'RCMAP_', 'RPM', 'TERRAIN_', 'WPNAV_']

# Parameter groups with redundant information (ie RCn_, SERVOn_)
# We can keep the documentation concise by only documenting these once
nparams = ['RCn_', 'SERVOn_', 'SRn_', 'BTNn_']

class MDEmit(Emit):

    def __init__(self):
        Emit.__init__(self)
        fname = 'Parameters.md'
        self.nparams = []
        self.f = open(fname, mode='w')

        self.blacklist = None
        
        # Flag to generate navigation header for BlueRobotics' ArduSub docs
        if os.getenv('BRDOC') is not None:
            self.header = """---\nlayout: default\ntitle: "Parameters"\npermalink: /parameters/\nnav:"""
        
        self.preamble = """\nThis is a complete list of the parameters which can be set via the MAVLink protocol in the EEPROM of your APM to control vehicle behaviour. This list is automatically generated from the latest ardupilot source code, and so may contain parameters which are not yet in the stable released versions of the code. Some parameters may only be available for developers, and are enabled at compile-time."""
        self.t = ''

    def close(self):
        # Write navigation header for BlueRobotics' ArduSub docs
        if os.getenv('BRDOC') is not None:
            self.f.write(self.header)
            self.f.write('\n---\n')
            
        self.f.write(self.preamble)
        self.f.write(self.t)
        self.f.close()

    def start_libraries(self):
        pass

    def emit(self, g):
        nparam = False # Flag indicating this is a parameter group with redundant information (ie RCn_, SERVOn_)
        
        if g.name == 'ArduSub':
            self.blacklist = sub_blacklist
        
        if self.blacklist is not None and g.name in self.blacklist:
            return
        
        pname = g.name
        
        # Check to see this is a parameter group with redundant information
        rename = re.sub('\d+', 'n', g.name)
        if rename in nparams:
            if rename in self.nparams:
                return
            else:
                self.nparams.append(rename)
                pname = rename
                nparam = True
        
        # Markdown!
        tag = '%s Parameters' % pname
        tag = tag.replace('_', '')
        link = tag.replace(' ', '-')
        
        # Add group to navigation header for BlueRobotics' ArduSub docs
        if os.getenv('BRDOC') is not None:
            self.header += "\n- %s: %s" % (link.split('-')[0],link.lower())
        
        t = '\n\n# %s' % tag
        
        for param in g.params:
            if not hasattr(param, 'DisplayName') or not hasattr(param, 'Description'):
                continue
            d = param.__dict__
            name = param.name.split(':')[-1]
            if nparam:
                name = re.sub('\d+', 'n', name, 1)
            tag = '%s: %s' % (name, param.DisplayName)
            t += '\n\n## %s' % tag
            if d.get('User', None) == 'Advanced':
                t += '\n\n*Note: This parameter is for advanced users*'
            t += "\n\n%s" % param.Description
            
            for field in param.__dict__.keys():
                if field not in ['name', 'DisplayName', 'Description', 'User'] and field in known_param_fields:
                    if field == 'Values' and Emit.prog_values_field.match(param.__dict__[field]):
                        values = (param.__dict__[field]).split(',')
                        t += "\n\n|Value|Meaning|"
                        t += "\n|:---:|:---:|"
                        for value in values:
                            v = value.split(':')
                            if len(v) != 2:
                                raise ValueError("Bad value (%s)" % v)
                            t += "\n|%s|%s|" % (v[0], v[1])
                    else:
                        t += "\n\n- %s: %s" % (field, param.__dict__[field])
        self.t += t

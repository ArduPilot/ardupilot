#!/usr/bin/env python
"""
Emit parameter documentation in markdown format
"""

from param import known_param_fields
from emit import Emit
import time
import re
import os

# ArduSub documentation uses the page title as the top level heading.
# OFFSET drops headings one level when compiling for those docs.
OFFSET = '#' if os.getenv('BRDOC') else ''

# Parameter groups disabled at compile time (Vehicle-specific)
sub_blacklist = ['RCMAP_', 'TERRAIN_']

# Parameter groups with redundant information (ie RCn_, SERVOn_)
# We can keep the documentation concise by only documenting these once
nparams = ['RCn_', 'SERVOn_', 'SRn_', 'BTNn_', 'INS_TCALn_', 'BATTn_', 'BAROn_GND_', 'BAROn_WCF_', 'CAN_Dn_', 'CAN_Dn_KDE_', 'CAN_Dn_PC_', 'CAN_Dn_UC_','CAN_Dn_TST_', 'CAN_Pn_', 'RNGFNDn_']

class MDEmit(Emit):

    def __init__(self, *args, **kwargs):
        Emit.__init__(self, *args, **kwargs)
        fname = 'Parameters.md'
        self.nparams = []
        self.f = open(fname, mode='w')

        self.blacklist = None
        
        # Flag to generate navigation header for BlueRobotics' ArduSub docs
        if os.getenv('BRDOC') is not None:
            now = time.strftime('%Y-%m-%dT%H:%M:%S%z')
            now = now[:-2] + ':' + now[-2:]
            self.header = """+++
title = "Parameters"
description = "Firmware parameter details."
date = %s
template = "docs/page.html"
sort_by = "weight"
weight = 10
draft = false
[extra]
toc = true
top = false""" % now
        
        self.preamble = """\nThis is a complete list of the parameters which can be set via the MAVLink protocol in the EEPROM of your autopilot to control vehicle behaviour. Some parameters may only be available for developers, and are enabled at compile-time."""
        self.t = ''

    def close(self):
        # Write navigation header for BlueRobotics' ArduSub docs
        if os.getenv('BRDOC') is not None:
            self.f.write(self.header)
            self.f.write('\n+++\n')
            
        self.f.write(self.preamble)
        self.f.write(self.t)
        self.f.close()

    def start_libraries(self):
        pass

    def emit(self, g):
        nparam = False # Flag indicating this is a parameter group with redundant information (ie RCn_, SERVOn_)
        
        if g.reference == 'ArduSub':
            self.blacklist = sub_blacklist
        
        if self.blacklist is not None and g.reference in self.blacklist:
            return
        
        pname = g.reference
        
        # Check to see this is a parameter group with redundant information
        rename = re.sub('\d+', 'n', g.reference)
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
        
        t = '\n\n%s# %s' % (OFFSET, tag)
        
        for param in g.params:
            if not hasattr(param, 'DisplayName') or not hasattr(param, 'Description'):
                continue
            d = param.__dict__
            name = param.name.split(':')[-1]
            if nparam:
                name = re.sub('\d+', 'n', name, 1)
            tag = '%s: %s' % (name, param.DisplayName)
            t += '\n\n%s## %s' % (OFFSET, tag)
            if os.getenv('DIFF'):
                continue
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

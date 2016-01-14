#!/usr/bin/env python
# encoding: utf-8
# (c) Siddharth Bharat Purohit, 3DRobotics Inc.

"""
The **mavgen.py** program is a code generator which creates mavlink header files.
"""

from waflib import Task, Utils, Node
from waflib.TaskGen import feature, before_method, extension
import os

class mavgen(Task.Task):
	"""generate mavlink header files"""
	color   = 'GREEN'
	run_str = '${PYTHON} ${MAVGEN} --lang=C --wire-protocol=1.0 --output ${TGT} ${SRC}'

def options(opt):
	opt.load('python')

@feature('mavgen')
@before_method('process_source')
def process_mavgen(self):
    inputs = self.to_nodes(self.source)
    outputs = []

    self.target = Utils.to_list(getattr(self, 'target', []))
    for t in self.target:
        if not isinstance(t, Node.Node):
            t = self.bld.bldnode.find_or_declare(t)
        outputs.append(t)

    self.source = []
    self.create_task('mavgen', inputs, outputs)

def configure(cfg):
	"""
	setup environment for mavlink header generator
	"""
	cfg.load('python')
	cfg.check_python_version(minver=(2,7,0))

	env = cfg.env
	cfg.env.env = dict(os.environ)

	cfg.start_msg('Checking for message_definitions')
	if not cfg.srcnode.find_resource('modules/mavlink/message_definitions/v1.0/ardupilotmega.xml'):
		cfg.fatal(env.MAV_MSG_DEFS + ' not found, please run: git submodule init && git submodule update', color='RED')
		return
	cfg.end_msg('success')

	env.MAVLINK_DIR = cfg.srcnode.find_dir('modules/mavlink/').abspath()

	env.MAVLINK_HEADERS = cfg.bldnode.make_node('/libraries/GCS_MAVLink/include/mavlink/v1.0/').abspath()

	env.MAVGEN = env.MAVLINK_DIR  + '/pymavlink/tools/mavgen.py'
	cfg.env.env['PYTHONPATH'] = env.MAVLINK_DIR

	env.MAV_MSG_DEFS = cfg.srcnode.find_resource('modules/mavlink/message_definitions/v1.0/ardupilotmega.xml').abspath()

#!/usr/bin/env python
# encoding: utf-8
# (c) Siddharth Bharat Purohit, 3DRobotics Inc.

"""
The **mavgen.py** program is a code generator which creates mavlink header files.
"""

from waflib import Logs, Task, Utils, Node
from waflib.TaskGen import feature, before_method, extension
import os

class mavgen(Task.Task):
    """generate mavlink header files"""
    color   = 'BLUE'
    before  = 'cxx c'

    def run(self):
        python = self.env.get_flat('PYTHON')
        mavgen = self.env.get_flat('MAVGEN')
        out = self.env.get_flat('OUTPUT_DIR')
        src = self.env.get_flat('SRC')
        ret = self.exec_command('{} {} --lang=C --wire-protocol=1.0 --output {} {}'.format(
                                python, mavgen, out, self.inputs[0].abspath()))

        if ret != 0:
            # ignore if there was a signal to the interpreter rather
            # than a real error in the script. Some environments use a
            # signed and some an unsigned return for this
            if ret > 128 or ret < 0:
                Logs.warn('mavgen crashed with code: {}'.format(ret))
                ret = 0
            else:
                Logs.error('mavgen returned {} error code'.format(ret))
        return ret

    def post_run(self):
        super(mavgen, self).post_run()
        for header in self.generator.output_dir.ant_glob("*.h **/*.h", remove=False):
            header.sig = header.cache_sig = self.cache_sig

def options(opt):
    opt.load('python')

@feature('mavgen')
@before_method('process_source')
def process_mavgen(self):
    if not hasattr(self, 'output_dir'):
        self.bld.fatal('mavgen: missing option output_dir')

    inputs = self.to_nodes(self.source)
    outputs = []

    self.source = []

    if not isinstance(self.output_dir, Node.Node):
        self.output_dir = self.bld.bldnode.find_or_declare(self.output_dir)

    task = self.create_task('mavgen', inputs, outputs)
    task.env['OUTPUT_DIR'] = self.output_dir.abspath()

    task.env.env = dict(os.environ)
    task.env.env['PYTHONPATH'] = task.env.MAVLINK_DIR

def configure(cfg):
    """
    setup environment for mavlink header generator
    """
    cfg.load('python')
    cfg.check_python_version(minver=(2,7,0))

    env = cfg.env

    env.MAVLINK_DIR = cfg.srcnode.make_node('modules/mavlink/').abspath()
    env.MAVGEN = env.MAVLINK_DIR  + '/pymavlink/tools/mavgen.py'

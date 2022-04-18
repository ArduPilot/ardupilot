#!/usr/bin/env python3
# encoding: utf-8

"""
generate DSDLC headers for Cyphal
"""

from waflib import Logs, Task, Utils, Node
from waflib.TaskGen import feature, before_method, extension
import os
import os.path
from xml.etree import ElementTree as et

class cyphalgen(Task.Task):
    """generate cyphal header files"""
    color   = 'BLUE'
    before  = 'cxx c'

    def run(self):
        python = "python3"
        out = self.env.get_flat('OUTPUT_DIR')
        params="--target-language c --target-endianness=little -v"
        reg_dir = self.env.get_flat("REG_PATH")
        uavcan_dir = self.env.get_flat("UAVCAN_PATH")
        nunavut_dir = self.env.get_flat("NUNAVUT_PATH")

        commands = (
            "{} -m pip install {}".format(python, nunavut_dir),
            "nnvg {} {} --lookup-dir {} --outdir {}".format(params, reg_dir, uavcan_dir, out),
            "nnvg {} {} --outdir {}".format(params, uavcan_dir, out)
        )

        for command in commands:
            ret = self.exec_command(command)
            if ret != 0:
                # ignore if there was a signal to the interpreter rather
                # than a real error in the script. Some environments use a
                # signed and some an unsigned return for this
                if ret > 128 or ret < 0:
                    Logs.warn('cyphalgen crashed with code: {}'.format(ret))
                    ret = 0
                else:
                    Logs.error('cyphalgen returned {} error code'.format(ret))
                    break
        return ret

    def post_run(self):
        super(cyphalgen, self).post_run()
        for header in self.generator.output_dir.ant_glob("*.hpp **/*.hpp", remove=False):
            header.sig = header.cache_sig = self.cache_sig

def options(opt):
    opt.load('python3')

@feature('cyphalgen')
@before_method('process_source')
def process_cyphalgen(self):
    if not hasattr(self, 'output_dir'):
        self.bld.fatal('cyphalgen: missing option output_dir')

    inputs = self.to_nodes(self.source)
    outputs = []

    self.source = []

    if not isinstance(self.output_dir, Node.Node):
        self.output_dir = self.bld.bldnode.find_or_declare(self.output_dir)

    task = self.create_task('cyphalgen', inputs, outputs)
    task.env['OUTPUT_DIR'] = self.output_dir.abspath()

    task.env.env = dict(os.environ)

def configure(cfg):
    """
    setup environment for cyphal header generator
    """
    cfg.load('python')
    cfg.check_python_version(minver=(2,7,0))

    env = cfg.env
    pub_reg_data_types_path = cfg.srcnode.make_node('modules/cyphal/public_regulated_data_types').abspath()
    env.REG_PATH = pub_reg_data_types_path + '/reg'
    env.UAVCAN_PATH = pub_reg_data_types_path + '/uavcan'
    env.NUNAVUT_PATH = cfg.srcnode.make_node('modules/cyphal/nunavut').abspath()

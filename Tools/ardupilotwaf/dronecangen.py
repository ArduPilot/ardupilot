# encoding: utf-8

# flake8: noqa

"""
generate DSDLC headers for uavcan
"""

from waflib import Logs, Task, Utils, Node
from waflib.TaskGen import feature, before_method, extension
import os
import os.path
from xml.etree import ElementTree as et
import subprocess

class dronecangen(Task.Task):
    """generate uavcan header files"""
    color   = 'BLUE'
    before  = 'cxx c'

    def run(self):
        python = self.env.get_flat('PYTHON')
        out = self.env.get_flat('OUTPUT_DIR')
        src = self.env.get_flat('SRC')
        dsdlc = self.env.get_flat("DC_DSDL_COMPILER_DIR")

        cmd = ['{}'.format(python),
               '{}/dronecan_dsdlc.py'.format(dsdlc),
               '-O{}'.format(out)] + [x.abspath() for x in self.inputs]
        ret = self.exec_command(cmd)
        if ret != 0:
            # ignore if there was a signal to the interpreter rather
            # than a real error in the script. Some environments use a
            # signed and some an unsigned return for this
            if ret > 128 or ret < 0:
                Logs.warn('dronecangen crashed with code: {}'.format(ret))
                ret = 0
            else:
                Logs.warn('dronecangen: cmd=%s ' % str(cmd))
                # re-run command with stdout visible to see errors
                subprocess.call(cmd)
                Logs.error('dronecangen returned {} error code'.format(ret))
        return ret

    def post_run(self):
        super(dronecangen, self).post_run()
        for header in self.generator.output_dir.ant_glob("*.h **/*.h", remove=False):
            header.sig = header.cache_sig = self.cache_sig

def options(opt):
    opt.load('python')

@feature('dronecangen')
@before_method('process_rule')
def process_dronecangen(self):
    if not hasattr(self, 'output_dir'):
        self.bld.fatal('dronecangen: missing option output_dir')

    inputs = self.to_nodes(self.source)
    # depend on each message file in the source so rebuilds will occur properly
    deps = []
    for inp in inputs:
        deps.extend(inp.ant_glob("**/*.uavcan"))
    # also depend on the generator source itself
    dsdlc_dir = self.env.get_flat("DC_DSDL_COMPILER_DIR")
    dsdlc = self.bld.root.find_node(dsdlc_dir) # expected to be absolute
    if dsdlc is None:
        self.bld.fatal("dronecangen: waf couldn't find dsdlc at abspath {}".format(dsdlc_dir))
    deps.extend(dsdlc.ant_glob("**/*.py **/*.em"))
    outputs = []

    self.source = []

    if not isinstance(self.output_dir, Node.Node):
        self.output_dir = self.bld.bldnode.find_or_declare(self.output_dir)

    task = self.create_task('dronecangen', inputs, outputs)
    task.dep_nodes = deps
    task.env['OUTPUT_DIR'] = self.output_dir.abspath()

    task.env.env = dict(os.environ)

def configure(cfg):
    """
    setup environment for uavcan header generator
    """
    env = cfg.env
    env.DC_DSDL_COMPILER_DIR = cfg.srcnode.make_node('modules/DroneCAN/dronecan_dsdlc/').abspath()
    cfg.msg('DC_DSDL compiler in', env.DC_DSDL_COMPILER_DIR)

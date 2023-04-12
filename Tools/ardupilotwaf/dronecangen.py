# encoding: utf-8

"""
generate DSDLC headers for uavcan
"""

from waflib import Logs, Task, Utils, Node
from waflib.TaskGen import feature, before_method, extension
import os
import os.path
from xml.etree import ElementTree as et

class dronecangen(Task.Task):
    """generate uavcan header files"""
    color   = 'BLUE'
    before  = 'cxx c'

    def run(self):
        python = self.env.get_flat('PYTHON')
        out = self.env.get_flat('OUTPUT_DIR')
        src = self.env.get_flat('SRC')
        dsdlc = self.env.get_flat("DC_DSDL_COMPILER")

        ret = self.exec_command(['{}'.format(python),
                                 '{}'.format(dsdlc),
                                 '-O{}'.format(out)] + [x.abspath() for x in self.inputs])
        if ret != 0:
            # ignore if there was a signal to the interpreter rather
            # than a real error in the script. Some environments use a
            # signed and some an unsigned return for this
            if ret > 128 or ret < 0:
                Logs.warn('dronecangen crashed with code: {}'.format(ret))
                ret = 0
            else:
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
    outputs = []

    self.source = []

    if not isinstance(self.output_dir, Node.Node):
        self.output_dir = self.bld.bldnode.find_or_declare(self.output_dir)

    task = self.create_task('dronecangen', inputs, outputs)
    task.env['OUTPUT_DIR'] = self.output_dir.abspath()

    task.env.env = dict(os.environ)

def configure(cfg):
    """
    setup environment for uavcan header generator
    """
    cfg.load('python')
    cfg.check_python_version(minver=(2,7,0))

    env = cfg.env
    env.DC_DSDL_COMPILER_DIR = cfg.srcnode.make_node('modules/DroneCAN/dronecan_dsdlc/').abspath()
    env.DC_DSDL_COMPILER = env.DC_DSDL_COMPILER_DIR + '/dronecan_dsdlc.py'
    cfg.msg('DC_DSDL compiler', env.DC_DSDL_COMPILER)

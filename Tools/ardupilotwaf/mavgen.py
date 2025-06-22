# encoding: utf-8
# (c) Siddharth Bharat Purohit, 3DRobotics Inc.

# flake8: noqa

"""
The **mavgen.py** program is a code generator which creates mavlink header files.
"""

from waflib import Logs, Task, Utils, Node
from waflib.TaskGen import feature, before_method, extension
import sys
import os.path
from xml.etree import ElementTree as et

class mavgen(Task.Task):
    """generate mavlink header files"""
    color   = 'BLUE'
    before  = 'cxx c'

    def scan(self):
        nodes = []
        names = []

        entry_point = self.inputs[0]
        queue = [entry_point]
        head = 0

        while head < len(queue):
            node = queue[head]
            head += 1

            tree = et.parse(node.abspath())
            root = tree.getroot()
            includes = root.findall('include')
            for i in includes:
                path = i.text.strip()
                n = node.parent.find_node(path)
                if n:
                    nodes.append(n)
                    if n not in queue:
                        queue.append(n)
                    continue

                path = os.path.join(
                    node.parent.path_from(entry_point.parent),
                    path
                )
                if path not in names:
                    names.append(path)

        return nodes, names

    def run(self):
        sys.path.insert(0,self.env.get_flat('MAVLINK_DIR'))
        from pymavlink.generator import mavgen
        class mavgen_options:
            language = 'C'
            wire_protocol = '2.0'
            validate = False
            output = self.env.get_flat('OUTPUT_DIR')
        xml = self.inputs[0].abspath()
        if mavgen.mavgen(mavgen_options(), [xml]):
            return 0
        return 1

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

    inputs = self.to_nodes(self.bld.srcnode.find_node(self.source))
    outputs = []

    self.source = []

    if not isinstance(self.output_dir, Node.Node):
        self.output_dir = self.bld.bldnode.find_or_declare(self.output_dir)

    task = self.create_task('mavgen', inputs, outputs)
    task.env['OUTPUT_DIR'] = self.output_dir.abspath()

def configure(cfg):
    """
    setup environment for mavlink header generator
    """
    env = cfg.env
    env.MAVLINK_DIR = cfg.srcnode.make_node('modules/mavlink/').abspath()

#!/usr/bin/env python
# encoding: utf-8
# (c) aptj

"""
The **telemetry_mqtt.py** program is a code generator which creates mavlink header files.
"""

from waflib import Logs, Task, Utils, Node
from waflib.TaskGen import feature, before_method, extension
import os
import os.path
from datetime import datetime

class mqtt(Task.Task):
    """copy Mqtt files to AP_Telemetry/Mqtt"""
    color   = 'BLUE'

    def run(self):
        print('Copy modules/Mqtt to libraries/AP_Telemetry/Mqtt')
        now = datetime.now().strftime('%Y\/%m\/%d %H:%M:%S')
        cpy = self.env.get_flat('INPUT_DIR')
        src = self.inputs[0]
        major = '1'
        minor = '0'
        mkfile = open(src.abspath() + '/Makefile')
        mklines = mkfile.readlines()
        for line in mklines:
            if 'MAJOR_VERSION' in line:
                major = line.split('=')[1].strip()
            if 'MINOR_VERSION' in line:
                minor = line.split('=')[1].strip()
                break
        version = '{}.{}'.format(major, minor)
        
        if not os.path.exists(cpy):
            os.mkdir(cpy)
        files = os.listdir(src.abspath() + '/src')
        
        for f in files:
            print("copying {}".format(f))
            if 'VersionInfo.h.in' in f:
                sedcmd = 'sed -e "s/@CLIENT_VERSION@/{}/g" -e "s/@BUILD_TIMESTAMP@/{}/g" {} > {}'.format(
                    version, now, src.abspath() + '/src/' + f, cpy + '/VersionInfo.h')
                self.exec_command(sedcmd)
            if '.c' in f or '.h' in f:
                self.exec_command('cp {} {}'.format(
                                src.abspath()+'/src/'+f, cpy))
        print("copied!")

    def post_run(self):
        super(mqtt, self).post_run()

def cleanup(bld):
    print('Cleanup Mqtt files from AP_Telemetry direcotry')
    print(os.path.abspath('.'))
    apdir = ('./libraries/AP_Telemetry')
    mqttdir = ('./modules/Mqtt/src')
    apmqttfiles = os.listdir(apdir)
    mqttfiles = os.listdir(mqttdir)
    for f in apmqttfiles:
        if f == 'VersionInfo.h':
            os.remove("{}/{}".format(apdir, f))
            continue
        for m in mqttfiles:
            if ('.c' in m or '.h' in m) and m == f:
                os.remove("{}/{}".format(apdir, f))

def options(opt):
    opt.load('python')
    cpy = self.env.get_flat('INPUT_DIR')
    if os.path.exists(cpy):
        os.rmdir(cpy)

@feature('mqtt')
@before_method('process_source')
def process_mqtt(self):
    if not hasattr(self, 'input_dir'):
        self.bld.fatal('mqtt: missing option input_dir')


    inputs = self.to_nodes(self.source)
    outputs = []

    self.source = []

    if not isinstance(self.input_dir, Node.Node):
        self.input_dir = self.bld.bldnode.find_or_declare(self.input_dir)

    task = self.create_task('mqtt', inputs, outputs)
    task.env['INPUT_DIR'] = self.input_dir.abspath()

    task.env.env = dict(os.environ)

def configure(cfg):
    print('configure')
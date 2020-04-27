#!/usr/bin/env python
# encoding: utf-8

from waflib import Task, Node
from waflib.TaskGen import feature, before_method

class luagen(Task.Task):
	ext_out = ['.h']
	def run(self):
		return self.exec_command ('{} -o ./libraries/AP_Scripting/lua_generated_bindings -i {}'.format (
			self.inputs[0].abspath(),
			self.inputs[1].abspath()
		) )

	def post_run(self):
		super(luagen, self).post_run()
		for node in self.outputs:
			node.sig = node.cache_sig = self.cache_sig


@feature('luagen')
@before_method('process_source')
def process_luagen(self):
	inputs = [self.bld.path.find_resource('libraries/AP_Scripting/generator/gen-bindings'),self.bld.path.find_resource('libraries/AP_Scripting/generator/description/bindings.desc')]
	outputs = []

	task = self.create_task('luagen', inputs, outputs)

def configure(cfg):
	pass

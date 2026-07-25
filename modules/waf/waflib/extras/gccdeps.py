#!/usr/bin/env python
# encoding: utf-8
# Thomas Nagy, 2008-2010 (ita)

"""
Execute the tasks with gcc -MD, read the dependencies from the .d file
and prepare the dependency calculation for the next run.
This affects the cxx class, so make sure to load Qt5 after this tool.

Usage::

	def options(opt):
		opt.load('compiler_cxx')
	def configure(conf):
		conf.load('compiler_cxx gccdeps')
"""

import os, re, threading
from waflib import Task, Logs, Utils, Errors
from waflib.Tools import asm, c, c_preproc, cxx
from waflib.TaskGen import before_method, feature

lock = threading.Lock()

gccdeps_flags = ['-MD']
if not c_preproc.go_absolute:
	gccdeps_flags = ['-MMD']

# Third-party tools are allowed to add extra names in here with append()
supported_compilers = ['gas', 'gcc', 'icc', 'clang']

re_o = re.compile(r"\.o$")
re_splitter = re.compile(r'(?<!\\)\s+') # split by space, except when spaces are escaped

def remove_makefile_rule_lhs(line):
	# Splitting on a plain colon would accidentally match inside a
	# Windows absolute-path filename, so we must search for a colon
	# followed by whitespace to find the divider between LHS and RHS
	# of the Makefile rule.
	rulesep = ': '

	sep_idx = line.find(rulesep)
	if sep_idx >= 0:
		return line[sep_idx + 2:]
	else:
		return line

def path_to_node(base_node, path, cached_nodes):
	# Take the base node and the path and return a node
	# Results are cached because searching the node tree is expensive
	# The following code is executed by threads, it is not safe, so a lock is needed...
	if getattr(path, '__hash__'):
		node_lookup_key = (base_node, path)
	else:
		# Not hashable, assume it is a list and join into a string
		node_lookup_key = (base_node, os.path.sep.join(path))

	try:
		node = cached_nodes[node_lookup_key]
	except KeyError:
		# retry with lock on cache miss
		with lock:
			try:
				node = cached_nodes[node_lookup_key]
			except KeyError:
				node = cached_nodes[node_lookup_key] = base_node.find_resource(path)

	return node

def post_run(self):
	if not self.__class__.__name__ in self.env.ENABLE_GCCDEPS:
		return super(self.derived_gccdeps, self).post_run()

	deps_filename = self.outputs[0].abspath()
	deps_filename = re_o.sub('.d', deps_filename)
	try:
		deps_txt = Utils.readf(deps_filename)
	except EnvironmentError:
		Logs.error('Could not find a .d dependency file, are cflags/cxxflags overwritten?')
		raise

	# Compilers have the choice to either output the file's dependencies
	# as one large Makefile rule:
	#
	#   /path/to/file.o: /path/to/dep1.h \
	#                    /path/to/dep2.h \
	#                    /path/to/dep3.h \
	#                    ...
	#
	# or as many individual rules:
	#
	#   /path/to/file.o: /path/to/dep1.h
	#   /path/to/file.o: /path/to/dep2.h
	#   /path/to/file.o: /path/to/dep3.h
	#   ...
	#
	# So the first step is to sanitize the input by stripping out the left-
	# hand side of all these lines. After that, whatever remains are the
	# implicit dependencies of task.outputs[0]
	deps_txt = '\n'.join([remove_makefile_rule_lhs(line) for line in deps_txt.splitlines()])

	# Now join all the lines together
	deps_txt = deps_txt.replace('\\\n', '')

	dep_paths = deps_txt.strip()
	dep_paths = [x.replace('\\ ', ' ') for x in re_splitter.split(dep_paths) if x]

	resolved_nodes = []
	unresolved_names = []
	bld = self.generator.bld

	# Dynamically bind to the cache
	try:
		cached_nodes = bld.cached_nodes
	except AttributeError:
		cached_nodes = bld.cached_nodes = {}

	for path in dep_paths:

		node = None
		if os.path.isabs(path):
			node = path_to_node(bld.root, path, cached_nodes)
		else:
			# TODO waf 1.9 - single cwd value
			base_node = getattr(bld, 'cwdx', bld.bldnode)
			# when calling find_resource, make sure the path does not contain '..'
			path = [k for k in Utils.split_path(path) if k and k != '.']
			while '..' in path:
				idx = path.index('..')
				if idx == 0:
					path = path[1:]
					base_node = base_node.parent
				else:
					del path[idx]
					del path[idx-1]

			node = path_to_node(base_node, path, cached_nodes)

		if not node:
			continue

		if id(node) == id(self.inputs[0]):
			# ignore the source file, it is already in the dependencies
			# this way, successful config tests may be retrieved from the cache
			continue

		resolved_nodes.append(node)

	Logs.debug('deps: gccdeps for %s returned %s', self, resolved_nodes)

	bld.node_deps[self.uid()] = resolved_nodes
	bld.raw_deps[self.uid()] = unresolved_names

	try:
		del self.cache_sig
	except AttributeError:
		pass

	Task.Task.post_run(self)

def scan(self):
	if not self.__class__.__name__ in self.env.ENABLE_GCCDEPS:
		return super(self.derived_gccdeps, self).scan()

	resolved_nodes = self.generator.bld.node_deps.get(self.uid(), [])
	unresolved_names = []
	return (resolved_nodes, unresolved_names)

def sig_implicit_deps(self):
	if not self.__class__.__name__ in self.env.ENABLE_GCCDEPS:
		return super(self.derived_gccdeps, self).sig_implicit_deps()
	bld = self.generator.bld

	try:
		return self.compute_sig_implicit_deps()
	except Errors.TaskNotReady:
		raise ValueError("Please specify the build order precisely with gccdeps (asm/c/c++ tasks)")
	except EnvironmentError:
		# If a file is renamed, assume the dependencies are stale and must be recalculated
		for x in bld.node_deps.get(self.uid(), []):
			if not x.is_bld() and not x.exists():
				try:
					del x.parent.children[x.name]
				except KeyError:
					pass

	key = self.uid()
	bld.node_deps[key] = []
	bld.raw_deps[key] = []
	return Utils.SIG_NIL

def wrap_compiled_task(classname):
	derived_class = type(classname, (Task.classes[classname],), {})
	derived_class.derived_gccdeps = derived_class
	derived_class.post_run = post_run
	derived_class.scan = scan
	derived_class.sig_implicit_deps = sig_implicit_deps

for k in ('asm', 'c', 'cxx'):
	if k in Task.classes:
		wrap_compiled_task(k)

@before_method('process_source')
@feature('force_gccdeps')
def force_gccdeps(self):
	self.env.ENABLE_GCCDEPS = ['asm', 'c', 'cxx']

def configure(conf):
	# in case someone provides a --enable-gccdeps command-line option
	if not getattr(conf.options, 'enable_gccdeps', True):
		return

	global gccdeps_flags
	flags = conf.env.GCCDEPS_FLAGS or gccdeps_flags
	if conf.env.ASM_NAME in supported_compilers:
		try:
			conf.check(fragment='', features='asm force_gccdeps', asflags=flags, compile_filename='test.S', msg='Checking for asm flags %r' % ''.join(flags))
		except Errors.ConfigurationError:
			pass
		else:
			conf.env.append_value('ASFLAGS', flags)
			conf.env.append_unique('ENABLE_GCCDEPS', 'asm')

	if conf.env.CC_NAME in supported_compilers:
		try:
			conf.check(fragment='int main() { return 0; }', features='c force_gccdeps', cflags=flags, msg='Checking for c flags %r' % ''.join(flags))
		except Errors.ConfigurationError:
			pass
		else:
			conf.env.append_value('CFLAGS', flags)
			conf.env.append_unique('ENABLE_GCCDEPS', 'c')

	if conf.env.CXX_NAME in supported_compilers:
		try:
			conf.check(fragment='int main() { return 0; }', features='cxx force_gccdeps', cxxflags=flags, msg='Checking for cxx flags %r' % ''.join(flags))
		except Errors.ConfigurationError:
			pass
		else:
			conf.env.append_value('CXXFLAGS', flags)
			conf.env.append_unique('ENABLE_GCCDEPS', 'cxx')

def options(opt):
	raise ValueError('Do not load gccdeps options')


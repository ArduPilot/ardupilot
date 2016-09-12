#!/usr/bin/env python
# encoding: utf-8

# Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
#
# This file is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
Waf tool for external builds with cmake. This tool defines the feature
'cmake_build', for building through the cmake interface.

You can use CMAKE_MIN_VERSION environment variable before loading this tool in
the configuration to set a minimum version required for cmake. Example::

    def configure(cfg):
        cfg.CMAKE_MIN_VERSION = '3.5.2'
        cfg.load('cmake')

Usage example::

    def build(bld):
        # cmake configuration
        foo = bld.cmake(
            name='foo',
            cmake_src='path/to/foosrc', # where is the source tree
            cmake_bld='path/to/foobld', # where to generate the build system
            cmake_vars=dict(
                CMAKE_BUILD_TYPE='Release',
                ...
            ),
        )

        # cmake build for external target 'bar'
        bld(
            features='cmake_build',
            cmake_config='foo', # this build depends on the cmake generation above defined
            cmake_target='bar', # what to pass to option --target of cmake
        )

        # cmake build for target 'baz' (syntactic sugar)
        foo.build('baz')

The keys of cmake_vars are sorted so that unnecessary execution is avoided. If
you want to ensure an order in which the variables are passed to cmake, use an
OrderedDict. Example::

    def build(bld):
        foo_vars = OrderedDict()
        foo_vars['CMAKE_BUILD_TYPE'] = 'Release'
        foo_vars['FOO'] = 'value_of_foo'
        foo_vars['BAR'] = 'value_of_bar'

        # cmake configuration
        foo =  bld.cmake(
            cmake_vars=foo_vars,
            ...
        )

There may be cases when you want to establish dependency between other tasks and
the external build system's products (headers and libraries, for example). In
that case, you can specify the specific files in the option 'target' of your
cmake_build task generator. Example::

    def build(bld):
        ...

        # declaring on target only what I'm interested in
        foo.build('baz', target='path/to/foobld/include/baz.h')

        # myprogram.c includes baz.h, so the dependency is (implicitly)
        # established
        bld.program(target='myprogram', source='myprogram.c')

        # another example
        foo.build('another', target='another.txt')

        bld(
            rule='${CP} ${SRC} ${TGT}',
            source=bld.bldnode.find_or_declare('another.txt'),
            target='another_copied.txt',
        )


You can also establish the dependency directly on a task object::

    @feature('myfeature')
    def process_myfeature(self):
        baz_taskgen = self.bld.get_tgen_by_name('baz')
        baz_taskgen.post()

        # every cmake_build taskgen stores its task in cmake_build_task
        baz_task = baz_taskgen.cmake_build_task

        tsk = self.create_task('mytask')

        tsk.set_run_after(baz_task)

        # tsk is run whenever baz_task changes its outputs, namely,
        # path/to/foobld/include/baz.h
        tsk.dep_nodes.extend(baz_task.outputs)

If your cmake build creates several files (that may be dependency for several
tasks), you can use the parameter cmake_output_patterns. It receives a pattern
or a list of patterns relative to the cmake build directory. After the build
task is run, the files that match those patterns are set as output of the cmake
build task, so that they get a signature. Example::

    def build(bld):
        ...

        foo.build('baz', cmake_output_patterns='include/*.h')

        ...
"""

from waflib import Context, Node, Task, Utils
from waflib.Configure import conf
from waflib.TaskGen import feature, taskgen_method

from collections import OrderedDict
import os
import re
import sys

class cmake_configure_task(Task.Task):
    vars = ['CMAKE_BLD_DIR']
    run_str = '${CMAKE} ${CMAKE_SRC_DIR} ${CMAKE_VARS} ${CMAKE_GENERATOR_OPTION}'
    color = 'BLUE'

    def exec_command(self, cmd, **kw):
        kw['stdout'] = sys.stdout
        return super(cmake_configure_task, self).exec_command(cmd, **kw)

    def uid(self):
        if not hasattr(self, 'uid_'):
            m = Utils.md5()
            def u(s):
                m.update(s.encode('utf-8'))
            u(self.__class__.__name__)
            u(self.env.get_flat('CMAKE_SRC_DIR'))
            u(self.env.get_flat('CMAKE_BLD_DIR'))
            u(self.env.get_flat('CMAKE_VARS'))
            self.uid_ = m.digest()

        return self.uid_

    def __str__(self):
        return self.cmake.name

    def keyword(self):
        return 'CMake Configure'

# Clean cmake configuration
cmake_configure_task._original_run = cmake_configure_task.run
def _cmake_configure_task_run(self):
    cmakecache_path = self.outputs[0].abspath()
    if os.path.exists(cmakecache_path):
        os.remove(cmakecache_path)
    self._original_run()
cmake_configure_task.run = _cmake_configure_task_run

class cmake_build_task(Task.Task):
    run_str = '${CMAKE} --build ${CMAKE_BLD_DIR} --target ${CMAKE_TARGET}'
    color = 'BLUE'
    # the cmake-generated build system is responsible of managing its own
    # dependencies
    always_run = True

    def exec_command(self, cmd, **kw):
        kw['stdout'] = sys.stdout
        return super(cmake_build_task, self).exec_command(cmd, **kw)

    def uid(self):
        if not hasattr(self, 'uid_'):
            m = Utils.md5()
            def u(s):
                m.update(s.encode('utf-8'))
            u(self.__class__.__name__)
            u(self.env.get_flat('CMAKE_BLD_DIR'))
            u(self.env.get_flat('CMAKE_TARGET'))
            self.uid_ = m.digest()

        return self.uid_

    def __str__(self):
        return '%s %s' % (self.cmake.name, self.cmake_target)

    def keyword(self):
        return 'CMake Build'

# allow tasks to depend on possible headers or other resources if the user
# declares outputs for the cmake build
cmake_build_task = Task.update_outputs(cmake_build_task)

cmake_build_task.original_post_run = cmake_build_task.post_run
def _cmake_build_task_post_run(self):
    self.output_patterns = Utils.to_list(self.output_patterns)
    if not self.output_patterns:
        return self.original_post_run()
    bldnode = self.cmake.bldnode
    for node in bldnode.ant_glob(self.output_patterns, remove=False):
        self.set_outputs(node)
    return self.original_post_run()
cmake_build_task.post_run = _cmake_build_task_post_run

class CMakeConfig(object):
    '''
    CMake configuration. This object shouldn't be instantiated directly. Use
    bld.cmake().
    '''
    def __init__(self, bld, name, srcnode, bldnode, cmake_vars):
        self.bld = bld
        self.name = name
        self.srcnode = srcnode
        self.bldnode = bldnode
        self.vars = cmake_vars

        self._config_task = None
        self.last_build_task = None

    def vars_keys(self):
        keys = list(self.vars.keys())
        if not isinstance(self.vars, OrderedDict):
            keys.sort()
        return keys

    def config_sig(self):
        m = Utils.md5()
        def u(s):
            m.update(s.encode('utf-8'))
        u(self.srcnode.abspath())
        u(self.bldnode.abspath())
        keys = self.vars_keys()
        for k in keys:
            u(k)
            u(self.vars[k])
        return m.digest()

    def config_task(self, taskgen):
        sig = self.config_sig()
        if self._config_task and self._config_task.cmake_config_sig == sig:
            return self._config_task

        self._config_task = taskgen.create_task('cmake_configure_task')
        self._config_task.cwd = self.bldnode
        self._config_task.cmake = self
        self._config_task.cmake_config_sig = sig

        env = self._config_task.env
        env.CMAKE_BLD_DIR = self.bldnode.abspath()
        env.CMAKE_SRC_DIR = self.srcnode.abspath()

        keys = self.vars_keys()
        env.CMAKE_VARS = ["-D%s='%s'" % (k, self.vars[k]) for k in keys]

        self._config_task.set_outputs(
            self.bldnode.find_or_declare('CMakeCache.txt'),
        )

        if self.last_build_task:
            self._config_task.set_run_after(self.last_build_task)

        self.bldnode.mkdir()

        return self._config_task

    def build(self, cmake_target, **kw):
        return self.bld.cmake_build(self.name, cmake_target, **kw)

_cmake_instances = {}
def get_cmake(name):
    if name not in _cmake_instances:
        raise Exception('cmake: configuration named "%s" not found' % name)
    return _cmake_instances[name]

@conf
def cmake(bld, name, cmake_src=None, cmake_bld=None, cmake_vars={}):
    '''
    This function has two signatures:
     - bld.cmake(name, cmake_src, cmake_bld, cmake_vars):
        Create a cmake configuration.
     - bld.cmake(name):
        Get the cmake configuration with name.
    '''
    if not cmake_src and not cmake_bld and not cmake_vars:
        return get_cmake(name)

    if name in _cmake_instances:
        bld.fatal('cmake: configuration named "%s" already exists' % name)

    if not isinstance(cmake_src, Node.Node):
        cmake_src = bld.path.find_dir(cmake_src)

    if not cmake_bld:
        cmake_bld = cmake_src.get_bld()
    elif not isinstance(cmake_bld, Node.Node):
        cmake_bld = bld.bldnode.make_node(cmake_bld)

    c = CMakeConfig(bld, name, cmake_src, cmake_bld, cmake_vars)
    _cmake_instances[name] = c
    return c

@feature('cmake_build')
def process_cmake_build(self):
    if not hasattr(self, 'cmake_target'):
        self.bld.fatal('cmake_build: taskgen is missing cmake_target')
    if not hasattr(self, 'cmake_config'):
        self.bld.fatal('cmake_build: taskgen is missing cmake_config')

    tsk = self.create_cmake_build_task(self.cmake_config, self.cmake_target)
    self.cmake_build_task = tsk

    outputs = Utils.to_list(getattr(self, 'target', ''))
    if not isinstance(outputs, list):
        outputs = [outputs]

    for o in outputs:
        if not isinstance(o, Node.Node):
            o = self.path.find_or_declare(o)
        tsk.set_outputs(o)

    tsk.output_patterns = getattr(self, 'cmake_output_patterns', [])

@conf
def cmake_build(bld, cmake_config, cmake_target, **kw):
    kw['cmake_config'] = cmake_config
    kw['cmake_target'] = cmake_target
    kw['features'] = Utils.to_list(kw.get('features', [])) + ['cmake_build']

    if 'name' not in kw:
        kw['name'] = '%s_%s' % (cmake_config, cmake_target)

    return bld(**kw)

@taskgen_method
def create_cmake_build_task(self, cmake_config, cmake_target):
    cmake = get_cmake(cmake_config)

    tsk = self.create_task('cmake_build_task')
    tsk.cmake = cmake
    tsk.cmake_target = cmake_target
    tsk.output_patterns = []
    tsk.env.CMAKE_BLD_DIR = cmake.bldnode.abspath()
    tsk.env.CMAKE_TARGET = cmake_target

    self.cmake_config_task = cmake.config_task(self)
    tsk.set_run_after(self.cmake_config_task)

    if cmake.last_build_task:
        tsk.set_run_after(cmake.last_build_task)
    cmake.last_build_task = tsk

    return tsk

def _check_min_version(cfg):
    cfg.start_msg('Checking cmake version')
    cmd = cfg.env.get_flat('CMAKE'), '--version'
    out = cfg.cmd_and_log(cmd, quiet=Context.BOTH)
    m = re.search(r'\d+\.\d+(\.\d+(\.\d+)?)?', out)
    if not m:
        cfg.end_msg(
            'unable to parse version, build is not guaranteed to succeed',
            color='YELLOW',
        )
    else:
        version = Utils.num2ver(m.group(0))
        minver_str = cfg.env.get_flat('CMAKE_MIN_VERSION')
        minver = Utils.num2ver(minver_str)
        if version < minver:
            cfg.fatal('cmake must be at least at version %s' % minver_str)
        cfg.end_msg(m.group(0))

generators = dict(
    default=[
        (['ninja', 'ninja-build'], 'Ninja'),
        (['make'], 'Unix Makefiles'),
    ],
    win32=[
        (['ninja', 'ninja-build'], 'Ninja'),
        (['nmake'], 'NMake Makefiles'),
    ],
)

def configure(cfg):
    cfg.find_program('cmake')

    if cfg.env.CMAKE_MIN_VERSION:
        _check_min_version(cfg)

    l = generators.get(Utils.unversioned_sys_platform(), generators['default'])
    for names, generator in l:
        if cfg.find_program(names, mandatory=False):
            cfg.env.CMAKE_GENERATOR_OPTION = '-G%s' % generator
            break
    else:
        cfg.fatal("cmake: couldn't find a suitable CMake generator. " +
                  "The ones supported by this Waf tool for this platform are: %s" % ', '.join(g for _, g in l))

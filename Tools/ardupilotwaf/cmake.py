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
Waf tool for external builds with cmake. This tool defines two features:
    - cmake_configure: for defining task generators that do cmake build
      configuration, i.e., build system generation.
    - cmake_build: for actual build through the cmake interface.

Example::

    def build(bld):
        # cmake configuration
        foo = bld(
            features='cmake_configure',
            name='foo',
            cmake_src='path/to/foosrc', # where is the source tree
            cmake_bld='path/to/foobld', # where to generate the build system
            cmake_vars=dict(
                CMAKE_BUILD_TYPE='Release',
                ...
            ),
        )

        # cmake build for target (cmake target) 'bar'
        bld(
            features='cmake_build',
            cmake_config='foo', # this build depends on the cmake generation above defined
            cmake_target='bar', # what to pass to option --target of cmake
        )

        # cmake build for target 'baz' (syntactic sugar)
        foo.cmake_build('baz')

The keys of cmake_vars are sorted so that unnecessary execution is avoided. If
you want to ensure an order in which the variables are passed to cmake, use an
OrderedDict. Example::

    def build(bld):
        foo_vars = OrderedDict()
        foo_vars['CMAKE_BUILD_TYPE'] = 'Release'
        foo_vars['FOO'] = 'value_of_foo'
        foo_vars['BAR'] = 'value_of_bar'

        # cmake configuration
        foo = bld(
            features='cmake_configure',
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
        foo.cmake_build('baz', target='path/to/foobld/include/baz.h')

        # myprogram.c includes baz.h, so the dependency is (implicitly)
        # established
        bld.program(target='myprogram', source='myprogram.c')

        # another example
        foo.cmake_build('another', target='another.txt')

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

        foo.cmake_build('baz', cmake_output_patterns='include/*.h')

        ...
"""

from waflib import Node, Task, Utils
from waflib.TaskGen import feature, taskgen_method

from collections import OrderedDict
import os
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
            self.uid_ = m.digest()

        return self.uid_

    def __str__(self):
        return self.generator.name

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
        config_name = self.config_taskgen.name
        target = self.cmake_target
        return '%s %s' % (config_name, target)

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
    bldnode = self.config_taskgen.cmake_bld
    for node in bldnode.ant_glob(self.output_patterns, remove=False):
        self.set_outputs(node)
    return self.original_post_run()
cmake_build_task.post_run = _cmake_build_task_post_run

# the cmake-generated build system is responsible of managing its own
# dependencies
cmake_build_task = Task.always_run(cmake_build_task)

@feature('cmake_configure')
def process_cmake_configure(self):
    if not hasattr(self, 'name'):
        self.bld.fatal('cmake_configure: taskgen is missing name')
    if not hasattr(self, 'cmake_src'):
        self.bld.fatal('cmake_configure: taskgen is missing cmake_src')

    if not isinstance(self.cmake_src, Node.Node):
        self.cmake_src = self.bld.path.find_dir(self.cmake_src)

    self.get_cmake_bldnode()
    self.cmake_bld.mkdir()

    self.last_build_task = None

    self.cmake_vars = getattr(self, 'cmake_vars', {})

    # NOTE: we'll probably need to use the full class name in waf 1.9
    tsk = self.cmake_config_task = self.create_task('cmake_configure')
    tsk.cwd = self.cmake_bld.abspath()
    tsk.env.CMAKE_BLD_DIR = self.cmake_bld.abspath()
    tsk.env.CMAKE_SRC_DIR = self.cmake_src.abspath()

    keys = list(self.cmake_vars.keys())
    if not isinstance(self.cmake_vars, OrderedDict):
        keys.sort()
    tsk.env.CMAKE_VARS = ["-D%s='%s'" % (k, self.cmake_vars[k]) for k in keys]

    tsk.set_outputs(self.cmake_bld.find_or_declare('CMakeCache.txt'))

@feature('cmake_build')
def process_cmake_build(self):
    if not hasattr(self, 'cmake_target'):
        self.bld.fatal('cmake_build: taskgen is missing cmake_target')
    if not hasattr(self, 'cmake_config'):
        self.bld.fatal('cmake_build: taskgen is missing cmake_config')

    self.config_taskgen = self.bld.get_tgen_by_name(self.cmake_config)

    if not getattr(self.config_taskgen, 'posted', False):
        self.config_taskgen.post()

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

def _get_config_tg(self, cmake_config):
    if not cmake_config:
        if 'cmake_configure' in self.features:
            cmake_config = self.name
        elif 'cmake_build' in self.features:
            if hasattr(self, 'cmake_config'):
                cmake_config = self.cmake_config
    if not cmake_config:
        self.bld.fatal('cmake: cmake_config is missing or invalid')

    return self.bld.get_tgen_by_name(cmake_config)

@taskgen_method
def cmake_build(self, cmake_target, cmake_config=None, **kw):
    tg = _get_config_tg(self, cmake_config)

    kw['cmake_config'] = tg.name
    kw['cmake_target'] = cmake_target

    if 'name' not in kw:
        kw['name'] = '%s_%s' % (tg.name, cmake_target)

    kw['features'] = Utils.to_list(kw.get('features', []))
    kw['features'].append('cmake_build')

    return self.bld(**kw)

@taskgen_method
def get_cmake_bldnode(self, cmake_config=None):
    tg = _get_config_tg(self, cmake_config)

    if not hasattr(tg, 'cmake_bld'):
        tg.cmake_bld = tg.cmake_src.get_bld()
    elif not isinstance(tg.cmake_bld, Node.Node):
        tg.cmake_bld = tg.bld.bldnode.make_node(tg.cmake_bld)

    return tg.cmake_bld

@taskgen_method
def create_cmake_build_task(self, cmake_config, cmake_target):
    # NOTE: we'll probably need to use the full class name in waf 1.9
    tsk = self.create_task('cmake_build')
    config_tg = self.bld.get_tgen_by_name(cmake_config)
    tsk.config_taskgen = config_tg
    tsk.cmake_target = cmake_target
    tsk.output_patterns = []
    tsk.env.CMAKE_BLD_DIR = config_tg.cmake_bld.abspath()
    tsk.env.CMAKE_TARGET = cmake_target
    tsk.set_run_after(config_tg.cmake_config_task)

    if config_tg.last_build_task:
        tsk.set_run_after(config_tg.last_build_task)
    config_tg.last_build_task = tsk

    return tsk

def configure(cfg):
    cfg.find_program('cmake')
    cfg.find_program(['ninja', 'ninja-build'], var='NINJA', mandatory=False)
    cfg.env.CMAKE_GENERATOR_OPTION = ''
    if cfg.env.NINJA:
        cfg.env.CMAKE_GENERATOR_OPTION = '-GNinja'

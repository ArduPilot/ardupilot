#!/usr/bin/env python
# encoding: utf-8

"""
gbenchmark is a Waf tool for benchmark builds in Ardupilot
"""

from waflib import Build, Context, Task
from waflib.TaskGen import feature, before_method, after_method
from waflib.Errors import WafError

def _configure_cmake(ctx, bldnode):
    env = ctx.env
    my_build_node = bldnode.find_dir(env.GBENCHMARK_BUILD_REL)
    if not my_build_node:
        bldnode.make_node(env.GBENCHMARK_BUILD_REL).mkdir()

    prefix_node = bldnode.make_node(env.GBENCHMARK_PREFIX_REL)

    cmake_vars = {
        'CMAKE_BUILD_TYPE': 'Release',
        'CMAKE_INSTALL_PREFIX:PATH': prefix_node.abspath(),
    }

    cmake_vars = ' '.join("-D%s='%s'" % v for v in cmake_vars.items())

    cmd = '%s %s %s %s' % (
              env.CMAKE[0],
              env.GBENCHMARK_SRC,
              cmake_vars,
              env.GBENCHMARK_GENERATOR_OPTION
          )

    ctx.cmd_and_log(
        cmd,
        cwd=env.GBENCHMARK_BUILD,
        quiet=Context.BOTH,
    )

def configure(cfg):
    env = cfg.env
    env.HAS_GBENCHMARK = False

    if env.TOOLCHAIN != 'native':
        cfg.msg(
            'Gbenchmark',
            'cross-compilation currently not supported',
             color='YELLOW',
        )
        return

    cfg.find_program('cmake', mandatory=False)

    if not env.CMAKE:
        return

    env.GBENCHMARK_CMAKE_GENERATOR = None

    cfg.find_program(['ninja', 'ninja-build'], var='NINJA', mandatory=False)
    if env.NINJA:
        env.GBENCHMARK_CMAKE_GENERATOR = 'Ninja'

    env.GBENCHMARK_GENERATOR_OPTION = ''
    if env.GBENCHMARK_CMAKE_GENERATOR:
        env.GBENCHMARK_GENERATOR_OPTION = '-G%s' % env.GBENCHMARK_CMAKE_GENERATOR

    bldnode = cfg.bldnode.make_node(cfg.variant)
    prefix_node = bldnode.make_node('gbenchmark')
    my_build_node = bldnode.make_node('gbenchmark_build')
    my_src_node = cfg.srcnode.make_node('modules/gbenchmark')

    env.GBENCHMARK_PREFIX_REL = prefix_node.path_from(bldnode)
    env.GBENCHMARK_BUILD = my_build_node.abspath()
    env.GBENCHMARK_BUILD_REL = my_build_node.path_from(bldnode)
    env.GBENCHMARK_SRC = my_src_node.abspath()

    env.INCLUDES_GBENCHMARK = [prefix_node.make_node('include').abspath()]
    env.LIBPATH_GBENCHMARK = [prefix_node.make_node('lib').abspath()]
    env.LIB_GBENCHMARK = ['benchmark']

    env.append_value('GIT_SUBMODULES', 'gbenchmark')
    env.HAS_GBENCHMARK = True

class gbenchmark_build(Task.Task):
    def __init__(self, *k, **kw):
        super(gbenchmark_build, self).__init__(*k, **kw)

        bldnode = self.generator.bld.bldnode
        output_list = [
            '%s/%s' % (self.env.GBENCHMARK_PREFIX_REL, path)
            for path in (
                'include/benchmark/benchmark.h',
                'include/benchmark/macros.h',
                'include/benchmark/benchmark_api.h',
                'include/benchmark/reporter.h',
                'lib/libbenchmark.a',
            )
        ]
        self.outputs.extend([bldnode.make_node(f) for f in output_list])

    def run(self):
        bld = self.generator.bld
        cmds = []

        try:
            # Generate build system first, if necessary
            my_build_node = bld.bldnode.find_dir(self.env.GBENCHMARK_BUILD_REL)
            if not (my_build_node and my_build_node.find_resource('CMakeCache.txt')):
                _configure_cmake(bld, bld.bldnode)

            # Build gbenchmark
            cmd = '%s --build %s --target install' % (
                    self.env.CMAKE[0],
                    self.env.GBENCHMARK_BUILD
                  )
            bld.cmd_and_log(
                cmd,
                cwd=self.env.GBENCHMARK_BUILD,
                quiet=Context.BOTH,
            )
            return 0
        except WafError as e:
            print(e)
            if hasattr(e, 'stderr'):
                print('')
                print(e.stderr)
            return 1

    def __str__(self):
        return 'Google Benchmark'

gbenchmark_build = Task.always_run(Task.update_outputs(gbenchmark_build))

build_task = None

@feature('gbenchmark')
@before_method('process_use')
def append_gbenchmark_use(self):
    self.use = self.to_list(getattr(self, 'use', []))
    if 'GBENCHMARK' not in self.use:
        self.use.append('GBENCHMARK')

@feature('gbenchmark')
@after_method('process_source')
def wait_for_gbenchmark_build(self):
    global build_task

    if not build_task:
        build_task = self.create_task('gbenchmark_build')

    for task in self.compiled_tasks:
        task.set_run_after(build_task)
        task.dep_nodes.extend(build_task.outputs)

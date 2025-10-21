# encoding: utf-8

"""
gbenchmark is a Waf tool for benchmark builds in Ardupilot
"""

from waflib import Build, Context, Task
from waflib.Configure import conf
from waflib.TaskGen import feature, before_method, after_method
from waflib.Errors import WafError

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

    cfg.load('cmake')

    env.GBENCHMARK_PREFIX_REL = 'gbenchmark'

    bldnode = cfg.bldnode.make_node(cfg.variant)
    prefix_node = bldnode.make_node(env.GBENCHMARK_PREFIX_REL)

    env.INCLUDES_GBENCHMARK = [prefix_node.make_node('include').abspath()]
    env.LIBPATH_GBENCHMARK = [prefix_node.make_node('lib').abspath()]
    env.LIB_GBENCHMARK = ['benchmark']

    env.append_value('GIT_SUBMODULES', 'gbenchmark')
    env.HAS_GBENCHMARK = True

@conf
def libbenchmark(bld):
    prefix_node = bld.bldnode.make_node(bld.env.GBENCHMARK_PREFIX_REL)

    gbenchmark = bld.cmake(
        name='gbenchmark',
        cmake_src='modules/gbenchmark',
        cmake_bld='gbenchmark_build',
        cmake_vars=dict(
            CMAKE_BUILD_TYPE='Release',
            CMAKE_INSTALL_PREFIX=prefix_node.abspath(),
            BENCHMARK_ENABLE_GTEST_TESTS='OFF',
            BENCHMARK_ENABLE_TESTING='OFF',
        ),
    )

    prefix_node = bld.bldnode.make_node(bld.env.GBENCHMARK_PREFIX_REL)
    output_paths = (
        'lib/libbenchmark.a',
        'include/benchmark/benchmark.h',
    )
    outputs = [prefix_node.make_node(path) for path in output_paths]
    gbenchmark.build('install', target=outputs)

@feature('gbenchmark')
@before_method('process_use')
def append_gbenchmark_use(self):
    self.use = self.to_list(getattr(self, 'use', []))
    if 'GBENCHMARK' not in self.use:
        self.use.append('GBENCHMARK')

@feature('gbenchmark')
@after_method('process_source')
def wait_for_gbenchmark_install(self):
    gbenchmark_install = self.bld.get_tgen_by_name('gbenchmark_install')
    gbenchmark_install.post()

    for task in self.compiled_tasks:
        task.set_run_after(gbenchmark_install.cmake_build_task)
        task.dep_nodes.extend(gbenchmark_install.cmake_build_task.outputs)

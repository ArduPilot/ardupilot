#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for defining ardupilot's submodules, so that they are kept up to date.
Submodules can be considered dynamic sources, since they are updated during the
build. Furthermore, they can be used to generate other dynamic sources (mavlink
headers generation, for example). Thus, the correct use of this tool should
have three build groups: first one for updating the submodules, second for
generating any dynamic source from them, and the last one for the build. And
post_mode should be set to POST_LAZY. Example::

    def build(bld):
        bld.post_mode = waflib.Build.POST_LAZY

        bld.add_group('git_submodules')
        # gtest submodule
        bld(
            features='git_submodule'
            git_submodule='gtest',
        )
        # mavlink submodule with syntactic sugar
        bld.git_submodule('mavlink')
        ...

        # now, for the dynamic sources
        bld.add_group('dynamic_sources')
        ...

        # now, below go the task generators for normal build process
        bld.add_group('build')
        ...
"""

from waflib import Context, Task, Utils
from waflib.Configure import conf
from waflib.TaskGen import before_method, feature, taskgen_method

import os.path

class update_submodule(Task.Task):
    color = 'BLUE'
    run_str = '${GIT} -C ${SRC_ROOT} submodule update --init -- ${SUBMODULE_PATH}'

    def runnable_status(self):
        e = self.env.get_flat
        cmd = e('GIT'), '-C', e('SRC_ROOT'), 'submodule', 'status', '--', e('SUBMODULE_PATH')
        out = self.generator.bld.cmd_and_log(cmd, quiet=Context.BOTH)

        # git submodule status uses a blank prefix for submodules that are up
        # to date
        if out[0] != ' ':
            return Task.RUN_ME

        return Task.SKIP_ME

    def uid(self):
        if not hasattr(self, 'uid_'):
            m = Utils.md5()
            m.update(self.__class__.__name__)
            m.update(self.env.get_flat('SUBMODULE_PATH'))
            self.uid_ = m.digest()

        return self.uid_

    def __str__(self):
        return 'Submodule update: %s' % self.submodule

def configure(cfg):
    cfg.find_program('git')

_submodules_tasks = {}

@taskgen_method
def git_submodule_update(self, name):
    if name not in _submodules_tasks:
        module_node = self.bld.srcnode.make_node(os.path.join('modules', name))

        tsk = self.create_task('update_submodule', submodule=name)
        tsk.env.SRC_ROOT = self.bld.srcnode.abspath()
        tsk.env.SUBMODULE_PATH = module_node.abspath()

        _submodules_tasks[name] = tsk

    return _submodules_tasks[name]


@feature('git_submodule')
@before_method('process_source')
def process_module_dependencies(self):
    self.git_submodule = getattr(self, 'git_submodule', '')
    if not self.git_submodule:
        self.bld.fatal('git_submodule: empty or missing git_submodule argument')
    self.git_submodule_update(self.git_submodule)

@conf
def git_submodule(bld, git_submodule, **kw):
    kw['git_submodule'] = git_submodule
    kw['features'] = Utils.to_list(kw.get('features', ''))
    kw['features'].append('git_submodule')

    return bld(**kw)

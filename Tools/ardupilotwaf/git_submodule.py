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

from waflib import Context, Logs, Task, Utils
from waflib.Configure import conf
from waflib.TaskGen import before_method, feature, taskgen_method

import os.path
import re

class update_submodule(Task.Task):
    color = 'BLUE'
    run_str = '${GIT} submodule update --recursive --init -- ${SUBMODULE_PATH}'

    fast_forward_diff_re = dict(
        removed=re.compile(r'-Subproject commit ([0-9a-f]+)'),
        added=re.compile(r'\+Subproject commit ([0-9a-f]+)')
    )

    def is_fast_forward(self, path):
        bld = self.generator.bld
        git = self.env.get_flat('GIT')

        cmd = git, 'diff', '--submodule=short', '--', os.path.basename(path)
        cwd = self.cwd.make_node(os.path.dirname(path))
        out = bld.cmd_and_log(cmd, quiet=Context.BOTH, cwd=cwd)

        m = self.fast_forward_diff_re['removed'].search(out)
        n = self.fast_forward_diff_re['added'].search(out)
        if not m or not n:
            bld.fatal('git_submodule: failed to parse diff')

        head = n.group(1)
        wanted = m.group(1)
        cmd = git, 'merge-base', head, wanted
        cwd = self.cwd.make_node(path)
        out = bld.cmd_and_log(cmd, quiet=Context.BOTH, cwd=cwd)

        return out.strip() == head

    def runnable_status(self):
        e = self.env.get_flat
        cmd = e('GIT'), 'submodule', 'status', '--recursive', '--', e('SUBMODULE_PATH')
        out = self.generator.bld.cmd_and_log(cmd, quiet=Context.BOTH, cwd=self.cwd)

        self.non_fast_forward = []

        # git submodule status uses a blank prefix for submodules that are up
        # to date
        r = Task.SKIP_ME
        for line in out.splitlines():
            prefix = line[0]
            path = line[1:].split()[1]
            if prefix == ' ':
                continue
            if prefix == '-':
                r = Task.RUN_ME
            if prefix == '+':
                if not self.is_fast_forward(path):
                    self.non_fast_forward.append(path)
                else:
                    r = Task.RUN_ME

        if self.non_fast_forward:
            r = Task.SKIP_ME

        return r

    def uid(self):
        if not hasattr(self, 'uid_'):
            m = Utils.md5()
            def u(s):
                m.update(s.encode('utf-8'))
            u(self.__class__.__name__)
            u(self.env.get_flat('SUBMODULE_PATH'))
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
        tsk.cwd = self.bld.srcnode
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

def _post_fun(bld):
    Logs.info('')
    for name, t in _submodules_tasks.items():
        if not t.non_fast_forward:
            continue
        Logs.warn("Submodule %s not updated: non-fastforward" % name)

@conf
def git_submodule_post_fun(bld):
    bld.add_post_fun(_post_fun)

def _git_head_hash(ctx, path, short=False):
    cmd = [ctx.env.get_flat('GIT'), 'rev-parse']
    if short:
        cmd.append('--short=8')
    cmd.append('HEAD')
    out = ctx.cmd_and_log(cmd, quiet=Context.BOTH, cwd=path)
    return out.strip()

@conf
def git_submodule_head_hash(self, name, short=False):
    module_node = self.srcnode.make_node(os.path.join('modules', name))
    return _git_head_hash(self, module_node.abspath(), short=short)

@conf
def git_head_hash(self, short=False):
    return _git_head_hash(self, self.srcnode.abspath(), short=short)

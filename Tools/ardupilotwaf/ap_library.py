# Copyright (C) 2016  Intel Corporation. All rights reserved.
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
Waf tool for Ardupilot libraries. The function bld.ap_library() creates the
necessary task generators for creating the objects of a library for a vehicle.
That includes the common objects, which are shared among vehicles. That
function is used by bld.ap_stlib() and shouldn't need to be called otherwise.

The environment variable AP_LIBRARIES_OBJECTS_KW is a dictionary of keyword
arguments to be passed to bld.objects() when during the creation of the task
generators. You can use it to pass extra arguments to that function (although
some of them will be rewritten, see the implementation for details).

This tool also checks if the headers used by the source files don't use
vehicle-related headers and fails the build if they do.
"""
import os
import re

from waflib import Errors, Task, Utils
from waflib.Configure import conf
from waflib.TaskGen import after_method, before_method, feature
from waflib.Tools import c_preproc

import ardupilotwaf as ap

UTILITY_SOURCE_EXTS = ['utility/' + glob for glob in ap.SOURCE_EXTS]

def _common_tgen_name(library):
    return 'objs/%s' % library

def _vehicle_tgen_name(library, vehicle):
    return 'objs/%s/%s' % (library, vehicle)

_vehicle_indexes = {}
def _vehicle_index(vehicle):
    """ Used for the objects taskgens idx parameter """
    if vehicle not in _vehicle_indexes:
        _vehicle_indexes[vehicle] = len(_vehicle_indexes) + 1
    return _vehicle_indexes[vehicle]

_vehicle_macros = ('SKETCHNAME', 'SKETCH', 'APM_BUILD_DIRECTORY',
                   'APM_BUILD_TYPE')
_macros_re = re.compile(r'\b(%s)\b' % '|'.join(_vehicle_macros))

def _remove_comments(s):
    return c_preproc.re_cpp.sub(c_preproc.repl, s)

_depends_on_vehicle_cache = {}
def _depends_on_vehicle(bld, source_node):
    path = source_node.srcpath()

    if path not in _depends_on_vehicle_cache:
        s = _remove_comments(source_node.read())
        _depends_on_vehicle_cache[path] = _macros_re.search(s) is not None

    return _depends_on_vehicle_cache[path]

@conf
def ap_library(bld, library, vehicle):
    try:
        common_tg = bld.get_tgen_by_name(_common_tgen_name(library))
    except Errors.WafError:
        common_tg = None

    try:
        vehicle_tg = bld.get_tgen_by_name(_vehicle_tgen_name(library, vehicle))
    except Errors.WafError:
        vehicle_tg = None

    if common_tg and vehicle_tg:
        return

    if library.find('*') != -1:
        # allow for wildcard patterns, used for submodules without direct waf support
        library_dir = bld.srcnode.find_dir('.')
        wildcard = library
    else:
        library_dir = bld.srcnode.find_dir('libraries/%s' % library)
        wildcard = ap.SOURCE_EXTS + UTILITY_SOURCE_EXTS

    if not library_dir:
        bld.fatal('ap_library: %s not found' % library)

    src = library_dir.ant_glob(wildcard)

    if not common_tg:
        kw = dict(bld.env.AP_LIBRARIES_OBJECTS_KW)
        kw['features'] = kw.get('features', []) + ['ap_library_object']
        kw.update(
            name=_common_tgen_name(library),
            source=[s for s in src if not _depends_on_vehicle(bld, s)],
            idx=0,
        )
        bld.objects(**kw)

    if not vehicle_tg:
        source = [s for s in src if _depends_on_vehicle(bld, s)]

        if not source:
            return

        kw = dict(bld.env.AP_LIBRARIES_OBJECTS_KW)
        kw['features'] = kw.get('features', []) + ['ap_library_object']
        kw.update(
            name=_vehicle_tgen_name(library, vehicle),
            source=source,
            defines=ap.get_legacy_defines(vehicle),
            idx=_vehicle_index(vehicle),
        )
        bld.objects(**kw)

@before_method('process_use')
@feature('cxxstlib')
def process_ap_libraries(self):
    self.use = Utils.to_list(getattr(self, 'use', []))
    libraries = Utils.to_list(getattr(self, 'ap_libraries', []))
    vehicle = getattr(self, 'ap_vehicle', None)

    for l in libraries:
        self.use.append(_common_tgen_name(l))
        if vehicle:
            self.use.append(_vehicle_tgen_name(l, vehicle))

class ap_library_check_headers(Task.Task):
    color = 'PINK'
    before  = 'cxx c'
    dispatched_headers = set()
    whitelist = (
        'libraries/AP_Vehicle/AP_Vehicle_Type.h',
        'libraries/AP_Camera/AP_RunCam.h',
    )
    whitelist = tuple(os.path.join(*p.split('/')) for p in whitelist)

    def run(self):
        for n in self.headers:
            s = _remove_comments(n.read())
            if _macros_re.search(s):
                raise Errors.WafError('%s: library header uses vehicle-dependent macros' % n.srcpath())

    def uid(self):
        try:
            return self._uid
        except AttributeError:
            self._uid = 'check_headers-%s' % self.compiled_task.uid()
            return self._uid

    def signature(self):
        bld = self.generator.bld
        # force scan() to be called
        bld.imp_sigs[self.uid()] = None
        s = super(ap_library_check_headers, self).signature()
        bld.ap_persistent_task_sigs[self.uid()] = s
        return s

    def scan(self):
        r = []
        self.headers = []

        srcnode_path = self.generator.bld.srcnode.abspath()

        # force dependency scan, if necessary
        self.compiled_task.signature()
        for n in self.generator.bld.node_deps[self.compiled_task.uid()]:
            # using common Node methods doesn't work here
            p = n.abspath()
            if not p.startswith(srcnode_path):
                continue
            if os.path.relpath(p, srcnode_path) in self.whitelist:
                continue

            r.append(n)
            if n not in self.dispatched_headers:
                self.headers.append(n)
                self.dispatched_headers.add(n)

        return r, []

    def __str__(self):
        return str(self.compiled_task)

    def keyword(self):
        return 'Checking included headers'

@feature('ap_library_object')
@after_method('process_source')
def ap_library_register_for_check(self):
    if not hasattr(self, 'compiled_tasks'):
        return

    if not self.env.ENABLE_HEADER_CHECKS:
        return

    for t in self.compiled_tasks:
        tsk = self.create_task('ap_library_check_headers')
        tsk.compiled_task = t

def configure(cfg):
    cfg.env.AP_LIBRARIES_OBJECTS_KW = dict()

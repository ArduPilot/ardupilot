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

This tool also provides a feature to check if the headers used by the source
files don't use vehicle-related headers. The task generators for performing the
checks are created with bld.ap_library_check_headers(). They should be created
in a build group that is processed after the compilation tasks, because they
use the result of Waf's C preprocessor to know what headers should be checked.
The results can be reported to the user by calling
bld.ap_library_check_summary() to add the post build callback.
"""
import re

from waflib import Errors, Logs, Task, Utils
from waflib.Configure import conf
from waflib.TaskGen import before_method, feature
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

    library_dir = bld.srcnode.find_dir('libraries/%s' % library)
    if not library_dir:
        bld.fatal('ap_library: %s not found' % library)

    src = library_dir.ant_glob(ap.SOURCE_EXTS + UTILITY_SOURCE_EXTS)

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

class ap_library_check_header(Task.Task):
    color = 'PINK'
    tasks = []

    def run(self):
        self.tasks.append(self)
        n = self.inputs[0]
        s = _remove_comments(n.read())
        self.guilty = _macros_re.search(s) is not None

    def post_run(self):
        super(ap_library_check_header, self).post_run()
        if self.guilty:
            self.generator.bld.task_sigs[self.uid()] = None

    def keyword(self):
        return 'Checking header'

_object_tgens = []
@feature('ap_library_object')
def ap_library_register_for_check(self):
    _object_tgens.append(self)

_headers_exceptions = (
    'libraries/AP_Vehicle/AP_Vehicle_Type.h',
)
_headers = set()
@feature('ap_library_check_headers')
def ap_library_process_headers(self):
    for o in _object_tgens:
        if not hasattr(o, 'compiled_tasks'):
            continue

        for t in o.compiled_tasks:
            for n in self.bld.node_deps[t.uid()]:
                if not n.is_src():
                    continue
                if n.srcpath() in _headers_exceptions:
                    continue
                if n in _headers:
                    continue
                _headers.add(n)
                self.create_task('ap_library_check_header', n)

@conf
def ap_library_check_headers(bld, name):
    return bld(name=name, features='ap_library_check_headers')

def _check_summary(bld):
    if not ap_library_check_header.tasks:
        return

    guilty = [t.inputs[0].srcpath() for t in ap_library_check_header.tasks if t.guilty]
    if guilty:
        Logs.warn('ap_libraries: the following headers use vehicle-dependent macros:')
        for path in guilty:
            Logs.warn('    %s' % path)
        bld.fatal('ap_libraries: there are headers using vehicle-dependent macros')
    else:
        Logs.info('ap_libraries: all headers okay')

@conf
def ap_library_check_summary(bld):
    bld.add_post_fun(_check_summary)

def configure(cfg):
    cfg.env.AP_LIBRARIES_OBJECTS_KW = dict()

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

from waflib import Errors, Task, Utils, Logs
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

# note that AP_NavEKF3_core.h is needed for AP_NavEKF3_feature.h
_vehicle_macros = ['APM_BUILD_DIRECTORY', 'AP_BUILD_TARGET_NAME',
                   'APM_BUILD_TYPE', 'APM_BUILD_COPTER_OR_HELI',
                   'AP_NavEKF3_core.h', 'lua_generated_bindings.h',
                   'AP_InertialSensor_rate_config.h']
_macros_re = re.compile(r'\b(%s)\b' % '|'.join(_vehicle_macros))

# some cpp files are not available at the time we run this check so need to be
# unilaterally added
_vehicle_cpp_need_macros = ['lua_generated_bindings.cpp']
_macros_cpp_re = re.compile(r'\b(%s)\b' % '|'.join(_vehicle_cpp_need_macros))

def _remove_comments(s):
    return c_preproc.re_cpp.sub(c_preproc.repl, s)

_depends_on_vehicle_cache = {}
def _depends_on_vehicle(bld, source_node):
    path = source_node.srcpath()

    if not bld.env.BUILDROOT:
        bld.env.BUILDROOT = bld.bldnode.make_node('').abspath()

    if _macros_cpp_re.search(path) is not None:
        _depends_on_vehicle_cache[path] = True

    if path not in _depends_on_vehicle_cache:
        try:
            s = _remove_comments(source_node.read())
            _depends_on_vehicle_cache[path] = _macros_re.search(s) is not None
        except Exception:
            return False

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

    # allow for dynamically generated sources in a library that inherit the
    # dependencies and includes
    if library in bld.env.AP_LIB_EXTRA_SOURCES:
        for s in bld.env.AP_LIB_EXTRA_SOURCES[library]:
            src.append(bld.bldnode.find_or_declare(os.path.join('libraries', library, s)))

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
            defines=ap.get_legacy_defines(vehicle, bld),
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

@before_method('process_source')
@feature('cxxstlib')
def dynamic_post(self):
    if not getattr(self, 'dynamic_source', None):
        return
    self.source = Utils.to_list(self.source)
    self.source.extend(self.bld.bldnode.ant_glob(self.dynamic_source))

class ap_library_check_headers(Task.Task):
    color = 'PINK'
    before  = 'cxx c'
    dispatched_headers = set()
    whitelist = (
        'libraries/AP_Vehicle/AP_Vehicle_Type.h',
        'libraries/AP_Common/AP_FWVersionDefine.h',
        'libraries/AP_Scripting/lua_generated_bindings.h',
        'libraries/AP_NavEKF3/AP_NavEKF3_feature.h',
        'libraries/AP_LandingGear/AP_LandingGear_config.h',
        'libraries/AP_InertialSensor/AP_InertialSensor_rate_config.h',
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
        if not self.compiled_task.uid() in self.generator.bld.node_deps:
            return r, []
        for n in self.generator.bld.node_deps[self.compiled_task.uid()]:
            # using common Node methods doesn't work here
            p = n.abspath()
            if not p.startswith(srcnode_path):
                continue
            rel_p = os.path.relpath(p, srcnode_path)
            if rel_p in self.whitelist:
                continue

            # check if the path ends with something in the white list
            # this is required for white listing files in 'build/' (for scripting generated bindings)
            found = False
            for m in self.whitelist:
                if rel_p.endswith(m):
                    found = True
                    break
            
            if found:
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

def custom_flags_check(tgen):
    '''
     check for tasks marked as having custom cpp or c flags
     a library can do this by setting AP_LIB_EXTRA_CXXFLAGS and AP_LIB_EXTRA_CFLAGS

     For example add this is the configure section of the library, using AP_DDS as an example:

        cfg.env.AP_LIB_EXTRA_CXXFLAGS['AP_DDS'] = ['-DSOME_CXX_FLAG']
        cfg.env.AP_LIB_EXTRA_CFLAGS['AP_DDS'] = ['-DSOME_C_FLAG']
    '''
    if not tgen.name.startswith("objs/"):
        return
    libname = tgen.name[5:]
    if libname in tgen.env.AP_LIB_EXTRA_CXXFLAGS:
        tgen.env.CXXFLAGS.extend(tgen.env.AP_LIB_EXTRA_CXXFLAGS[libname])
    if libname in tgen.env.AP_LIB_EXTRA_CFLAGS:
        tgen.env.CFLAGS.extend(tgen.env.AP_LIB_EXTRA_CFLAGS[libname])


def double_precision_check(tasks):
    '''check for tasks marked as double precision'''

    for t in tasks:
        if len(t.inputs) == 1:
            # get a list of tasks we need to change to be double precision
            double_tasks = []
            for library in t.env.DOUBLE_PRECISION_SOURCES.keys():
                for s in t.env.DOUBLE_PRECISION_SOURCES[library]:
                    double_tasks.append([library, s])

            src = str(t.inputs[0]).split('/')[-2:]
            double_library = t.env.DOUBLE_PRECISION_LIBRARIES.get(src[0],False)

            if double_library or src in double_tasks:
                t.env.CXXFLAGS = t.env.CXXFLAGS[:]
                for opt in ['-fsingle-precision-constant', '-cl-single-precision-constant']:
                    try:
                        t.env.CXXFLAGS.remove(opt)
                    except ValueError:
                        pass
                t.env.CXXFLAGS.append("-DALLOW_DOUBLE_MATH_FUNCTIONS")


def gsoap_library_check(bld, tasks):
    '''check for tasks marked as gSOAP library source'''

    for t in tasks:
        if len(t.inputs) == 1:
            gsoap_tasks = []
            for s in t.env.AP_LIB_EXTRA_SOURCES["AP_ONVIF"]:
                gsoap_tasks.append(bld.bldnode.find_or_declare(os.path.join('libraries', "AP_ONVIF", s)))

            if t.inputs[0] in gsoap_tasks:
                t.env.CXXFLAGS += [
                    '-Wno-shadow',
                ]
                if 'clang++' not in t.env.COMPILER_CXX:
                    t.env.CXXFLAGS += [
                        '-Wno-suggest-override',
                    ]


@feature('ap_library_object')
@after_method('process_source')
def ap_library_register_for_check(self):
    if not hasattr(self, 'compiled_tasks'):
        return

    custom_flags_check(self)
    double_precision_check(self.compiled_tasks)
    if self.env.ENABLE_ONVIF:
        gsoap_library_check(self.bld, self.compiled_tasks)

    if not self.env.ENABLE_HEADER_CHECKS:
        return

    for t in self.compiled_tasks:
        tsk = self.create_task('ap_library_check_headers')
        tsk.compiled_task = t

def write_compilation_database(bld):
    """
    Write the compilation database as JSON
    """
    database_file = bld.bldnode.find_or_declare('compile_commands.json')
    # don't remove the file at clean

    Logs.info('Build commands will be stored in %s', database_file.path_from(bld.path))
    try:
        root = database_file.read_json()
    except IOError:
        root = []
    compile_cmd_db = dict((x['file'], x) for x in root)
    for task in bld.compilation_database_tasks:
        try:
            cmd = task.last_cmd
        except AttributeError:
            continue
        f_node = task.inputs[0]
        filename = f_node.path_from(task.get_cwd())
        entry = {
            "directory": task.get_cwd().abspath(),
            "arguments": cmd,
            "file": filename,
        }
        compile_cmd_db[filename] = entry
    root = list(compile_cmd_db.values())
    database_file.write_json(root)

def target_list_changed(bld, targets):
    """
    Check if the list of targets has changed recorded in target_list file
    """
    # target_list file is in the root build directory
    target_list_file = bld.bldnode.find_or_declare('target_list')
    try:
        with open(target_list_file.abspath(), 'r') as f:
            old_targets = f.read().strip().split(',')
    except IOError:
        Logs.info('No target_list file found, creating')
        old_targets = []
    if old_targets != targets:
        with open(target_list_file.abspath(), 'w') as f:
            f.write(','.join(targets))
        return True
    return False

@conf
def remove_target_list(cfg):
    target_list_file = cfg.bldnode.make_node(cfg.options.board + '/target_list')
    try:
        Logs.info('Removing target_list file %s', target_list_file.abspath())
        os.remove(target_list_file.abspath())
    except OSError:
        pass

@feature('cxxprogram', 'cxxstlib')
@after_method('propagate_uselib_vars')
def dry_run_compilation_database(self):
    if not hasattr(self, 'bld'):
        return
    bld = self.bld
    bld.compilation_database_tasks = []
    targets = bld.targets.split(',')
    use = self.use
    if isinstance(use, str):
        use = [use]
    # if targets have not changed and neither has configuration, 
    # we can skip compilation database generation
    if not target_list_changed(bld, targets + use):
        Logs.info('Targets have not changed, skipping compilation database compile_commands.json generation')
        return
    Logs.info('Generating compile_commands.json')
    # we need only to generate last_cmd, so override
    # exec_command temporarily
    def exec_command(bld, *k, **kw):
        return 0

    for g in bld.groups:
        for tg in g:
            # we only care to list targets and library objects
            if not hasattr(tg, 'name'):
                continue
            if (tg.name not in targets) and (tg.name not in self.use):
                continue
            try:
                f = tg.post
            except AttributeError:
                pass
            else:
                f()

            if isinstance(tg, Task.Task):
                lst = [tg]
            else:
                lst = tg.tasks
            for tsk in lst:
                if tsk.__class__.__name__ == "swig":
                    tsk.runnable_status()
                    if hasattr(tsk, 'more_tasks'):
                        lst.extend(tsk.more_tasks)
                # Not all dynamic tasks can be processed, in some cases
                # one may have to call the method "run()" like this:
                # elif tsk.__class__.__name__ == 'src2c':
                #    tsk.run()
                #    if hasattr(tsk, 'more_tasks'):
                #        lst.extend(tsk.more_tasks)

                tup = tuple(y for y in [Task.classes.get(x) for x in ('c', 'cxx')] if y)
                if isinstance(tsk, tup):
                    bld.compilation_database_tasks.append(tsk)
                    tsk.nocache = True
                    old_exec = tsk.exec_command
                    tsk.exec_command = exec_command
                    tsk.run()
                    tsk.exec_command = old_exec

    write_compilation_database(bld)

def configure(cfg):
    cfg.env.AP_LIBRARIES_OBJECTS_KW = dict()
    cfg.env.AP_LIB_EXTRA_SOURCES = dict()
    cfg.env.AP_LIB_EXTRA_CXXFLAGS = dict()
    cfg.env.AP_LIB_EXTRA_CFLAGS = dict()
    cfg.env.DOUBLE_PRECISION_SOURCES = dict()
    cfg.env.DOUBLE_PRECISION_LIBRARIES = dict()

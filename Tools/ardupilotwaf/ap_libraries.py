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

NOTE: This tool is provisional, it shouldn't be necessary when all libraries
become independent of vehicle. Thus, this tool should contain the only
*minimal* functionality required and addition of features should be avoided.
"""
from waflib import Utils
from waflib.Configure import conf
from waflib.TaskGen import before_method, feature

import ardupilotwaf as ap

UTILITY_SOURCE_EXTS = ['utility/' + glob for glob in ap.SOURCE_EXTS]

AP_LIBRARIES_VEHICLE_DEPENDENT_SRC = [
    'libraries/AP_AccelCal/AP_AccelCal.cpp',
    'libraries/AP_BattMonitor/AP_BattMonitor.cpp',
    'libraries/AP_Compass/AP_Compass.cpp',
    'libraries/AP_HAL_Linux/Storage.cpp',
    'libraries/AP_HAL_PX4/HAL_PX4_Class.cpp',
    'libraries/AP_HAL_PX4/Storage.cpp',
    'libraries/AP_HAL_QURT/mainapp/mainapp.cpp',
    'libraries/AP_HAL_SITL/SITL_cmdline.cpp',
    'libraries/AP_HAL_VRBRAIN/HAL_VRBRAIN_Class.cpp',
    'libraries/AP_HAL_VRBRAIN/Storage.cpp',
    'libraries/AP_InertialSensor/AP_InertialSensor.cpp',
    'libraries/AP_Mission/AP_Mission.cpp',
    'libraries/AP_NavEKF2/AP_NavEKF2.cpp',
    'libraries/AP_NavEKF/AP_NavEKF_core.cpp',
    'libraries/AP_NavEKF/AP_NavEKF.cpp',
    'libraries/AP_Rally/AP_Rally.cpp',
    'libraries/AP_Scheduler/AP_Scheduler.cpp',
]

_common_objects_tgens = {}
""" Mapped by library """
_vehicle_objects_tgens = {}
""" Mapped by (library, vehicle) """

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

@conf
def ap_library(bld, library, vehicle):
    if library in _common_objects_tgens and \
       (library, vehicle) in _vehicle_objects_tgens:
        return

    library_dir = bld.srcnode.find_dir('libraries/%s' % library)
    if not library_dir:
        bld.fatal('ap_libraries: %s not found' % library)

    vehicle_dependent = AP_LIBRARIES_VEHICLE_DEPENDENT_SRC
    sources = library_dir.ant_glob(ap.SOURCE_EXTS + UTILITY_SOURCE_EXTS)

    if library not in _common_objects_tgens:
        kw = dict(bld.env.AP_LIBRARIES_OBJECTS_KW)
        kw['features'] = kw.get('features', []) + bld.env.AP_LIBRARY_FEATURES
        kw.update(
            name=_common_tgen_name(library),
            source=[
                s for s in sources
                if s.path_from(bld.srcnode) not in vehicle_dependent
            ],
            idx=0,
        )
        _common_objects_tgens[library] = bld.objects(**kw)

    if (library, vehicle) not in _vehicle_objects_tgens:
        source = [
            s for s in sources
            if s.path_from(bld.srcnode) in vehicle_dependent
        ]

        if not source:
            return

        kw = dict(bld.env.AP_LIBRARIES_OBJECTS_KW)
        kw['features'] = kw.get('features', []) + bld.env.AP_LIBRARY_FEATURES
        kw.update(
            name=_vehicle_tgen_name(library, vehicle),
            source=source,
            defines=ap.get_legacy_defines(vehicle),
            idx=_vehicle_index(vehicle),
        )
        _vehicle_objects_tgens[(library, vehicle)] = bld.objects(**kw)

@before_method('process_use')
@feature('cxxstlib')
def process_ap_libraries(self):
    self.use = Utils.to_list(getattr(self, 'use', []))
    libraries = Utils.to_list(getattr(self, 'ap_libraries', []))
    vehicle = getattr(self, 'ap_vehicle', None)

    for l in libraries:
        if l not in _common_objects_tgens:
            self.bld.fatal('ap_libraries: common objects for %s not declared' % l)

        self.use.append(_common_tgen_name(l))

        if not vehicle or (l, vehicle) not in _vehicle_objects_tgens:
            continue

        self.use.append(_vehicle_tgen_name(l, vehicle))

def configure(cfg):
    cfg.env.AP_LIBRARIES_OBJECTS_KW = dict()

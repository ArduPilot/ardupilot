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
Module that changes Waf to keep persistent information across clean operations
in for performance improvement.
"""
from waflib import Build, Task

Build.SAVED_ATTRS.append('ap_persistent_task_sigs')
Build.SAVED_ATTRS.append('ap_persistent_imp_sigs')
Build.SAVED_ATTRS.append('ap_persistent_node_deps')

_original_signature = Task.Task.signature

_original_sig_implicit_deps = Task.Task.sig_implicit_deps
if hasattr(_original_sig_implicit_deps, '__func__'):
    _original_sig_implicit_deps = _original_sig_implicit_deps.__func__

def _signature(self):
    s = _original_signature(self)
    real_fn = self.sig_implicit_deps.__func__
    if not self.scan or _original_sig_implicit_deps != real_fn:
        return s
    bld = self.generator.bld
    bld.ap_persistent_imp_sigs[self.uid()] = bld.imp_sigs[self.uid()]
    bld.ap_persistent_node_deps[self.uid()] = bld.node_deps[self.uid()]
    return s
Task.Task.signature = _signature

class CleanContext(Build.CleanContext):
    def clean(self):
        if not self.options.clean_all_sigs:
            saved_task_sigs = dict(self.ap_persistent_task_sigs)
            saved_imp_sigs = dict(self.ap_persistent_imp_sigs)
            saved_node_deps = dict(self.ap_persistent_node_deps)

        super(CleanContext, self).clean()

        if not self.options.clean_all_sigs:
            self.task_sigs.update(saved_task_sigs)
            self.ap_persistent_task_sigs.update(saved_task_sigs)

            self.imp_sigs.update(saved_imp_sigs)
            self.ap_persistent_imp_sigs.update(saved_imp_sigs)

            self.node_deps.update(saved_node_deps)
            self.ap_persistent_node_deps.update(saved_node_deps)

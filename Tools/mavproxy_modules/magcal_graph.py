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
'''
This module shows the geodesic sections hit by the samples collected during
compass calibration, and also some status data. The objective of this module is
to provide a reference on how to interpret the field `completion_mask` from the
MAG_CAL_PROGRESS mavlink message. That information can be used in order to
guide the vehicle user during calibration.

The plot shown by this module isn't very helpful to the end user, but it might
help developers during development of internal calibration support in ground
control stations.
'''
from MAVProxy.modules.lib import mp_module, mp_util
import multiprocessing

class MagcalGraph():
    def __init__(self):
        self.parent_pipe, self.child_pipe = multiprocessing.Pipe()
        self.ui_process = None
        self._last_mavlink_msgs = {}

    def start(self):
        if self.is_active():
            return
        if self.ui_process:
            self.ui_process.join()

        for l in self._last_mavlink_msgs.values():
            for m in l:
                if not m:
                    continue
                self.parent_pipe.send(m)

        self.ui_process = multiprocessing.Process(target=self.ui_task)
        self.ui_process.start()

    def stop(self):
        if not self.is_active():
            return

        self.parent_pipe.send('close')
        self.ui_process.join()

    def ui_task(self):
        mp_util.child_close_fds()

        from MAVProxy.modules.lib import wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx
        from lib.magcal_graph_ui import MagcalFrame

        app = wx.App(False)
        app.frame = MagcalFrame(self.child_pipe)
        app.frame.Show()
        app.MainLoop()

    def is_active(self):
        return self.ui_process is not None and self.ui_process.is_alive()

    def mavlink_packet(self, m):
        if m.compass_id not in self._last_mavlink_msgs:
            # Keep the two last messages so that, if one is the calibration
            # report message, the previous one is the last progress message.
            self._last_mavlink_msgs[m.compass_id] = [None, m]
        else:
            l = self._last_mavlink_msgs[m.compass_id]
            l[0] = l[1]
            l[1] = m

        if not self.is_active():
            return
        self.parent_pipe.send(m)

class MagcalGraphModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(MagcalGraphModule, self).__init__(mpstate, 'magcal_graph')
        self.add_command(
            'magcal_graph',
            self.cmd_magcal_graph,
            'open a window to report magcal progress and plot geodesic ' +
            'sections hit by the collected data in real time',
        )

        self.graph = MagcalGraph()

    def cmd_magcal_graph(self, args):
        self.graph.start()

    def mavlink_packet(self, m):
        if m.get_type() not in ('MAG_CAL_PROGRESS', 'MAG_CAL_REPORT'):
            return
        self.graph.mavlink_packet(m)

    def unload(self):
        self.graph.stop()

def init(mpstate):
    return MagcalGraphModule(mpstate)

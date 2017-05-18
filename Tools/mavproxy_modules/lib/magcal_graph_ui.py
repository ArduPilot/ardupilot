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
import matplotlib.pyplot as plt
from matplotlib.backends.backend_wxagg import FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pymavlink.mavutil import mavlink

from MAVProxy.modules.lib import wx_processguard
from MAVProxy.modules.lib.wx_loader import wx

import geodesic_grid as grid

class MagcalPanel(wx.Panel):
    _status_markup_strings = {
        mavlink.MAG_CAL_NOT_STARTED: 'Not started',
        mavlink.MAG_CAL_WAITING_TO_START: 'Waiting to start',
        mavlink.MAG_CAL_RUNNING_STEP_ONE: 'Step one',
        mavlink.MAG_CAL_RUNNING_STEP_TWO: 'Step two',
        mavlink.MAG_CAL_SUCCESS: '<span color="blue">Success</span>',
        mavlink.MAG_CAL_FAILED: '<span  color="red">Failed</span>',
    }

    _empty_color = '#7ea6ce'
    _filled_color = '#4680b9'

    def __init__(self, *k, **kw):
        super(MagcalPanel, self).__init__(*k, **kw)

        facecolor = self.GetBackgroundColour().GetAsString(wx.C2S_HTML_SYNTAX)
        fig = plt.figure(facecolor=facecolor, figsize=(1,1))

        self._canvas = FigureCanvas(self, wx.ID_ANY, fig)
        self._canvas.SetMinSize((300,300))

        self._id_text = wx.StaticText(self, wx.ID_ANY)
        self._status_text = wx.StaticText(self, wx.ID_ANY)
        self._completion_pct_text = wx.StaticText(self, wx.ID_ANY)

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self._id_text)
        sizer.Add(self._status_text)
        sizer.Add(self._completion_pct_text)
        sizer.Add(self._canvas, proportion=1, flag=wx.EXPAND)
        self.SetSizer(sizer)

        ax = fig.add_subplot(111, axis_bgcolor=facecolor, projection='3d')
        self.configure_plot(ax)

    def configure_plot(self, ax):
        extra = .5
        lim = grid.radius + extra
        ax.set_xlim3d(-lim, lim)
        ax.set_ylim3d(-lim, lim)
        ax.set_zlim3d(-lim, lim)

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        ax.invert_zaxis()
        ax.invert_xaxis()

        ax.set_aspect('equal')

        self._polygons_collection = Poly3DCollection(
            grid.sections_triangles,
            edgecolors='#386694',
        )
        ax.add_collection3d(self._polygons_collection)

    def update_status_from_mavlink(self, m):
        status_string = self._status_markup_strings.get(m.cal_status, '???')
        self._status_text.SetLabelMarkup(
            '<b>Status:</b> %s' % status_string,
        )

    def mavlink_magcal_report(self, m):
        self.update_status_from_mavlink(m)
        self._completion_pct_text.SetLabel('')

    def mavlink_magcal_progress(self, m):
        facecolors = []
        for i, mask in enumerate(m.completion_mask):
            for j in range(8):
                section = i * 8 + j
                if mask & 1 << j:
                    facecolor = self._filled_color
                else:
                    facecolor = self._empty_color
                facecolors.append(facecolor)
        self._polygons_collection.set_facecolors(facecolors)
        self._canvas.draw()

        self._id_text.SetLabelMarkup(
            '<b>Compass id:</b> %d' % m.compass_id
        )

        self._completion_pct_text.SetLabelMarkup(
            '<b>Completion:</b> %d%%' % m.completion_pct
        )

        self.update_status_from_mavlink(m)

    _legend_panel = None
    @staticmethod
    def legend_panel(*k, **kw):
        if MagcalPanel._legend_panel:
            return MagcalPanel._legend_panel

        p = MagcalPanel._legend_panel = wx.Panel(*k, **kw)
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        p.SetSizer(sizer)

        marker = wx.Panel(p, wx.ID_ANY, size=(10, 10))
        marker.SetBackgroundColour(MagcalPanel._empty_color)
        sizer.Add(marker, flag=wx.ALIGN_CENTER)
        text = wx.StaticText(p, wx.ID_ANY)
        text.SetLabel('Sections not hit')
        sizer.Add(text, border=4, flag=wx.ALIGN_CENTER | wx.LEFT)

        marker = wx.Panel(p, wx.ID_ANY, size=(10, 10))
        marker.SetBackgroundColour(MagcalPanel._filled_color)
        sizer.Add(marker, border=10, flag=wx.ALIGN_CENTER | wx.LEFT)
        text = wx.StaticText(p, wx.ID_ANY)
        text.SetLabel('Sections hit')
        sizer.Add(text, border=4, flag=wx.ALIGN_CENTER | wx.LEFT)
        return p

class MagcalFrame(wx.Frame):
    def __init__(self, conn):
        super(MagcalFrame, self).__init__(
            None,
            wx.ID_ANY,
            title='Magcal Graph',
        )

        self.SetMinSize((300, 300))

        self._conn = conn

        self._main_panel = wx.ScrolledWindow(self, wx.ID_ANY)
        self._main_panel.SetScrollbars(1, 1, 1, 1)

        self._magcal_panels = {}

        self._sizer = wx.BoxSizer(wx.VERTICAL)
        self._main_panel.SetSizer(self._sizer)

        idle_text = wx.StaticText(self._main_panel, wx.ID_ANY)
        idle_text.SetLabelMarkup('<i>No calibration messages received yet...</i>')
        idle_text.SetForegroundColour('#444444')

        self._sizer.AddStretchSpacer()
        self._sizer.Add(
            idle_text,
            proportion=0,
            flag=wx.ALIGN_CENTER | wx.ALL,
            border=10,
        )
        self._sizer.AddStretchSpacer()

        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.timer_callback, self._timer)
        self._timer.Start(200)

    def add_compass(self, id):
        if not self._magcal_panels:
            self._sizer.Clear(deleteWindows=True)
            self._magcal_panels_sizer = wx.BoxSizer(wx.HORIZONTAL)

            self._sizer.Add(
                self._magcal_panels_sizer,
                proportion=1,
                flag=wx.EXPAND,
            )

            legend = MagcalPanel.legend_panel(self._main_panel, wx.ID_ANY)
            self._sizer.Add(
                legend,
                proportion=0,
                flag=wx.ALIGN_CENTER,
            )

        self._magcal_panels[id] = MagcalPanel(self._main_panel, wx.ID_ANY)
        self._magcal_panels_sizer.Add(
            self._magcal_panels[id],
            proportion=1,
            border=10,
            flag=wx.EXPAND | wx.ALL,
        )

    def timer_callback(self, evt):
        close_requested = False
        mavlink_msgs = {}
        while self._conn.poll():
            m = self._conn.recv()
            if isinstance(m, str) and m == 'close':
                close_requested = True
                continue
            if m.compass_id not in mavlink_msgs:
                # Keep the last two messages so that we get the last progress
                # if the last message is the calibration report.
                mavlink_msgs[m.compass_id] = [None, m]
            else:
                l = mavlink_msgs[m.compass_id]
                l[0] = l[1]
                l[1] = m

        if close_requested:
            self._timer.Stop()
            self.Destroy()
            return

        if not mavlink_msgs:
            return

        needs_fit = False
        for k in mavlink_msgs:
            if k not in self._magcal_panels:
                self.add_compass(k)
                needs_fit = True
        if needs_fit:
            self._sizer.Fit(self)

        for k, l in mavlink_msgs.items():
            for m in l:
                if not m:
                    continue
                panel = self._magcal_panels[k]
                if m.get_type() == 'MAG_CAL_PROGRESS':
                    panel.mavlink_magcal_progress(m)
                elif m.get_type() == 'MAG_CAL_REPORT':
                    panel.mavlink_magcal_report(m)

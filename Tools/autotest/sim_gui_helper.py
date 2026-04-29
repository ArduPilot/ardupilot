import wx
import os
import sys

class SimVehicleGUI(wx.App):
    def __init__(self, opts, vinfo, vehicle_map, autodetected=None):
        self.opts = opts
        self.vinfo = vinfo
        self.vehicle_map = vehicle_map
        self.autodetected = autodetected
        self.launch = False
        super().__init__()

    def show_and_run(self):
        frame = GUIFrame(None, "ArduPilot SITL GUI Launcher", self.opts, self.vinfo, self.vehicle_map, self.autodetected)
        frame.Centre()
        frame.Show()
        self.MainLoop()
        return self.launch

class GUIFrame(wx.Frame):
    def __init__(self, parent, title, opts, vinfo, vehicle_map, autodetected):
        super().__init__(parent, title=title, size=(480, 750))
        self.opts = opts
        self.vinfo = vinfo
        self.autodetected = autodetected
        
        # Initialize Config
        self.config = wx.Config("ArduPilotSITLGUI")
        
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # --- Header ---
        header_text = wx.StaticText(panel, label="SITL Simulation Setup")
        header_font = header_text.GetFont()
        header_font.SetPointSize(14)
        header_font.SetWeight(wx.FONTWEIGHT_BOLD)
        header_text.SetFont(header_font)
        main_sizer.Add(header_text, 0, wx.ALL | wx.ALIGN_CENTER_HORIZONTAL, 15)

        # --- Vehicle Selection ---
        main_sizer.Add(wx.StaticText(panel, label="Vehicle Type:"), 0, wx.LEFT | wx.RIGHT | wx.TOP, 15)
        
        display_names = ["ArduCopter", "AntennaTracker", "ArduPlane", "ArduSub", "Blimp", "Rover", "Helicopter"]
        
        default_idx = 0
        status_label = "Select vehicle type to simulate"
        status_color = wx.Colour(100, 100, 100)

        target_vehicle = opts.vehicle or autodetected
        if target_vehicle in display_names:
            default_idx = display_names.index(target_vehicle)
            if autodetected:
                status_label = f"Locked to {target_vehicle} (Context Detected)"
                status_color = wx.Colour(139, 0, 0) # Dark Red for locked
            else:
                status_label = f"Default set to {target_vehicle} (CLI Argument)"
                status_color = wx.Colour(0, 128, 0)

        self.vehicle_cb = wx.ComboBox(panel, choices=display_names, style=wx.CB_READONLY)
        self.vehicle_cb.SetSelection(default_idx)
        
        # Enforce strict directory-locking logic if autodetected
        if autodetected:
            self.vehicle_cb.Disable()

        main_sizer.Add(self.vehicle_cb, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 15)
        
        self.status_text = wx.StaticText(panel, label=status_label)
        self.status_text.SetForegroundColour(status_color)
        main_sizer.Add(self.status_text, 0, wx.LEFT | wx.RIGHT | wx.TOP, 10)

        # --- Location Selection ---
        main_sizer.Add(wx.StaticLine(panel), 0, wx.EXPAND | wx.ALL, 15)
        main_sizer.Add(wx.StaticText(panel, label="Start Location (-L):"), 0, wx.LEFT, 15)
        
        locations = self._parse_locations()
        self.loc_cb = wx.ComboBox(panel, choices=locations, style=wx.CB_DROPDOWN | wx.TE_PROCESS_ENTER)
        
        # CLI priority over Config over Default
        target_loc = opts.location if opts.location else self.config.Read("LastLocation", "CMAC")
        self.loc_cb.SetValue(target_loc)
        main_sizer.Add(self.loc_cb, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 15)

        # --- Aircraft / Scenario History ---
        main_sizer.Add(wx.StaticText(panel, label="Aircraft / Scenario Name (-A):"), 0, wx.LEFT | wx.TOP, 15)

        # Load history
        history_str = self.config.Read("AircraftHistory", "")
        self.aircraft_history = history_str.split('|') if history_str else []

        self.aircraft_cb = wx.ComboBox(panel, choices=self.aircraft_history, style=wx.CB_DROPDOWN | wx.TE_PROCESS_ENTER)
        self.aircraft_cb.SetHint("e.g. MyTestCopter")
        
        # CLI priority over Config
        target_aircraft = opts.aircraft if opts.aircraft else self.config.Read("LastAircraft", "")
        self.aircraft_cb.SetValue(target_aircraft)
        main_sizer.Add(self.aircraft_cb, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 15)

        # --- Instance Settings ---
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.StaticText(panel, label="Number of Vehicles (-n):"), 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT, 15)
        self.instance_spc = wx.SpinCtrl(panel, value=str(getattr(opts, 'count', 1)), min=1, max=20)
        hbox.Add(self.instance_spc, 0, wx.LEFT, 10)
        main_sizer.Add(hbox, 0, wx.EXPAND | wx.TOP, 10)

        # --- MAVProxy Options & Modules ---
        main_sizer.Add(wx.StaticLine(panel), 0, wx.EXPAND | wx.ALL, 15)
        main_sizer.Add(wx.StaticText(panel, label="SITL & MAVProxy Modules:"), 0, wx.LEFT, 15)
        
        self.no_mavproxy_chk = wx.CheckBox(panel, label="Disable MAVProxy (--no-mavproxy)")
        self.map_chk = wx.CheckBox(panel, label="Enable Map (--map)")
        self.console_chk = wx.CheckBox(panel, label="Enable Console (--console)")
        self.osd_chk = wx.CheckBox(panel, label="Enable OSD (--osd)")
        
        # Resolve initial states (CLI priority > Config)
        init_no_mav = opts.no_mavproxy if getattr(opts, 'no_mavproxy', None) is not None else self.config.ReadBool("NoMavproxy", False)
        init_map = opts.map if opts.map is not None else self.config.ReadBool("Map", False)
        init_console = opts.console if opts.console is not None else self.config.ReadBool("Console", False)
        init_osd = getattr(opts, 'OSD', None) if getattr(opts, 'OSD', None) is not None else self.config.ReadBool("OSD", False)

        self.no_mavproxy_chk.SetValue(init_no_mav)
        self.map_chk.SetValue(init_map)
        self.console_chk.SetValue(init_console)
        self.osd_chk.SetValue(init_osd)
        
        self.no_mavproxy_chk.Bind(wx.EVT_CHECKBOX, self._on_no_mavproxy_toggle)

        main_sizer.Add(self.no_mavproxy_chk, 0, wx.LEFT | wx.TOP, 15)
        main_sizer.Add(self.map_chk, 0, wx.LEFT | wx.TOP, 10)
        main_sizer.Add(self.console_chk, 0, wx.LEFT | wx.TOP, 10)
        main_sizer.Add(self.osd_chk, 0, wx.LEFT | wx.TOP, 10)

        # Apply interdependency logic on boot
        self._enforce_mavproxy_dependencies()

        # --- Launch Button ---
        main_sizer.AddStretchSpacer()
        self.launch_btn = wx.Button(panel, label="START SIMULATION", size=(-1, 60))
        self.launch_btn.SetBackgroundColour(wx.Colour(34, 139, 34)) # Forest Green
        self.launch_btn.SetForegroundColour(wx.WHITE)
        
        btn_font = self.launch_btn.GetFont()
        btn_font.SetWeight(wx.FONTWEIGHT_BOLD)
        btn_font.SetPointSize(12)
        self.launch_btn.SetFont(btn_font)
        
        self.launch_btn.Bind(wx.EVT_BUTTON, self.on_launch)
        main_sizer.Add(self.launch_btn, 0, wx.EXPAND | wx.ALL, 20)

        panel.SetSizer(main_sizer)
        self.Layout()

    def _parse_locations(self):
        """Parses Tools/autotest/locations.txt for location names."""
        locs = ["CMAC"] # Absolute fallback
        # Assuming sim_gui_helper.py is in Tools/autotest/
        loc_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'locations.txt')
        
        if os.path.exists(loc_path):
            parsed_locs = set()
            with open(loc_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        parts = line.split('=')
                        if len(parts) >= 2:
                            parsed_locs.add(parts[0].strip())
            if parsed_locs:
                locs = sorted(list(parsed_locs))
        return locs

    def _on_no_mavproxy_toggle(self, event):
        self._enforce_mavproxy_dependencies()

    def _enforce_mavproxy_dependencies(self):
        """Disables and unchecks dependent modules if MAVProxy is disabled."""
        disabled = self.no_mavproxy_chk.GetValue()
        deps = [self.map_chk, self.console_chk, self.osd_chk]
        
        for chk in deps:
            if disabled:
                chk.SetValue(False)
                chk.Disable()
            else:
                chk.Enable()

    def _save_state(self, aircraft_val, loc_val):
        """Saves current GUI selections to wx.Config."""
        # Save checkboxes
        self.config.WriteBool("NoMavproxy", self.no_mavproxy_chk.GetValue())
        self.config.WriteBool("Map", self.map_chk.GetValue())
        self.config.WriteBool("Console", self.console_chk.GetValue())
        self.config.WriteBool("OSD", self.osd_chk.GetValue())
        
        # Save Location
        self.config.Write("LastLocation", loc_val)
        self.config.Write("LastAircraft", aircraft_val)
        
        # Update Aircraft History (keep last 5 unique)
        if aircraft_val:
            if aircraft_val in self.aircraft_history:
                self.aircraft_history.remove(aircraft_val)
            self.aircraft_history.insert(0, aircraft_val)
            self.aircraft_history = self.aircraft_history[:5]
            self.config.Write("AircraftHistory", "|".join(self.aircraft_history))
            
        self.config.Flush()

    def on_launch(self, event):
        # 1. Gather values
        aircraft_val = self.aircraft_cb.GetValue().strip()
        loc_val = self.loc_cb.GetValue().strip()
        choice = self.vehicle_cb.GetValue()
        
        # 2. Persist State
        self._save_state(aircraft_val, loc_val)

        # 3. Inject back into opts
        self.opts.vehicle = "ArduCopter" if choice == "Helicopter" else choice
        self.opts.count = self.instance_spc.GetValue()
        self.opts.aircraft = aircraft_val if aircraft_val else None
        self.opts.location = loc_val if loc_val else None
        
        self.opts.no_mavproxy = self.no_mavproxy_chk.GetValue()
        self.opts.map = self.map_chk.GetValue()
        self.opts.console = self.console_chk.GetValue()
        self.opts.OSD = self.osd_chk.GetValue()
        
        wx.GetApp().launch = True
        self.Close()
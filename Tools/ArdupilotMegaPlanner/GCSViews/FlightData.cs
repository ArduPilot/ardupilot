using System;
using System.Collections.Generic; // Lists
using System.Text; // stringbuilder
using System.Drawing; // pens etc
using System.IO; // file io
using System.IO.Ports; // serial
using System.Windows.Forms; // Forms
using System.Collections; // hashs
using System.Text.RegularExpressions; // regex
using System.Xml; // GE xml alt reader
using System.Net; // dns, ip address
using System.Net.Sockets; // tcplistner
using GMap.NET;
using GMap.NET.WindowsForms;
using System.Globalization; // language
using GMap.NET.WindowsForms.Markers;
using ZedGraph; // Graphs
using System.Drawing.Drawing2D;

// written by michael oborne
namespace ArdupilotMega.GCSViews
{
    partial class FlightData : MyUserControl
    {
        ArdupilotMega.MAVLink comPort = MainV2.comPort;
        public static int threadrun = 0;
        StreamWriter swlog;
        int tickStart = 0;
        RollingPointPairList list1 = new RollingPointPairList(1200);
        RollingPointPairList list2 = new RollingPointPairList(1200);
        RollingPointPairList list3 = new RollingPointPairList(1200);
        RollingPointPairList list4 = new RollingPointPairList(1200);
        RollingPointPairList list5 = new RollingPointPairList(1200);
        RollingPointPairList list6 = new RollingPointPairList(1200);
        RollingPointPairList list7 = new RollingPointPairList(1200);
        RollingPointPairList list8 = new RollingPointPairList(1200);
        RollingPointPairList list9 = new RollingPointPairList(1200);
        RollingPointPairList list10 = new RollingPointPairList(1200);

        System.Reflection.PropertyInfo list1item = null;
        System.Reflection.PropertyInfo list2item = null;
        System.Reflection.PropertyInfo list3item = null;
        System.Reflection.PropertyInfo list4item = null;
        System.Reflection.PropertyInfo list5item = null;
        System.Reflection.PropertyInfo list6item = null;
        System.Reflection.PropertyInfo list7item = null;
        System.Reflection.PropertyInfo list8item = null;
        System.Reflection.PropertyInfo list9item = null;
        System.Reflection.PropertyInfo list10item = null;

        CurveItem list1curve;
        CurveItem list2curve;
        CurveItem list3curve;
        CurveItem list4curve;
        CurveItem list5curve;
        CurveItem list6curve;
        CurveItem list7curve;
        CurveItem list8curve;
        CurveItem list9curve;
        CurveItem list10curve;

        internal static GMapOverlay kmlpolygons;
        internal static GMapOverlay geofence;

        bool huddropout = false;
        bool huddropoutresize = false;

        List<PointLatLng> trackPoints = new List<PointLatLng>();

        const float rad2deg = (float)(180 / Math.PI);

        const float deg2rad = (float)(1.0 / rad2deg);

        public static hud.HUD myhud = null;
        public static GMapControl mymap = null;

        AviWriter aviwriter;

        public SplitContainer MainHcopy = null;

        protected override void Dispose(bool disposing)
        {
            threadrun = 0;
            MainV2.comPort.logreadmode = false;
            MainV2.config["FlightSplitter"] = MainH.SplitterDistance.ToString();
            System.Threading.Thread.Sleep(100);
            base.Dispose(disposing);
        }

        public FlightData()
        {
            InitializeComponent();

            mymap = gMapControl1;
            myhud = hud1;
            MainHcopy = MainH;

            // setup default tuning graph
            if (MainV2.config["Tuning_Graph_Selected"] != null)
            {
                string line = MainV2.config["Tuning_Graph_Selected"].ToString();
                string[] lines = line.Split(new char[] { '|' }, StringSplitOptions.RemoveEmptyEntries);
                foreach (string option in lines)
                {
                    chk_box_CheckedChanged((object)(new CheckBox() { Name = option, Checked = true }), new EventArgs());
                }
            }
            else
            {
                chk_box_CheckedChanged((object)(new CheckBox() { Name = "roll", Checked = true }), new EventArgs());
                chk_box_CheckedChanged((object)(new CheckBox() { Name = "pitch", Checked = true }), new EventArgs());
                chk_box_CheckedChanged((object)(new CheckBox() { Name = "nav_roll", Checked = true }), new EventArgs());
                chk_box_CheckedChanged((object)(new CheckBox() { Name = "nav_pitch", Checked = true }), new EventArgs());
            }

            List<string> list = new List<string>();

            //foreach (object obj in Enum.GetValues(typeof(MAVLink.MAV_ACTION)))
#if MAVLINK10
            {
                list.Add("LOITER_UNLIM");
                list.Add("RETURN_TO_LAUNCH");
                list.Add("PREFLIGHT_CALIBRATION");
            }
#else
            {
                list.Add("RETURN");
                list.Add("HALT");
                list.Add("CONTINUE");
                list.Add("SET_MANUAL");
                list.Add("SET_AUTO");
                list.Add("STORAGE_READ");
                list.Add("STORAGE_WRITE");
                list.Add("CALIBRATE_RC");
                list.Add("NAVIGATE");
                list.Add("LOITER");
                list.Add("TAKEOFF");
                list.Add("CALIBRATE_GYRO");
            }
#endif

            CMB_action.DataSource = list;

            CMB_modes.DataSource = Enum.GetNames(typeof(Common.apmmodes));

            CMB_setwp.SelectedIndex = 0;

            zg1.Visible = true;

            CreateChart(zg1);

            // config map             
            gMapControl1.MapType = MapType.GoogleSatellite;
            gMapControl1.MinZoom = 1;
            gMapControl1.CacheLocation = Path.GetDirectoryName(Application.ExecutablePath) + "/gmapcache/";

            gMapControl1.OnMapZoomChanged += new MapZoomChanged(gMapControl1_OnMapZoomChanged);

            gMapControl1.Zoom = 3;

            gMapControl1.RoutesEnabled = true;
            gMapControl1.PolygonsEnabled = true;

            kmlpolygons = new GMapOverlay(gMapControl1, "kmlpolygons");
            gMapControl1.Overlays.Add(kmlpolygons);

            geofence = new GMapOverlay(gMapControl1, "geofence");
            gMapControl1.Overlays.Add(geofence);

            polygons = new GMapOverlay(gMapControl1, "polygons");
            gMapControl1.Overlays.Add(polygons);

            routes = new GMapOverlay(gMapControl1, "routes");
            gMapControl1.Overlays.Add(routes);

            try
            {
                if (MainV2.getConfig("GspeedMAX") != "")
                {
                    Gspeed.MaxValue = float.Parse(MainV2.getConfig("GspeedMAX"));
                }
            }
            catch { }
        }

        void tabControl1_DrawItem(object sender, DrawItemEventArgs e)
        {
            // Draw the background of the ListBox control for each item.
            //e.DrawBackground();
            // Define the default color of the brush as black.
            Brush myBrush = Brushes.Black;

            LinearGradientBrush linear = new LinearGradientBrush(e.Bounds, Color.FromArgb(0x94, 0xc1, 0x1f), Color.FromArgb(0xcd, 0xe2, 0x96), LinearGradientMode.Vertical);

            e.Graphics.FillRectangle(linear, e.Bounds);

            // Draw the current item text based on the current Font 
            // and the custom brush settings.
            e.Graphics.DrawString(((TabControl)sender).TabPages[e.Index].Text.ToString(),
                e.Font, myBrush, e.Bounds, StringFormat.GenericDefault);
            // If the ListBox has focus, draw a focus rectangle around the selected item.
            e.DrawFocusRectangle();

        }

        void gMapControl1_OnMapZoomChanged()
        {
            Zoomlevel.Value = Convert.ToDecimal(gMapControl1.Zoom);
        }

        private void FlightData_Load(object sender, EventArgs e)
        {

            System.Threading.Thread t11 = new System.Threading.Thread(new System.Threading.ThreadStart(mainloop))
            {
                IsBackground = true,
                Name = "FlightData updater"
            };

            t11.Start();
            //MainH.threads.Add(t11);

            Zoomlevel.Minimum = gMapControl1.MinZoom;
            Zoomlevel.Maximum = gMapControl1.MaxZoom + 1;
            Zoomlevel.Value = Convert.ToDecimal(gMapControl1.Zoom);
        }

        private void mainloop()
        {
            //System.Threading.Thread.CurrentThread.CurrentUICulture = new System.Globalization.CultureInfo("en-US");
            //System.Threading.Thread.CurrentThread.CurrentCulture = new System.Globalization.CultureInfo("en-US");
            threadrun = 1;
            EndPoint Remote = (EndPoint)(new IPEndPoint(IPAddress.Any, 0));

            DateTime lastdata = DateTime.MinValue;

            DateTime tracklast = DateTime.Now.AddSeconds(0);

            DateTime tunning = DateTime.Now.AddSeconds(0);

            DateTime vidrec = DateTime.Now.AddSeconds(0);

            DateTime waypoints = DateTime.Now.AddSeconds(0);

            //comPort.stopall(true);

            while (threadrun == 1)
            {
                if (threadrun == 0) { return; }

                if (MainV2.givecomport == true)
                {
                    System.Threading.Thread.Sleep(20);
                    continue;
                }
                if (!comPort.BaseStream.IsOpen)
                    lastdata = DateTime.MinValue;
                // re-request servo data
                if (!(lastdata.AddSeconds(8) > DateTime.Now) && comPort.BaseStream.IsOpen)
                {
                    //Console.WriteLine("REQ streams - flightdata");
                    try
                    {
                        //System.Threading.Thread.Sleep(1000);

                        //comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RAW_CONTROLLER, 0); // request servoout
                        comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_EXTENDED_STATUS, MainV2.cs.ratestatus); // mode
                        comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_POSITION, MainV2.cs.rateposition); // request gps
                        comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA1, MainV2.cs.rateattitude); // request attitude
                        comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA2, MainV2.cs.rateattitude); // request vfr
                        comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RAW_SENSORS, MainV2.cs.ratesensors); // request raw sensor
                        comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RC_CHANNELS, MainV2.cs.raterc); // request rc info
                    }
                    catch { }
                    lastdata = DateTime.Now; // prevent flooding
                }

                if (!MainV2.comPort.logreadmode)
                    System.Threading.Thread.Sleep(100); // max is only ever 10 hz

                try
                {
                    if (aviwriter != null && vidrec.AddMilliseconds(100) <= DateTime.Now)
                    {
                        vidrec = DateTime.Now;

                        hud1.streamjpgenable = true;

                        //aviwriter.avi_start("test.avi");
                        // add a frame
                        aviwriter.avi_add(hud1.streamjpg.ToArray(), (uint)hud1.streamjpg.Length);
                        // write header - so even partial files will play
                        aviwriter.avi_end(hud1.Width, hud1.Height, 10);
                    }
                }
                catch { }

                if (MainV2.comPort.logreadmode && MainV2.comPort.logplaybackfile != null)
                {
                    if (threadrun == 0) { return; }

                        this.BeginInvoke((System.Windows.Forms.MethodInvoker)delegate()
{
    try
    {
        BUT_playlog.Text = "Pause";
    }
    catch { }
});

                    if (comPort.BaseStream.IsOpen)
                        MainV2.comPort.logreadmode = false;

                    DateTime logplayback = MainV2.comPort.lastlogread;
                    try
                    {
                        MainV2.comPort.readPacket();
                    }
                    catch { }

                    updateLogPlayPosition();

                    int act = (int)(MainV2.comPort.lastlogread - logplayback).TotalMilliseconds;

                    if (act > 9999 || act < 0)
                        act = 0;

                    int ts = 0;
                    try
                    {
                        ts = (int)(act / (double)NUM_playbackspeed.Value);
                    }
                    catch { }
                    if (ts > 0)
                        System.Threading.Thread.Sleep(ts);

                    if (threadrun == 0) { return; }

                    tracklast = tracklast.AddMilliseconds(ts - act);
                    tunning = tunning.AddMilliseconds(ts - act);

                    if (tracklast.Month != DateTime.Now.Month)
                    {
                        tracklast = DateTime.Now;
                        tunning = DateTime.Now;
                    }

                    if (MainV2.comPort.logplaybackfile.BaseStream.Position == MainV2.comPort.logplaybackfile.BaseStream.Length)
                    {
                        MainV2.comPort.logreadmode = false;
                    }
                }
                else
                {
                    if (threadrun == 0) { return; }
                    try
                    {
                        this.Invoke((System.Windows.Forms.MethodInvoker)delegate()
    {
        BUT_playlog.Text = "Play";
    });
                    }
                    catch { }
                }

                try
                {
                    //Console.WriteLine(DateTime.Now.Millisecond);
                    MainV2.cs.UpdateCurrentSettings(bindingSource1);
                    //Console.WriteLine(DateTime.Now.Millisecond + " done ");

                    if (tunning.AddMilliseconds(50) < DateTime.Now && CB_tuning.Checked == true)
                    {

                        double time = (Environment.TickCount - tickStart) / 1000.0;
                        if (list1item != null)
                            list1.Add(time, (float)list1item.GetValue((object)MainV2.cs, null));
                        if (list2item != null)
                            list2.Add(time, (float)list2item.GetValue((object)MainV2.cs, null));
                        if (list3item != null)
                            list3.Add(time, (float)list3item.GetValue((object)MainV2.cs, null));
                        if (list4item != null)
                            list4.Add(time, (float)list4item.GetValue((object)MainV2.cs, null));
                        if (list5item != null)
                            list5.Add(time, (float)list5item.GetValue((object)MainV2.cs, null));
                        if (list6item != null)
                            list6.Add(time, (float)list6item.GetValue((object)MainV2.cs, null));
                        if (list7item != null)
                            list7.Add(time, (float)list7item.GetValue((object)MainV2.cs, null));
                        if (list8item != null)
                            list8.Add(time, (float)list8item.GetValue((object)MainV2.cs, null));
                        if (list9item != null)
                            list9.Add(time, (float)list9item.GetValue((object)MainV2.cs, null));
                        if (list10item != null)
                            list10.Add(time, (float)list10item.GetValue((object)MainV2.cs, null));
                    }

                    if (tracklast.AddSeconds(1) < DateTime.Now)
                    {
                        gMapControl1.HoldInvalidation = true;

                        if (trackPoints.Count > int.Parse(MainV2.config["NUM_tracklength"].ToString()))
                        {
                            trackPoints.RemoveRange(0, trackPoints.Count - int.Parse(MainV2.config["NUM_tracklength"].ToString()));
                        }
                        if (MainV2.cs.lat != 0)
                            trackPoints.Add(new PointLatLng(MainV2.cs.lat, MainV2.cs.lng));



                        //                        if (CB_tuning.Checked == false) // draw if in view
                        {

                            if (MainV2.comPort.logreadmode && MainV2.comPort.logplaybackfile != null)
                            {
                                // this is for the pulled wp list from a mavlink logfile
                                FlightPlanner.pointlist.Clear();
                                FlightPlanner.pointlist.AddRange(MainV2.comPort.wps);
                            }

                            

                            routes.Markers.Clear();
                            routes.Routes.Clear();

                            route = new GMapRoute(trackPoints, "track");
                            //track.Stroke = Pens.Red;
                            //route.Stroke = new Pen(Color.FromArgb(144, Color.Red));
                            //route.Stroke.Width = 5;
                            //route.Tag = "track";

                            routes.Routes.Add(route);

                            if (waypoints.AddSeconds(10) < DateTime.Now)
                            {
                                //Console.WriteLine("Doing FD WP's");
                                polygons.Markers.Clear();

                                foreach (PointLatLngAlt plla in FlightPlanner.pointlist)
                                {
                                    if (plla == null || plla.Lng == 0 || plla.Lat == 0)
                                        break;
                                    addpolygonmarker(plla.Tag, plla.Lng, plla.Lat, (int)plla.Alt,plla.color);
                                }

                                RegeneratePolygon();

                                waypoints = DateTime.Now;
                            }

                            //routes.Polygons.Add(poly);    

                            if (trackPoints.Count > 0)
                            {
                                PointLatLng currentloc = new PointLatLng(MainV2.cs.lat, MainV2.cs.lng);


                                if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
                                {
                                    routes.Markers.Add(new GMapMarkerPlane(currentloc, MainV2.cs.yaw, MainV2.cs.groundcourse, MainV2.cs.nav_bearing, MainV2.cs.target_bearing));
                                }
                                else
                                {
                                    routes.Markers.Add(new GMapMarkerQuad(currentloc, MainV2.cs.yaw, MainV2.cs.groundcourse, MainV2.cs.nav_bearing));
                                }

                                if (trackPoints[trackPoints.Count - 1].Lat != 0 && (DateTime.Now.Second % 4 == 0) && CHK_autopan.Checked)
                                {
                                    updateMapPosition(currentloc);
                                }

                                if (trackPoints.Count == 1 && gMapControl1.Zoom == 3) // 3 is the default load zoom
                                {
                                    updateMapPosition(currentloc);
                                    updateMapZoom(17);
                                    //gMapControl1.ZoomAndCenterMarkers("routes");// ZoomAndCenterRoutes("routes");
                                }
                            }

                            gMapControl1.HoldInvalidation = false;

                            gMapControl1.Invalidate();

                            Application.DoEvents();

                            GC.Collect();
                        }

                        tracklast = DateTime.Now;
                    }
                }
                catch (Exception ex) { Console.WriteLine("FD Main loop exception " + ex.ToString()); }
            }
            Console.WriteLine("FD Main loop exit");
        }

        private void updateMapPosition(PointLatLng currentloc)
        {
            this.BeginInvoke((MethodInvoker)delegate()
            {
                try
                {
                    gMapControl1.Position = currentloc;
                    hud1.Refresh();
                }
                catch { }
            });
        }

        private void updateMapZoom(int zoom)
        {
            this.BeginInvoke((MethodInvoker)delegate()
            {
                try
                {
                    gMapControl1.Zoom = zoom;
                }
                catch { }
            });
        }

        private void updateLogPlayPosition()
        {
            this.BeginInvoke((MethodInvoker)delegate()
            {
                try
                {
                    tracklog.Value = (int)(MainV2.comPort.logplaybackfile.BaseStream.Position / (double)MainV2.comPort.logplaybackfile.BaseStream.Length * 100);

                    lbl_logpercent.Text = (MainV2.comPort.logplaybackfile.BaseStream.Position / (double)MainV2.comPort.logplaybackfile.BaseStream.Length).ToString("0.00%");
                }
                catch { }
            });
        }

        private void addpolygonmarker(string tag, double lng, double lat, int alt, Color? color)
        {
            try
            {
                PointLatLng point = new PointLatLng(lat, lng);
                GMapMarkerGoogleGreen m = new GMapMarkerGoogleGreen(point);
                m.ToolTipMode = MarkerTooltipMode.Always;
                m.ToolTipText = tag;
                m.Tag = tag;

                GMapMarkerRect mBorders = new GMapMarkerRect(point);
                {
                    
                        mBorders.InnerMarker = m;
                        try
                        {
                            mBorders.wprad = (int)float.Parse(ArdupilotMega.MainV2.config["TXT_WPRad"].ToString());
                        }
                        catch { }
                        mBorders.MainMap = gMapControl1;
                        if (color.HasValue)
                        {
                            mBorders.Color = color.Value;
                        }
                }

                polygons.Markers.Add(m);
                polygons.Markers.Add(mBorders);
            }
            catch (Exception) { }
        }

        /// <summary>
        /// used to redraw the polygon
        /// </summary>
        void RegeneratePolygon()
        {
            List<PointLatLng> polygonPoints = new List<PointLatLng>();

            if (routes == null)
                return;

            foreach (GMapMarker m in polygons.Markers)
            {
                if (m is GMapMarkerRect)
                {
                    m.Tag = polygonPoints.Count;
                    polygonPoints.Add(m.Position);
                }
            }

            if (polygon == null)
            {
                polygon = new GMapPolygon(polygonPoints, "polygon test");
                polygons.Polygons.Add(polygon);
            }
            else
            {
                polygon.Points.Clear();
                polygon.Points.AddRange(polygonPoints);

                polygon.Stroke = new Pen(Color.Yellow, 4);

                if (polygons.Polygons.Count == 0)
                {
                    polygons.Polygons.Add(polygon);
                }
                else
                {
                    gMapControl1.UpdatePolygonLocalPosition(polygon);
                }
            }
        }

        GMapPolygon polygon;
        GMapOverlay polygons;
        GMapOverlay routes;
        GMapRoute route;

        public void CreateChart(ZedGraphControl zgc)
        {
            GraphPane myPane = zgc.GraphPane;

            // Set the titles and axis labels
            myPane.Title.Text = "Tuning";
            myPane.XAxis.Title.Text = "Time (s)";
            myPane.YAxis.Title.Text = "Unit";

            // Show the x axis grid
            myPane.XAxis.MajorGrid.IsVisible = true;

            myPane.XAxis.Scale.Min = 0;
            myPane.XAxis.Scale.Max = 5;

            // Make the Y axis scale red
            myPane.YAxis.Scale.FontSpec.FontColor = Color.White;
            myPane.YAxis.Title.FontSpec.FontColor = Color.White;
            // turn off the opposite tics so the Y tics don't show up on the Y2 axis
            myPane.YAxis.MajorTic.IsOpposite = false;
            myPane.YAxis.MinorTic.IsOpposite = false;
            // Don't display the Y zero line
            myPane.YAxis.MajorGrid.IsZeroLine = true;
            // Align the Y axis labels so they are flush to the axis
            myPane.YAxis.Scale.Align = AlignP.Inside;
            // Manually set the axis range
            //myPane.YAxis.Scale.Min = -1;
            //myPane.YAxis.Scale.Max = 1;

            // Fill the axis background with a gradient
            //myPane.Chart.Fill = new Fill(Color.White, Color.LightGray, 45.0f);

            // Sample at 50ms intervals
            ZedGraphTimer.Interval = 100;
            //timer1.Enabled = true;
            //timer1.Start();


            // Calculate the Axis Scale Ranges
            zgc.AxisChange();

            tickStart = Environment.TickCount;


        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            try
            {
                // Make sure that the curvelist has at least one curve
                if (zg1.GraphPane.CurveList.Count <= 0)
                    return;

                // Get the first CurveItem in the graph
                LineItem curve = zg1.GraphPane.CurveList[0] as LineItem;
                if (curve == null)
                    return;

                // Get the PointPairList
                IPointListEdit list = curve.Points as IPointListEdit;
                // If this is null, it means the reference at curve.Points does not
                // support IPointListEdit, so we won't be able to modify it
                if (list == null)
                    return;

                // Time is measured in seconds
                double time = (Environment.TickCount - tickStart) / 1000.0;

                // Keep the X scale at a rolling 30 second interval, with one
                // major step between the max X value and the end of the axis
                Scale xScale = zg1.GraphPane.XAxis.Scale;
                if (time > xScale.Max - xScale.MajorStep)
                {
                    xScale.Max = time + xScale.MajorStep;
                    xScale.Min = xScale.Max - 10.0;
                }

                // Make sure the Y axis is rescaled to accommodate actual data
                zg1.AxisChange();

                // Force a redraw

                zg1.Invalidate();
            }
            catch { }

        }

        private void FlightData_FormClosing(object sender, FormClosingEventArgs e)
        {
            ZedGraphTimer.Stop();
            threadrun = 0;
            if (comPort.BaseStream.IsOpen)
            {
                comPort.Close();
            }
        }

        private void BUT_clear_track_Click(object sender, EventArgs e)
        {
            trackPoints.Clear();
        }

        private void BUT_save_log_Click(object sender, EventArgs e)
        {
            // close existing log first
            if (swlog != null)
                swlog.Close();

            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs");
                swlog = new StreamWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs" + Path.DirectorySeparatorChar + DateTime.Now.ToString("yyyy-MM-dd hh-mm") + " telem.log");
            }
            catch { MessageBox.Show("Log creation error"); }
        }

        private void BUTactiondo_Click(object sender, EventArgs e)
        {
            try
            {
                ((Button)sender).Enabled = false;
#if MAVLINK10
                comPort.doCommand((MAVLink.MAV_CMD)Enum.Parse(typeof(MAVLink.MAV_CMD), CMB_action.Text),0,0,0,0,0,0,0);
#else
                comPort.doAction((MAVLink.MAV_ACTION)Enum.Parse(typeof(MAVLink.MAV_ACTION), "MAV_ACTION_" + CMB_action.Text));
#endif
            }
            catch { MessageBox.Show("The Command failed to execute"); }
            ((Button)sender).Enabled = true;
        }

        private void BUTrestartmission_Click(object sender, EventArgs e)
        {
            try
            {
                ((Button)sender).Enabled = false;

                //comPort.doAction(MAVLink.MAV_ACTION.MAV_ACTION_RETURN); // set nav from
                //System.Threading.Thread.Sleep(100);
                comPort.setWPCurrent(1); // set nav to
                //System.Threading.Thread.Sleep(100);
                //comPort.doAction(MAVLink.MAV_ACTION.MAV_ACTION_SET_AUTO); // set auto
            }
            catch { MessageBox.Show("The command failed to execute"); }
            ((Button)sender).Enabled = true;
        }

        private void FlightData_Resize(object sender, EventArgs e)
        {
            //Gspeed;
            //Galt;
            //Gheading;
            //attitudeIndicatorInstrumentControl1;
        }

        private void CB_tuning_CheckedChanged(object sender, EventArgs e)
        {
            if (CB_tuning.Checked)
            {
                splitContainer1.Panel1Collapsed = false;
                ZedGraphTimer.Enabled = true;
                ZedGraphTimer.Start();
                zg1.Visible = true;
                zg1.Refresh();
            }
            else
            {
                splitContainer1.Panel1Collapsed = true;
                ZedGraphTimer.Enabled = false;
                ZedGraphTimer.Stop();
                zg1.Visible = false;
            }
        }

        private void SubMainHT_Panel1_Resize(object sender, EventArgs e)
        {
            hud1.Width = MainH.SplitterDistance;

            SubMainLeft.SplitterDistance = hud1.Height + 2;
        }

        private void BUT_RAWSensor_Click(object sender, EventArgs e)
        {
            Form temp = new RAW_Sensor();
            MainV2.fixtheme(temp);
            temp.Show();
        }

        private void gMapControl1_Click(object sender, EventArgs e)
        {

        }

        PointLatLng gotolocation = new PointLatLng();

        private void gMapControl1_MouseDown(object sender, MouseEventArgs e)
        {
            gotolocation = gMapControl1.FromLocalToLatLng(e.X, e.Y);
        }

        private void goHereToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (!MainV2.comPort.BaseStream.IsOpen)
            {
                MessageBox.Show("Please Connect First");
                return;
            }

            string alt = "100";

            if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2)
            {
                alt = (10 * MainV2.cs.multiplierdist).ToString("0");
            }
            else
            {
                alt = (100 * MainV2.cs.multiplierdist).ToString("0");
            }
            if (DialogResult.Cancel == Common.InputBox("Enter Alt", "Enter Guided Mode Alt", ref alt))
                return;

            int intalt = (int)(100 * MainV2.cs.multiplierdist);
            if (!int.TryParse(alt, out intalt))
            {
                MessageBox.Show("Bad Alt");
                return;
            }

            if (gotolocation.Lat == 0 || gotolocation.Lng == 0)
            {
                MessageBox.Show("Bad Lat/Long");
                return;
            }

            Locationwp gotohere = new Locationwp();

            gotohere.id = (byte)MAVLink.MAV_CMD.WAYPOINT;
            gotohere.alt = (float)(intalt / MainV2.cs.multiplierdist); // back to m
            gotohere.lat = (float)(gotolocation.Lat);
            gotohere.lng = (float)(gotolocation.Lng);

            try
            {
                MainV2.givecomport = true;

                MainV2.comPort.setWP(gotohere, 0, MAVLink.MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT, (byte)2);

                MainV2.givecomport = false;
            }
            catch (Exception ex) { MainV2.givecomport = false; MessageBox.Show("Error sending command : " + ex.Message); }

        }

        private void Zoomlevel_ValueChanged(object sender, EventArgs e)
        {
            try
            {
                if (gMapControl1.MaxZoom + 1 == (double)Zoomlevel.Value)
                {
                    gMapControl1.Zoom = (double)Zoomlevel.Value - .1;
                }
                else
                {
                    gMapControl1.Zoom = (double)Zoomlevel.Value;
                }
            }
            catch { }
        }

        private void gMapControl1_MouseMove(object sender, MouseEventArgs e)
        {
            PointLatLng point = gMapControl1.FromLocalToLatLng(e.X, e.Y);

            if (e.Button == MouseButtons.Left)
            {
                double latdif = gotolocation.Lat - point.Lat;
                double lngdif = gotolocation.Lng - point.Lng;

                try
                {
                    gMapControl1.Position = new PointLatLng(gMapControl1.Position.Lat + latdif, gMapControl1.Position.Lng + lngdif);
                }
                catch { }
            }
        }

        private void FlightData_ParentChanged(object sender, EventArgs e)
        {
            if (MainV2.cam != null)
            {
                MainV2.cam.camimage += new WebCamService.CamImage(cam_camimage);
            }
        }

        void cam_camimage(Image camimage)
        {
            hud1.bgimage = camimage;
        }

        private void BUT_Homealt_Click(object sender, EventArgs e)
        {
            if (MainV2.cs.altoffsethome != 0)
            {
                MainV2.cs.altoffsethome = 0;
            }
            else
            {
                MainV2.cs.altoffsethome = MainV2.cs.alt / MainV2.cs.multiplierdist;
            }
        }

        private void gMapControl1_Resize(object sender, EventArgs e)
        {
            gMapControl1.Zoom = gMapControl1.Zoom + 0.01;
        }

        private void BUT_loadtelem_Click(object sender, EventArgs e)
        {
            OpenFileDialog fd = new OpenFileDialog();
            fd.AddExtension = true;
            fd.Filter = "Ardupilot Telemtry log (*.tlog)|*.tlog|Mavlink Log (*.mavlog)|*.mavlog";
            fd.InitialDirectory = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs";
            fd.DefaultExt = ".tlog";
            DialogResult result = fd.ShowDialog();
            string file = fd.FileName;
            if (file != "")
            {
                try
                {
                    BUT_clear_track_Click(sender, e);

                    MainV2.comPort.logreadmode = false;
                    MainV2.comPort.logplaybackfile = new BinaryReader(File.OpenRead(file));
                    MainV2.comPort.lastlogread = DateTime.MinValue;

                    tracklog.Value = 0;
                    tracklog.Minimum = 0;
                    tracklog.Maximum = 100;
                }
                catch { MessageBox.Show("Error: Failed to write log file"); }
            }
        }

        private void BUT_playlog_Click(object sender, EventArgs e)
        {
            if (MainV2.comPort.logreadmode)
            {
                MainV2.comPort.logreadmode = false;
            }
            else
            {
                BUT_clear_track_Click(sender, e);
                MainV2.comPort.logreadmode = true;
            }
        }

        private void tracklog_Scroll(object sender, EventArgs e)
        {
            BUT_clear_track_Click(sender, e);

            MainV2.comPort.lastlogread = DateTime.MinValue;

            if (MainV2.comPort.logplaybackfile != null)
                MainV2.comPort.logplaybackfile.BaseStream.Position = (long)(MainV2.comPort.logplaybackfile.BaseStream.Length * (tracklog.Value / 100.0));

            updateLogPlayPosition();
        }

        bool loaded = false;

        private void MainH_SplitterMoved(object sender, SplitterEventArgs e)
        {
            if (loaded == true)
            { // startup check
                MainV2.config["FlightSplitter"] = MainH.SplitterDistance.ToString();
            }
            loaded = true;
            hud1.Width = MainH.Panel1.Width;
        }

        private void tabPage1_Resize(object sender, EventArgs e)
        {
            int mywidth;

            if (tabGauges.Width < 500)
            {
                Gvspeed.Visible = false;
                mywidth = tabGauges.Width / 3;

                Gspeed.Height = mywidth;
                Galt.Height = mywidth;
                Gheading.Height = mywidth;

                Gspeed.Location = new Point(0, 0);
            }
            else
            {
                Gvspeed.Visible = true;
                mywidth = tabGauges.Width / 4;

                Gvspeed.Height = mywidth;
                Gspeed.Height = mywidth;
                Galt.Height = mywidth;
                Gheading.Height = mywidth;

                Gvspeed.Location = new Point(0, 0);
                Gspeed.Location = new Point(Gvspeed.Right, 0);
            }

            Galt.Location = new Point(Gspeed.Right, 0);
            Gheading.Location = new Point(Galt.Right, 0);

        }

        private void BUT_setmode_Click(object sender, EventArgs e)
        {
            MainV2.comPort.setMode(CMB_modes.Text);
        }

        private void BUT_setwp_Click(object sender, EventArgs e)
        {
            try
            {
                ((Button)sender).Enabled = false;
                comPort.setWPCurrent((ushort)CMB_setwp.SelectedIndex); // set nav to
            }
            catch { MessageBox.Show("The command failed to execute"); }
            ((Button)sender).Enabled = true;
        }

        private void CMB_setwp_Click(object sender, EventArgs e)
        {
            CMB_setwp.Items.Clear();

            CMB_setwp.Items.Add("0 (Home)");

            if (MainV2.comPort.param["CMD_TOTAL"] != null)
            {
                int wps = int.Parse(MainV2.comPort.param["CMD_TOTAL"].ToString());
                for (int z = 1; z <= wps; z++)
                {
                    CMB_setwp.Items.Add(z.ToString());
                }
            }
        }

        private void BUT_quickauto_Click(object sender, EventArgs e)
        {
            try
            {
                ((Button)sender).Enabled = false;
#if MAVLINK10
				MainV2.comPort.setMode("AUTO");
#else
                comPort.doAction(MAVLink.MAV_ACTION.MAV_ACTION_SET_AUTO);
#endif
            }
            catch { MessageBox.Show("The Command failed to execute"); }
            ((Button)sender).Enabled = true;
        }

        private void BUT_quickrtl_Click(object sender, EventArgs e)
        {
            try
            {
                ((Button)sender).Enabled = false;
#if MAVLINK10
				MainV2.comPort.setMode("RTL");
#else
                comPort.doAction(MAVLink.MAV_ACTION.MAV_ACTION_RETURN);
#endif
            }
            catch { MessageBox.Show("The Command failed to execute"); }
            ((Button)sender).Enabled = true;
        }

        private void BUT_quickmanual_Click(object sender, EventArgs e)
        {
            try
            {
                ((Button)sender).Enabled = false;
#if MAVLINK10
				MainV2.comPort.setMode("MANUAL");
#else
                comPort.doAction(MAVLink.MAV_ACTION.MAV_ACTION_SET_MANUAL);
#endif
            }
            catch { MessageBox.Show("The Command failed to execute"); }
            ((Button)sender).Enabled = true;
        }

        private void BUT_log2kml_Click(object sender, EventArgs e)
        {
            if (DialogResult.Cancel == Common.MessageShowAgain("Tlog to KML Firmware Version", "When converting logs, ensure your Firmware version is set correctly.\n(Near your com port selection.)"))
            {
                return;
            }

            Form frm = new MavlinkLog();
            MainV2.fixtheme(frm);
            frm.ShowDialog();
        }

        private void BUT_joystick_Click(object sender, EventArgs e)
        {
            Form joy = new JoystickSetup();
            MainV2.fixtheme(joy);
            joy.Show();
        }

        private void CMB_modes_Click(object sender, EventArgs e)
        {
            CMB_modes.DataSource = Enum.GetNames(Common.getModes());
        }

        private void hud1_DoubleClick(object sender, EventArgs e)
        {
            if (huddropout)
                return;

            SubMainLeft.Panel1Collapsed = true;
            Form dropout = new Form();
            dropout.Size = new System.Drawing.Size(hud1.Width, hud1.Height + 20);
            SubMainLeft.Panel1.Controls.Remove(hud1);
            dropout.Controls.Add(hud1);
            dropout.Resize += new EventHandler(dropout_Resize);
            dropout.FormClosed += new FormClosedEventHandler(dropout_FormClosed);
            dropout.Show();
            huddropout = true;
        }

        void dropout_FormClosed(object sender, FormClosedEventArgs e)
        {
            SubMainLeft.Panel1.Controls.Add(hud1);
            SubMainLeft.Panel1Collapsed = false;
            huddropout = false;
        }

        void dropout_Resize(object sender, EventArgs e)
        {
            if (huddropoutresize)
                return;

            huddropoutresize = true;

            int hudw = hud1.Width;
            int hudh = hud1.Height;

            int formh = ((Form)sender).Height - 30;
            int formw = ((Form)sender).Width;

            if (((Form)sender).Height < hudh)
            {
                if (((Form)sender).WindowState == FormWindowState.Maximized)
                {
                    Point tl = ((Form)sender).DesktopLocation;
                    ((Form)sender).WindowState = FormWindowState.Normal;
                    ((Form)sender).Location = tl;
                }
                ((Form)sender).Width = (int)(formh * 1.333f);
                ((Form)sender).Height = (int)(formh) + 20;
            }
            hud1.Refresh();
            huddropoutresize = false;
        }

        private void tabControl1_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (tabControl1.SelectedTab == tabStatus)
            {
                tabControl1.SuspendLayout();

                foreach (Control temp in tabStatus.Controls)
                {
                    temp.DataBindings.Clear();
                    //temp.Dispose();
                }
                //tabStatus.Controls.Clear();

                int x = 10;
                int y = 10;

                object thisBoxed = MainV2.cs;
                Type test = thisBoxed.GetType();

                foreach (var field in test.GetProperties())
                {
                    // field.Name has the field's name.
                    object fieldValue;
                    try
                    {
                        fieldValue = field.GetValue(thisBoxed, null); // Get value
                    }
                    catch { continue; }

                    // Get the TypeCode enumeration. Multiple types get mapped to a common typecode.
                    TypeCode typeCode = Type.GetTypeCode(fieldValue.GetType());

                    bool add = true;

                    MyLabel lbl1 = new MyLabel();
                    MyLabel lbl2 = new MyLabel();
                    try
                    {
                        lbl1 = (MyLabel)tabStatus.Controls.Find(field.Name, false)[0];

                        lbl2 = (MyLabel)tabStatus.Controls.Find(field.Name + "value", false)[0];

                        add = false;
                    }
                    catch { }

                    lbl1.Location = new Point(x, y);
                    lbl1.Size = new System.Drawing.Size(75, 13);
                    lbl1.Text = field.Name;
                    lbl1.Name = field.Name;
                    lbl2.AutoSize = false;

                    lbl2.Location = new Point(lbl1.Right + 5, y);
                    lbl2.Size = new System.Drawing.Size(50, 13);
                    //if (lbl2.Name == "")
                    lbl2.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.bindingSource1, field.Name, true, System.Windows.Forms.DataSourceUpdateMode.OnValidation, ""));
                    lbl2.Name = field.Name + "value";
                    //lbl2.Text = fieldValue.ToString();

                    if (add)
                    {
                        tabStatus.Controls.Add(lbl1);
                        tabStatus.Controls.Add(lbl2);
                    }

                    //Application.DoEvents();

                    x += 0;
                    y += 15;

                    if (y > tabStatus.Height - 30)
                    {
                        x += 140;
                        y = 10;
                    }
                }

                tabStatus.Width = x;

                tabControl1.ResumeLayout();
            }
            else
            {
                foreach (Control temp in tabStatus.Controls)
                {
                    temp.DataBindings.Clear();
                }
            }
        }

        private void Gspeed_DoubleClick(object sender, EventArgs e)
        {
            string max = "60";
            if (DialogResult.OK == Common.InputBox("Enter Max", "Enter Max Speed", ref max))
            {
                Gspeed.MaxValue = float.Parse(max);
                MainV2.config["GspeedMAX"] = Gspeed.MaxValue.ToString();
            }
        }

        private void recordHudToAVIToolStripMenuItem_Click(object sender, EventArgs e)
        {
            stopRecordToolStripMenuItem_Click(sender, e);

            MessageBox.Show("Output avi will be saved to the log folder");

            aviwriter = new AviWriter();
            Directory.CreateDirectory(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs");
            aviwriter.avi_start(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs" + Path.DirectorySeparatorChar + DateTime.Now.ToString("yyyy-MM-dd hh-mm-ss") + ".avi");

            recordHudToAVIToolStripMenuItem.Text = "Recording";
        }

        private void stopRecordToolStripMenuItem_Click(object sender, EventArgs e)
        {
            recordHudToAVIToolStripMenuItem.Text = "Start Recording";

            if (aviwriter != null)
                aviwriter.avi_close();

            aviwriter = null;
        }

        void setupPropertyInfo(ref System.Reflection.PropertyInfo input, string name, object source)
        {
            Type test = source.GetType();

            foreach (var field in test.GetProperties())
            {
                if (field.Name == name)
                {
                    input = field;
                    return;
                }
            }


        }

        private void zg1_DoubleClick(object sender, EventArgs e)
        {
            Form selectform = new Form()
            {
                Name = "select",
                Width = 50,
                Height = 250,
                Text = "Graph This"
            };

            int x = 10;
            int y = 10;

            {
                CheckBox chk_box = new CheckBox();
                chk_box.Text = "Logarithmic";
                chk_box.Name = "Logarithmic";
                chk_box.Location = new Point(x, y);
                chk_box.Size = new System.Drawing.Size(100, 20);
                chk_box.CheckedChanged += new EventHandler(chk_log_CheckedChanged);

                selectform.Controls.Add(chk_box);
            }

            y += 20;

            object thisBoxed = MainV2.cs;
            Type test = thisBoxed.GetType();

            foreach (var field in test.GetProperties())
            {
                // field.Name has the field's name.
                object fieldValue;
                try
                {
                    fieldValue = field.GetValue(thisBoxed, null); // Get value
                }
                catch { continue; }

                // Get the TypeCode enumeration. Multiple types get mapped to a common typecode.
                TypeCode typeCode = Type.GetTypeCode(fieldValue.GetType());

                if (!(typeCode == TypeCode.Single))
                    continue;

                CheckBox chk_box = new CheckBox();

                if (list1item != null && list1item.Name == field.Name)
                    chk_box.Checked = true;
                if (list2item != null && list2item.Name == field.Name)
                    chk_box.Checked = true;
                if (list3item != null && list3item.Name == field.Name)
                    chk_box.Checked = true;
                if (list4item != null && list4item.Name == field.Name)
                    chk_box.Checked = true;
                if (list5item != null && list5item.Name == field.Name)
                    chk_box.Checked = true;
                if (list6item != null && list6item.Name == field.Name)
                    chk_box.Checked = true;
                if (list7item != null && list7item.Name == field.Name)
                    chk_box.Checked = true;
                if (list8item != null && list8item.Name == field.Name)
                    chk_box.Checked = true;
                if (list9item != null && list9item.Name == field.Name)
                    chk_box.Checked = true;
                if (list10item != null && list10item.Name == field.Name)
                    chk_box.Checked = true;

                chk_box.Text = field.Name;
                chk_box.Name = field.Name;
                chk_box.Location = new Point(x, y);
                chk_box.Size = new System.Drawing.Size(100, 20);
                chk_box.CheckedChanged += new EventHandler(chk_box_CheckedChanged);

                selectform.Controls.Add(chk_box);

                Application.DoEvents();

                x += 0;
                y += 20;

                if (y > selectform.Height - 50)
                {
                    x += 100;
                    y = 10;

                    selectform.Width = x + 100;
                }
            }
            MainV2.fixtheme(selectform);
            selectform.Show();
        }

        void chk_log_CheckedChanged(object sender, EventArgs e)
        {
            if (((CheckBox)sender).Checked)
            {
                zg1.GraphPane.YAxis.Type = AxisType.Log;
            }
            else
            {
                zg1.GraphPane.YAxis.Type = AxisType.Linear;
            }
        }

        void chk_box_CheckedChanged(object sender, EventArgs e)
        {
            if (((CheckBox)sender).Checked)
            {
                if (list1item == null)
                {
                    setupPropertyInfo(ref list1item, ((CheckBox)sender).Name, MainV2.cs);
                    list1curve = zg1.GraphPane.AddCurve(((CheckBox)sender).Name, list1, Color.Red, SymbolType.None);
                }
                else if (list2item == null)
                {
                    setupPropertyInfo(ref list2item, ((CheckBox)sender).Name, MainV2.cs);
                    list2curve = zg1.GraphPane.AddCurve(((CheckBox)sender).Name, list2, Color.Blue, SymbolType.None);
                }
                else if (list3item == null)
                {
                    setupPropertyInfo(ref list3item, ((CheckBox)sender).Name, MainV2.cs);
                    list3curve = zg1.GraphPane.AddCurve(((CheckBox)sender).Name, list3, Color.Green, SymbolType.None);
                }
                else if (list4item == null)
                {
                    setupPropertyInfo(ref list4item, ((CheckBox)sender).Name, MainV2.cs);
                    list4curve = zg1.GraphPane.AddCurve(((CheckBox)sender).Name, list4, Color.Orange, SymbolType.None);
                }
                else if (list5item == null)
                {
                    setupPropertyInfo(ref list5item, ((CheckBox)sender).Name, MainV2.cs);
                    list5curve = zg1.GraphPane.AddCurve(((CheckBox)sender).Name, list5, Color.Yellow, SymbolType.None);
                }
                else if (list6item == null)
                {
                    setupPropertyInfo(ref list6item, ((CheckBox)sender).Name, MainV2.cs);
                    list6curve = zg1.GraphPane.AddCurve(((CheckBox)sender).Name, list6, Color.Magenta, SymbolType.None);
                }
                else if (list7item == null)
                {
                    setupPropertyInfo(ref list7item, ((CheckBox)sender).Name, MainV2.cs);
                    list7curve = zg1.GraphPane.AddCurve(((CheckBox)sender).Name, list7, Color.Purple, SymbolType.None);
                }
                else if (list8item == null)
                {
                    setupPropertyInfo(ref list8item, ((CheckBox)sender).Name, MainV2.cs);
                    list8curve = zg1.GraphPane.AddCurve(((CheckBox)sender).Name, list8, Color.LimeGreen, SymbolType.None);
                }
                else if (list9item == null)
                {
                    setupPropertyInfo(ref list9item, ((CheckBox)sender).Name, MainV2.cs);
                    list9curve = zg1.GraphPane.AddCurve(((CheckBox)sender).Name, list9, Color.Cyan, SymbolType.None);
                }
                else if (list10item == null)
                {
                    setupPropertyInfo(ref list10item, ((CheckBox)sender).Name, MainV2.cs);
                    list10curve = zg1.GraphPane.AddCurve(((CheckBox)sender).Name, list10, Color.Violet, SymbolType.None);
                }
                else
                {
                    MessageBox.Show("Max 10 at a time.");
                    ((CheckBox)sender).Checked = false;
                }
                MainV2.fixtheme(this);

                string selected = "";
                try
                {
                    selected = selected + zg1.GraphPane.CurveList[0].Label.Text + "|";
                    selected = selected + zg1.GraphPane.CurveList[1].Label.Text + "|";
                    selected = selected + zg1.GraphPane.CurveList[2].Label.Text + "|";
                    selected = selected + zg1.GraphPane.CurveList[3].Label.Text + "|";
                    selected = selected + zg1.GraphPane.CurveList[4].Label.Text + "|";
                    selected = selected + zg1.GraphPane.CurveList[5].Label.Text + "|";
                    selected = selected + zg1.GraphPane.CurveList[6].Label.Text + "|";
                    selected = selected + zg1.GraphPane.CurveList[7].Label.Text + "|";
                    selected = selected + zg1.GraphPane.CurveList[8].Label.Text + "|";
                    selected = selected + zg1.GraphPane.CurveList[9].Label.Text + "|";
                    selected = selected + zg1.GraphPane.CurveList[10].Label.Text + "|";
                }
                catch { }
                MainV2.config["Tuning_Graph_Selected"] = selected;
            }
            else
            {
                // reset old stuff
                if (list1item != null && list1item.Name == ((CheckBox)sender).Name)
                {
                    list1item = null;
                    zg1.GraphPane.CurveList.Remove(list1curve);
                }
                if (list2item != null && list2item.Name == ((CheckBox)sender).Name)
                {
                    list2item = null;
                    zg1.GraphPane.CurveList.Remove(list2curve);
                }
                if (list3item != null && list3item.Name == ((CheckBox)sender).Name)
                {
                    list3item = null;
                    zg1.GraphPane.CurveList.Remove(list3curve);
                }
                if (list4item != null && list4item.Name == ((CheckBox)sender).Name)
                {
                    list4item = null;
                    zg1.GraphPane.CurveList.Remove(list4curve);
                }
                if (list5item != null && list5item.Name == ((CheckBox)sender).Name)
                {
                    list5item = null;
                    zg1.GraphPane.CurveList.Remove(list5curve);
                }
                if (list6item != null && list6item.Name == ((CheckBox)sender).Name)
                {
                    list6item = null;
                    zg1.GraphPane.CurveList.Remove(list6curve);
                }
                if (list7item != null && list7item.Name == ((CheckBox)sender).Name)
                {
                    list7item = null;
                    zg1.GraphPane.CurveList.Remove(list7curve);
                }
                if (list8item != null && list8item.Name == ((CheckBox)sender).Name)
                {
                    list8item = null;
                    zg1.GraphPane.CurveList.Remove(list8curve);
                }
                if (list9item != null && list9item.Name == ((CheckBox)sender).Name)
                {
                    list9item = null;
                    zg1.GraphPane.CurveList.Remove(list9curve);
                }
                if (list10item != null && list10item.Name == ((CheckBox)sender).Name)
                {
                    list10item = null;
                    zg1.GraphPane.CurveList.Remove(list10curve);
                }
            }
        }

        private void pointCameraHereToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (!MainV2.comPort.BaseStream.IsOpen)
            {
                MessageBox.Show("Please Connect First");
                return;
            }

            string alt = (100 * MainV2.cs.multiplierdist).ToString("0");
            Common.InputBox("Enter Alt", "Enter Target Alt (absolute)", ref alt);

            int intalt = (int)(100 * MainV2.cs.multiplierdist);
            if (!int.TryParse(alt, out intalt))
            {
                MessageBox.Show("Bad Alt");
                return;
            }

            if (gotolocation.Lat == 0 || gotolocation.Lng == 0)
            {
                MessageBox.Show("Bad Lat/Long");
                return;
            }

            MainV2.comPort.setMountConfigure(MAVLink.MAV_MOUNT_MODE.MAV_MOUNT_MODE_GPS_POINT, true, true, true);
            MainV2.comPort.setMountControl(gotolocation.Lat, gotolocation.Lng, (int)(intalt / MainV2.cs.multiplierdist), true);

        }

        private void BUT_script_Click(object sender, EventArgs e)
        {

            System.Threading.Thread t11 = new System.Threading.Thread(new System.Threading.ThreadStart(ScriptStart))
            {
                IsBackground = true,
                Name = "Script Thread"
            };
            t11.Start();
        }

        void ScriptStart()
        {
            string myscript = @"
# cs.???? = currentstate, any variable on the status tab in the planner can be used.
# Script = options are 
# Script.Sleep(ms)
# Script.ChangeParam(name,value)
# Script.GetParam(name)
# Script.ChangeMode(mode) - same as displayed in mode setup screen 'AUTO'
# Script.WaitFor(string,timeout)
# Script.SendRC(channel,pwm,sendnow)
# 

print 'Start Script'
for chan in range(1,9):
    Script.SendRC(chan,1500,False)
Script.SendRC(3,Script.GetParam('RC3_MIN'),True)

Script.Sleep(5000)
while cs.lat == 0:
    print 'Waiting for GPS'
    Script.Sleep(1000)
print 'Got GPS'
jo = 10 * 13
print jo
Script.SendRC(3,1000,False)
Script.SendRC(4,2000,True)
cs.messages.Clear()
Script.WaitFor('ARMING MOTORS',30000)
Script.SendRC(4,1500,True)
print 'Motors Armed!'

Script.SendRC(3,1700,True)
while cs.alt < 50:
    Script.Sleep(50)

Script.SendRC(5,2000,True) # acro

Script.SendRC(1,2000,False) # roll
Script.SendRC(3,1370,True) # throttle
while cs.roll > -45: # top hald 0 - 180
    Script.Sleep(5)
while cs.roll < -45: # -180 - -45
    Script.Sleep(5)

Script.SendRC(5,1500,False) # stabalise
Script.SendRC(1,1500,True) # level roll
Script.Sleep(2000) # 2 sec to stabalise
Script.SendRC(3,1300,True) # throttle back to land

thro = 1350 # will decend

while cs.alt > 0.1:
    Script.Sleep(300)

Script.SendRC(3,1000,False)
Script.SendRC(4,1000,True)
Script.WaitFor('DISARMING MOTORS',30000)
Script.SendRC(4,1500,True)

print 'Roll complete'

";

            MessageBox.Show("This is Very ALPHA");

            Form scriptedit = new Form();

            scriptedit.Size = new System.Drawing.Size(500,500);

            TextBox tb = new TextBox();

            tb.Dock = DockStyle.Fill;

            tb.ScrollBars = ScrollBars.Both;
            tb.Multiline = true;

            tb.Location = new Point(0,0);
            tb.Size = new System.Drawing.Size(scriptedit.Size.Width-30,scriptedit.Size.Height-30);

            scriptedit.Controls.Add(tb);

            tb.Text = myscript;

            scriptedit.ShowDialog();

            if (DialogResult.Yes == MessageBox.Show("Run Script", "Run this script?", MessageBoxButtons.YesNo))
            {

                Script scr = new Script();

                scr.runScript(tb.Text);
            }
        }
    }
}
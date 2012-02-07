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
using System.Resources;
using System.Reflection;
using System.ComponentModel;
using System.Threading;
using SharpKml.Base;
using SharpKml.Dom;



namespace ArdupilotMega.GCSViews
{
    partial class FlightPlanner : MyUserControl
    {
        int selectedrow = 0;
        bool quickadd = false;
        bool isonline = true;
        bool sethome = false;
        bool polygongridmode = false;
        Hashtable param = new Hashtable();

        public static List<PointLatLngAlt> pointlist = new List<PointLatLngAlt>(); // used to calc distance
        static public Object thisLock = new Object();
        private TextBox textBox1;
        private ComponentResourceManager rm = new ComponentResourceManager(typeof(FlightPlanner));

        private Dictionary<string, string[]> cmdParamNames = new Dictionary<string, string[]>();


        /// <summary>
        /// Reads defines.h for all valid commands and eeprom positions
        /// </summary>
        /// <param name="file">File Path</param>
        /// <returns></returns>
        public bool readdefines(string file)
        {
            if (!File.Exists(file))
            {
                return false;
            }
            try
            {
                StreamReader sr = new StreamReader(file); //"defines.h"
                while (!sr.EndOfStream)
                {
                    string line = sr.ReadLine();
                    Regex regex2 = new Regex(@"define\s+([^\s]+)\s+([^\s]+)", RegexOptions.IgnoreCase);
                    if (regex2.IsMatch(line))
                    {
                        MatchCollection matchs = regex2.Matches(line);
                        for (int i = 0; i < matchs.Count; i++)
                        {
                            int num = 0;
                            if (matchs[i].Groups[2].Value.ToString().ToLower().Contains("0x"))
                            {
                                try
                                {
                                    num = Convert.ToInt32(matchs[i].Groups[2].Value.ToString(), 16);
                                }
                                catch (Exception) { System.Diagnostics.Debug.WriteLine("BAD hex " + matchs[i].Groups[1].Value.ToString()); }
                            }
                            else
                            {
                                try
                                {
                                    num = Convert.ToInt32(matchs[i].Groups[2].Value.ToString(), 10);
                                }
                                catch (Exception) { System.Diagnostics.Debug.WriteLine("BAD dec " + matchs[i].Groups[1].Value.ToString()); }
                            }
                            System.Diagnostics.Debug.WriteLine(matchs[i].Groups[1].Value.ToString() + " = " + matchs[i].Groups[2].Value.ToString() + " = " + num.ToString());
                            try
                            {
                             //   hashdefines.Add(matchs[i].Groups[1].Value.ToString(), num);
                            }
                            catch (Exception) { }
                        }
                    }
                }

                sr.Close();


               // if (!hashdefines.ContainsKey("WP_START_BYTE"))
                {
                    MessageBox.Show("Your Ardupilot Mega project defines.h is Invalid");
                    //return false;
                }
            }
            catch (Exception)
            {
                MessageBox.Show("Can't open file!");
                return false;
            }
            return true;
        }

        /// <summary>
        /// Read from waypoint writter *.h file
        /// </summary>
        /// <param name="file">File Path</param>
        /// <returns></returns>
        bool readwaypointwritterfile(string file)
        {

            byte wp_rad = 30;
            byte loit_rad = 45;
            int alt_hold = 100;
            byte wp_count = 0;
            bool error = false;
            List<Locationwp> cmds = new List<Locationwp>();

            cmds.Add(new Locationwp());

            try
            {
                StreamReader sr = new StreamReader(file); //"defines.h"
                while (!error && !sr.EndOfStream)
                {
                    string line = sr.ReadLine();
                    // defines
                    Regex regex2 = new Regex(@"define\s+([^\s]+)\s+([^\s]+)", RegexOptions.IgnoreCase);
                    if (regex2.IsMatch(line))
                    {
                        MatchCollection matchs = regex2.Matches(line);
                        for (int i = 0; i < matchs.Count; i++)
                        {
                            if (matchs[i].Groups[1].Value.ToString().Equals("WP_RADIUS"))
                                wp_rad = (byte)double.Parse(matchs[i].Groups[2].Value.ToString());
                            if (matchs[i].Groups[1].Value.ToString().Equals("LOITER_RADIUS"))
                                loit_rad = (byte)double.Parse(matchs[i].Groups[2].Value.ToString());
                            if (matchs[i].Groups[1].Value.ToString().Equals("ALT_TO_HOLD"))
                                alt_hold = (int)double.Parse(matchs[i].Groups[2].Value.ToString());
                        }
                    }
                    // waypoints
                    regex2 = new Regex(@"([^,{]+),([^,]+),([^,]+),([^,]+),([^,}]+)", RegexOptions.IgnoreCase);
                    if (regex2.IsMatch(line))
                    {
                        MatchCollection matchs = regex2.Matches(line);
                        for (int i = 0; i < matchs.Count; i++)
                        {
                            Locationwp temp = new Locationwp();
                            temp.options = 1;
                            temp.id = (byte)(int)Enum.Parse(typeof(MAVLink.MAV_CMD), matchs[i].Groups[1].Value.ToString().Replace("NAV_", ""), false);
                            temp.p1 = byte.Parse(matchs[i].Groups[2].Value.ToString());

                            if (temp.id < (byte)MAVLink.MAV_CMD.LAST)
                            {
                                temp.alt = (float)(double.Parse(matchs[i].Groups[3].Value.ToString(), new System.Globalization.CultureInfo("en-US")));
                                temp.lat = (float)(double.Parse(matchs[i].Groups[4].Value.ToString(), new System.Globalization.CultureInfo("en-US")));
                                temp.lng = (float)(double.Parse(matchs[i].Groups[5].Value.ToString(), new System.Globalization.CultureInfo("en-US")));
                            }
                            else
                            {
                                temp.alt = (float)(double.Parse(matchs[i].Groups[3].Value.ToString(), new System.Globalization.CultureInfo("en-US")));
                                temp.lat = (float)(double.Parse(matchs[i].Groups[4].Value.ToString(), new System.Globalization.CultureInfo("en-US")));
                                temp.lng = (float)(double.Parse(matchs[i].Groups[5].Value.ToString(), new System.Globalization.CultureInfo("en-US")));
                            }
                            cmds.Add(temp);

                            wp_count++;
                            if (wp_count == byte.MaxValue)
                                break;
                        }
                        if (wp_count == byte.MaxValue)
                        {
                            MessageBox.Show("To many Waypoints!!!");
                            break;
                        }
                    }
                }

                sr.Close();

                TXT_DefaultAlt.Text = (alt_hold).ToString();
                TXT_WPRad.Text = (wp_rad).ToString();
                TXT_loiterrad.Text = (loit_rad).ToString();

                processToScreen(cmds);

                writeKML();

                MainMap.ZoomAndCenterMarkers("objects");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Can't open file! " + ex.ToString());
                return false;
            }
            return true;

        }

        /// <summary>
        /// used to adjust existing point in the datagrid including "Home"
        /// </summary>
        /// <param name="pointno"></param>
        /// <param name="lat"></param>
        /// <param name="lng"></param>
        /// <param name="alt"></param>
        public void callMeDrag(string pointno, double lat, double lng, int alt)
        {
            if (pointno == "")
            {
                return;
            }

            // dragging a WP
            if (pointno == "Home")
            {
                if (isonline && CHK_geheight.Checked)
                {
                    TXT_homealt.Text = getGEAlt(lat, lng).ToString();
                }
                else
                {
                    // no change
                    //TXT_homealt.Text = alt.ToString();
                }
                TXT_homelat.Text = lat.ToString();
                TXT_homelng.Text = lng.ToString();
                return;
            }

            try
            {
                selectedrow = int.Parse(pointno) - 1;
                Commands.CurrentCell = Commands[1, selectedrow];
            }
            catch
            {
                return;
            }

            setfromGE(lat, lng, alt);
        }
        /// <summary>
        /// Actualy Sets the values into the datagrid and verifys height if turned on
        /// </summary>
        /// <param name="lat"></param>
        /// <param name="lng"></param>
        /// <param name="alt"></param>
        void setfromGE(double lat, double lng, int alt)
        {
            if (selectedrow > Commands.RowCount)
            {
                MessageBox.Show("Invalid coord, How did you do this?");
                return;
            }
            DataGridViewTextBoxCell cell;
            if (Commands.Columns[Lat.Index].HeaderText.Equals(cmdParamNames["WAYPOINT"][4]/*"Lat"*/))
            {
                cell = Commands.Rows[selectedrow].Cells[Lat.Index] as DataGridViewTextBoxCell;
                cell.Value = lat.ToString("0.0000000");
                cell.DataGridView.EndEdit();
            }
            if (Commands.Columns[Lon.Index].HeaderText.Equals(cmdParamNames["WAYPOINT"][5]/*"Long"*/))
            {
                cell = Commands.Rows[selectedrow].Cells[Lon.Index] as DataGridViewTextBoxCell;
                cell.Value = lng.ToString("0.0000000");
                cell.DataGridView.EndEdit();
            }
            if (alt != -1 && Commands.Columns[Alt.Index].HeaderText.Equals(cmdParamNames["WAYPOINT"][6]/*"Alt"*/))
            {
                cell = Commands.Rows[selectedrow].Cells[Alt.Index] as DataGridViewTextBoxCell;

                {
                    float result;
                    bool pass = float.TryParse(TXT_homealt.Text, out result);

                    if (result == 0 || pass == false)
                    {
                        MessageBox.Show("You must have a home altitude");
                        return;
                    }
                    int results1;
                    if (!int.TryParse(TXT_DefaultAlt.Text, out results1))
                    {
                        MessageBox.Show("Your default alt is not valid");
                        return;
                    }
                }

                cell.Value = TXT_DefaultAlt.Text;

                float ans;
                if (float.TryParse(cell.Value.ToString(), out ans))
                {
                    ans = (int)ans;
                    if (alt != 0) // use passed in value;
                        cell.Value = alt.ToString();
                    if (ans == 0)
                        cell.Value = 50;
                    //   online          verify height
                    if (isonline && CHK_geheight.Checked)
                    {
                        if (CHK_altmode.Checked)
                        {
                            cell.Value = ((int)getGEAlt(lat, lng) + int.Parse(TXT_DefaultAlt.Text)).ToString();
                        }
                        else
                        {
                            cell.Value = ((int)getGEAlt(lat, lng) + int.Parse(TXT_DefaultAlt.Text) - float.Parse(TXT_homealt.Text)).ToString();
                        }
                    }
                    else
                    {
                        // is absolute but no verify
                        if (CHK_altmode.Checked)
                        {
                            cell.Value = (float.Parse(TXT_homealt.Text) + int.Parse(TXT_DefaultAlt.Text)).ToString();
                        } // is relative and check height
                        else if (isonline && CHK_geheight.Checked)
                        {
                            alt = (int)getGEAlt(lat, lng);

                            if (float.Parse(TXT_homealt.Text) + int.Parse(TXT_DefaultAlt.Text) < alt) // calced height is less that GE ground height
                            {
                                MessageBox.Show("Altitude appears to be low!! (you will fly into a hill)\nGoogle Ground height: " + alt + " Meters\nYour height: " + ((float.Parse(TXT_homealt.Text) + int.Parse(TXT_DefaultAlt.Text))) + " Meters");
                                cell.Style.BackColor = Color.Red;
                            }
                            else
                            {
                                cell.Style.BackColor = Color.LightGreen;
                            }
                        }

                    }
                    cell.DataGridView.EndEdit();
                }
                else
                {
                    MessageBox.Show("Invalid Home or wp Alt");
                    cell.Style.BackColor = Color.Red;
                }

            }
            writeKML();
            Commands.EndEdit();
        }
        /// <summary>
        /// Used for current mouse position
        /// </summary>
        /// <param name="lat"></param>
        /// <param name="lng"></param>
        /// <param name="alt"></param>
        public void callMeDisplay(double lat, double lng, int alt)
        {
            TXT_mouselat.Text = lat.ToString();
            TXT_mouselong.Text = lng.ToString();
            TXT_mousealt.Text = srtm.getAltitude(lat, lng, MainMap.Zoom).ToString("0");

            try
            {
                double lastdist = MainMap.Manager.GetDistance(polygon.Points[polygon.Points.Count - 1], currentMarker.Position);

                lbl_prevdist.Text = rm.GetString("lbl_prevdist.Text") + ": " + FormatDistance(lastdist, true);

                double homedist = MainMap.Manager.GetDistance(currentMarker.Position, polygon.Points[0]);

                lbl_homedist.Text = rm.GetString("lbl_homedist.Text") + ": " + FormatDistance(homedist, true);
            }
            catch { }
        }

        /// <summary>
        /// Used to create a new WP
        /// </summary>
        /// <param name="lat"></param>
        /// <param name="lng"></param>
        /// <param name="alt"></param>
        public void callMe(double lat, double lng, int alt)
        {
            if (polygongridmode)
            {
                addPolygonPointToolStripMenuItem_Click(null, null);
                return;
            }

            if (sethome)
            {
                sethome = false;
                callMeDrag("Home", lat, lng, alt);
                return;
            }
            // creating a WP

            Commands.Rows.Add();
            selectedrow = Commands.RowCount - 1;
            Commands.CurrentCell = Commands.Rows[selectedrow].Cells[Param1.Index];

            setfromGE(lat, lng, alt);
        }

        public FlightPlanner()
        {
            InitializeComponent();

            // config map             
            //MainMap.MapType = MapType.GoogleSatellite;
            MainMap.MinZoom = 1;
            MainMap.CacheLocation = Path.GetDirectoryName(Application.ExecutablePath) + "/gmapcache/";

            //MainMap.Manager.ImageCacheLocal.PutImageToCache(,MapType.None,new GPoint(),17);

            // map events
            MainMap.OnCurrentPositionChanged += new CurrentPositionChanged(MainMap_OnCurrentPositionChanged);
            MainMap.OnTileLoadStart += new TileLoadStart(MainMap_OnTileLoadStart);
            MainMap.OnTileLoadComplete += new TileLoadComplete(MainMap_OnTileLoadComplete);
            MainMap.OnMarkerClick += new MarkerClick(MainMap_OnMarkerClick);
            MainMap.OnMapZoomChanged += new MapZoomChanged(MainMap_OnMapZoomChanged);
            MainMap.OnMapTypeChanged += new MapTypeChanged(MainMap_OnMapTypeChanged);
            MainMap.MouseMove += new MouseEventHandler(MainMap_MouseMove);
            MainMap.MouseDown += new MouseEventHandler(MainMap_MouseDown);
            MainMap.MouseUp += new MouseEventHandler(MainMap_MouseUp);
            MainMap.OnMarkerEnter += new MarkerEnter(MainMap_OnMarkerEnter);
            MainMap.OnMarkerLeave += new MarkerLeave(MainMap_OnMarkerLeave);

            MainMap.MapScaleInfoEnabled = false;
            MainMap.ScalePen = new Pen(Color.Red);

            MainMap.ForceDoubleBuffer = false;

            WebRequest.DefaultWebProxy.Credentials = System.Net.CredentialCache.DefaultCredentials;

            // get map type
            comboBoxMapType.DataSource = Enum.GetValues(typeof(MapType));
            comboBoxMapType.SelectedItem = MainMap.MapType;

            comboBoxMapType.SelectedValueChanged += new System.EventHandler(this.comboBoxMapType_SelectedValueChanged);
            /*
            // acccess mode
            comboBoxMode.DataSource = Enum.GetValues(typeof(AccessMode));
            comboBoxMode.SelectedItem = GMaps.Instance.Mode;

            // get position
            textBoxLat.Text = MainMap.Position.Lat.ToString(CultureInfo.InvariantCulture);
            textBoxLng.Text = MainMap.Position.Lng.ToString(CultureInfo.InvariantCulture);
             */

            MainMap.RoutesEnabled = true;

            //MainMap.MaxZoom = 18;

            // get zoom  
            trackBar1.Minimum = MainMap.MinZoom;
            trackBar1.Maximum = MainMap.MaxZoom + 0.99;

            // draw this layer first
            kmlpolygons = new GMapOverlay(MainMap, "kmlpolygons");
            MainMap.Overlays.Add(kmlpolygons);

            geofence = new GMapOverlay(MainMap, "geofence");
            MainMap.Overlays.Add(geofence);

            routes = new GMapOverlay(MainMap, "routes");
            MainMap.Overlays.Add(routes);

            polygons = new GMapOverlay(MainMap, "polygons");
            MainMap.Overlays.Add(polygons);

            objects = new GMapOverlay(MainMap, "objects");
            MainMap.Overlays.Add(objects);

            drawnpolygons = new GMapOverlay(MainMap, "drawnpolygons");
            MainMap.Overlays.Add(drawnpolygons);



            top = new GMapOverlay(MainMap, "top");
            //MainMap.Overlays.Add(top);

            objects.Markers.Clear();

            // set current marker
            currentMarker = new GMapMarkerGoogleRed(MainMap.Position);
            //top.Markers.Add(currentMarker);

            // map center
            center = new GMapMarkerCross(MainMap.Position);
            top.Markers.Add(center);

            MainMap.Zoom = 3;

            //set home
            try
            {
                MainMap.Position = new PointLatLng(double.Parse(TXT_homelat.Text), double.Parse(TXT_homelng.Text));
                MainMap.Zoom = 13;

            }
            catch (Exception) { }

            RegeneratePolygon();

            if (MainV2.getConfig("MapType") != "")
            {
                try
                {
                    comboBoxMapType.SelectedItem = Enum.Parse(typeof(MapType), MainV2.getConfig("MapType"));
                }
                catch { }
            }

            updateCMDParams();

            Up.Image = global::ArdupilotMega.Properties.Resources.up;
            Down.Image = global::ArdupilotMega.Properties.Resources.down;
        }

        void updateCMDParams()
        {
            cmdParamNames = readCMDXML();

            List<string> cmds = new List<string>();

            foreach (string item in cmdParamNames.Keys)
            {
                cmds.Add(item);
            }

            Command.DataSource = cmds;
        }

        Dictionary<string, string[]> readCMDXML()
        {
            Dictionary<string, string[]> cmd = new Dictionary<string, string[]>();

            // do lang stuff here

            string file = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "mavcmd.xml";

            using (XmlReader reader = XmlReader.Create(file))
            {
                reader.Read();
                reader.ReadStartElement("CMD");
                if (MainV2.APMFirmware == MainV2.Firmwares.ArduPlane)
                {
                    reader.ReadToFollowing("APM");
                }
                else
                {
                    reader.ReadToFollowing("AC2");
                }

                XmlReader inner = reader.ReadSubtree();

                inner.Read();

                inner.MoveToElement();

                inner.Read();

                while (inner.Read())
                {
                    inner.MoveToElement();
                    if (inner.IsStartElement())
                    {
                        string cmdname = inner.Name;
                        string[] cmdarray = new string[7];
                        int b = 0;

                        XmlReader inner2 = inner.ReadSubtree();

                        inner2.Read();

                        while (inner2.Read())
                        {
                            inner2.MoveToElement();
                            if (inner2.IsStartElement())
                            {
                                cmdarray[b] = inner2.ReadString();
                                b++;
                            }
                        }

                        cmd[cmdname] = cmdarray;
                    }
                }
            }

            return cmd;
        }

        void Commands_DataError(object sender, DataGridViewDataErrorEventArgs e)
        {
            Console.WriteLine(e.Exception.ToString() + " " + e.Context + " col " + e.ColumnIndex);
            e.Cancel = false;
            e.ThrowException = false;
            //throw new NotImplementedException();
        }
        /// <summary>
        /// Adds a new row to the datagrid
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BUT_Add_Click(object sender, EventArgs e)
        {
            if (Commands.CurrentRow == null)
            {
                selectedrow = 0;
            }
            else
            {
                selectedrow = Commands.CurrentRow.Index;
            }

            if (Commands.RowCount <= 1)
            {
                Commands.Rows.Add();
            }
            else
            {
                if (Commands.RowCount == selectedrow + 1)
                {
                    DataGridViewRow temp = Commands.Rows[selectedrow];
                    Commands.Rows.Add();
                }
                else
                {
                    Commands.Rows.Insert(selectedrow + 1, 1);
                }
            }
            writeKML();
        }

        private void Planner_Load(object sender, EventArgs e)
        {
            quickadd = true;

            config(false);

            if (MainV2.HomeLocation.Lat != 0 && MainV2.HomeLocation.Lng != 0)
            {
                TXT_homelat.Text = MainV2.HomeLocation.Lat.ToString();

                TXT_homelng.Text = MainV2.HomeLocation.Lng.ToString();

                TXT_homealt.Text = MainV2.HomeLocation.Alt.ToString();
            }


            quickadd = false;

            if (MainV2.config["WMSserver"] != null)
                MainMap.Manager.CustomWMSURL = MainV2.config["WMSserver"].ToString();

            trackBar1.Value = (int)MainMap.Zoom;

            // check for net and set offline if needed
            try
            {
                IPAddress[] addresslist = Dns.GetHostAddresses("www.google.com");
            }
            catch (Exception)
            { // here if dns failed
                isonline = false;
            }

            // setup geofence
            List<PointLatLng> polygonPoints = new List<PointLatLng>();
            gf = new GMapPolygon(polygonPoints, "geofence");
            gf.Stroke = new Pen(Color.Pink, 5);

            //setup drawnpolgon
            List<PointLatLng> polygonPoints2 = new List<PointLatLng>();
            drawnpolygon = new GMapPolygon(polygonPoints2, "drawnpoly");
            drawnpolygon.Stroke = new Pen(Color.Red, 2);

            updateCMDParams();

            // mono
            panelMap.Dock = DockStyle.None;
            panelMap.Dock = DockStyle.Fill;
            panelMap_Resize(null, null);

            writeKML();
        }

        void parser_ElementAdded(object sender, SharpKml.Base.ElementEventArgs e)
        {
            processKML(e.Element);
        }

        private void processKML(SharpKml.Dom.Element Element)
        {
            try
            {
                Console.WriteLine(Element.ToString() + " " + Element.Parent);
            }
            catch { }

            SharpKml.Dom.Document doc = Element as SharpKml.Dom.Document;
            SharpKml.Dom.Placemark pm = Element as SharpKml.Dom.Placemark;
            SharpKml.Dom.Folder folder = Element as SharpKml.Dom.Folder;
            SharpKml.Dom.Polygon polygon = Element as SharpKml.Dom.Polygon;
            SharpKml.Dom.LineString ls = Element as SharpKml.Dom.LineString;

            if (doc != null)
            {
                foreach (var feat in doc.Features)
                {
                    //Console.WriteLine("feat " + feat.GetType());
                    //processKML((Element)feat);
                }
            }
            else
                if (folder != null)
                {
                    foreach (Feature feat in folder.Features)
                    {
                        //Console.WriteLine("feat "+feat.GetType());
                        //processKML(feat);
                    }
                }
                else if (pm != null)
                {

                }
                else if (polygon != null)
                {
                    GMapPolygon kmlpolygon = new GMapPolygon(new List<PointLatLng>(), "kmlpolygon");

                    kmlpolygon.Stroke.Color = Color.Purple;

                    foreach (var loc in polygon.OuterBoundary.LinearRing.Coordinates)
                    {
                        kmlpolygon.Points.Add(new PointLatLng(loc.Latitude, loc.Longitude));
                    }

                    kmlpolygons.Polygons.Add(kmlpolygon);
                }
                else if (ls != null)
                {
                    GMapRoute kmlroute = new GMapRoute(new List<PointLatLng>(), "kmlroute");

                    kmlroute.Stroke.Color = Color.Purple;

                    foreach (var loc in ls.Coordinates)
                    {
                        kmlroute.Points.Add(new PointLatLng(loc.Latitude, loc.Longitude));
                    }

                    kmlpolygons.Routes.Add(kmlroute);
                }
        }

        private void ChangeColumnHeader(string command)
        {
            try
            {
                if (cmdParamNames.ContainsKey(command))
                    for (int i = 1; i <= 7; i++)
                        Commands.Columns[i].HeaderText = cmdParamNames[command][i - 1];
                else
                    for (int i = 1; i <= 7; i++)
                        Commands.Columns[i].HeaderText = "setme";
            }
            catch (Exception ex) { MessageBox.Show(ex.ToString()); }
        }

        /// <summary>
        /// Used to update column headers
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Commands_RowEnter(object sender, DataGridViewCellEventArgs e)
        {
            if (quickadd)
                return;
            try
            {
                selectedrow = e.RowIndex;
                string option = Commands[Command.Index, selectedrow].EditedFormattedValue.ToString();
                string cmd;
                try
                {
                    cmd = Commands[Command.Index, selectedrow].Value.ToString();
                }
                catch { cmd = option; }
                //Console.WriteLine("editformat " + option + " value " + cmd);
                ChangeColumnHeader(cmd);
            }
            catch (Exception ex) { MessageBox.Show(ex.ToString()); }
        }

        private void Commands_RowsAdded(object sender, DataGridViewRowsAddedEventArgs e)
        {
            for (int i = 0; i < Commands.ColumnCount; i++)
            {
                DataGridViewCell tcell = Commands.Rows[e.RowIndex].Cells[i];
                if (tcell.GetType() == typeof(DataGridViewTextBoxCell))
                {
                    if (tcell.Value == null)
                        tcell.Value = "0";
                }
            }

            DataGridViewComboBoxCell cell = Commands.Rows[e.RowIndex].Cells[Command.Index] as DataGridViewComboBoxCell;
            if (cell.Value == null)
            {
                cell.Value = "WAYPOINT";
                Commands.Rows[e.RowIndex].Cells[Delete.Index].Value = "X";
                if (!quickadd)
                {
                    Commands_RowEnter(sender, new DataGridViewCellEventArgs(0, e.RowIndex - 0)); // do header labels
                    Commands_RowValidating(sender, new DataGridViewCellCancelEventArgs(0, e.RowIndex)); // do default values
                }
            }

            if (quickadd)
                return;

            try
            {
                Commands.CurrentCell = Commands.Rows[e.RowIndex].Cells[0];

                if (Commands.Rows[e.RowIndex - 1].Cells[Command.Index].Value.ToString() == "WAYPOINT")
                {
                    Commands.Rows[e.RowIndex].Selected = true; // highlight row
                }
                else
                {
                    Commands.CurrentCell = Commands[1, e.RowIndex - 1];
                    //Commands_RowEnter(sender, new DataGridViewCellEventArgs(0, e.RowIndex-1));
                }
            }
            catch (Exception) { }
            // Commands.EndEdit();
        }
        private void Commands_RowValidating(object sender, DataGridViewCellCancelEventArgs e)
        {
            selectedrow = e.RowIndex;
            Commands_RowEnter(sender, new DataGridViewCellEventArgs(0, e.RowIndex - 0)); // do header labels - encure we dont 0 out valid colums
            int cols = Commands.Columns.Count;
            for (int a = 1; a < cols; a++)
            {
                DataGridViewTextBoxCell cell;
                cell = Commands.Rows[selectedrow].Cells[a] as DataGridViewTextBoxCell;

                if (Commands.Columns[a].HeaderText.Equals("") && cell != null && cell.Value == null)
                {
                    cell.Value = "0";
                }
                else
                {
                    if (cell != null && (cell.Value == null || cell.Value.ToString() == ""))
                    {
                        cell.Value = "?";
                    }
                    else
                    {
                        // not a text box
                    }
                }
            }
        }

        /// <summary>
        /// used to add a marker to the map display
        /// </summary>
        /// <param name="tag"></param>
        /// <param name="lng"></param>
        /// <param name="lat"></param>
        /// <param name="alt"></param>
        private void addpolygonmarker(string tag, double lng, double lat, int alt, Color? color)
        {
            try
            {
                PointLatLng point = new PointLatLng(lat, lng);
                GMapMarkerGoogleGreen m = new GMapMarkerGoogleGreen(point);
                m.ToolTipMode = MarkerTooltipMode.Always;
                m.ToolTipText = tag;
                m.Tag = tag;

                //ArdupilotMega.GMapMarkerRectWPRad mBorders = new ArdupilotMega.GMapMarkerRectWPRad(point, (int)float.Parse(TXT_WPRad.Text), MainMap);
                GMapMarkerRect mBorders = new GMapMarkerRect(point);
                {
                    mBorders.InnerMarker = m;
                    mBorders.wprad = (int)float.Parse(TXT_WPRad.Text);
                    mBorders.MainMap = MainMap;
                    if (color.HasValue)
                    {
                        mBorders.Color = color.Value;
                    }
                }

                objects.Markers.Add(m);
                objects.Markers.Add(mBorders);
            }
            catch (Exception) { }
        }

        private void addpolygonmarkergrid(string tag, double lng, double lat, int alt)
        {
            try
            {
                PointLatLng point = new PointLatLng(lat, lng);
                GMapMarkerGoogleRed m = new GMapMarkerGoogleRed(point);
                m.ToolTipMode = MarkerTooltipMode.Never;
                m.ToolTipText = "grid" + tag;
                m.Tag = "grid" + tag;

                //ArdupilotMega.GMapMarkerRectWPRad mBorders = new ArdupilotMega.GMapMarkerRectWPRad(point, (int)float.Parse(TXT_WPRad.Text), MainMap);
                GMapMarkerRect mBorders = new GMapMarkerRect(point);
                {
                    mBorders.InnerMarker = m;
                }

                drawnpolygons.Markers.Add(m);
                drawnpolygons.Markers.Add(mBorders);
            }
            catch (Exception ex) { Console.WriteLine(ex.ToString()); }
        }

        /// <summary>
        /// used to write a KML, update the Map view polygon, and update the row headers
        /// </summary>
        private void writeKML()
        {
            // quickadd is for when loading wps from eeprom or file, to prevent slow, loading times
            if (quickadd)
                return;

            // this is to share the current mission with the data tab
            pointlist = new List<PointLatLngAlt>();

            System.Diagnostics.Debug.WriteLine(DateTime.Now);
            try
            {
                if (objects != null) // hasnt been created yet
                {
                    objects.Markers.Clear();
                }

                // process and add home to the list
                string home;
                if (TXT_homealt.Text != "" && TXT_homelat.Text != "" && TXT_homelng.Text != "")
                {
                    home = string.Format("{0},{1},{2}\r\n", TXT_homelng.Text, TXT_homelat.Text, TXT_DefaultAlt.Text);
                    if (objects != null) // during startup
                    {
                        pointlist.Add(new PointLatLngAlt(double.Parse(TXT_homelat.Text), double.Parse(TXT_homelng.Text), (int)double.Parse(TXT_homealt.Text), "Home"));
                        addpolygonmarker("Home", double.Parse(TXT_homelng.Text), double.Parse(TXT_homelat.Text), 0, null);
                    }
                }
                else
                {
                    home = "";
                }

                // setup for centerpoint calc etc.
                double avglat = 0;
                double avglong = 0;
                double maxlat = -180;
                double maxlong = -180;
                double minlat = 180;
                double minlong = 180;
                double homealt = 0;
                try
                {
                    homealt = (int)double.Parse(TXT_homealt.Text);
                }
                catch { }
                if (CHK_altmode.Checked)
                {
                    homealt = 0; // for absolute we dont need to add homealt
                }

                int usable = 0;

                // number rows 
                System.Threading.Thread t1 = new System.Threading.Thread(delegate()
                {
                    // thread for updateing row numbers
                    for (int a = 0; a < Commands.Rows.Count - 0; a++)
                    {
                        try
                        {
                            if (Commands.Rows[a].HeaderCell.Value == null)
                            {
                                Commands.Rows[a].HeaderCell.Style.Alignment = DataGridViewContentAlignment.MiddleCenter;
                                Commands.Rows[a].HeaderCell.Value = (a + 1).ToString();
                            }
                            // skip rows with the correct number
                            string rowno = Commands.Rows[a].HeaderCell.Value.ToString();
                            if (!rowno.Equals((a + 1).ToString()))
                            {
                                // this code is where the delay is when deleting.
                                Commands.Rows[a].HeaderCell.Value = (a + 1).ToString();
                            }
                        }
                        catch (Exception) { }
                    }
                });
                t1.Name = "Row number updater";
                t1.IsBackground = true;
                t1.Start();
                MainV2.threads.Add(t1);

                long temp = System.Diagnostics.Stopwatch.GetTimestamp();

                string lookat = "";
                for (int a = 0; a < Commands.Rows.Count - 0; a++)
                {
                    try
                    {
                        int command = (byte)(int)Enum.Parse(typeof(MAVLink.MAV_CMD), Commands.Rows[a].Cells[Command.Index].Value.ToString(), false);
                        if (command < (byte)MAVLink.MAV_CMD.LAST && command != (byte)MAVLink.MAV_CMD.TAKEOFF)
                        {
                            string cell2 = Commands.Rows[a].Cells[Alt.Index].Value.ToString(); // alt
                            string cell3 = Commands.Rows[a].Cells[Lat.Index].Value.ToString(); // lat
                            string cell4 = Commands.Rows[a].Cells[Lon.Index].Value.ToString(); // lng

                            if (cell4 == "0" || cell3 == "0")
                                continue;
                            if (cell4 == "?" || cell3 == "?")
                                continue;

                            
                            if (command == (byte)MAVLink.MAV_CMD.LOITER_TIME || command == (byte)MAVLink.MAV_CMD.LOITER_TURNS || command == (byte)MAVLink.MAV_CMD.LOITER_UNLIM) {
                                pointlist.Add(new PointLatLngAlt(double.Parse(cell3), double.Parse(cell4), (int)double.Parse(cell2) + homealt, (a + 1).ToString()){ color  = Color.LightBlue });
                                addpolygonmarker((a + 1).ToString(), double.Parse(cell4), double.Parse(cell3), (int)double.Parse(cell2) , Color.LightBlue);
                            } else {
                                pointlist.Add(new PointLatLngAlt(double.Parse(cell3), double.Parse(cell4), (int)double.Parse(cell2) + homealt, (a + 1).ToString()));
                                addpolygonmarker((a + 1).ToString(), double.Parse(cell4), double.Parse(cell3), (int)double.Parse(cell2) ,null);
                            }

                            avglong += double.Parse(Commands.Rows[a].Cells[Lon.Index].Value.ToString());
                            avglat += double.Parse(Commands.Rows[a].Cells[Lat.Index].Value.ToString());
                            usable++;

                            maxlong = Math.Max(double.Parse(Commands.Rows[a].Cells[Lon.Index].Value.ToString()), maxlong);
                            maxlat = Math.Max(double.Parse(Commands.Rows[a].Cells[Lat.Index].Value.ToString()), maxlat);
                            minlong = Math.Min(double.Parse(Commands.Rows[a].Cells[Lon.Index].Value.ToString()), minlong);
                            minlat = Math.Min(double.Parse(Commands.Rows[a].Cells[Lat.Index].Value.ToString()), minlat);

                            System.Diagnostics.Debug.WriteLine(temp - System.Diagnostics.Stopwatch.GetTimestamp());
                        }
                    }
                    catch (Exception e) { Console.WriteLine("writekml - bad wp data " + e.ToString()); }
                }

                if (usable > 0)
                {
                    avglat = avglat / usable;
                    avglong = avglong / usable;
                    double latdiff = maxlat - minlat;
                    double longdiff = maxlong - minlong;
                    float range = 4000;

                    Locationwp loc1 = new Locationwp();
                    loc1.lat = (float)(minlat);
                    loc1.lng = (float)(minlong);
                    Locationwp loc2 = new Locationwp();
                    loc2.lat = (float)(maxlat);
                    loc2.lng = (float)(maxlong);

                    //double distance = getDistance(loc1, loc2);  // same code as ardupilot
                    double distance = 2000;

                    if (usable > 1)
                    {
                        range = (float)(distance * 2);
                    }
                    else
                    {
                        range = 4000;
                    }

                    if (avglong != 0 && usable < 3)
                    {
                        // no autozoom
                        lookat = "<LookAt>     <longitude>" + (minlong + longdiff / 2).ToString(new System.Globalization.CultureInfo("en-US")) + "</longitude>     <latitude>" + (minlat + latdiff / 2).ToString(new System.Globalization.CultureInfo("en-US")) + "</latitude> <range>" + range + "</range> </LookAt>";
                        //MainMap.ZoomAndCenterMarkers("objects");
                        //MainMap.Zoom -= 1;
                        //MainMap_OnMapZoomChanged();
                    }
                }
                else if (home.Length > 5 && usable == 0)
                {
                    lookat = "<LookAt>     <longitude>" + TXT_homelng.Text.ToString(new System.Globalization.CultureInfo("en-US")) + "</longitude>     <latitude>" + TXT_homelat.Text.ToString(new System.Globalization.CultureInfo("en-US")) + "</latitude> <range>4000</range> </LookAt>";

                    RectLatLng? rect = MainMap.GetRectOfAllMarkers("objects");
                    if (rect.HasValue)
                    {
                        MainMap.Position = rect.Value.LocationMiddle;
                    }

                    MainMap.Zoom = 17;

                    MainMap_OnMapZoomChanged();
                }

                RegeneratePolygon();

                if (polygon != null && polygon.Points.Count > 0)
                {
                    double homedist = 0;

                    if (home.Length > 5)
                    {
                        pointlist.Add(new PointLatLngAlt(double.Parse(TXT_homelat.Text), double.Parse(TXT_homelng.Text), (int)double.Parse(TXT_homealt.Text), "Home"));

                        homedist = MainMap.Manager.GetDistance(polygon.Points[polygon.Points.Count - 1], polygon.Points[0]);

                        lbl_homedist.Text = rm.GetString("lbl_homedist.Text") + ": " + FormatDistance(homedist, true);
                    }
                    lbl_distance.Text = rm.GetString("lbl_distance.Text") + ": " + FormatDistance(polygon.Distance + homedist, false);
                }


                config(true);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.ToString());
            }

            System.Diagnostics.Debug.WriteLine(DateTime.Now);
        }
        /// <summary>
        /// Saves a waypoint writer file
        /// </summary>
        private void savewaypoints()
        {
            SaveFileDialog fd = new SaveFileDialog();
            fd.Filter = "Ardupilot Mission (*.txt)|*.*";
            fd.DefaultExt = ".txt";
            DialogResult result = fd.ShowDialog();
            string file = fd.FileName;
            if (file != "")
            {
                try
                {
                    StreamWriter sw = new StreamWriter(file);
                    sw.WriteLine("QGC WPL 110");
                    try
                    {
                        sw.WriteLine("0\t1\t0\t16\t0\t0\t0\t0\t" + double.Parse(TXT_homelat.Text).ToString("0.000000", new System.Globalization.CultureInfo("en-US")) + "\t" + double.Parse(TXT_homelng.Text).ToString("0.000000", new System.Globalization.CultureInfo("en-US")) + "\t" + double.Parse(TXT_homealt.Text).ToString("0.000000", new System.Globalization.CultureInfo("en-US")) + "\t1");
                    }
                    catch
                    {
                        sw.WriteLine("0\t1\t0\t0\t0\t0\t0\t0\t0\t0\t0\t1");
                    }
                    for (int a = 0; a < Commands.Rows.Count - 0; a++)
                    {
                        byte mode = (byte)(MAVLink.MAV_CMD)Enum.Parse(typeof(MAVLink.MAV_CMD), Commands.Rows[a].Cells[0].Value.ToString());

                        sw.Write((a + 1)); // seq
                        sw.Write("\t" + 0); // current
                        sw.Write("\t" + (CHK_altmode.Checked == true ? (byte)MAVLink.MAV_FRAME.MAV_FRAME_GLOBAL : (byte)MAVLink.MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT)); //frame 
                        sw.Write("\t" + mode);
                        sw.Write("\t" + double.Parse(Commands.Rows[a].Cells[Param1.Index].Value.ToString()).ToString("0.000000", new System.Globalization.CultureInfo("en-US")));
                        sw.Write("\t" + double.Parse(Commands.Rows[a].Cells[Param2.Index].Value.ToString()).ToString("0.000000", new System.Globalization.CultureInfo("en-US")));
                        sw.Write("\t" + double.Parse(Commands.Rows[a].Cells[Param3.Index].Value.ToString()).ToString("0.000000", new System.Globalization.CultureInfo("en-US")));
                        sw.Write("\t" + double.Parse(Commands.Rows[a].Cells[Param4.Index].Value.ToString()).ToString("0.000000", new System.Globalization.CultureInfo("en-US")));
                        sw.Write("\t" + double.Parse(Commands.Rows[a].Cells[Lat.Index].Value.ToString()).ToString("0.000000", new System.Globalization.CultureInfo("en-US")));
                        sw.Write("\t" + double.Parse(Commands.Rows[a].Cells[Lon.Index].Value.ToString()).ToString("0.000000", new System.Globalization.CultureInfo("en-US")));
                        sw.Write("\t" + (double.Parse(Commands.Rows[a].Cells[Alt.Index].Value.ToString()) / MainV2.cs.multiplierdist).ToString("0.000000", new System.Globalization.CultureInfo("en-US")));
                        sw.Write("\t" + 1);
                        sw.WriteLine("");
                    }
                    sw.Close();
                }
                catch (Exception) { MessageBox.Show("Error writing file"); }
            }
        }

        private void SaveFile_Click(object sender, EventArgs e)
        {
            savewaypoints();
            writeKML();
        }

        /// <summary>
        /// Reads the EEPROM from a com port
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        internal void BUT_read_Click(object sender, EventArgs e)
        {

            List<Locationwp> cmds = new List<Locationwp>();
            int error = 0;

            System.Threading.Thread t12 = new System.Threading.Thread(delegate()
            {
                try
                {
                    MAVLink port = MainV2.comPort;

                    if (!port.BaseStream.IsOpen)
                    {
                        throw new Exception("Please Connect First!");
                    }

                    MainV2.givecomport = true;

                    param = port.param;

                    Console.WriteLine("Getting WP #");
                    int cmdcount = port.getWPCount();

                    for (ushort a = 0; a < cmdcount; a++)
                    {
                        Console.WriteLine("Getting WP" + a);
                        cmds.Add(port.getWP(a));
                    }

                    port.setWPACK();

                    Console.WriteLine("Done");
                }
                catch (Exception ex) { error = 1; MessageBox.Show("Error : " + ex.ToString()); }
                try
                {
                    this.BeginInvoke((System.Threading.ThreadStart)delegate()
                    {
                        if (error == 0)
                        {
                            try
                            {
                                processToScreen(cmds);
                            }
                            catch (Exception exx) { Console.WriteLine(exx.ToString()); }
                        }

                        MainV2.givecomport = false;

                        BUT_read.Enabled = true;

                        writeKML();

                    });
                }
                catch (Exception exx) { Console.WriteLine(exx.ToString()); }
            });
            t12.IsBackground = true;
            t12.Name = "Read wps";
            t12.Start();
            MainV2.threads.Add(t12);

            BUT_read.Enabled = false;
        }

        /// <summary>
        /// Writes the mission from the datagrid and values to the EEPROM
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BUT_write_Click(object sender, EventArgs e)
        {
            if (CHK_altmode.Checked)
            {
                if (DialogResult.No == MessageBox.Show("Absolute Alt is ticked are you sure?", "Alt Mode", MessageBoxButtons.YesNo))
                {
                    CHK_altmode.Checked = false;
                }
            }

            // check for invalid grid data
            for (int a = 0; a < Commands.Rows.Count - 0; a++)
            {
                for (int b = 0; b < Commands.ColumnCount - 0; b++)
                {
                    double answer;
                    if (b >= 1 && b <= 4)
                    {
                        if (!double.TryParse(Commands[b, a].Value.ToString(), out answer))
                        {
                            MessageBox.Show("There are errors in your mission");
                            return;
                        }
                    }
                }
            }

            System.Threading.Thread t12 = new System.Threading.Thread(delegate()
            {
                try
                {
                    MAVLink port = MainV2.comPort;

                    if (!port.BaseStream.IsOpen)
                    {
                        throw new Exception("Please Connect First!");
                    }

                    MainV2.givecomport = true;

                    Locationwp home = new Locationwp();

                    try
                    {
                        home.id = (byte)MAVLink.MAV_CMD.WAYPOINT;
                        home.lat = (float.Parse(TXT_homelat.Text));
                        home.lng = (float.Parse(TXT_homelng.Text));
                        home.alt = (float.Parse(TXT_homealt.Text) / MainV2.cs.multiplierdist); // use saved home
                    }
                    catch { throw new Exception("Your home location is invalid"); }

                    port.setWPTotal((ushort)(Commands.Rows.Count + 1)); // + home

                    port.setWP(home, (ushort)0, MAVLink.MAV_FRAME.MAV_FRAME_GLOBAL, 0);

                    MAVLink.MAV_FRAME frame = MAVLink.MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;

                    // process grid to memory eeprom
                    for (int a = 0; a < Commands.Rows.Count - 0; a++)
                    {
                        Locationwp temp = new Locationwp();
                        temp.id = (byte)(int)Enum.Parse(typeof(MAVLink.MAV_CMD), Commands.Rows[a].Cells[Command.Index].Value.ToString(), false);
                        temp.p1 = float.Parse(Commands.Rows[a].Cells[Param1.Index].Value.ToString());
                        if (temp.id < (byte)MAVLink.MAV_CMD.LAST || temp.id == (byte)MAVLink.MAV_CMD.DO_SET_HOME)
                        {
                            if (CHK_altmode.Checked)
                            {
                                frame = MAVLink.MAV_FRAME.MAV_FRAME_GLOBAL;
                            }
                            else
                            {
                                frame = MAVLink.MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
                            }
                        }

                        temp.alt = (float)(double.Parse(Commands.Rows[a].Cells[Alt.Index].Value.ToString()) / MainV2.cs.multiplierdist);
                        temp.lat = (float)(double.Parse(Commands.Rows[a].Cells[Lat.Index].Value.ToString()));
                        temp.lng = (float)(double.Parse(Commands.Rows[a].Cells[Lon.Index].Value.ToString()));

                        temp.p2 = (float)(double.Parse(Commands.Rows[a].Cells[Param2.Index].Value.ToString()));
                        temp.p3 = (float)(double.Parse(Commands.Rows[a].Cells[Param3.Index].Value.ToString()));
                        temp.p4 = (float)(double.Parse(Commands.Rows[a].Cells[Param4.Index].Value.ToString()));

                        port.setWP(temp, (ushort)(a + 1), frame, 0);
                    }

                    port.setWPACK();


                    if (CHK_holdalt.Checked)
                    {
                        port.setParam("ALT_HOLD_RTL", int.Parse(TXT_DefaultAlt.Text) / MainV2.cs.multiplierdist);
                    }
                    else
                    {
                        port.setParam("ALT_HOLD_RTL", -1);
                    }

                    port.setParam("WP_RADIUS", (byte)int.Parse(TXT_WPRad.Text) / MainV2.cs.multiplierdist);

                    try
                    {
                        port.setParam("WP_LOITER_RAD", (byte)(int.Parse(TXT_loiterrad.Text) / MainV2.cs.multiplierdist));
                    }
                    catch
                    {
                        port.setParam("LOITER_RAD", (byte)int.Parse(TXT_loiterrad.Text) / MainV2.cs.multiplierdist);
                    }

                }
                catch (Exception ex) { MessageBox.Show("Error : " + ex.ToString()); }

                MainV2.givecomport = false;

                try
                {
                    this.BeginInvoke((System.Threading.ThreadStart)delegate()
                    {
                        BUT_write.Enabled = true;
                    });
                }
                catch { }

            });
            t12.IsBackground = true;
            t12.Name = "Write wps";
            t12.Start();
            MainV2.threads.Add(t12);

            MainMap.Focus();

            BUT_write.Enabled = false;
        }

        /// <summary>
        /// Processes a loaded EEPROM to the map and datagrid
        /// </summary>
        void processToScreen(List<Locationwp> cmds)
        {
            quickadd = true;
            Commands.Rows.Clear();

            int i = -1;
            foreach (Locationwp temp in cmds)
            {
                i++;
                //Console.WriteLine("FP processToScreen " + i);
                if (temp.id == 0 && i != 0) // 0 and not home
                    break;
                if (temp.id == 255 && i != 0) // bad record - never loaded any WP's - but have started the board up.
                    break;
                if (i + 1 >= Commands.Rows.Count)
                {
                    Commands.Rows.Add();
                }
                //if (i == 0 && temp.alt == 0) // skip 0 home
                  //  continue;
                DataGridViewTextBoxCell cell;
                DataGridViewComboBoxCell cellcmd;
                cellcmd = Commands.Rows[i].Cells[Command.Index] as DataGridViewComboBoxCell;
                cellcmd.Value = "WAYPOINT";

                foreach (object value in Enum.GetValues(typeof(MAVLink.MAV_CMD)))
                {
                    if ((int)value == temp.id)
                    {
                        cellcmd.Value = value.ToString();
                        break;
                    }
                }

                if (temp.id < (byte)MAVLink.MAV_CMD.LAST || temp.id == (byte)MAVLink.MAV_CMD.DO_SET_HOME)
                {
                    if ((temp.options & 0x1) == 0 && i != 0) // home is always abs
                    {
                        CHK_altmode.Checked = true;
                    }
                    else
                    {
                        CHK_altmode.Checked = false;
                    }



                }

                cell = Commands.Rows[i].Cells[Alt.Index] as DataGridViewTextBoxCell;
                cell.Value = Math.Round((temp.alt * MainV2.cs.multiplierdist), 0);
                cell = Commands.Rows[i].Cells[Lat.Index] as DataGridViewTextBoxCell;
                cell.Value = (double)temp.lat;
                cell = Commands.Rows[i].Cells[Lon.Index] as DataGridViewTextBoxCell;
                cell.Value = (double)temp.lng;

                cell = Commands.Rows[i].Cells[Param1.Index] as DataGridViewTextBoxCell;
                cell.Value = temp.p1;
                cell = Commands.Rows[i].Cells[Param2.Index] as DataGridViewTextBoxCell;
                cell.Value = temp.p2;
                cell = Commands.Rows[i].Cells[Param3.Index] as DataGridViewTextBoxCell;
                cell.Value = temp.p3;
                cell = Commands.Rows[i].Cells[Param4.Index] as DataGridViewTextBoxCell;
                cell.Value = temp.p4;
            }
            try
            {
                DataGridViewTextBoxCell cellhome;
                cellhome = Commands.Rows[0].Cells[Lat.Index] as DataGridViewTextBoxCell;
                if (cellhome.Value != null)
                {
                    if (cellhome.Value.ToString() != TXT_homelat.Text && cellhome.Value.ToString() != "0")
                    {
                        DialogResult dr = MessageBox.Show("Reset Home to loaded coords", "Reset Home Coords", MessageBoxButtons.YesNo);

                        if (dr == DialogResult.Yes)
                        {
                            TXT_homelat.Text = (double.Parse(cellhome.Value.ToString())).ToString();
                            cellhome = Commands.Rows[0].Cells[Lon.Index] as DataGridViewTextBoxCell;
                            TXT_homelng.Text = (double.Parse(cellhome.Value.ToString())).ToString();
                            cellhome = Commands.Rows[0].Cells[Alt.Index] as DataGridViewTextBoxCell;
                            TXT_homealt.Text = (double.Parse(cellhome.Value.ToString()) * MainV2.cs.multiplierdist).ToString();
                        }
                    }
                }

                string hold_alt = ((int)((float)param["ALT_HOLD_RTL"] * MainV2.cs.multiplierdist)).ToString();

                if (!hold_alt.Equals("-1"))
                {
                    TXT_DefaultAlt.Text = hold_alt;
                }

                TXT_WPRad.Text = ((int)((float)param["WP_RADIUS"] * MainV2.cs.multiplierdist)).ToString();
                try
                {
                    TXT_loiterrad.Text = ((int)((float)param["LOITER_RADIUS"] * MainV2.cs.multiplierdist)).ToString();
                }
                catch
                {
                    TXT_loiterrad.Text = ((int)((float)param["WP_LOITER_RAD"] * MainV2.cs.multiplierdist)).ToString();
                }
                CHK_holdalt.Checked = Convert.ToBoolean((float)param["ALT_HOLD_RTL"] > 0);

            }
            catch (Exception) { } // if there is no valid home

            if (Commands.RowCount > 0)
            {
                Commands.Rows.Remove(Commands.Rows[0]); // remove home row
            }

            quickadd = false;

            writeKML();

            MainMap.ZoomAndCenterMarkers("objects");

            MainMap_OnMapZoomChanged();
        }



        /// <summary>
        /// Saves this forms config to MAIN, where it is written in a global config
        /// </summary>
        /// <param name="write">true/false</param>
        private void config(bool write)
        {
            if (write)
            {
                ArdupilotMega.MainV2.config["TXT_homelat"] = TXT_homelat.Text;
                ArdupilotMega.MainV2.config["TXT_homelng"] = TXT_homelng.Text;
                ArdupilotMega.MainV2.config["TXT_homealt"] = TXT_homealt.Text;


                ArdupilotMega.MainV2.config["TXT_WPRad"] = TXT_WPRad.Text;

                ArdupilotMega.MainV2.config["TXT_loiterrad"] = TXT_loiterrad.Text;

                ArdupilotMega.MainV2.config["TXT_DefaultAlt"] = TXT_DefaultAlt.Text;

                ArdupilotMega.MainV2.config["CHK_altmode"] = CHK_altmode.Checked;

            }
            else
            {
                Hashtable temp = new Hashtable((Hashtable)ArdupilotMega.MainV2.config.Clone());

                foreach (string key in temp.Keys)
                {
                    switch (key)
                    {
                        case "TXT_WPRad":
                            TXT_WPRad.Text = ArdupilotMega.MainV2.config[key].ToString();
                            break;
                        case "TXT_loiterrad":
                            TXT_loiterrad.Text = ArdupilotMega.MainV2.config[key].ToString();
                            break;
                        case "TXT_DefaultAlt":
                            TXT_DefaultAlt.Text = ArdupilotMega.MainV2.config[key].ToString();
                            break;
                        case "CHK_altmode":
                            CHK_altmode.Checked = false;//bool.Parse(ArdupilotMega.MainV2.config[key].ToString());
                            break;
                        default:
                            break;
                    }
                }

            }
        }

        private void TXT_WPRad_KeyPress(object sender, KeyPressEventArgs e)
        {
            int isNumber = 0;
            if (e.KeyChar.ToString() == "\b")
                return;
            e.Handled = !int.TryParse(e.KeyChar.ToString(), out isNumber);
        }

        private void TXT_WPRad_Leave(object sender, EventArgs e)
        {
            int isNumber = 0;
            if (!int.TryParse(TXT_WPRad.Text, out isNumber))
            {
                TXT_WPRad.Text = "30";
            }
            if (isNumber > 127)
            {
                MessageBox.Show("The value can only be between 0 and 127");
                TXT_WPRad.Text = "127";
            }
            writeKML();
        }

        private void TXT_loiterrad_KeyPress(object sender, KeyPressEventArgs e)
        {
            int isNumber = 0;
            if (e.KeyChar.ToString() == "\b")
                return;
            e.Handled = !int.TryParse(e.KeyChar.ToString(), out isNumber);
        }

        private void TXT_loiterrad_Leave(object sender, EventArgs e)
        {
            int isNumber = 0;
            if (!int.TryParse(TXT_loiterrad.Text, out isNumber))
            {
                TXT_loiterrad.Text = "45";
            }
            if (isNumber > 127)
            {
                MessageBox.Show("The value can only be between 0 and 127");
                TXT_loiterrad.Text = "127";
            }
        }

        private void TXT_DefaultAlt_KeyPress(object sender, KeyPressEventArgs e)
        {
            int isNumber = 0;
            if (e.KeyChar.ToString() == "\b")
                return;
            e.Handled = !int.TryParse(e.KeyChar.ToString(), out isNumber);
        }

        private void TXT_DefaultAlt_Leave(object sender, EventArgs e)
        {
            int isNumber = 0;
            if (!int.TryParse(TXT_DefaultAlt.Text, out isNumber))
            {
                TXT_DefaultAlt.Text = "100";
            }
        }


        /// <summary>
        /// used to control buttons in the datagrid
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Commands_CellContentClick(object sender, DataGridViewCellEventArgs e)
        {
            try
            {
                if (e.RowIndex < 0)
                    return;
                if (e.ColumnIndex == Delete.Index && (e.RowIndex + 0) < Commands.RowCount) // delete
                {
                    quickadd = true;
                    Commands.Rows.RemoveAt(e.RowIndex);
                    quickadd = false;
                    writeKML();
                }
                if (e.ColumnIndex == Up.Index && e.RowIndex != 0) // up
                {
                    DataGridViewRow myrow = Commands.CurrentRow;
                    Commands.Rows.Remove(myrow);
                    Commands.Rows.Insert(e.RowIndex - 1, myrow);
                    writeKML();
                }
                if (e.ColumnIndex == Down.Index && e.RowIndex < Commands.RowCount - 1) // down
                {
                    DataGridViewRow myrow = Commands.CurrentRow;
                    Commands.Rows.Remove(myrow);
                    Commands.Rows.Insert(e.RowIndex + 1, myrow);
                    writeKML();
                }
            }
            catch (Exception) { MessageBox.Show("Row error"); }
        }

        private void Commands_DefaultValuesNeeded(object sender, DataGridViewRowEventArgs e)
        {
            e.Row.Cells[Delete.Index].Value = "X";
            e.Row.Cells[Up.Index].Value = global::ArdupilotMega.Properties.Resources.up;
            e.Row.Cells[Down.Index].Value = global::ArdupilotMega.Properties.Resources.down;
        }

        private void TXT_homelat_TextChanged(object sender, EventArgs e)
        {
            sethome = false;
            try
            {
                MainV2.HomeLocation.Lat = double.Parse(TXT_homelat.Text);
            }
            catch { }
            writeKML();

        }

        private void TXT_homelng_TextChanged(object sender, EventArgs e)
        {
            sethome = false;
            try
            {
                MainV2.HomeLocation.Lng = double.Parse(TXT_homelng.Text);
            }
            catch { }
            writeKML();
        }

        private void TXT_homealt_TextChanged(object sender, EventArgs e)
        {
            sethome = false;
            try
            {
                MainV2.HomeLocation.Alt = double.Parse(TXT_homealt.Text);
            }
            catch { }
            writeKML();
        }

        private void Planner_FormClosing(object sender, FormClosingEventArgs e)
        {
        }

        private void BUT_loadwpfile_Click(object sender, EventArgs e)
        {
            OpenFileDialog fd = new OpenFileDialog();
            fd.Filter = "Ardupilot Mission (*.txt)|*.*";
            fd.DefaultExt = ".txt";
            DialogResult result = fd.ShowDialog();
            string file = fd.FileName;
            if (file != "")
            {
                if (file.ToLower().EndsWith(".h"))
                {
                    readwaypointwritterfile(file);
                }
                else
                {
                    readQGC110wpfile(file);
                }
            }
        }

        void readQGC110wpfile(string file)
        {
            int wp_count = 0;
            bool error = false;
            List<Locationwp> cmds = new List<Locationwp>();

            try
            {
                StreamReader sr = new StreamReader(file); //"defines.h"
                string header = sr.ReadLine();
                if (header == null || !header.Contains("QGC WPL 110"))
                {
                    MessageBox.Show("Invalid Waypoint file");
                    return;
                }
                while (!error && !sr.EndOfStream)
                {
                    string line = sr.ReadLine();
                    // waypoints

                    if (line.StartsWith("#"))
                        continue;

                    string[] items = line.Split(new char[] { (char)'\t', ' ' }, StringSplitOptions.RemoveEmptyEntries);

                    if (items.Length <= 9)
                        continue;

                    try
                    {

                        Locationwp temp = new Locationwp();
                        if (items[2] == "3")
                        { // abs MAV_FRAME_GLOBAL_RELATIVE_ALT=3
                            temp.options = 1;
                        }
                        else
                        {
                            temp.options = 0;
                        }
                        temp.id = (byte)(int)Enum.Parse(typeof(MAVLink.MAV_CMD), items[3], false);
                        temp.p1 = float.Parse(items[4], new System.Globalization.CultureInfo("en-US"));

                        if (temp.id == 99)
                            temp.id = 0;

                        temp.alt = (float)(double.Parse(items[10], new System.Globalization.CultureInfo("en-US")));
                        temp.lat = (float)(double.Parse(items[8], new System.Globalization.CultureInfo("en-US")));
                        temp.lng = (float)(double.Parse(items[9], new System.Globalization.CultureInfo("en-US")));

                        temp.p2 = (float)(double.Parse(items[5], new System.Globalization.CultureInfo("en-US")));
                        temp.p3 = (float)(double.Parse(items[6], new System.Globalization.CultureInfo("en-US")));
                        temp.p4 = (float)(double.Parse(items[7], new System.Globalization.CultureInfo("en-US")));

                        cmds.Add(temp);

                        wp_count++;

                    }
                    catch { MessageBox.Show("Line invalid\n" + line); }

                    if (wp_count == byte.MaxValue)
                    {
                        MessageBox.Show("To many Waypoints!!!");
                        break;
                    }

                }

                sr.Close();

                processToScreen(cmds);

                writeKML();

                MainMap.ZoomAndCenterMarkers("objects");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Can't open file! " + ex.ToString());
            }
        }

        private void trackBar1_ValueChanged(object sender, EventArgs e)
        {
            try
            {
                lock (thisLock)
                {
                    MainMap.Zoom = trackBar1.Value;
                }
            }
            catch { }
        }

        // marker
        GMapMarker currentMarker;
        GMapMarker center = new GMapMarkerCross(new PointLatLng(0.0, 0.0));

        // polygons
        GMapPolygon polygon;
        GMapPolygon drawnpolygon;
        GMapPolygon gf;

        GMapOverlay kmlpolygons;
        GMapOverlay geofence;


        // layers
        GMapOverlay top;
        internal GMapOverlay objects;
        static GMapOverlay routes;// static so can update from gcs
        static GMapOverlay polygons;
        GMapOverlay drawnpolygons;

        // etc
        readonly Random rnd = new Random();
        string mobileGpsLog = string.Empty;
        GMapMarkerRect CurentRectMarker = null;
        bool isMouseDown = false;
        bool isMouseDraging = false;
        PointLatLng start;
        PointLatLng end;

        //public long ElapsedMilliseconds;

        #region -- map events --
        void MainMap_OnMarkerLeave(GMapMarker item)
        {
            if (!isMouseDown)
            {
                if (item is GMapMarkerRect)
                {
                    CurentRectMarker = null;

                    GMapMarkerRect rc = item as GMapMarkerRect;
                    rc.Pen.Color = Color.Blue;
                    MainMap.Invalidate(false);
                }
            }
        }

        void MainMap_OnMarkerEnter(GMapMarker item)
        {
            if (!isMouseDown)
            {
                if (item is GMapMarkerRect)
                {
                    GMapMarkerRect rc = item as GMapMarkerRect;
                    rc.Pen.Color = Color.Red;
                    MainMap.Invalidate(false);

                    CurentRectMarker = rc;
                }
            }
        }

        void MainMap_OnMapTypeChanged(MapType type)
        {
            comboBoxMapType.SelectedItem = MainMap.MapType;

            trackBar1.Minimum = MainMap.MinZoom;
            trackBar1.Maximum = MainMap.MaxZoom + 0.99;

            MainMap.ZoomAndCenterMarkers("objects");

            if (type == MapType.CustomWMS)
            {
                string url = "";
                if (MainV2.config["WMSserver"] != null)
                    url = MainV2.config["WMSserver"].ToString();
                Common.InputBox("WMS Server", "Enter the WMS server URL", ref url);
                MainV2.config["WMSserver"] = url;
                MainMap.Manager.CustomWMSURL = url;
            }
        }

        void MainMap_MouseUp(object sender, MouseEventArgs e)
        {
            end = MainMap.FromLocalToLatLng(e.X, e.Y);

            if (e.Button == MouseButtons.Right) // ignore right clicks
            {
                return;
            }

            if (isMouseDown) // mouse down on some other object and dragged to here.
            {
                if (e.Button == MouseButtons.Left)
                {
                    isMouseDown = false;
                }
                if (!isMouseDraging)
                {
                    if (CurentRectMarker != null)
                    {
                        // cant add WP in existing rect
                    }
                    else
                    {
                        callMe(currentMarker.Position.Lat, currentMarker.Position.Lng, 0);
                    }
                }
                else
                {
                    if (CurentRectMarker != null)
                    {
                        if (CurentRectMarker.InnerMarker.Tag.ToString().Contains("grid"))
                        {
                            try
                            {
                                drawnpolygon.Points[int.Parse(CurentRectMarker.InnerMarker.Tag.ToString().Replace("grid", "")) - 1] = new PointLatLng(end.Lat, end.Lng);
                                MainMap.UpdatePolygonLocalPosition(drawnpolygon);
                                MainMap.Invalidate();
                            }
                            catch { }
                        }
                        else
                        {
                            callMeDrag(CurentRectMarker.InnerMarker.Tag.ToString(), currentMarker.Position.Lat, currentMarker.Position.Lng, -1);
                        }
                        CurentRectMarker = null;
                    }
                }
            }

            isMouseDraging = false;
        }

        void MainMap_MouseDown(object sender, MouseEventArgs e)
        {
            start = MainMap.FromLocalToLatLng(e.X, e.Y);

            if (e.Button == MouseButtons.Left && Control.ModifierKeys != Keys.Alt)
            {
                isMouseDown = true;
                isMouseDraging = false;

                if (currentMarker.IsVisible)
                {
                    currentMarker.Position = MainMap.FromLocalToLatLng(e.X, e.Y);
                }
            }
        }

        // move current marker with left holding
        void MainMap_MouseMove(object sender, MouseEventArgs e)
        {
            PointLatLng point = MainMap.FromLocalToLatLng(e.X, e.Y);

            currentMarker.Position = point;

            if (!isMouseDown)
            {
                callMeDisplay(point.Lat, point.Lng, 0);
            }

            //draging
            if (e.Button == MouseButtons.Left && isMouseDown)
            {
                isMouseDraging = true;
                if (CurentRectMarker == null) // left click pan
                {
                    double latdif = start.Lat - point.Lat;
                    double lngdif = start.Lng - point.Lng;

                    try
                    {
                        lock (thisLock)
                        {
                            MainMap.Position = new PointLatLng(center.Position.Lat + latdif, center.Position.Lng + lngdif);
                        }
                    }
                    catch { }
                }
                else // move rect marker
                {
                    try
                    {
                        if (CurentRectMarker.InnerMarker.Tag.ToString().Contains("grid"))
                        {
                            drawnpolygon.Points[int.Parse(CurentRectMarker.InnerMarker.Tag.ToString().Replace("grid", "")) - 1] = new PointLatLng(point.Lat, point.Lng);
                            MainMap.UpdatePolygonLocalPosition(drawnpolygon);
                            MainMap.Invalidate();
                        }
                    }
                    catch { }

                    PointLatLng pnew = MainMap.FromLocalToLatLng(e.X, e.Y);

                    int? pIndex = (int?)CurentRectMarker.Tag;
                    if (pIndex.HasValue)
                    {
                        if (pIndex < polygon.Points.Count)
                        {
                            polygon.Points[pIndex.Value] = pnew;
                            lock (thisLock)
                            {
                                MainMap.UpdatePolygonLocalPosition(polygon);
                            }
                        }
                    }

                    if (currentMarker.IsVisible)
                    {
                        currentMarker.Position = pnew;
                    }
                    CurentRectMarker.Position = pnew;

                    if (CurentRectMarker.InnerMarker != null)
                    {
                        CurentRectMarker.InnerMarker.Position = pnew;
                    }
                }
            }

        }

        // MapZoomChanged
        void MainMap_OnMapZoomChanged()
        {
            if (MainMap.Zoom > 0)
            {
                try
                {
                    trackBar1.Value = (int)(MainMap.Zoom);
                }
                catch { }
                //textBoxZoomCurrent.Text = MainMap.Zoom.ToString();
                center.Position = MainMap.Position;
            }
        }

        // click on some marker
        void MainMap_OnMarkerClick(GMapMarker item, MouseEventArgs e)
        {
            int answer;
            try // when dragging item can sometimes be null
            {
                if (int.TryParse(item.Tag.ToString(), out answer))
                {

                    Commands.CurrentCell = Commands[0, answer - 1];

                }
            }
            catch { }
            return;
        }

        // loader start loading tiles
        void MainMap_OnTileLoadStart()
        {
            MethodInvoker m = delegate()
            {
                lbl_status.Text = "Status: loading tiles...";
            };
            try
            {
                BeginInvoke(m);
            }
            catch
            {
            }
        }

        // loader end loading tiles
        void MainMap_OnTileLoadComplete(long ElapsedMilliseconds)
        {

            //MainMap.ElapsedMilliseconds = ElapsedMilliseconds;

            MethodInvoker m = delegate()
            {
                lbl_status.Text = "Status: loaded tiles";

                //panelMenu.Text = "Menu, last load in " + MainMap.ElapsedMilliseconds + "ms";

                //textBoxMemory.Text = string.Format(CultureInfo.InvariantCulture, "{0:0.00}MB of {1:0.00}MB", MainMap.Manager.MemoryCacheSize, MainMap.Manager.MemoryCacheCapacity);
            };
            try
            {
                BeginInvoke(m);
            }
            catch
            {
            }

        }

        // current point changed
        void MainMap_OnCurrentPositionChanged(PointLatLng point)
        {
            if (point.Lat > 90) { point.Lat = 90; }
            if (point.Lat < -90) { point.Lat = -90; }
            if (point.Lng > 180) { point.Lng = 180; }
            if (point.Lng < -180) { point.Lng = -180; }
            center.Position = point;
            TXT_mouselat.Text = point.Lat.ToString(CultureInfo.InvariantCulture);
            TXT_mouselong.Text = point.Lng.ToString(CultureInfo.InvariantCulture);
        }

        // center markers on start
        private void MainForm_Load(object sender, EventArgs e)
        {
            if (objects.Markers.Count > 0)
            {
                MainMap.ZoomAndCenterMarkers(null);
            }
            trackBar1.Value = (int)MainMap.Zoom;
        }

        // ensure focus on map, trackbar can have it too
        private void MainMap_MouseEnter(object sender, EventArgs e)
        {
            // MainMap.Focus();
        }
        #endregion

        /// <summary>
        /// used to redraw the polygon
        /// </summary>
        void RegeneratePolygon()
        {
            List<PointLatLng> polygonPoints = new List<PointLatLng>();

            if (objects == null)
                return;

            foreach (GMapMarker m in objects.Markers)
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
                    lock (thisLock)
                    {
                        MainMap.UpdatePolygonLocalPosition(polygon);
                    }
                }
            }
        }

        private void comboBoxMapType_SelectedValueChanged(object sender, EventArgs e)
        {
            try
            {
                MainMap.MapType = (MapType)comboBoxMapType.SelectedItem;
                FlightData.mymap.MapType = (MapType)comboBoxMapType.SelectedItem;
                MainV2.config["MapType"] = comboBoxMapType.Text;
            }
            catch { MessageBox.Show("Map change failed. try zomming out first."); }
        }

        private void Commands_EditingControlShowing(object sender, DataGridViewEditingControlShowingEventArgs e)
        {
            if (e.Control.GetType() == typeof(DataGridViewComboBoxEditingControl))
            {
                var temp = ((ComboBox)e.Control);
                ((ComboBox)e.Control).SelectionChangeCommitted -= new EventHandler(Commands_SelectionChangeCommitted);
                ((ComboBox)e.Control).SelectionChangeCommitted += new EventHandler(Commands_SelectionChangeCommitted);
                ((ComboBox)e.Control).ForeColor = Color.White;
                ((ComboBox)e.Control).BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                System.Diagnostics.Debug.WriteLine("Setting event handle");
            }
        }

        void Commands_SelectionChangeCommitted(object sender, EventArgs e)
        {
            // update row headers
            ((ComboBox)sender).ForeColor = Color.White;
            ChangeColumnHeader(((ComboBox)sender).Text);
            try
            {
                for (int i = 0; i < Commands.ColumnCount; i++)
                {
                    DataGridViewCell tcell = Commands.Rows[selectedrow].Cells[i];
                    if (tcell.GetType() == typeof(DataGridViewTextBoxCell))
                    {
                        if (tcell.Value.ToString() == "?")
                            tcell.Value = "0";
                    }
                }
            }
            catch { }
        }
        /// <summary>
        /// Get the Google earth ALT for a given coord
        /// </summary>
        /// <param name="lat"></param>
        /// <param name="lng"></param>
        /// <returns>Altitude</returns>
        double getGEAlt(double lat, double lng)
        {
            double alt = 0;
            //http://maps.google.com/maps/api/elevation/xml

            try
            {
                using (XmlTextReader xmlreader = new XmlTextReader("http://maps.google.com/maps/api/elevation/xml?locations=" + lat.ToString(new System.Globalization.CultureInfo("en-US")) + "," + lng.ToString(new System.Globalization.CultureInfo("en-US")) + "&sensor=true"))
                {
                    while (xmlreader.Read())
                    {
                        xmlreader.MoveToElement();
                        switch (xmlreader.Name)
                        {
                            case "elevation":
                                alt = double.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                break;
                            default:
                                break;
                        }
                    }
                }
            }
            catch { }

            return alt * MainV2.cs.multiplierdist;
        }

        private void TXT_homelat_Enter(object sender, EventArgs e)
        {
            sethome = true;
            MessageBox.Show("Click on the Map to set Home ");
        }

        private void Planner_Resize(object sender, EventArgs e)
        {
            MainMap.Zoom = trackBar1.Value;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            writeKML();
            double homealt;
            double.TryParse(TXT_homealt.Text, out homealt);
            Form temp = new ElevationProfile(pointlist, homealt);
            MainV2.fixtheme(temp);
            temp.ShowDialog();
        }

        private void CHK_altmode_CheckedChanged(object sender, EventArgs e)
        {
            if (Commands.RowCount > 0 && !quickadd)
                MessageBox.Show("You will need to change your altitudes");
        }

        protected override void OnPaint(PaintEventArgs pe)
        {
            try
            {
                base.OnPaint(pe);
            }
            catch (Exception)
            {
            }
        }

        private void Commands_CellEndEdit(object sender, DataGridViewCellEventArgs e)
        {
            Commands_RowEnter(null, new DataGridViewCellEventArgs(Commands.CurrentCell.ColumnIndex, Commands.CurrentCell.RowIndex));
        }

        private void MainMap_Resize(object sender, EventArgs e)
        {
            MainMap.Zoom = MainMap.Zoom + 0.01;
        }

        private void trackBar1_Scroll(object sender, EventArgs e)
        {
            try
            {
                lock (thisLock)
                {
                    MainMap.Zoom = trackBar1.Value;
                }
            }
            catch { }
        }

        private void panel4_PanelCollapsing(object sender, BSE.Windows.Forms.XPanderStateChangeEventArgs e)
        {

        }

        private void BUT_Prefetch_Click(object sender, EventArgs e)
        {
            RectLatLng area = MainMap.SelectedArea;
            if (area.IsEmpty)
            {
                DialogResult res = MessageBox.Show("No ripp area defined, ripp displayed on screen?", "Rip", MessageBoxButtons.YesNo);
                if (res == DialogResult.Yes)
                {
                    area = MainMap.CurrentViewArea;
                }
            }

            if (!area.IsEmpty)
            {
                DialogResult res = MessageBox.Show("Ready ripp at Zoom = " + (int)MainMap.Zoom + " ?", "GMap.NET", MessageBoxButtons.YesNo);

                for (int i = 1; i <= MainMap.MaxZoom; i++)
                {
                    if (res == DialogResult.Yes)
                    {
                        TilePrefetcher obj = new TilePrefetcher();
                        obj.ShowCompleteMessage = false;
                        obj.Start(area, MainMap.Projection, i, MainMap.MapType, 100);
                    }
                    else if (res == DialogResult.No)
                    {
                        continue;
                    }
                    else if (res == DialogResult.Cancel)
                    {
                        break;
                    }
                }
            }
            else
            {
                MessageBox.Show("Select map area holding ALT", "GMap.NET", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
            }
        }

        /// <summary>
        /// from http://stackoverflow.com/questions/1119451/how-to-tell-if-a-line-intersects-a-polygon-in-c
        /// </summary>
        /// <param name="start1"></param>
        /// <param name="end1"></param>
        /// <param name="start2"></param>
        /// <param name="end2"></param>
        /// <returns></returns>
        public PointLatLng FindLineIntersection(PointLatLng start1, PointLatLng end1, PointLatLng start2, PointLatLng end2)
        {
            double denom = ((end1.Lng - start1.Lng) * (end2.Lat - start2.Lat)) - ((end1.Lat - start1.Lat) * (end2.Lng - start2.Lng));
            //  AB & CD are parallel         
            if (denom == 0)
                return PointLatLng.Zero;
            double numer = ((start1.Lat - start2.Lat) * (end2.Lng - start2.Lng)) - ((start1.Lng - start2.Lng) * (end2.Lat - start2.Lat));
            double r = numer / denom;
            double numer2 = ((start1.Lat - start2.Lat) * (end1.Lng - start1.Lng)) - ((start1.Lng - start2.Lng) * (end1.Lat - start1.Lat));
            double s = numer2 / denom;
            if ((r < 0 || r > 1) || (s < 0 || s > 1))
                return PointLatLng.Zero;
            // Find intersection point      
            PointLatLng result = new PointLatLng();
            result.Lng = start1.Lng + (r * (end1.Lng - start1.Lng));
            result.Lat = start1.Lat + (r * (end1.Lat - start1.Lat));
            return result;
        }

        RectLatLng getPolyMinMax(GMapPolygon poly)
        {
            if (poly.Points.Count == 0)
                return new RectLatLng();

            double minx, miny, maxx, maxy;

            minx = maxx = poly.Points[0].Lng;
            miny = maxy = poly.Points[0].Lat;

            foreach (PointLatLng pnt in poly.Points)
            {
                //Console.WriteLine(pnt.ToString());
                minx = Math.Min(minx, pnt.Lng);
                maxx = Math.Max(maxx, pnt.Lng);

                miny = Math.Min(miny, pnt.Lat);
                maxy = Math.Max(maxy, pnt.Lat);
            }

            return new RectLatLng(maxy, minx, Math.Abs(maxx - minx), Math.Abs(miny - maxy));
        }

        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        private void BUT_grid_Click(object sender, EventArgs e)
        {
            polygongridmode = false;

            if (drawnpolygon == null || drawnpolygon.Points.Count == 0)
            {
                MessageBox.Show("Right click the map to draw a polygon", "Area", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                return;
            }
            GMapPolygon area = drawnpolygon;
            area.Points.Add(area.Points[0]); // make a full loop
            RectLatLng arearect = getPolyMinMax(area);
            if (area.Distance > 0)
            {

                PointLatLng topright = new PointLatLng(arearect.LocationTopLeft.Lat, arearect.LocationRightBottom.Lng);
                PointLatLng bottomleft = new PointLatLng(arearect.LocationRightBottom.Lat, arearect.LocationTopLeft.Lng);

                double diagdist = MainMap.Manager.GetDistance(arearect.LocationTopLeft, arearect.LocationRightBottom) * 1000;
                double heightdist = MainMap.Manager.GetDistance(arearect.LocationTopLeft, bottomleft) * 1000;
                double widthdist = MainMap.Manager.GetDistance(arearect.LocationTopLeft, topright) * 1000;



                string alt = (100 * MainV2.cs.multiplierdist).ToString("0");
                Common.InputBox("Altitude", "Relative Altitude", ref alt);


                string distance = (50 * MainV2.cs.multiplierdist).ToString("0");
                Common.InputBox("Distance", "Distance between lines", ref distance);

                //string overshoot = (30 * MainV2.cs.multiplierdist).ToString("0");
                //Common.InputBox("Overshoot", "Enter of line overshoot amount", ref overshoot);

                string angle = (90).ToString("0");
                Common.InputBox("Angle", "Enter the line direction (0-180)", ref angle);

                double tryme = 0;

                if (!double.TryParse(angle, out tryme))
                {
                    MessageBox.Show("Invalid Angle");
                    return;
                }
                if (!double.TryParse(alt, out tryme))
                {
                    MessageBox.Show("Invalid Alt");
                    return;
                }
                if (!double.TryParse(distance, out tryme))
                {
                    MessageBox.Show("Invalid Distance");
                    return;
                }

#if DEBUG
                //Commands.Rows.Clear();
#endif
                // get x y components
                double x1 = Math.Cos((double.Parse(angle)) * deg2rad);
                double y1 = Math.Sin((double.Parse(angle)) * deg2rad);

                // get x y step amount in lat lng from m
                double latdiff = arearect.HeightLat / ((heightdist / (double.Parse(distance) * (y1) / MainV2.cs.multiplierdist)));
                double lngdiff = arearect.WidthLng / ((widthdist / (double.Parse(distance) * (x1) / MainV2.cs.multiplierdist)));

                double latlngdiff = Math.Sqrt(latdiff * latdiff + lngdiff * lngdiff);

                double fulllatdiff = arearect.HeightLat * x1 * 2;
                double fulllngdiff = arearect.WidthLng * y1 * 2;

                // lat - up down
                // lng - left right

                int overshootdist = 0;// (int)(double.Parse(overshoot) / MainV2.cs.multiplierdist);

                int altitude = (int)(double.Parse(alt) / MainV2.cs.multiplierdist);

                double overshootdistlng = arearect.WidthLng / widthdist * overshootdist;

                bool dir = false;

                int count = 0;

                double x = bottomleft.Lat - Math.Abs(fulllatdiff);
                double y = bottomleft.Lng - Math.Abs(fulllngdiff);

                Console.WriteLine("{0} < {1} {2} < {3}", x, (topright.Lat), y, (topright.Lng));

                while (x < (topright.Lat + Math.Abs(fulllatdiff)) && y < (topright.Lng + Math.Abs(fulllngdiff)))
                {
                    if (double.Parse(angle) < 45)
                    {
                        x = bottomleft.Lat;
                        y += latlngdiff;
                    }
                    else if (double.Parse(angle) > 135)
                    {
                        x = arearect.LocationTopLeft.Lat; //arearect.LocationTopLeft.Lat;
                        y += latlngdiff;
                    }
                    else if (double.Parse(angle) > 90)
                    {
                        y = bottomleft.Lng; //arearect.LocationTopLeft.Lat;
                        x += latlngdiff;
                    }
                    else
                    {
                        y = bottomleft.Lng;
                        x += latlngdiff;
                    }

                    //callMe(x , y, 0);
                    //callMe(x + (fulllatdiff), y + (fulllngdiff), 0);

                    //continue;

                    PointLatLng closestlatlong = PointLatLng.Zero;
                    PointLatLng farestlatlong = PointLatLng.Zero;

                    double noc = double.MaxValue;
                    double nof = double.MinValue;

                    if (dir)
                    {
                        double ax = x;
                        double ay = y;

                        double bx = x + fulllatdiff;
                        double by = y + fulllngdiff;
                        int a = -1;
                        PointLatLng newlatlong = PointLatLng.Zero;
                        foreach (PointLatLng pnt in area.Points)
                        {
                            a++;
                            if (a == 0)
                            {
                                continue;
                            }
                            newlatlong = FindLineIntersection(area.Points[a - 1], area.Points[a], new PointLatLng(ax, ay), new PointLatLng(bx, by));
                            if (!newlatlong.IsZero)
                            {
                                if (noc > MainMap.Manager.GetDistance(new PointLatLng(ax, ay), newlatlong))
                                {
                                    closestlatlong.Lat = newlatlong.Lat;
                                    closestlatlong.Lng = newlatlong.Lng;
                                    noc = MainMap.Manager.GetDistance(new PointLatLng(ax, ay), newlatlong);
                                }
                                if (nof < MainMap.Manager.GetDistance(new PointLatLng(ax, ay), newlatlong))
                                {
                                    farestlatlong.Lat = newlatlong.Lat;
                                    farestlatlong.Lng = newlatlong.Lng;
                                    nof = MainMap.Manager.GetDistance(new PointLatLng(ax, ay), newlatlong);
                                }
                            }
                        }

                        if (!farestlatlong.IsZero)
                            callMe(farestlatlong.Lat, farestlatlong.Lng, altitude);
                        if (!closestlatlong.IsZero)
                            callMe(closestlatlong.Lat, closestlatlong.Lng - overshootdistlng, altitude);

                        //callMe(x, topright.Lng, altitude);
                        //callMe(x, bottomleft.Lng - overshootdistlng, altitude);
                    }
                    else
                    {
                        double ax = x;
                        double ay = y;

                        double bx = x + fulllatdiff;
                        double by = y + fulllngdiff;
                        int a = -1;
                        PointLatLng newlatlong = PointLatLng.Zero;
                        foreach (PointLatLng pnt in area.Points)
                        {
                            a++;
                            if (a == 0)
                            {
                                continue;
                            }
                            newlatlong = FindLineIntersection(area.Points[a - 1], area.Points[a], new PointLatLng(ax, ay), new PointLatLng(bx, by));
                            if (!newlatlong.IsZero)
                            {
                                if (noc > MainMap.Manager.GetDistance(new PointLatLng(ax, ay), newlatlong))
                                {
                                    closestlatlong.Lat = newlatlong.Lat;
                                    closestlatlong.Lng = newlatlong.Lng;
                                    noc = MainMap.Manager.GetDistance(new PointLatLng(ax, ay), newlatlong);
                                }
                                if (nof < MainMap.Manager.GetDistance(new PointLatLng(ax, ay), newlatlong))
                                {
                                    farestlatlong.Lat = newlatlong.Lat;
                                    farestlatlong.Lng = newlatlong.Lng;
                                    nof = MainMap.Manager.GetDistance(new PointLatLng(ax, ay), newlatlong);
                                }
                            }
                        }
                        if (!closestlatlong.IsZero)
                            callMe(closestlatlong.Lat, closestlatlong.Lng, altitude);
                        if (!farestlatlong.IsZero)
                            callMe(farestlatlong.Lat, farestlatlong.Lng + overshootdistlng, altitude);
                        //callMe(x, bottomleft.Lng, altitude);
                        //callMe(x, topright.Lng + overshootdistlng, altitude);
                    }

                    dir = !dir;

                    count++;

                    if (Commands.RowCount > 150)
                    {
                        MessageBox.Show("Stopping at 150 WP's");
                        break;
                    }
                }

                //drawnpolygon.Points.Clear();
                //drawnpolygons.Markers.Clear();
                MainMap.Refresh();

            }
        }

        private void label4_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            if (MainV2.cs.lat != 0)
            {
                TXT_homealt.Text = (MainV2.cs.alt).ToString("0");
                TXT_homelat.Text = MainV2.cs.lat.ToString();
                TXT_homelng.Text = MainV2.cs.lng.ToString();
            }
            else
            {
                MessageBox.Show("If you're at the field, connect to your APM and wait for GPS lock. Then click 'Home Location' link to set home to your location");
            }
        }


        /// <summary>
        /// Format distance according to prefer distance unit
        /// </summary>
        /// <param name="distInKM">distance in kilometers</param>
        /// <param name="toMeterOrFeet">convert distance to meter or feet if true, covert to km or miles if false</param>
        /// <returns>formatted distance with unit</returns>
        private string FormatDistance(double distInKM, bool toMeterOrFeet)
        {
            string sunits = MainV2.getConfig("distunits");
            Common.distances units = Common.distances.Meters;

            if (sunits != "")
                try
                {
                    units = (Common.distances)Enum.Parse(typeof(Common.distances), sunits);
                }
                catch (Exception) { }

            switch (units)
            {
                case Common.distances.Feet:
                    return toMeterOrFeet ? string.Format((distInKM * 3280.8399).ToString("0.00 ft")) :
                        string.Format((distInKM * 0.621371).ToString("0.0000 miles"));
                case Common.distances.Meters:
                default:
                    return toMeterOrFeet ? string.Format((distInKM * 1000).ToString("0.00 m")) :
                        string.Format(distInKM.ToString("0.0000 km"));
            }
        }

        PointLatLng startmeasure = new PointLatLng();

        private void ContextMeasure_Click(object sender, EventArgs e)
        {
            if (startmeasure.IsZero)
            {
                startmeasure = start;
                polygons.Markers.Add(new GMapMarkerGoogleRed(start));
                MainMap.Invalidate();
            }
            else
            {
                List<PointLatLng> polygonPoints = new List<PointLatLng>();
                polygonPoints.Add(startmeasure);
                polygonPoints.Add(start);

                GMapPolygon line = new GMapPolygon(polygonPoints, "measure dist");
                line.Stroke.Color = Color.Green;

                polygons.Polygons.Add(line);

                polygons.Markers.Add(new GMapMarkerGoogleRed(start));
                MainMap.Invalidate();
                MessageBox.Show("Distance: " + FormatDistance(MainMap.Manager.GetDistance(startmeasure, start), true) + " AZ: " + (MainMap.Manager.GetBearing(startmeasure, start).ToString("0")));
                polygons.Polygons.Remove(line);
                polygons.Markers.Clear();
                startmeasure = new PointLatLng();
            }
        }

        private void rotateMapToolStripMenuItem_Click(object sender, EventArgs e)
        {
            string heading = "0";
            Common.InputBox("Rotate map to heading", "Enter new UP heading", ref heading);
            float ans = 0;
            if (float.TryParse(heading, out ans))
            {
                MainMap.Bearing = ans;
                FlightData.mymap.Bearing = ans;
            }
        }

        private void addPolygonPointToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (polygongridmode == false)
            {
                MessageBox.Show("You will remain in polygon mode until you clear the polygon or create a grid/upload a fence");
            }

            polygongridmode = true;

            List<PointLatLng> polygonPoints = new List<PointLatLng>();
            if (drawnpolygons.Polygons.Count == 0)
            {
                drawnpolygon.Points.Clear();
                drawnpolygons.Polygons.Add(drawnpolygon);
            }
            drawnpolygon.Points.Add(new PointLatLng(start.Lat, start.Lng));

            addpolygonmarkergrid(drawnpolygon.Points.Count.ToString(), start.Lng, start.Lat, 0);

            MainMap.UpdatePolygonLocalPosition(drawnpolygon);

            MainMap.Invalidate();

        }

        private void clearPolygonToolStripMenuItem_Click(object sender, EventArgs e)
        {
            polygongridmode = false;
            if (drawnpolygon == null)
                return;
            drawnpolygon.Points.Clear();
            drawnpolygons.Markers.Clear();
            MainMap.Invalidate();

            writeKML();
        }

        private void clearMissionToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Commands.Rows.Clear();
            selectedrow = 0;
            writeKML();
        }

        private void loiterForeverToolStripMenuItem_Click(object sender, EventArgs e)
        {
            selectedrow = Commands.Rows.Add();

            Commands.Rows[selectedrow].Cells[Command.Index].Value = MAVLink.MAV_CMD.LOITER_UNLIM.ToString();

            ChangeColumnHeader(MAVLink.MAV_CMD.LOITER_UNLIM.ToString());

            setfromGE(end.Lat, end.Lng, (int)float.Parse(TXT_DefaultAlt.Text));

            writeKML();
        }

        private void jumpstartToolStripMenuItem_Click(object sender, EventArgs e)
        {
            string repeat = "5";
            Common.InputBox("Jump repeat", "Number of times to Repeat", ref repeat);

            int row = Commands.Rows.Add();

            Commands.Rows[row].Cells[Command.Index].Value = MAVLink.MAV_CMD.DO_JUMP.ToString();

            Commands.Rows[row].Cells[Param1.Index].Value = 1;

            Commands.Rows[row].Cells[Param2.Index].Value = repeat;

            writeKML();
        }

        private void jumpwPToolStripMenuItem_Click(object sender, EventArgs e)
        {
            string wp = "1";
            Common.InputBox("WP No", "Jump to WP no?", ref wp);
            string repeat = "5";
            Common.InputBox("Jump repeat", "Number of times to Repeat", ref repeat);

            int row = Commands.Rows.Add();

            Commands.Rows[row].Cells[Command.Index].Value = MAVLink.MAV_CMD.DO_JUMP.ToString();

            Commands.Rows[row].Cells[Param1.Index].Value = wp;

            Commands.Rows[row].Cells[Param2.Index].Value = repeat;

            writeKML();
        }

        private void deleteWPToolStripMenuItem_Click(object sender, EventArgs e)
        {
            int no = 0;
            if (CurentRectMarker != null)
            {
                if (int.TryParse(CurentRectMarker.InnerMarker.Tag.ToString(), out no))
                {
                    Commands.Rows.RemoveAt(no - 1); // home is 0
                }
                else if (int.TryParse(CurentRectMarker.InnerMarker.Tag.ToString().Replace("grid", ""), out no))
                {
                    try
                    {
                        drawnpolygon.Points.RemoveAt(no - 1);
                        drawnpolygons.Markers.Clear();

                        int a = 1;
                        foreach (PointLatLng pnt in drawnpolygon.Points)
                        {
                            addpolygonmarkergrid(a.ToString(), pnt.Lng, pnt.Lat, 0);
                            a++;
                        }

                        MainMap.UpdatePolygonLocalPosition(drawnpolygon);

                        MainMap.Invalidate();
                    }
                    catch {
                        MessageBox.Show("Remove point Failed. Please try again.");
                    }
                }
            }

            CurentRectMarker = null;

            writeKML();
        }

        private void loitertimeToolStripMenuItem_Click(object sender, EventArgs e)
        {
            string time = "5";
            Common.InputBox("Loiter Time", "Loiter Time", ref time);

            selectedrow = Commands.Rows.Add();

            Commands.Rows[selectedrow].Cells[Command.Index].Value = MAVLink.MAV_CMD.LOITER_TIME.ToString();

            Commands.Rows[selectedrow].Cells[Param1.Index].Value = time;

            ChangeColumnHeader(MAVLink.MAV_CMD.LOITER_TIME.ToString());

            setfromGE(end.Lat, end.Lng, (int)float.Parse(TXT_DefaultAlt.Text));

            writeKML();
        }

        private void loitercirclesToolStripMenuItem_Click(object sender, EventArgs e)
        {
            string turns = "3";
            Common.InputBox("Loiter Turns", "Loiter Turns", ref turns);

            selectedrow = Commands.Rows.Add();

            Commands.Rows[selectedrow].Cells[Command.Index].Value = MAVLink.MAV_CMD.LOITER_TURNS.ToString();

            Commands.Rows[selectedrow].Cells[Param1.Index].Value = turns;

            ChangeColumnHeader(MAVLink.MAV_CMD.LOITER_TURNS.ToString());

            setfromGE(end.Lat, end.Lng, (int)float.Parse(TXT_DefaultAlt.Text));

            writeKML();
        }

        private void BUT_Camera_Click(object sender, EventArgs e)
        {
            Camera form = new Camera();
            MainV2.fixtheme(form);
            form.Show();
        }

        private void panelMap_Resize(object sender, EventArgs e)
        {
            // this is a mono fix for the zoom bar
            //Console.WriteLine("panelmap "+panelMap.Size.ToString());
            MainMap.Size = new Size(panelMap.Size.Width - 50, panelMap.Size.Height);
            trackBar1.Location = new System.Drawing.Point(panelMap.Size.Width - 50, trackBar1.Location.Y);
            trackBar1.Size = new System.Drawing.Size(trackBar1.Size.Width, panelMap.Size.Height - trackBar1.Location.Y);
            label11.Location = new System.Drawing.Point(panelMap.Size.Width - 50, label11.Location.Y);
        }

        private void BUT_zoomto_Click(object sender, EventArgs e)
        {
            string place = "Perth Airport, Australia";
            if (DialogResult.OK == Common.InputBox("Location", "Enter your location", ref place))
            {

                GeoCoderStatusCode status = MainMap.SetCurrentPositionByKeywords(place);
                if (status != GeoCoderStatusCode.G_GEO_SUCCESS)
                {
                    MessageBox.Show("Google Maps Geocoder can't find: '" + place + "', reason: " + status.ToString(), "GMap.NET", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                }
                else
                {
                    MainMap.Zoom = 15;
                }
            }
        }

        private void BUT_loadkml_Click(object sender, EventArgs e)
        {

            OpenFileDialog fd = new OpenFileDialog();
            fd.Filter = "Google Earth KML (*.kml)|*.kml";
            fd.DefaultExt = ".kml";
            DialogResult result = fd.ShowDialog();
            string file = fd.FileName;
            if (file != "")
            {
                try
                {
                    kmlpolygons.Polygons.Clear();
                    kmlpolygons.Routes.Clear();

                    FlightData.kmlpolygons.Routes.Clear();
                    FlightData.kmlpolygons.Polygons.Clear();

                    string kml = new StreamReader(File.OpenRead(file)).ReadToEnd();

                    kml = kml.Replace("<Snippet/>", "");

                    var parser = new SharpKml.Base.Parser();

                    parser.ElementAdded += parser_ElementAdded;
                    parser.ParseString(kml, true);

                    if (DialogResult.Yes == MessageBox.Show("Do you want to load this into the flight data screen?", "Load data", MessageBoxButtons.YesNo))
                    {
                        foreach (var temp in kmlpolygons.Polygons)
                        {
                            FlightData.kmlpolygons.Polygons.Add(temp);
                        }
                        foreach (var temp in kmlpolygons.Routes)
                        {
                            FlightData.kmlpolygons.Routes.Add(temp);
                        }
                    }

                }
                catch (Exception ex) { MessageBox.Show("Bad KML File :" + ex.ToString()); }
            }

        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            try
            {
                routes.Markers.Clear();

                if (MainV2.cs.lat == 0 || MainV2.cs.lng == 0)
                    return;

                PointLatLng currentloc = new PointLatLng(MainV2.cs.lat, MainV2.cs.lng);

                if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
                {
                    routes.Markers.Add(new GMapMarkerPlane(currentloc, MainV2.cs.yaw, MainV2.cs.groundcourse, MainV2.cs.nav_bearing, MainV2.cs.target_bearing) { ToolTipText = MainV2.cs.alt.ToString("0"), ToolTipMode = MarkerTooltipMode.Always });
                }
                else
                {
                    routes.Markers.Add(new GMapMarkerQuad(currentloc, MainV2.cs.yaw, MainV2.cs.groundcourse, MainV2.cs.nav_bearing));
                }
            }
            catch { }
        }

        private void GeoFenceuploadToolStripMenuItem_Click(object sender, EventArgs e)
        {
            polygongridmode = false;
            //FENCE_TOTAL
            if (MainV2.comPort.param["FENCE_ACTION"] == null)
            {
                MessageBox.Show("Not Supported");
                return;
            }

            if (drawnpolygon == null)
            {
                MessageBox.Show("No polygon to upload");
                return;
            }

            if (geofence.Markers.Count == 0)
            {
                MessageBox.Show("No return location set");
                return;
            }

            if (drawnpolygon.Points.Count == 0)
            {
                MessageBox.Show("No polygon drawn");
                return;
            }

            // check if return is inside polygon
            List<PointLatLng> plll = new List<PointLatLng>(drawnpolygon.Points.ToArray());
            // close it
            plll.Add(plll[0]);
            // check it
            if (!pnpoly(plll.ToArray(), geofence.Markers[0].Position.Lat, geofence.Markers[0].Position.Lng))
            {
                MessageBox.Show("Your return location is outside the polygon");
                return;
            }

            string minalts = (int.Parse(MainV2.comPort.param["FENCE_MINALT"].ToString()) * MainV2.cs.multiplierdist).ToString("0");
            Common.InputBox("Min Alt", "Box Minimum Altitude?", ref minalts);

            string maxalts = (int.Parse(MainV2.comPort.param["FENCE_MAXALT"].ToString()) * MainV2.cs.multiplierdist).ToString("0");
            Common.InputBox("Max Alt", "Box Maximum Altitude?", ref maxalts);

            int minalt = 0;
            int maxalt = 0;

            if (!int.TryParse(minalts, out minalt))
            {
                MessageBox.Show("Bad Min Alt");
                return;
            }

            if (!int.TryParse(maxalts, out maxalt))
            {
                MessageBox.Show("Bad Max Alt");
                return;
            }

            try
            {
                MainV2.comPort.setParam("FENCE_MINALT", minalt);
                MainV2.comPort.setParam("FENCE_MAXALT", maxalt);
            }
            catch
            {
                MessageBox.Show("Failed to set min/max fence alt");
                return;
            }

            try
            {
                if (MainV2.comPort.param["FENCE_ACTION"].ToString() != "0")
                    MainV2.comPort.setParam("FENCE_ACTION", 0);
            }
            catch
            {
                MessageBox.Show("Failed to set FENCE_ACTION");
                return;
            }

            // points + return + close
            byte pointcount = (byte)(drawnpolygon.Points.Count + 2);

            MainV2.comPort.setParam("FENCE_TOTAL", pointcount);

            byte a = 0;

            // add return loc
            MainV2.comPort.setFencePoint(a, new PointLatLngAlt(geofence.Markers[0].Position), pointcount);
            a++;

            // add points
            foreach (var pll in drawnpolygon.Points)
            {
                MainV2.comPort.setFencePoint(a, new PointLatLngAlt(pll), pointcount);
                a++;
            }
            // add polygon close
            MainV2.comPort.setFencePoint(a, new PointLatLngAlt(drawnpolygon.Points[0]), pointcount);

            // clear everything
            drawnpolygons.Polygons.Clear();
            drawnpolygons.Markers.Clear();
            geofence.Polygons.Clear();
            gf.Points.Clear();

            // add polygon
            gf.Points.AddRange(drawnpolygon.Points.ToArray());

            drawnpolygon.Points.Clear();

            geofence.Polygons.Add(gf);

            // update flightdata
            FlightData.geofence.Markers.Clear();
            FlightData.geofence.Polygons.Clear();
            FlightData.geofence.Polygons.Add(new GMapPolygon(gf.Points, "gf fd") { Stroke = gf.Stroke });
            FlightData.geofence.Markers.Add(new GMapMarkerGoogleRed(geofence.Markers[0].Position) { ToolTipText = geofence.Markers[0].ToolTipText, ToolTipMode = geofence.Markers[0].ToolTipMode });

            MainMap.UpdatePolygonLocalPosition(gf);
            MainMap.UpdateMarkerLocalPosition(geofence.Markers[0]);

            MainMap.Invalidate();
        }

        private void GeoFencedownloadToolStripMenuItem_Click(object sender, EventArgs e)
        {
            polygongridmode = false;
            int count = 1;

            if (MainV2.comPort.param["FENCE_ACTION"] == null || MainV2.comPort.param["FENCE_TOTAL"] == null)
            {
                MessageBox.Show("Not Supported");
                return;
            }

            if (int.Parse(MainV2.comPort.param["FENCE_TOTAL"].ToString()) <= 1)
            {
                MessageBox.Show("Nothing to download");
                return;
            }

            geofence.Polygons.Clear();
            geofence.Markers.Clear();
            gf.Points.Clear();


            for (int a = 0; a < count; a++)
            {
                PointLatLngAlt plla = MainV2.comPort.getFencePoint(a, ref count);
                gf.Points.Add(new PointLatLng(plla.Lat, plla.Lng));
            }

            // do return location
            geofence.Markers.Add(new GMapMarkerGoogleRed(new PointLatLng(gf.Points[0].Lat, gf.Points[0].Lng)) { ToolTipMode = MarkerTooltipMode.OnMouseOver, ToolTipText = "GeoFence Return" });
            gf.Points.RemoveAt(0);

            // add now - so local points are calced
            geofence.Polygons.Add(gf);

            // update flight data
            FlightData.geofence.Markers.Clear();
            FlightData.geofence.Polygons.Clear();
            FlightData.geofence.Polygons.Add(new GMapPolygon(gf.Points, "gf fd") { Stroke = gf.Stroke });
            FlightData.geofence.Markers.Add(new GMapMarkerGoogleRed(geofence.Markers[0].Position) { ToolTipText = geofence.Markers[0].ToolTipText, ToolTipMode = geofence.Markers[0].ToolTipMode });

            MainMap.UpdatePolygonLocalPosition(gf);
            MainMap.UpdateMarkerLocalPosition(geofence.Markers[0]);

            MainMap.Invalidate();
        }

        private void setReturnLocationToolStripMenuItem_Click(object sender, EventArgs e)
        {
            geofence.Markers.Clear();
            geofence.Markers.Add(new GMapMarkerGoogleRed(new PointLatLng(start.Lat, start.Lng)) { ToolTipMode = MarkerTooltipMode.OnMouseOver, ToolTipText = "GeoFence Return" });

            MainMap.Invalidate();
        }

        /// <summary>
        /// from http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
        /// </summary>
        /// <param name="array"> a closed polygon</param>
        /// <param name="testx"></param>
        /// <param name="testy"></param>
        /// <returns> true = outside</returns>
        bool pnpoly(PointLatLng[] array, double testx, double testy)
        {
            int nvert = array.Length;
            int i, j = 0;
            bool c = false;
            for (i = 0, j = nvert - 1; i < nvert; j = i++)
            {
                if (((array[i].Lng > testy) != (array[j].Lng > testy)) &&
                 (testx < (array[j].Lat - array[i].Lat) * (testy - array[i].Lng) / (array[j].Lng - array[i].Lng) + array[i].Lat))
                    c = !c;
            }
            return c;
        }

        private void loadFromFileToolStripMenuItem_Click(object sender, EventArgs e)
        {
            OpenFileDialog fd = new OpenFileDialog();
            fd.Filter = "Fence (*.fen)|*.fen";
            fd.ShowDialog();
            if (File.Exists(fd.FileName))
            {
                StreamReader sr = new StreamReader(fd.OpenFile());

                drawnpolygons.Markers.Clear();
                drawnpolygons.Polygons.Clear();
                drawnpolygon.Points.Clear();

                int a = 0;

                while (!sr.EndOfStream)
                {
                    string line = sr.ReadLine();
                    if (line.StartsWith("#"))
                    {
                        continue;
                    }
                    else
                    {
                        string[] items = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);

                        if (a == 0)
                        {
                            geofence.Markers.Clear();
                            geofence.Markers.Add(new GMapMarkerGoogleRed(new PointLatLng(double.Parse(items[0]), double.Parse(items[1]))) { ToolTipMode = MarkerTooltipMode.OnMouseOver, ToolTipText = "GeoFence Return" });
                            MainMap.UpdateMarkerLocalPosition(geofence.Markers[0]);
                        }
                        else
                        {
                            drawnpolygon.Points.Add(new PointLatLng(double.Parse(items[0]), double.Parse(items[1])));
                            addpolygonmarkergrid(drawnpolygon.Points.Count.ToString(), double.Parse(items[1]), double.Parse(items[0]), 0);
                        }
                        a++;
                    }
                }

                // remove loop close
                if (drawnpolygon.Points[0] == drawnpolygon.Points[drawnpolygon.Points.Count - 1])
                {
                    drawnpolygon.Points.RemoveAt(drawnpolygon.Points.Count - 1);
                }

                drawnpolygons.Polygons.Add(drawnpolygon);

                MainMap.UpdatePolygonLocalPosition(drawnpolygon);

                MainMap.Invalidate();
            }
        }

        private void saveToFileToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (geofence.Markers.Count == 0)
            {
                MessageBox.Show("Please set a return location");
                return;
            }


            SaveFileDialog sf = new SaveFileDialog();
            sf.Filter = "Fence (*.fen)|*.fen";
            sf.ShowDialog();
            if (sf.FileName != "")
            {
                try
                {
                    StreamWriter sw = new StreamWriter(sf.OpenFile());

                    sw.WriteLine("#saved by APM Planner " + Application.ProductVersion);

                    sw.WriteLine(geofence.Markers[0].Position.Lat + " " + geofence.Markers[0].Position.Lng);
                    if (drawnpolygon.Points.Count > 0)
                    {
                        foreach (var pll in drawnpolygon.Points)
                        {
                            sw.WriteLine(pll.Lat + " " + pll.Lng);
                        }

                        PointLatLng pll2 = drawnpolygon.Points[0];

                        sw.WriteLine(pll2.Lat + " " + pll2.Lng);
                    }
                    else
                    {
                        foreach (var pll in gf.Points)
                        {
                            sw.WriteLine(pll.Lat + " " + pll.Lng);
                        }

                        PointLatLng pll2 = gf.Points[0];

                        sw.WriteLine(pll2.Lat + " " + pll2.Lng);
                    }

                    sw.Close();
                }
                catch { MessageBox.Show("Failed to write fence file"); }
            }
        }

        public T DeepClone<T>(T obj)
        {
            using (var ms = new System.IO.MemoryStream())
            {
                var formatter = new System.Runtime.Serialization.Formatters.Binary.BinaryFormatter();

                formatter.Serialize(ms, obj);

                ms.Position = 0;

                return (T)formatter.Deserialize(ms);
            }
        }
    }
}
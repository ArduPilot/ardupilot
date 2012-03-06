using System;
using System.Collections.Generic;
using System.Collections;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;
using System.Text.RegularExpressions;
//using KMLib;
//using KMLib.Feature;
//using KMLib.Geometry;
//using Core.Geometry;
using ICSharpCode.SharpZipLib.Zip;
using ICSharpCode.SharpZipLib.Checksums;
using ICSharpCode.SharpZipLib.Core;

using SharpKml.Base;
using SharpKml.Dom;
using SharpKml.Dom.GX;

using System.Reflection;
using System.Xml;
using log4net;
using ZedGraph; // Graphs

namespace ArdupilotMega
{
    public partial class MavlinkLog : Form
    {
        private static readonly ILog log = LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        List<CurrentState> flightdata = new List<CurrentState>();

        List<string> selection = new List<string>();

        public MavlinkLog()
        {
            InitializeComponent();
        }

        private void writeKML(string filename)
        {
            SharpKml.Dom.AltitudeMode altmode = SharpKml.Dom.AltitudeMode.Absolute;

            if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
                altmode = SharpKml.Dom.AltitudeMode.Absolute;
            }
            else if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2)
            {
                altmode = SharpKml.Dom.AltitudeMode.RelativeToGround;
            }

            Color[] colours = { Color.Red, Color.Orange, Color.Yellow, Color.Green, Color.Blue, Color.Indigo, Color.Violet, Color.Pink };

            Document kml = new Document();

            Tour tour = new Tour() { Name = "First Person View" };
            Playlist tourplaylist = new Playlist();

            AddNamespace(kml, "gx", "http://www.google.com/kml/ext/2.2");

            Style style = new Style();
            style.Id = "yellowLineGreenPoly";
            style.Line = new LineStyle(new Color32(HexStringToColor("7f00ffff")), 4);



            PolygonStyle pstyle = new PolygonStyle();
            pstyle.Color = new Color32(HexStringToColor("7f00ff00"));
            style.Polygon = pstyle;

            kml.AddStyle(style);

            Style stylet = new Style();
            stylet.Id = "track";
            SharpKml.Dom.IconStyle ico = new SharpKml.Dom.IconStyle();
            LabelStyle lst = new LabelStyle();
            lst.Scale = 0;
            stylet.Icon = ico;
            ico.Icon = new IconStyle.IconLink(new Uri("http://earth.google.com/images/kml-icons/track-directional/track-none.png"));
            stylet.Icon.Scale = 0.5;
            stylet.Label = lst;

            kml.AddStyle(stylet);

            // create sub folders
            Folder planes = new Folder();
            planes.Name = "Planes";
            kml.AddFeature(planes);

            Folder points = new Folder();
            points.Name = "Points";
            kml.AddFeature(points);


            // coords for line string
            CoordinateCollection coords = new CoordinateCollection();

            int a = 1;
            int c = -1;
            DateTime lasttime = DateTime.MaxValue;
            DateTime starttime = DateTime.MinValue;
            Color stylecolor = Color.AliceBlue;
            string mode = "";
            if (flightdata.Count > 0)
            {
                mode = flightdata[0].mode;
            }
            foreach (CurrentState cs in flightdata)
            {
                progressBar1.Value = 50 + (int)((float)a / (float)flightdata.Count * 100.0f / 2.0f);
                progressBar1.Refresh();

                if (starttime == DateTime.MinValue)
                {
                    starttime = cs.datetime;
                    lasttime = cs.datetime;
                }

                if (mode != cs.mode || flightdata.Count == a)
                {
                    c++;

                    LineString ls = new LineString();
                    ls.AltitudeMode = altmode;
                    ls.Extrude = true;

                    ls.Coordinates = coords;

                    Placemark pm = new Placemark();

                    pm.Name = c + " Flight Path " + mode;
                    pm.StyleUrl = new Uri("#yellowLineGreenPoly", UriKind.Relative);
                    pm.Geometry = ls;

                    SharpKml.Dom.TimeSpan ts = new SharpKml.Dom.TimeSpan();
                    ts.Begin = starttime;
                    ts.End = cs.datetime;

                    pm.Time = ts;

                    // setup for next line
                    mode = cs.mode;
                    starttime = cs.datetime;

                    stylecolor = colours[c % (colours.Length - 1)];

                    Style style2 = new Style();
                    style2.Line = new LineStyle(new Color32(stylecolor), 4);

                    pm.StyleSelector = style2;

                    kml.AddFeature(pm);

                    coords = new CoordinateCollection();
                }

                coords.Add(new Vector(cs.lat, cs.lng, cs.alt));

                SharpKml.Dom.Timestamp tstamp = new SharpKml.Dom.Timestamp();
                tstamp.When = cs.datetime;

                FlyTo flyto = new FlyTo();

                flyto.Duration = (cs.datetime - lasttime).TotalMilliseconds / 1000.0;

                flyto.Mode = FlyToMode.Smooth;
                SharpKml.Dom.Camera cam = new SharpKml.Dom.Camera();
                cam.AltitudeMode = altmode;
                cam.Latitude = cs.lat;
                cam.Longitude = cs.lng;
                cam.Altitude = cs.alt;
                cam.Heading = cs.yaw;
                cam.Roll = -cs.roll;
                cam.Tilt = (90 - (cs.pitch * -1));

                cam.GXTimePrimitive = tstamp;

                flyto.View = cam;
                //if (Math.Abs(flyto.Duration.Value) > 0.1)
                {
                    tourplaylist.AddTourPrimitive(flyto);
                    lasttime = cs.datetime;
                }


                Placemark pmplane = new Placemark();
                pmplane.Name = "Point " + a;



                pmplane.Time = tstamp;

                pmplane.Visibility = false;

                SharpKml.Dom.Location loc = new SharpKml.Dom.Location();
                loc.Latitude = cs.lat;
                loc.Longitude = cs.lng;
                loc.Altitude = cs.alt;

                if (loc.Altitude < 0)
                    loc.Altitude = 0.01;

                SharpKml.Dom.Orientation ori = new SharpKml.Dom.Orientation();
                ori.Heading = cs.yaw;
                ori.Roll = -cs.roll;
                ori.Tilt = -cs.pitch;

                SharpKml.Dom.Scale sca = new SharpKml.Dom.Scale();

                sca.X = 2;
                sca.Y = 2;
                sca.Z = 2;

                Model model = new Model();
                model.Location = loc;
                model.Orientation = ori;
                model.AltitudeMode = altmode;
                model.Scale = sca;

                try
                {
                    Description desc = new Description();
                    desc.Text = @"<![CDATA[
              <table>
                <tr><td>Roll: " + model.Orientation.Roll.Value.ToString("0.00") + @" </td></tr>
                <tr><td>Pitch: " + model.Orientation.Tilt.Value.ToString("0.00") + @" </td></tr>
                <tr><td>Yaw: " + model.Orientation.Heading.Value.ToString("0.00") + @" </td></tr>
                <tr><td>Time: " + cs.datetime.ToString("HH:mm:sszzzzzz") + @" </td></tr>
              </table> ";
//            ]]>";

                    pmplane.Description = desc;
                }
                catch { }

                SharpKml.Dom.Link link = new SharpKml.Dom.Link();
                link.Href = new Uri("block_plane_0.dae", UriKind.Relative);

                model.Link = link;

                pmplane.Geometry = model;

                planes.AddFeature(pmplane);

                ///

                Placemark pmt = new Placemark();

                SharpKml.Dom.Point pnt = new SharpKml.Dom.Point();
                pnt.AltitudeMode = altmode;
                pnt.Coordinate = new Vector(cs.lat,cs.lng,cs.alt);

                pmt.Name = "" + a;

                pmt.Description = pmplane.Description;

                pmt.Time = tstamp;

                pmt.Geometry = pnt;
                pmt.StyleUrl = new Uri("#track", UriKind.Relative);

                points.AddFeature(pmt);

                a++;
            }

            tour.Playlist = tourplaylist;

            kml.AddFeature(tour);

            Serializer serializer = new Serializer();
            serializer.Serialize(kml);


            //Console.WriteLine(serializer.Xml);

            StreamWriter sw = new StreamWriter(filename);
            sw.Write(serializer.Xml);
            sw.Close();

            // create kmz - aka zip file

            FileStream fs = File.Open(filename.Replace(Path.GetExtension(filename), ".kmz"), FileMode.Create);
            ZipOutputStream zipStream = new ZipOutputStream(fs);
            zipStream.SetLevel(9); //0-9, 9 being the highest level of compression
            zipStream.UseZip64 = UseZip64.Off; // older zipfile

            // entry 1
            string entryName = ZipEntry.CleanName(Path.GetFileName(filename)); // Removes drive from name and fixes slash direction
            ZipEntry newEntry = new ZipEntry(entryName);
            newEntry.DateTime = DateTime.Now;

            zipStream.PutNextEntry(newEntry);

            // Zip the file in buffered chunks
            // the "using" will close the stream even if an exception occurs
            byte[] buffer = new byte[4096];
            using (FileStream streamReader = File.Open(filename,FileMode.Open,FileAccess.Read,FileShare.ReadWrite))
            {
                StreamUtils.Copy(streamReader, zipStream, buffer);
            }
            zipStream.CloseEntry();

            filename = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "block_plane_0.dae";

            // entry 2
            entryName = ZipEntry.CleanName(Path.GetFileName(filename)); // Removes drive from name and fixes slash direction
            newEntry = new ZipEntry(entryName);
            newEntry.DateTime = DateTime.Now;

            zipStream.PutNextEntry(newEntry);

            // Zip the file in buffered chunks
            // the "using" will close the stream even if an exception occurs
            buffer = new byte[4096];
            using (FileStream streamReader = File.OpenRead(filename))
            {
                StreamUtils.Copy(streamReader, zipStream, buffer);
            }
            zipStream.CloseEntry();


            zipStream.IsStreamOwner = true;	// Makes the Close also Close the underlying stream
            zipStream.Close();

            flightdata.Clear();
        }

        static void AddNamespace(Element element, string prefix, string uri)
        {
            // The Namespaces property is marked as internal.
            PropertyInfo property = typeof(Element).GetProperty(
                "Namespaces",
                BindingFlags.Instance | BindingFlags.NonPublic);

            var namespaces = (XmlNamespaceManager)property.GetValue(element, null);
            namespaces.AddNamespace(prefix, uri);
        }

        private void Log_FormClosing(object sender, FormClosingEventArgs e)
        {

        }

        private void BUT_redokml_Click(object sender, EventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.Filter = "*.tlog|*.tlog";
            openFileDialog1.FilterIndex = 2;
            openFileDialog1.RestoreDirectory = true;
            openFileDialog1.Multiselect = true;
            try
            {
                openFileDialog1.InitialDirectory = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs" + Path.DirectorySeparatorChar;
            }
            catch { } // incase dir doesnt exist

            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                foreach (string logfile in openFileDialog1.FileNames)
                {

                    MAVLink mine = new MAVLink();
                    mine.logplaybackfile = new BinaryReader(File.Open(logfile, FileMode.Open, FileAccess.Read, FileShare.Read));
                    mine.logreadmode = true;

                    mine.packets.Initialize(); // clear

                    CurrentState cs = new CurrentState();

                    float oldlatlngalt = 0;

                    int appui = 0;

                    while (mine.logplaybackfile.BaseStream.Position < mine.logplaybackfile.BaseStream.Length)
                    {
                        // bar moves to 50 % in this step
                        progressBar1.Value = (int)((float)mine.logplaybackfile.BaseStream.Position / (float)mine.logplaybackfile.BaseStream.Length * 100.0f / 2.0f);
                        progressBar1.Invalidate();
                        progressBar1.Refresh();

                        byte[] packet = mine.readPacket();

                        cs.datetime = mine.lastlogread;

                        cs.UpdateCurrentSettings(null, true, mine);

                        if (appui != DateTime.Now.Second)
                        {
                            // cant do entire app as mixes with flightdata timer
                            this.Refresh();
                            appui = DateTime.Now.Second;
                        }

                        try
                        {
                            if (MainV2.talk != null)
                                MainV2.talk.SpeakAsyncCancelAll();
                        }
                        catch { } // ignore because of this Exception System.PlatformNotSupportedException: No voice installed on the system or none available with the current security setting.

                        if ((float)(cs.lat + cs.lng) != oldlatlngalt
                            && cs.lat != 0 && cs.lng != 0)
                        {
                            Console.WriteLine(cs.lat + " " + cs.lng + " " + cs.alt + "   lah " + (float)(cs.lat + cs.lng) + "!=" + oldlatlngalt);
                            CurrentState cs2 = (CurrentState)cs.Clone();

                            flightdata.Add(cs2);

                            oldlatlngalt = (cs.lat + cs.lng);
                        }
                    }

                    mine.logreadmode = false;
                    mine.logplaybackfile.Close();
                    mine.logplaybackfile = null;

                    Application.DoEvents();

                    writeKML(logfile + ".kml");

                    progressBar1.Value = 100;

                }
            }
        }

        public static Color HexStringToColor(string hexColor)
        {
            string hc = (hexColor);
            if (hc.Length != 8)
            {
                // you can choose whether to throw an exception
                //throw new ArgumentException("hexColor is not exactly 6 digits.");
                return Color.Empty;
            }
            string a = hc.Substring(0, 2);
            string r = hc.Substring(6, 2);
            string g = hc.Substring(4, 2);
            string b = hc.Substring(2, 2);
            Color color = Color.Empty;
            try
            {
                int ai
                   = Int32.Parse(a, System.Globalization.NumberStyles.HexNumber);
                int ri
                   = Int32.Parse(r, System.Globalization.NumberStyles.HexNumber);
                int gi
                   = Int32.Parse(g, System.Globalization.NumberStyles.HexNumber);
                int bi
                   = Int32.Parse(b, System.Globalization.NumberStyles.HexNumber);
                color = Color.FromArgb(ai, ri, gi, bi);
            }
            catch
            {
                // you can choose whether to throw an exception
                //throw new ArgumentException("Conversion failed.");
                return Color.Empty;
            }
            return color;
        }

        private void BUT_humanreadable_Click(object sender, EventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.Filter = "*.tlog|*.tlog";
            openFileDialog1.FilterIndex = 2;
            openFileDialog1.RestoreDirectory = true;
            openFileDialog1.Multiselect = true;
            try
            {
                openFileDialog1.InitialDirectory = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs" + Path.DirectorySeparatorChar;
            }
            catch { } // incase dir doesnt exist

            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                foreach (string logfile in openFileDialog1.FileNames)
                {

                    MAVLink mine = new MAVLink();
                    mine.logplaybackfile = new BinaryReader(File.Open(logfile, FileMode.Open, FileAccess.Read, FileShare.Read));
                    mine.logreadmode = true;

                    mine.packets.Initialize(); // clear

                    StreamWriter sw = new StreamWriter(Path.GetDirectoryName(logfile)+ Path.DirectorySeparatorChar + Path.GetFileNameWithoutExtension(logfile) + ".txt");

                    while (mine.logplaybackfile.BaseStream.Position < mine.logplaybackfile.BaseStream.Length)
                    {
                        // bar moves to 100 % in this step
                        progressBar1.Value = (int)((float)mine.logplaybackfile.BaseStream.Position / (float)mine.logplaybackfile.BaseStream.Length * 100.0f / 1.0f);

                        progressBar1.Refresh();
                        //Application.DoEvents();

                        byte[] packet = mine.readPacket();
                        string text = "";
                        mine.DebugPacket(packet, ref text);

                        sw.Write(mine.lastlogread +" "+text);
                    }

                    sw.Close();

                    progressBar1.Value = 100;

                    mine.logreadmode = false;
                    mine.logplaybackfile.Close();
                    mine.logplaybackfile = null;

                }
            }
        }

        private void BUT_graphmavlog_Click(object sender, EventArgs e)
        {

            //http://devreminder.wordpress.com/net/net-framework-fundamentals/c-dynamic-math-expression-evaluation/
            //http://www.c-sharpcorner.com/UploadFile/mgold/CodeDomCalculator08082005003253AM/CodeDomCalculator.aspx

//string mathExpression = "(1+1)*3";
            //Console.WriteLine(String.Format("{0}={1}",mathExpression, Evaluate(mathExpression)));


            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.Filter = "*.tlog|*.tlog";
            openFileDialog1.FilterIndex = 2;
            openFileDialog1.RestoreDirectory = true;
            openFileDialog1.Multiselect = false;
            try
            {
                openFileDialog1.InitialDirectory = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs" + Path.DirectorySeparatorChar;
            }
            catch { } // incase dir doesnt exist

            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                List<string> fields = GetLogFileValidFields(openFileDialog1.FileName);

                zg1.GraphPane.CurveList.Clear();

                GetLogFileData(zg1, openFileDialog1.FileName, fields);

                try
                {
                    // fix new line types
                    ThemeManager.ApplyThemeTo(this);

                    zg1.Invalidate();
                    zg1.AxisChange();
                }
                catch { }
            }
        }

        static int[] ColourValues = new int[] {  
        0xFF0000,0x00FF00,0x0000FF,0xFFFF00,0xFF00FF,0x00FFFF,0x000000,  
        0x800000,0x008000,0x000080,0x808000,0x800080,0x008080,0x808080,  
        0xC00000,0x00C000,0x0000C0,0xC0C000,0xC000C0,0x00C0C0,0xC0C0C0,  
        0x400000,0x004000,0x000040,0x404000,0x400040,0x004040,0x404040,  
        0x200000,0x002000,0x000020,0x202000,0x200020,0x002020,0x202020,  
        0x600000,0x006000,0x000060,0x606000,0x600060,0x006060,0x606060,  
        0xA00000,0x00A000,0x0000A0,0xA0A000,0xA000A0,0x00A0A0,0xA0A0A0,  
        0xE00000,0x00E000,0x0000E0,0xE0E000,0xE000E0,0x00E0E0,0xE0E0E0,  
    }; 


        private void GetLogFileData(ZedGraphControl zg1, string logfile, List<string> lookforfields)
        {
            if (zg1 == null)
                return;

            if (lookforfields != null && lookforfields.Count == 0)
                return;

            PointPairList[] lists = new PointPairList[lookforfields.Count];

            Random rand = new Random();

            int step = 0;

            // setup display and arrays
            for (int a = 0; a < lookforfields.Count; a++)
            {
                lists[a] = new PointPairList();

                LineItem myCurve;

                int colorvalue = ColourValues[step % ColourValues.Length];
                step++;

                myCurve = zg1.GraphPane.AddCurve(lookforfields[a].Replace("__mavlink_", ""), lists[a], Color.FromArgb(unchecked( colorvalue + (int)0xff000000)), SymbolType.None);
            }

            {

                MAVLink MavlinkInterface = new MAVLink();
                MavlinkInterface.logplaybackfile = new BinaryReader(File.Open(logfile, FileMode.Open, FileAccess.Read, FileShare.Read));
                MavlinkInterface.logreadmode = true;

                MavlinkInterface.packets.Initialize(); // clear

                int appui = 0;

                // to get first packet time
                MavlinkInterface.readPacket();

                DateTime startlogtime = MavlinkInterface.lastlogread;

                while (MavlinkInterface.logplaybackfile.BaseStream.Position < MavlinkInterface.logplaybackfile.BaseStream.Length)
                {
                    progressBar1.Value = (int)((float)MavlinkInterface.logplaybackfile.BaseStream.Position / (float)MavlinkInterface.logplaybackfile.BaseStream.Length * 100.0f);
                    progressBar1.Refresh();

                    byte[] packet = MavlinkInterface.readPacket();

                    object data = MavlinkInterface.DebugPacket(packet, false);

                    Type test = data.GetType();

                    foreach (var field in test.GetFields())
                    {
                        // field.Name has the field's name.

                        object fieldValue = field.GetValue(data); // Get value

                        if (field.FieldType.IsArray)
                        {

                        }
                        else
                        {
                            string currentitem = field.Name + " " + field.DeclaringType.Name;
                            int a = 0;
                            foreach (var lookforfield in lookforfields)
                            {

                                if (currentitem == lookforfield)
                                {
                                    object value = field.GetValue(data);
                                    // seconds scale
                                    double time = (MavlinkInterface.lastlogread - startlogtime).TotalMilliseconds / 1000.0;

                                    if (value.GetType() == typeof(Single))
                                    {
                                        lists[a].Add(time, (Single)field.GetValue(data));
                                    }
                                    else if (value.GetType() == typeof(short))
                                    {
                                        lists[a].Add(time, (short)field.GetValue(data));
                                    }
                                    else if (value.GetType() == typeof(ushort))
                                    {
                                        lists[a].Add(time, (ushort)field.GetValue(data));
                                    }
                                    else if (value.GetType() == typeof(byte))
                                    {
                                        lists[a].Add(time, (byte)field.GetValue(data));
                                    }
                                    else if (value.GetType() == typeof(Int32))
                                    {
                                        lists[a].Add(time, (Int32)field.GetValue(data));
                                    }
                                }
                                a++;
                            }
                        }
                    }

                    if (appui != DateTime.Now.Second)
                    {
                        // cant do entire app as mixes with flightdata timer
                        this.Refresh();
                        appui = DateTime.Now.Second;
                    }
                }

                MavlinkInterface.logreadmode = false;
                MavlinkInterface.logplaybackfile.Close();
                MavlinkInterface.logplaybackfile = null;


                //writeKML(logfile + ".kml");

                progressBar1.Value = 100;
            }
        }

        private List<string> GetLogFileValidFields(string logfile)
        {
            Form selectform = SelectDataToGraphForm();

            Hashtable seenIt = new Hashtable();
            
            {

                MAVLink mine = new MAVLink();
                mine.logplaybackfile = new BinaryReader(File.Open(logfile, FileMode.Open, FileAccess.Read, FileShare.Read));
                mine.logreadmode = true;

                mine.packets.Initialize(); // clear

                while (mine.logplaybackfile.BaseStream.Position < mine.logplaybackfile.BaseStream.Length)
                {
                    progressBar1.Value = (int)((float)mine.logplaybackfile.BaseStream.Position / (float)mine.logplaybackfile.BaseStream.Length * 100.0f);
                    this.Refresh();

                    byte[] packet = mine.readPacket();

                    object data = mine.DebugPacket(packet, false);

                    Type test = data.GetType();

                    foreach (var field in test.GetFields())
                    {
                        // field.Name has the field's name.

                        object fieldValue = field.GetValue(data); // Get value

                        if (field.FieldType.IsArray)
                        {

                        }
                        else
                        {
                            if (!seenIt.ContainsKey(field.DeclaringType.Name + "." + field.Name))
                            {
                                AddDataOption(selectform, field.Name + " " + field.DeclaringType.Name);
                                seenIt[field.DeclaringType.Name + "." + field.Name] = 1;
                            }
                        }
                    }
                }

                mine.logreadmode = false;
                mine.logplaybackfile.Close();
                mine.logplaybackfile = null;

                selectform.ShowDialog();

                progressBar1.Value = 100;

            }

            return selection;
        }

        private void AddDataOption(Form selectform, string Name)
        {

            CheckBox chk_box = new CheckBox();

            log.Info("Add Option " + Name);

            chk_box.Text = Name;
            chk_box.Name = Name;
            chk_box.Location = new System.Drawing.Point(x, y);
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

        void chk_box_CheckedChanged(object sender, EventArgs e)
        {
            if (((CheckBox)sender).Checked)
            {
                selection.Add(((CheckBox)sender).Name);
            }
            else
            {
                selection.Remove(((CheckBox)sender).Name);
            }
        }

        int x = 10;
        int y = 10;

        private Form SelectDataToGraphForm()
        {
            Form selectform = new Form()
            {
                Name = "select",
                Width = 50,
                Height = 500,
                Text = "Graph This"
            };

            x = 10;
            y = 10;

            {
                CheckBox chk_box = new CheckBox();
                chk_box.Text = "Logarithmic";
                chk_box.Name = "Logarithmic";
                chk_box.Location = new System.Drawing.Point(x, y);
                chk_box.Size = new System.Drawing.Size(100, 20);
                //chk_box.CheckedChanged += new EventHandler(chk_log_CheckedChanged);

                selectform.Controls.Add(chk_box);
            }

            y += 20;

            ThemeManager.ApplyThemeTo(selectform);

            return selectform;
        }
    }
}
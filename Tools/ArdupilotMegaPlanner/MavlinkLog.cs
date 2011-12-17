using System;
using System.Collections.Generic;
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


namespace ArdupilotMega
{
    public partial class MavlinkLog : Form
    {
        List<CurrentState> flightdata = new List<CurrentState>();

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

                Link link = new Link();
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

                    DateTime appui = DateTime.Now;

                    while (mine.logplaybackfile.BaseStream.Position < mine.logplaybackfile.BaseStream.Length)
                    {
                        // bar moves to 50 % in this step
                        progressBar1.Value = (int)((float)mine.logplaybackfile.BaseStream.Position / (float)mine.logplaybackfile.BaseStream.Length * 100.0f / 2.0f);
                        progressBar1.Invalidate();
                        progressBar1.Refresh();

                        byte[] packet = mine.readPacket();

                        cs.datetime = mine.lastlogread;

                        cs.UpdateCurrentSettings(null, true, mine);

                        if (appui != DateTime.Now)
                        {
                            // cant do entire app as mixes with flightdata timer
                            this.Refresh();
                            appui = DateTime.Now;
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

    }
}
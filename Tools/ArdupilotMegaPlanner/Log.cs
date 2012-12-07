using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Reflection;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;
using System.Text.RegularExpressions;
using KMLib;
using KMLib.Feature;
using KMLib.Geometry;
using Core.Geometry;
using ICSharpCode.SharpZipLib.Zip;
using ICSharpCode.SharpZipLib.Checksums;
using ICSharpCode.SharpZipLib.Core;
using log4net;
using ArdupilotMega.Comms;


namespace ArdupilotMega
{
    public partial class Log : Form
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        ICommsSerial comPort;
        int logcount = 0;
        serialstatus status = serialstatus.Connecting;
        StreamWriter sw;
        int currentlog = 0;
        bool threadrun = true;
        string logfile = "";
        int receivedbytes = 0;
        List<Data> flightdata = new List<Data>();
        //List<Model> orientation = new List<Model>();
        Model runmodel = new Model();
        Object thisLock = new Object();
        DateTime start = DateTime.Now;

        public struct Data
        {
            public Model model;
            public string[] ntun;
            public string[] ctun;
            public int datetime;
        }

        enum serialstatus
        {
            Connecting,
            Createfile,
            Closefile,
            Reading,
            Waiting,
            Done
        }

        public Log()
        {
            InitializeComponent();
        }

        private void waitandsleep(int time)
        {
            DateTime start = DateTime.Now;

            while ((DateTime.Now - start).TotalMilliseconds < time)
            {
                try
                {
                    if (comPort.BytesToRead > 0)
                    {
                        return;
                    }
                }
                catch { threadrun = false; return; }
            }
        }

        private void readandsleep(int time)
        {
            DateTime start = DateTime.Now;

            while ((DateTime.Now - start).TotalMilliseconds < time)
            {
                try
                {
                    if (comPort.BytesToRead > 0)
                    {
                        comPort_DataReceived((object)null, (SerialDataReceivedEventArgs)null);
                    }
                }
                catch { threadrun = false; return; }
            }
        }

        private void Log_Load(object sender, EventArgs e)
        {
            status = serialstatus.Connecting;

            comPort = MainV2.comPort.BaseStream;

            try
            {

                comPort.toggleDTR();

                comPort.DiscardInBuffer();

                // 10 sec
                waitandsleep(10000);
            }
            catch (Exception ex)
            {
                log.Error("Error opening comport", ex);
                CustomMessageBox.Show("Error opening comport");
            }

            var t11 = new System.Threading.Thread(delegate()
            {
                var start = DateTime.Now;

                threadrun = true;

                readandsleep(100);

                try
                {
                    comPort.Write("\n\n\n\n"); // more in "connecting"
                }
                catch
                {
                }

                while (threadrun)
                {
                    try
                    {
                        updateDisplay();

                        System.Threading.Thread.Sleep(10);
                        if (!comPort.IsOpen)
                            break;
                        while (comPort.BytesToRead >= 4)
                        {
                            comPort_DataReceived((object)null, (SerialDataReceivedEventArgs)null);
                        }
                    }
                    catch (Exception ex)
                    {
                        log.Error("crash in comport reader " + ex);
                    } // cant exit unless told to
                }
                log.Info("Comport thread close");
            }) {Name = "comport reader"};
            t11.Start();

            // doesnt seem to work on mac
            //comPort.DataReceived += new SerialDataReceivedEventHandler(comPort_DataReceived);
        }

        void genchkcombo(int logcount)
        {
            MethodInvoker m = delegate()
            {
                //CHK_logs.Items.Clear();
                //for (int a = 1; a <= logcount; a++)
                if (!CHK_logs.Items.Contains(logcount))
                {
                    CHK_logs.Items.Add(logcount);
                }
            };
            try
            {
                BeginInvoke(m);
            }
            catch
            {
            }
        }

        void updateDisplay()
        {
            if (start.Second != DateTime.Now.Second)
            {
                this.BeginInvoke((System.Windows.Forms.MethodInvoker)delegate()
{
    try
    {
        TXT_status.Text = status.ToString() + " " + receivedbytes + " " + comPort.BytesToRead;
    }
    catch { }
});
                start = DateTime.Now;
            }
        }

        void comPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {


                while (comPort.BytesToRead > 0 && threadrun)
                {
                    updateDisplay();

                    string line = "";

                    comPort.ReadTimeout = 500;
                    try
                    {
                        line = comPort.ReadLine(); //readline(comPort);
                        if (!line.Contains("\n"))
                            line = line + "\n";
                    }
                    catch
                    {
                        line = comPort.ReadExisting();
                        //byte[] data = readline(comPort);
                        //line = Encoding.ASCII.GetString(data, 0, data.Length);
                    }

                    receivedbytes += line.Length;

                    //string line = Encoding.ASCII.GetString(data, 0, data.Length);

                    switch (status)
                    {
                        case serialstatus.Connecting:

                            if (line.Contains("ENTER") || line.Contains("GROUND START") || line.Contains("reset to FLY") || line.Contains("interactive setup") || line.Contains("CLI") || line.Contains("Ardu"))
                            {
                                try
                                {
                                    comPort.Write("\n\n\n\n");
                                }
                                catch { }

                                comPort.Write("logs\r");
                                status = serialstatus.Done;
                            }
                            break;
                        case serialstatus.Closefile:
                            sw.Close();
                            TextReader tr = new StreamReader(logfile);

                            this.Invoke((System.Windows.Forms.MethodInvoker)delegate()
{
    TXT_seriallog.AppendText("Createing KML for " + logfile);
});

                            while (tr.Peek() != -1)
                            {
                                processLine(tr.ReadLine());
                            }

                            tr.Close();

                            try
                            {
                                writeKML(logfile + ".kml");
                            }
                            catch { } // usualy invalid lat long error
                            status = serialstatus.Done;
                            break;
                        case serialstatus.Createfile:
                            receivedbytes = 0;
                            Directory.CreateDirectory(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs");
                            logfile = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs" + Path.DirectorySeparatorChar + DateTime.Now.ToString("yyyy-MM-dd HH-mm") + " " + currentlog + ".log";
                            sw = new StreamWriter(logfile);
                            status = serialstatus.Waiting;
                            lock (thisLock)
                            {
                                this.Invoke((System.Windows.Forms.MethodInvoker)delegate()
{
    TXT_seriallog.Clear();
});
                            }
                            //if (line.Contains("Dumping Log"))
                            {
                                status = serialstatus.Reading;
                            }
                            break;
                        case serialstatus.Done:
                            // 
                            if (line.Contains("start") && line.Contains("end"))
                            {
                                Regex regex2 = new Regex(@"^Log ([0-9]+),", RegexOptions.IgnoreCase);
                                if (regex2.IsMatch(line))
                                {
                                    MatchCollection matchs = regex2.Matches(line);
                                    logcount = int.Parse(matchs[0].Groups[1].Value);
                                    genchkcombo(logcount);
                                    //status = serialstatus.Done;
                                }
                            }
                            if (line.Contains("No logs"))
                            {
                                status = serialstatus.Done;
                            }
                            break;
                        case serialstatus.Reading:
                            if (line.Contains("packets read") || line.Contains("Done") || line.Contains("logs enabled"))
                            {
                                status = serialstatus.Closefile;
                                break;
                            }
                            sw.Write(line);
                            continue;
                        case serialstatus.Waiting:
                            if (line.Contains("Dumping Log") || line.Contains("GPS:") || line.Contains("NTUN:") || line.Contains("CTUN:") || line.Contains("PM:"))
                            {
                                status = serialstatus.Reading;
                            }
                            break;
                    }
                    lock (thisLock)
                    {
                        this.BeginInvoke((MethodInvoker)delegate()
                        {

                            Console.Write(line);

                            TXT_seriallog.AppendText(line.Replace((char)0x0,' '));

                            // auto scroll
                            if (TXT_seriallog.TextLength >= 10000)
                            {
                                TXT_seriallog.Text = TXT_seriallog.Text.Substring(TXT_seriallog.TextLength / 2);
                            }

                            TXT_seriallog.SelectionStart = TXT_seriallog.Text.Length;

                            TXT_seriallog.ScrollToCaret();

                            TXT_seriallog.Refresh();

                        });



                    }
                }

                log.Info("exit while");
            }
            catch (Exception ex) { CustomMessageBox.Show("Error reading data" + ex.ToString()); }
        }

        string lastline = "";
        string[] ctunlast = new string[] { "", "", "", "", "", "", "", "", "", "", "", "", "", "" };
        string[] ntunlast = new string[] { "", "", "", "", "", "", "", "", "", "", "", "", "", "" };
        List<PointLatLngAlt> cmd = new List<PointLatLngAlt>();
        Point3D oldlastpos = new Point3D();
        Point3D lastpos = new Point3D();

        private void processLine(string line)
        {
            try
            {
                Application.DoEvents();

                line = line.Replace(", ", ",");
                line = line.Replace(": ", ":");

                string[] items = line.Split(',', ':');

                if (items[0].Contains("CMD"))
                {
                    if (flightdata.Count == 0)
                    {
                        if (int.Parse(items[2]) <= (int)MAVLink.MAV_CMD.LAST) // wps
                        {
                            PointLatLngAlt temp = new PointLatLngAlt(double.Parse(items[5], new System.Globalization.CultureInfo("en-US")) / 10000000, double.Parse(items[6], new System.Globalization.CultureInfo("en-US")) / 10000000, double.Parse(items[4], new System.Globalization.CultureInfo("en-US")) / 100, items[1].ToString());
                            cmd.Add(temp);
                        }
                    }
                }
                if (items[0].Contains("MOD"))
                {
                    positionindex++;
                    modelist.Add(""); // i cant be bothered doing this properly
                    modelist.Add("");
                    modelist[positionindex] = (items[1]);
                }
                if (items[0].Contains("GPS") && items[2] == "1" && items[4] != "0" && items[4] != "-1" && lastline != line) // check gps line and fixed status
                {
                    MainV2.comPort.MAV.cs.firmware = MainV2.Firmwares.ArduPlane;

                    if (position[positionindex] == null)
                        position[positionindex] = new List<Point3D>();

                    if (double.Parse(items[4], new System.Globalization.CultureInfo("en-US")) == 0)
                        return;

                    double alt = double.Parse(items[6], new System.Globalization.CultureInfo("en-US"));

                    if (items.Length == 11 && items[6] == "0.0000")
                        alt = double.Parse(items[7], new System.Globalization.CultureInfo("en-US"));
                    if (items.Length == 11 && items[6] == "0")
                        alt = double.Parse(items[7], new System.Globalization.CultureInfo("en-US"));


                    position[positionindex].Add(new Point3D(double.Parse(items[5], new System.Globalization.CultureInfo("en-US")), double.Parse(items[4], new System.Globalization.CultureInfo("en-US")), alt));
                    oldlastpos = lastpos;
                    lastpos = (position[positionindex][position[positionindex].Count - 1]);
                    lastline = line;
                }
                if (items[0].Contains("GPS") && items[4] != "0" && items[4] != "-1" && items.Length <= 9) // AC
                {
                    MainV2.comPort.MAV.cs.firmware = MainV2.Firmwares.ArduCopter2;

                    if (position[positionindex] == null)
                        position[positionindex] = new List<Point3D>();

                    if (double.Parse(items[4], new System.Globalization.CultureInfo("en-US")) == 0)
                        return;

                    double alt = double.Parse(items[5], new System.Globalization.CultureInfo("en-US"));

                    position[positionindex].Add(new Point3D(double.Parse(items[4], new System.Globalization.CultureInfo("en-US")), double.Parse(items[3], new System.Globalization.CultureInfo("en-US")), alt));
                    oldlastpos = lastpos;
                    lastpos = (position[positionindex][position[positionindex].Count - 1]);
                    lastline = line;

                }
                if (items[0].Contains("CTUN"))
                {
                    ctunlast = items;
                }
                if (items[0].Contains("NTUN"))
                {
                    ntunlast = items;
                    line = "ATT:" + double.Parse(ctunlast[3], new System.Globalization.CultureInfo("en-US")) * 100 + "," + double.Parse(ctunlast[6], new System.Globalization.CultureInfo("en-US")) * 100 + "," + double.Parse(items[1], new System.Globalization.CultureInfo("en-US")) * 100;
                    items = line.Split(',', ':');
                }
                if (items[0].Contains("ATT"))
                {
                    try
                    {
                        if (lastpos.X != 0 && oldlastpos.X != lastpos.X && oldlastpos.Y != lastpos.Y)
                        {
                            Data dat = new Data();

                            try
                            {
                                dat.datetime = int.Parse(lastline.Split(',', ':')[1]);
                            }
                            catch { }

                            runmodel = new Model();

                            runmodel.Location.longitude = lastpos.X;
                            runmodel.Location.latitude = lastpos.Y;
                            runmodel.Location.altitude = lastpos.Z;

                            oldlastpos = lastpos;

                            runmodel.Orientation.roll = double.Parse(items[1], new System.Globalization.CultureInfo("en-US")) / -100;
                            runmodel.Orientation.tilt = double.Parse(items[2], new System.Globalization.CultureInfo("en-US")) / -100;
                            runmodel.Orientation.heading = double.Parse(items[3], new System.Globalization.CultureInfo("en-US")) / 100;

                            dat.model = runmodel;
                            dat.ctun = ctunlast;
                            dat.ntun = ntunlast;

                            flightdata.Add(dat);
                        }
                    }
                    catch { }
                }
            }
            catch (Exception)
            {
                // if items is to short or parse fails.. ignore
            }
        }

        List<string> modelist = new List<string>();
        List<Core.Geometry.Point3D>[] position = new List<Core.Geometry.Point3D>[200];
        int positionindex = 0;

        private void writeGPX(string filename)
        {
            System.Xml.XmlTextWriter xw = new System.Xml.XmlTextWriter(Path.GetDirectoryName(filename) + Path.DirectorySeparatorChar + Path.GetFileNameWithoutExtension(filename) + ".gpx",Encoding.ASCII);

            xw.WriteStartElement("gpx");

            xw.WriteStartElement("trk");

            xw.WriteStartElement("trkseg");

            DateTime start = new DateTime(DateTime.Now.Year,DateTime.Now.Month,DateTime.Now.Day,0,0,0);

            foreach (Data mod in flightdata)
            {
                xw.WriteStartElement("trkpt");
                xw.WriteAttributeString("lat", mod.model.Location.latitude.ToString(new System.Globalization.CultureInfo("en-US")));
                xw.WriteAttributeString("lon", mod.model.Location.longitude.ToString(new System.Globalization.CultureInfo("en-US")));

                xw.WriteElementString("ele", mod.model.Location.altitude.ToString(new System.Globalization.CultureInfo("en-US")));
                xw.WriteElementString("time", start.AddMilliseconds(mod.datetime).ToString("yyyy-MM-ddTHH:mm:sszzzzzz"));
                xw.WriteElementString("course", (mod.model.Orientation.heading).ToString(new System.Globalization.CultureInfo("en-US")));

                xw.WriteElementString("roll", mod.model.Orientation.roll.ToString(new System.Globalization.CultureInfo("en-US")));
                xw.WriteElementString("pitch", mod.model.Orientation.tilt.ToString(new System.Globalization.CultureInfo("en-US")));
                //xw.WriteElementString("speed", mod.model.Orientation.);
                //xw.WriteElementString("fix", mod.model.Location.altitude);

                xw.WriteEndElement();
            }

            xw.WriteEndElement();
            xw.WriteEndElement();
            xw.WriteEndElement();

            xw.Close();
        }

        private void writeKML(string filename)
        {
            try
            {
                writeGPX(filename);
            }
            catch { }

            Color[] colours = { Color.Red, Color.Orange, Color.Yellow, Color.Green, Color.Blue, Color.Indigo, Color.Violet, Color.Pink };

            AltitudeMode altmode = AltitudeMode.absolute;

            if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduCopter2)
            {
                altmode = AltitudeMode.relativeToGround; // because of sonar, this is both right and wrong. right for sonar, wrong in terms of gps as the land slopes off.
            }

            KMLRoot kml = new KMLRoot();
            Folder fldr = new Folder("Log");

            Style style = new Style();
            style.Id = "yellowLineGreenPoly";
            style.Add(new LineStyle(HexStringToColor("7f00ffff"), 4));

            PolyStyle pstyle = new PolyStyle();
            pstyle.Color = HexStringToColor("7f00ff00");
            style.Add(pstyle);

            kml.Document.AddStyle(style);

            int stylecode = 0xff;
            int g = -1;
            foreach (List<Point3D> poslist in position)
            {
                g++;
                if (poslist == null)
                    continue;

                LineString ls = new LineString();
                ls.AltitudeMode = altmode;
                ls.Extrude = true;
                //ls.Tessellate = true;

                Coordinates coords = new Coordinates();
                coords.AddRange(poslist);

                ls.coordinates = coords;

                Placemark pm = new Placemark();

                string mode = "";
                if (g < modelist.Count)
                    mode = modelist[g];

                pm.name = g + " Flight Path " + mode;
                pm.styleUrl = "#yellowLineGreenPoly";
                pm.LineString = ls;

                stylecode = colours[g % (colours.Length - 1)].ToArgb();

                Style style2 = new Style();
                Color color = Color.FromArgb(0xff, (stylecode >> 16) & 0xff, (stylecode >> 8) & 0xff, (stylecode >> 0) & 0xff);
                log.Info("colour " + color.ToArgb().ToString("X") + " " + color.ToKnownColor().ToString());
                style2.Add(new LineStyle(color, 4));



                pm.AddStyle(style2);

                fldr.Add(pm);
            }

            Folder planes = new Folder();
            planes.name = "Planes";
            fldr.Add(planes);

            Folder waypoints = new Folder();
            waypoints.name = "Waypoints";
            fldr.Add(waypoints);


            LineString lswp = new LineString();
            lswp.AltitudeMode = AltitudeMode.relativeToGround;
            lswp.Extrude = true;

            Coordinates coordswp = new Coordinates();

            foreach (PointLatLngAlt p1 in cmd)
            {
                coordswp.Add(new Point3D(p1.Lng, p1.Lat, p1.Alt));
            }

            lswp.coordinates = coordswp;

            Placemark pmwp = new Placemark();

            pmwp.name = "Waypoints";
            //pm.styleUrl = "#yellowLineGreenPoly";
            pmwp.LineString = lswp;

            waypoints.Add(pmwp);

            int a = 0;
            int l = -1;

            Model lastmodel = null;

            foreach (Data mod in flightdata)
            {
                l++;
                if (mod.model.Location.latitude == 0)
                    continue;

                if (lastmodel != null)
                {
                    if (lastmodel.Location.Equals(mod.model.Location))
                    {
                        continue;
                    }
                }
                Placemark pmplane = new Placemark();
                pmplane.name = "Plane " + a;

                pmplane.visibility = false;

                Model model = mod.model;
                model.AltitudeMode = altmode;
                model.Scale.x = 2;
                model.Scale.y = 2;
                model.Scale.z = 2;

                try
                {
                    pmplane.description = @"<![CDATA[
              <table>
                <tr><td>Roll: " + model.Orientation.roll + @" </td></tr>
                <tr><td>Pitch: " + model.Orientation.tilt + @" </td></tr>
                <tr><td>Yaw: " + model.Orientation.heading + @" </td></tr>
                <tr><td>WP dist " + mod.ntun[2] + @" </td></tr>
				<tr><td>tar bear " + mod.ntun[3] + @" </td></tr>
				<tr><td>nav bear " + mod.ntun[4] + @" </td></tr>
				<tr><td>alt error " + mod.ntun[5] + @" </td></tr>
              </table>
            ]]>";
                }
                catch { }

                try
                {

                    pmplane.Point = new KmlPoint((float)model.Location.longitude, (float)model.Location.latitude, (float)model.Location.altitude);
                    pmplane.Point.AltitudeMode = altmode;

                    Link link = new Link();
                    link.href = "block_plane_0.dae";

                    model.Link = link;

                    pmplane.Model = model;

                    planes.Add(pmplane);
                }
                catch { } // bad lat long value

                lastmodel = mod.model;

                a++;
            }

            kml.Document.Add(fldr);

            kml.Save(filename);

            // create kmz - aka zip file

            FileStream fs = File.Open(filename.Replace(".log.kml", ".kmz"), FileMode.Create);
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
            using (FileStream streamReader = File.OpenRead(filename))
            {
                StreamUtils.Copy(streamReader, zipStream, buffer);
            }
            zipStream.CloseEntry();

            File.Delete(filename);

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

            positionindex = 0;
            modelist.Clear();
            flightdata.Clear();
            position = new List<Core.Geometry.Point3D>[200];
            cmd.Clear();
        }

        private void Log_FormClosing(object sender, FormClosingEventArgs e)
        {
            threadrun = false;
            System.Threading.Thread.Sleep(500);
            if (comPort.IsOpen)
            {
                //comPort.Close();
            }
            System.Threading.Thread.Sleep(500);
        }

        private void CHK_logs_Click(object sender, EventArgs e)
        {
            ListBox lb = sender as ListBox;

        }

        private void BUT_DLall_Click(object sender, EventArgs e)
        {
            if (status == serialstatus.Done)
            {
                System.Threading.Thread t11 = new System.Threading.Thread(delegate() { downloadthread(1, logcount); });
                t11.Name = "Log Download All thread";
                t11.Start();
            }
        }

        private void downloadthread(int startlognum, int endlognum)
        {
            for (int a = startlognum; a <= endlognum; a++)
            {
                currentlog = a;
                System.Threading.Thread.Sleep(500);
                comPort.Write("dump ");
                System.Threading.Thread.Sleep(100);
                comPort.Write(a.ToString() + "\r");
                status = serialstatus.Createfile;

                while (status != serialstatus.Done)
                {
                    System.Threading.Thread.Sleep(100);
                }

            }
        }

        private void downloadsinglethread()
        {
            for (int i = 0; i < CHK_logs.CheckedItems.Count; ++i)
            {
                int a = (int)CHK_logs.CheckedItems[i];
                {
                    currentlog = a;
                    System.Threading.Thread.Sleep(500);
                    comPort.Write("dump ");
                    System.Threading.Thread.Sleep(100);
                    comPort.Write(a.ToString() + "\r");
                    status = serialstatus.Createfile;

                    while (status != serialstatus.Done)
                    {
                        System.Threading.Thread.Sleep(100);
                    }
                }
            }
        }

        private void BUT_DLthese_Click(object sender, EventArgs e)
        {
            if (status == serialstatus.Done)
            {
                System.Threading.Thread t11 = new System.Threading.Thread(delegate() { downloadsinglethread(); });
                t11.Name = "Log download single thread";
                t11.Start();
            }
        }

        private void BUT_clearlogs_Click(object sender, EventArgs e)
        {
            System.Threading.Thread.Sleep(500);
            comPort.Write("erase\r");
            System.Threading.Thread.Sleep(100);
            TXT_seriallog.AppendText("!!Allow 30-90 seconds for erase\n");
            status = serialstatus.Done;
            CHK_logs.Items.Clear();
        }

        private void BUT_redokml_Click(object sender, EventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.Filter = "*.log|*.log";
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
                    TXT_seriallog.AppendText("\n\nProcessing " + logfile + "\n");
                    this.Refresh();
                    try
                    {
                        TextReader tr = new StreamReader(logfile);

                        while (tr.Peek() != -1)
                        {
                            processLine(tr.ReadLine());
                        }

                        tr.Close();
                    }
                    catch (Exception ex) { CustomMessageBox.Show("Error processing file. Make sure the file is not in use.\n"+ex.ToString()); }

                    writeKML(logfile + ".kml");

                    TXT_seriallog.AppendText("Done\n");
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

        private void BUT_firstperson_Click(object sender, EventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.Filter = "*.log|*.log";
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
                    TXT_seriallog.AppendText("\n\nProcessing " + logfile + "\n");
                    this.Refresh();

                    try
                    {
                        TextReader tr = new StreamReader(logfile);

                        while (tr.Peek() != -1)
                        {
                            processLine(tr.ReadLine());
                        }

                        tr.Close();
                    }
                    catch (Exception ex) { CustomMessageBox.Show("Error processing log. Is it still downloading? " + ex.Message); continue; }

                    writeKMLFirstPerson(logfile + ".kml");

                    TXT_seriallog.AppendText("Done\n");
                }
            }
        }

        private void writeKMLFirstPerson(string filename)
        {
            StreamWriter stream = new StreamWriter(File.Open(filename, FileMode.Create));
            System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();
            string header = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">\n     <Document>   <name>Paths</name>    <description>Path</description>\n    <Style id=\"yellowLineGreenPoly\">      <LineStyle>        <color>7f00ffff</color>        <width>4</width>      </LineStyle>      <PolyStyle>        <color>7f00ff00</color>      </PolyStyle>    </Style>\n  ";
            stream.Write(header);

            StringBuilder kml = new StringBuilder();
            StringBuilder data = new StringBuilder();

            double lastlat = 0;
            double lastlong = 0;
            int gpspackets = 0;
            int lastgpspacket = 0;

            foreach (Data mod in flightdata)
            {
                if (mod.model.Location.latitude == 0)
                    continue;

                gpspackets++;
                if (lastlat == mod.model.Location.latitude && lastlong == mod.model.Location.longitude)
                    continue;
                // double speed 0.05 - assumeing 10hz in log file
                // 1 speed = 0.1    10 / 1  = 0.1
                data.Append(@"
        <gx:FlyTo>
            <gx:duration>" + ((gpspackets - lastgpspacket) * 0.1) + @"</gx:duration>
            <gx:flyToMode>smooth</gx:flyToMode>
            <Camera>
                <longitude>" + mod.model.Location.longitude.ToString(new System.Globalization.CultureInfo("en-US")) + @"</longitude>
                <latitude>" + mod.model.Location.latitude.ToString(new System.Globalization.CultureInfo("en-US")) + @"</latitude>
                <altitude>" + mod.model.Location.altitude.ToString(new System.Globalization.CultureInfo("en-US")) + @"</altitude>
                <roll>" + mod.model.Orientation.roll.ToString(new System.Globalization.CultureInfo("en-US")) + @"</roll>
                <tilt>" + (90 - mod.model.Orientation.tilt).ToString(new System.Globalization.CultureInfo("en-US")) + @"</tilt>
                <heading>" + mod.model.Orientation.heading.ToString(new System.Globalization.CultureInfo("en-US")) + @"</heading>              
                <altitudeMode>absolute</altitudeMode>
            </Camera>
        </gx:FlyTo>
");
                lastlat = mod.model.Location.latitude;
                lastlong = mod.model.Location.longitude;
                lastgpspacket = gpspackets;
            }

            kml.Append(@"
        <Folder>
            <name>Flight</name> 
            <gx:Tour>
                <name>Flight Do</name> 
                <gx:Playlist>
                    " + data +
                @"</gx:Playlist> 
            </gx:Tour>
        </Folder>
    </Document>
</kml>
");

            stream.Write(kml.ToString());
            stream.Close();

            // create kmz - aka zip file

            FileStream fs = File.Open(filename.Replace(".log.kml", ".kmz"), FileMode.Create);
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
            using (FileStream streamReader = File.OpenRead(filename))
            {
               StreamUtils.Copy(streamReader, zipStream, buffer);
            }
            zipStream.CloseEntry();

            File.Delete(filename);

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

            positionindex = 0;
            modelist.Clear();
            flightdata.Clear();
            position = new List<Core.Geometry.Point3D>[200];
            cmd.Clear();
        }

        private void BUT_dumpdf_Click(object sender, EventArgs e)
        {
            if (status == serialstatus.Done)
            {
                // add -1 entry
                CHK_logs.Items.Add(-1, true);

                System.Threading.Thread t11 = new System.Threading.Thread(delegate() { downloadsinglethread(); });
                t11.Name = "Log download single thread";
                t11.Start();
            }
        }

    }
}
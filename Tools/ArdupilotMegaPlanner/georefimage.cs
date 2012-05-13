using System;
using System.Collections.Generic;
using System.Collections;
using System.Reflection;
using System.IO;
using System.Windows.Forms;
using com.drew.imaging.jpg;
using com.drew.metadata;
using log4net;
using SharpKml.Base;
using SharpKml.Dom;
using System.Drawing;
using System.Drawing.Imaging;
using System.Text;

namespace ArdupilotMega
{
    public class Georefimage : Form
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        private OpenFileDialog openFileDialog1;
        private ArdupilotMega.Controls.MyButton BUT_browselog;
        private ArdupilotMega.Controls.MyButton BUT_browsedir;
        private TextBox TXT_logfile;
        private TextBox TXT_jpgdir;
        private TextBox TXT_offsetseconds;
        private ArdupilotMega.Controls.MyButton BUT_doit;
        private FolderBrowserDialog folderBrowserDialog1;
        private Label label1;
        private TextBox TXT_outputlog;
        private ArdupilotMega.Controls.MyButton BUT_estoffset;

        int latpos = 4, lngpos = 5, altpos = 7, cogpos = 9;
        private NumericUpDown numericUpDown1;
        private NumericUpDown numericUpDown2;
        private NumericUpDown numericUpDown3;
        private NumericUpDown numericUpDown4;
        private Label label2;
        private Label label3;
        private Label label4;
        private Label label5;
        private Label label6;
        private Controls.MyButton BUT_networklinkgeoref;
        private ArdupilotMega.Controls.MyButton BUT_Geotagimages;

        internal Georefimage() {
            InitializeComponent();
        }

        Hashtable filedatecache = new Hashtable();
        Hashtable photocoords = new Hashtable();

        Hashtable timecoordcache = new Hashtable();
        Hashtable imagetotime = new Hashtable();

        List<string[]> logcache = new List<string[]>();

        DateTime getPhotoTime(string fn)
        {
            DateTime dtaken = DateTime.MinValue;

            if (filedatecache.ContainsKey(fn))
            {
                return (DateTime)filedatecache[fn];
            }

            try
            {
                
                Metadata lcMetadata = null;
                try
                {
                    FileInfo lcImgFile = new FileInfo(fn);
                    // Loading all meta data
                    lcMetadata = JpegMetadataReader.ReadMetadata(lcImgFile);
                }
                catch (JpegProcessingException e)
                {
                    log.InfoFormat(e.Message);
                    return dtaken;
                }

                foreach (AbstractDirectory lcDirectory in lcMetadata)
                {

                    if (lcDirectory.ContainsTag(0x9003))
                    {
                        dtaken = lcDirectory.GetDate(0x9003);
                        log.InfoFormat("does " + lcDirectory.GetTagName(0x9003) + " " + dtaken);

                        filedatecache[fn] = dtaken;

                        break;
                    }

                }


                


                ////// old method, works, just slow
                /*
                Image myImage = Image.FromFile(fn);
                PropertyItem propItem = myImage.GetPropertyItem(36867); // 36867  // 306

                //Convert date taken metadata to a DateTime object 
                string sdate = Encoding.UTF8.GetString(propItem.Value).Trim();
                string secondhalf = sdate.Substring(sdate.IndexOf(" "), (sdate.Length - sdate.IndexOf(" ")));
                string firsthalf = sdate.Substring(0, 10);
                firsthalf = firsthalf.Replace(":", "-");
                sdate = firsthalf + secondhalf;
                dtaken = DateTime.Parse(sdate);

                myImage.Dispose();
                 */
            }
            catch { }

            return dtaken;
        }

        List<string[]> readLog(string fn)
        {
            if (logcache.Count > 0)
                return logcache;

            List<string[]> list = new List<string[]>();

            if (fn.ToLower().EndsWith("tlog"))
            {
                MAVLink mine = new MAVLink();
                mine.logplaybackfile = new BinaryReader(File.Open(fn, FileMode.Open, FileAccess.Read, FileShare.Read));
                mine.logreadmode = true;

                mine.packets.Initialize(); // clear

                CurrentState cs = new CurrentState();

                string[] oldvalues = {""};

                while (mine.logplaybackfile.BaseStream.Position < mine.logplaybackfile.BaseStream.Length)
                {

                    byte[] packet = mine.readPacket();

                    cs.datetime = mine.lastlogread;

                    cs.UpdateCurrentSettings(null, true, mine);

                    //		line	"GPS: 82686250, 1, 8, -34.1406480, 118.5441900, 0.0000, 309.1900, 315.9500, 0.0000, 279.1200"	string


                    string[] vals = new string[] { "GPS", (cs.datetime.ToUniversalTime() - new DateTime(cs.datetime.Year,cs.datetime.Month,cs.datetime.Day,0,0,0,DateTimeKind.Utc)).TotalMilliseconds.ToString(), "1",
                    cs.satcount.ToString(),cs.lat.ToString(),cs.lng.ToString(),"0.0",cs.alt.ToString(),cs.alt.ToString(),cs.groundspeed.ToString(),cs.yaw.ToString()};

                    if (oldvalues.Length > 2 && oldvalues[latpos] == vals[latpos]
                        && oldvalues[lngpos] == vals[lngpos]
                        && oldvalues[altpos] == vals[altpos])
                        continue;

                    oldvalues = vals;

                    list.Add(vals);
                    // 4 5 7
                    Console.Write((mine.logplaybackfile.BaseStream.Position * 100 / mine.logplaybackfile.BaseStream.Length) + "    \r");
                    
                }

                mine.logplaybackfile.Close();

                logcache = list;

                return list;
            }

            StreamReader sr = new StreamReader(fn);

            string lasttime = "0";

            while (!sr.EndOfStream)
            {
                string line = sr.ReadLine();

                if (line.ToLower().StartsWith("gps"))
                {
                    string[] vals = line.Split(new char[] {',',':'});

                    if (lasttime == vals[1])
                        continue;

                    lasttime = vals[1];

                    list.Add(vals);
                }
            }

            sr.Close();
            sr.Dispose();

            logcache = list;

            return list;
        }

        public void dowork(string logFile, string dirWithImages, float offsetseconds, bool dooffset)
        {
            DateTime localmin = DateTime.MaxValue;
            DateTime localmax = DateTime.MinValue;

            DateTime startTime = DateTime.MinValue;

            photocoords = new Hashtable();
            timecoordcache = new Hashtable();
            imagetotime = new Hashtable();

            //logFile = @"C:\Users\hog\Pictures\farm 1-10-2011\100SSCAM\2011-10-01 11-48 1.log";

            List<string[]> list = readLog(logFile);

            //dirWithImages = @"C:\Users\hog\Pictures\farm 1-10-2011\100SSCAM";

            string[] files = Directory.GetFiles(dirWithImages);

            Document kml = new Document();

            StreamWriter sw4 = new StreamWriter(dirWithImages + Path.DirectorySeparatorChar + "loglocation.csv");

            StreamWriter sw3 = new StreamWriter(dirWithImages + Path.DirectorySeparatorChar + "location.kml");

            StreamWriter sw2 = new StreamWriter(dirWithImages + Path.DirectorySeparatorChar + "location.txt");

            StreamWriter sw = new StreamWriter(dirWithImages + Path.DirectorySeparatorChar + "location.tel");
            sw.WriteLine("version=1");

            sw.WriteLine("#seconds offset - " + TXT_offsetseconds.Text);
            sw.WriteLine("#longitude and latitude - in degrees");
            sw.WriteLine("#name	utc	longitude	latitude	height");

            int first = 0;
            int matchs = 0;

            int lastmatchindex = 0;

            foreach (string filename in files)
            {
                if (filename.ToLower().EndsWith(".jpg") && !filename.ToLower().Contains("_geotag"))
                {
                    DateTime photodt = getPhotoTime(filename);

                    if (startTime == DateTime.MinValue)
                    {
                        startTime = new DateTime(photodt.Year, photodt.Month, photodt.Day, 0, 0, 0, 0, DateTimeKind.Utc).ToLocalTime();

                        foreach (string[] arr in list)
                        {
                            DateTime crap = startTime.AddMilliseconds(int.Parse(arr[1])).AddSeconds(offsetseconds);

                            if (localmin > crap)
                                localmin = crap;
                            if (localmax < crap)
                                localmax = crap;
                        }

                        log.InfoFormat("min " + localmin + " max " + localmax);
                        TXT_outputlog.AppendText("Log min " + localmin + " max " + localmax + "\r\n");
                    }

                    TXT_outputlog.AppendText("Photo  " + Path.GetFileNameWithoutExtension(filename) + " time  " + photodt + "\r\n");
                    //Application.DoEvents();

                    int a = 0;

                    foreach (string[] arr in list)
                    {
                        a++;

                        if (lastmatchindex > (a))
                            continue;
                        //Application.DoEvents();

                        DateTime logdt = startTime.AddMilliseconds(int.Parse(arr[1])).AddSeconds(offsetseconds);

                        if (first == 0)
                        {
                            TXT_outputlog.AppendText("Photo " + Path.GetFileNameWithoutExtension(filename) + " " + photodt + " vs Log " + logdt + "\r\n");

                            TXT_outputlog.AppendText("offset should be about " + (photodt - logdt).TotalSeconds + "\r\n");

                            if (dooffset)
                                return;

                            first++;
                        }

                        //Console.Write("ph " + dt + " log " + crap + "         \r");


                        timecoordcache[(long)(logdt.AddSeconds(-offsetseconds) - DateTime.MinValue).TotalSeconds] = new double[] {
                            double.Parse(arr[latpos]), double.Parse(arr[lngpos]), double.Parse(arr[altpos]) 
                        };

                        sw4.WriteLine("ph " + filename + " " + photodt + " log " + logdt);

                        if (photodt.ToString("yyyy-MM-ddTHH:mm:ss") == logdt.ToString("yyyy-MM-ddTHH:mm:ss"))
                        {
                            lastmatchindex = a;
                             
                            TXT_outputlog.AppendText("MATCH Photo " + Path.GetFileNameWithoutExtension(filename) + " " + photodt + "\r\n");

                            matchs++;

                            

                             SharpKml.Dom.Timestamp tstamp = new SharpKml.Dom.Timestamp();

                             tstamp.When = photodt;

                             kml.AddFeature(
                                 new Placemark()
                                 {
                                     Time = tstamp,
                                     Name = Path.GetFileNameWithoutExtension(filename),
                                     Geometry = new SharpKml.Dom.Point()
                                     {
                                         Coordinate = new Vector(double.Parse(arr[latpos]), double.Parse(arr[lngpos]), double.Parse(arr[altpos]))
                                     },
                                     Description = new Description()
                                     {
                                         Text = "<table><tr><td><img src=\"" + Path.GetFileName(filename).ToLower() + "\" width=500 /></td></tr></table>"
                                     },
                                     StyleSelector = new Style()
                                     {
                                         Balloon = new BalloonStyle() { Text = "$[name]<br>$[description]" }
                                     }
                                 }
                             );

                             photocoords[filename] = new double[] { double.Parse(arr[latpos]), double.Parse(arr[lngpos]), double.Parse(arr[altpos]), double.Parse(arr[cogpos]) };

                             imagetotime[filename] = (long)(logdt.AddSeconds(-offsetseconds) - DateTime.MinValue).TotalSeconds;

                            sw2.WriteLine(Path.GetFileNameWithoutExtension(filename) + " " + arr[lngpos] + " " + arr[latpos] + " " + arr[altpos]);
                            sw.WriteLine(Path.GetFileNameWithoutExtension(filename) + "\t" + logdt.ToString("yyyy:MM:dd HH:mm:ss") + "\t" + arr[lngpos] + "\t" + arr[latpos] + "\t" + arr[altpos]);
                            sw.Flush();
                            sw2.Flush();
                            log.InfoFormat(Path.GetFileNameWithoutExtension(filename) + " " + arr[lngpos] + " " + arr[latpos] + " " + arr[altpos] + "           ");
                            break;
                        }
                        //Console.WriteLine(crap);
                    }
                }


            }

            Serializer serializer = new Serializer();
            serializer.Serialize(kml);
            sw3.Write(serializer.Xml);
            sw3.Close();

            MainV2.instance.georefkml = serializer.Xml;

            writeGPX(dirWithImages + Path.DirectorySeparatorChar + "location.gpx");

            sw4.Close();

            sw2.Close();
            sw.Close();

            TXT_outputlog.AppendText("Done " + matchs + " matchs");
        }

        private void writeGPX(string filename)
        {

            using (System.Xml.XmlTextWriter xw = new System.Xml.XmlTextWriter(Path.GetDirectoryName(filename) + Path.DirectorySeparatorChar + Path.GetFileNameWithoutExtension(filename) + ".gpx", Encoding.ASCII))
            {

                xw.WriteStartElement("gpx");

                xw.WriteStartElement("trk");

                xw.WriteStartElement("trkseg");

                List<string> items = new List<string>();

                foreach (string photo in photocoords.Keys)
                {
                    items.Add(photo);
                }

                items.Sort();

                foreach (string photo in items)
                {


                    xw.WriteStartElement("trkpt");
                    xw.WriteAttributeString("lat", ((double[])photocoords[photo])[0].ToString(new System.Globalization.CultureInfo("en-US")));
                    xw.WriteAttributeString("lon", ((double[])photocoords[photo])[1].ToString(new System.Globalization.CultureInfo("en-US")));

                    // must stay as above

                    xw.WriteElementString("time", ((DateTime)filedatecache[photo]).ToString("yyyy-MM-ddTHH:mm:ssZ"));

                    xw.WriteElementString("ele", ((double[])photocoords[photo])[2].ToString(new System.Globalization.CultureInfo("en-US")));
                    xw.WriteElementString("course", ((double[])photocoords[photo])[3].ToString(new System.Globalization.CultureInfo("en-US")));

                    xw.WriteElementString("compass", ((double[])photocoords[photo])[3].ToString(new System.Globalization.CultureInfo("en-US")));

                    xw.WriteEndElement();
                }

                xw.WriteEndElement();
                xw.WriteEndElement();
                xw.WriteEndElement();

                xw.Close();
            }
        }

        private void InitializeComponent()
        {
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.TXT_logfile = new System.Windows.Forms.TextBox();
            this.TXT_jpgdir = new System.Windows.Forms.TextBox();
            this.TXT_offsetseconds = new System.Windows.Forms.TextBox();
            this.folderBrowserDialog1 = new System.Windows.Forms.FolderBrowserDialog();
            this.TXT_outputlog = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.BUT_Geotagimages = new ArdupilotMega.Controls.MyButton();
            this.BUT_estoffset = new ArdupilotMega.Controls.MyButton();
            this.BUT_doit = new ArdupilotMega.Controls.MyButton();
            this.BUT_browsedir = new ArdupilotMega.Controls.MyButton();
            this.BUT_browselog = new ArdupilotMega.Controls.MyButton();
            this.numericUpDown1 = new System.Windows.Forms.NumericUpDown();
            this.numericUpDown2 = new System.Windows.Forms.NumericUpDown();
            this.numericUpDown3 = new System.Windows.Forms.NumericUpDown();
            this.numericUpDown4 = new System.Windows.Forms.NumericUpDown();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.BUT_networklinkgeoref = new ArdupilotMega.Controls.MyButton();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown3)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown4)).BeginInit();
            this.SuspendLayout();
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // TXT_logfile
            // 
            this.TXT_logfile.Location = new System.Drawing.Point(28, 14);
            this.TXT_logfile.Name = "TXT_logfile";
            this.TXT_logfile.Size = new System.Drawing.Size(317, 20);
            this.TXT_logfile.TabIndex = 2;
            this.TXT_logfile.Text = "C:\\Users\\hog\\Pictures\\farm 1-10-2011\\100SSCAM\\2011-10-01 11-48 1.log";
            this.TXT_logfile.TextChanged += new System.EventHandler(this.TXT_logfile_TextChanged);
            // 
            // TXT_jpgdir
            // 
            this.TXT_jpgdir.Location = new System.Drawing.Point(28, 43);
            this.TXT_jpgdir.Name = "TXT_jpgdir";
            this.TXT_jpgdir.Size = new System.Drawing.Size(317, 20);
            this.TXT_jpgdir.TabIndex = 3;
            this.TXT_jpgdir.Text = "C:\\Users\\hog\\Pictures\\farm 1-10-2011\\100SSCAM";
            // 
            // TXT_offsetseconds
            // 
            this.TXT_offsetseconds.Location = new System.Drawing.Point(180, 69);
            this.TXT_offsetseconds.Name = "TXT_offsetseconds";
            this.TXT_offsetseconds.Size = new System.Drawing.Size(100, 20);
            this.TXT_offsetseconds.TabIndex = 4;
            this.TXT_offsetseconds.Text = "-86158";
            // 
            // TXT_outputlog
            // 
            this.TXT_outputlog.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.TXT_outputlog.Location = new System.Drawing.Point(28, 190);
            this.TXT_outputlog.Multiline = true;
            this.TXT_outputlog.Name = "TXT_outputlog";
            this.TXT_outputlog.ReadOnly = true;
            this.TXT_outputlog.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.TXT_outputlog.Size = new System.Drawing.Size(397, 160);
            this.TXT_outputlog.TabIndex = 6;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(94, 75);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(78, 13);
            this.label1.TabIndex = 7;
            this.label1.Text = "Seconds offset";
            // 
            // BUT_Geotagimages
            // 
            this.BUT_Geotagimages.Enabled = false;
            this.BUT_Geotagimages.Location = new System.Drawing.Point(259, 161);
            this.BUT_Geotagimages.Name = "BUT_Geotagimages";
            this.BUT_Geotagimages.Size = new System.Drawing.Size(75, 23);
            this.BUT_Geotagimages.TabIndex = 9;
            this.BUT_Geotagimages.Text = "GeoTag Images";
            this.BUT_Geotagimages.UseVisualStyleBackColor = true;
            this.BUT_Geotagimages.Click += new System.EventHandler(this.BUT_Geotagimages_Click);
            // 
            // BUT_estoffset
            // 
            this.BUT_estoffset.Location = new System.Drawing.Point(286, 67);
            this.BUT_estoffset.Name = "BUT_estoffset";
            this.BUT_estoffset.Size = new System.Drawing.Size(75, 23);
            this.BUT_estoffset.TabIndex = 8;
            this.BUT_estoffset.Text = "Estimate Offset";
            this.BUT_estoffset.UseVisualStyleBackColor = true;
            this.BUT_estoffset.Click += new System.EventHandler(this.BUT_estoffset_Click);
            // 
            // BUT_doit
            // 
            this.BUT_doit.Location = new System.Drawing.Point(97, 161);
            this.BUT_doit.Name = "BUT_doit";
            this.BUT_doit.Size = new System.Drawing.Size(75, 23);
            this.BUT_doit.TabIndex = 5;
            this.BUT_doit.Text = "Do It";
            this.BUT_doit.UseVisualStyleBackColor = true;
            this.BUT_doit.Click += new System.EventHandler(this.BUT_doit_Click);
            // 
            // BUT_browsedir
            // 
            this.BUT_browsedir.Location = new System.Drawing.Point(351, 41);
            this.BUT_browsedir.Name = "BUT_browsedir";
            this.BUT_browsedir.Size = new System.Drawing.Size(75, 23);
            this.BUT_browsedir.TabIndex = 1;
            this.BUT_browsedir.Text = "Browse Directory";
            this.BUT_browsedir.UseVisualStyleBackColor = true;
            this.BUT_browsedir.Click += new System.EventHandler(this.BUT_browsedir_Click);
            // 
            // BUT_browselog
            // 
            this.BUT_browselog.Location = new System.Drawing.Point(351, 12);
            this.BUT_browselog.Name = "BUT_browselog";
            this.BUT_browselog.Size = new System.Drawing.Size(75, 23);
            this.BUT_browselog.TabIndex = 0;
            this.BUT_browselog.Text = "Browse Log";
            this.BUT_browselog.UseVisualStyleBackColor = true;
            this.BUT_browselog.Click += new System.EventHandler(this.BUT_browselog_Click);
            // 
            // numericUpDown1
            // 
            this.numericUpDown1.Location = new System.Drawing.Point(130, 116);
            this.numericUpDown1.Name = "numericUpDown1";
            this.numericUpDown1.Size = new System.Drawing.Size(42, 20);
            this.numericUpDown1.TabIndex = 10;
            this.numericUpDown1.Value = new decimal(new int[] {
            4,
            0,
            0,
            0});
            // 
            // numericUpDown2
            // 
            this.numericUpDown2.Location = new System.Drawing.Point(178, 116);
            this.numericUpDown2.Name = "numericUpDown2";
            this.numericUpDown2.Size = new System.Drawing.Size(42, 20);
            this.numericUpDown2.TabIndex = 11;
            this.numericUpDown2.Value = new decimal(new int[] {
            5,
            0,
            0,
            0});
            // 
            // numericUpDown3
            // 
            this.numericUpDown3.Location = new System.Drawing.Point(226, 116);
            this.numericUpDown3.Name = "numericUpDown3";
            this.numericUpDown3.Size = new System.Drawing.Size(42, 20);
            this.numericUpDown3.TabIndex = 12;
            this.numericUpDown3.Value = new decimal(new int[] {
            7,
            0,
            0,
            0});
            // 
            // numericUpDown4
            // 
            this.numericUpDown4.Location = new System.Drawing.Point(274, 116);
            this.numericUpDown4.Name = "numericUpDown4";
            this.numericUpDown4.Size = new System.Drawing.Size(42, 20);
            this.numericUpDown4.TabIndex = 13;
            this.numericUpDown4.Value = new decimal(new int[] {
            9,
            0,
            0,
            0});
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(127, 100);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(22, 13);
            this.label2.TabIndex = 14;
            this.label2.Text = "Lat";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(175, 100);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(25, 13);
            this.label3.TabIndex = 15;
            this.label3.Text = "Lon";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(223, 100);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(19, 13);
            this.label4.TabIndex = 16;
            this.label4.Text = "Alt";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(271, 100);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(47, 13);
            this.label5.TabIndex = 17;
            this.label5.Text = "Heading";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(63, 118);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(61, 13);
            this.label6.TabIndex = 18;
            this.label6.Text = "Log Offsets";
            // 
            // BUT_networklinkgeoref
            // 
            this.BUT_networklinkgeoref.Location = new System.Drawing.Point(178, 161);
            this.BUT_networklinkgeoref.Name = "BUT_networklinkgeoref";
            this.BUT_networklinkgeoref.Size = new System.Drawing.Size(75, 23);
            this.BUT_networklinkgeoref.TabIndex = 19;
            this.BUT_networklinkgeoref.Text = "Location Kml";
            this.BUT_networklinkgeoref.UseVisualStyleBackColor = true;
            this.BUT_networklinkgeoref.Click += new System.EventHandler(this.BUT_networklinkgeoref_Click);
            // 
            // Georefimage
            // 
            this.ClientSize = new System.Drawing.Size(452, 362);
            this.Controls.Add(this.BUT_networklinkgeoref);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.numericUpDown4);
            this.Controls.Add(this.numericUpDown3);
            this.Controls.Add(this.numericUpDown2);
            this.Controls.Add(this.numericUpDown1);
            this.Controls.Add(this.BUT_Geotagimages);
            this.Controls.Add(this.BUT_estoffset);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.TXT_outputlog);
            this.Controls.Add(this.BUT_doit);
            this.Controls.Add(this.TXT_offsetseconds);
            this.Controls.Add(this.TXT_jpgdir);
            this.Controls.Add(this.TXT_logfile);
            this.Controls.Add(this.BUT_browsedir);
            this.Controls.Add(this.BUT_browselog);
            this.Name = "Georefimage";
            this.Text = "Geo Ref Images";
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown3)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown4)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        private void BUT_browselog_Click(object sender, EventArgs e)
        {
            logcache.Clear();

            openFileDialog1.Filter = "Logs|*.log;*.tlog";
            openFileDialog1.ShowDialog();

            if (File.Exists(openFileDialog1.FileName))
            {
                TXT_logfile.Text = openFileDialog1.FileName;
            }
        }

        private void BUT_browsedir_Click(object sender, EventArgs e)
        {
            folderBrowserDialog1.ShowDialog();

            if (folderBrowserDialog1.SelectedPath != "")
            {
                TXT_jpgdir.Text = folderBrowserDialog1.SelectedPath;
            }
        }

        private void BUT_doit_Click(object sender, EventArgs e)
        {
            if (!File.Exists(TXT_logfile.Text))
                return;
            if (!Directory.Exists(TXT_jpgdir.Text))
                return;
            float seconds;
            if (float.TryParse(TXT_offsetseconds.Text, out seconds) == false)
                return;

            BUT_doit.Enabled = false;
            try
            {
                dowork(TXT_logfile.Text, TXT_jpgdir.Text, seconds, false);
            }
            catch { }
            BUT_doit.Enabled = true;
            BUT_Geotagimages.Enabled = true;
        }

        private void BUT_estoffset_Click(object sender, EventArgs e)
        {
            dowork(TXT_logfile.Text, TXT_jpgdir.Text, 0, true);
        }

        private void BUT_Geotagimages_Click(object sender, EventArgs e)
        {

            foreach (string file in photocoords.Keys)
            {
                WriteCoordinatesToImage(file, ((double[])photocoords[file])[0], ((double[])photocoords[file])[1], ((double[])photocoords[file])[2]);
            }

        }

        byte[] coordtobytearray(double coordin)
        {
            double coord = Math.Abs(coordin);

            byte[] output = new byte[sizeof(double) * 3];

            int d = (int)coord;
            int m = (int)((coord - d) * 60);
            double s = ((((coord - d) * 60) - m) * 60);
            /*
21 00 00 00 01 00 00 00--> 33/1
18 00 00 00 01 00 00 00--> 24/1
06 02 00 00 0A 00 00 00--> 518/10
*/

            Array.Copy(BitConverter.GetBytes((uint)d), 0, output, 0, sizeof(uint));
            Array.Copy(BitConverter.GetBytes((uint)1), 0, output, 4, sizeof(uint));
            Array.Copy(BitConverter.GetBytes((uint)m), 0, output, 8, sizeof(uint));
            Array.Copy(BitConverter.GetBytes((uint)1), 0, output, 12, sizeof(uint));
            Array.Copy(BitConverter.GetBytes((uint)(s * 10)), 0, output, 16, sizeof(uint));
            Array.Copy(BitConverter.GetBytes((uint)10), 0, output, 20, sizeof(uint));

            return output;
        }

        void WriteCoordinatesToImage(string Filename, double dLat, double dLong, double alt)
        {
            using (MemoryStream ms = new MemoryStream(File.ReadAllBytes(Filename)))
            {
                TXT_outputlog.AppendText("GeoTagging "+Filename + "\n");
                Application.DoEvents();

                using (Image Pic = Image.FromStream(ms))
                {
                    PropertyItem[] pi = Pic.PropertyItems;

                    pi[0].Id = 0x0004;
                    pi[0].Type = 5;
                    pi[0].Len = sizeof(ulong) * 3;
                    pi[0].Value = coordtobytearray(dLong);
                    Pic.SetPropertyItem(pi[0]);

                    pi[0].Id = 0x0002;
                    pi[0].Type = 5;
                    pi[0].Len = sizeof(ulong) * 3;
                    pi[0].Value = coordtobytearray(dLat);
                    Pic.SetPropertyItem(pi[0]);

                    pi[0].Id = 0x0006;
                    pi[0].Type = 5;
                    pi[0].Len = 8;
                    pi[0].Value = new Rational(alt).GetBytes();
                    Pic.SetPropertyItem(pi[0]);

                    pi[0].Id = 1;
                    pi[0].Len = 2;
                    pi[0].Type = 2;
                    if (dLat < 0)
                    {
                        pi[0].Value = new byte[] { (byte)'S', 0 };
                    }
                    else
                    {
                        pi[0].Value = new byte[] { (byte)'N', 0 };
                    }
                    Pic.SetPropertyItem(pi[0]);

                    pi[0].Id = 3;
                    pi[0].Len = 2;
                    pi[0].Type = 2;
                    if (dLong < 0)
                    {
                        pi[0].Value = new byte[] { (byte)'W', 0 };
                    }
                    else
                    {
                        pi[0].Value = new byte[] { (byte)'E', 0 };
                    }
                    Pic.SetPropertyItem(pi[0]);



                    string outputfilename = Path.GetDirectoryName(Filename) + Path.DirectorySeparatorChar + Path.GetFileNameWithoutExtension(Filename) + "_geotag" + Path.GetExtension(Filename);

                    File.Delete(outputfilename);

                    Pic.Save(outputfilename);
                }
            }
        }

        private void BUT_networklinkgeoref_Click(object sender, EventArgs e)
        {
            System.Diagnostics.Process.Start(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "m3u" + Path.DirectorySeparatorChar + "GeoRefnetworklink.kml");
        }

        private void TXT_logfile_TextChanged(object sender, EventArgs e)
        {
            logcache.Clear();
        }
    }

    public class Rational
    {
        uint dem = 0;
        uint num = 0;

        public Rational(double input)
        {
            Value = input;
        }

        public byte[] GetBytes()
        {
            byte[] answer = new byte[8];

            Array.Copy(BitConverter.GetBytes((uint)num), 0, answer, 0, sizeof(uint));
            Array.Copy(BitConverter.GetBytes((uint)dem), 0, answer, 4, sizeof(uint));

            return answer;
        }

        public double Value { 
            get { 
                return num / dem;
            } 
            set {
                if ((value % 1.0) != 0)
                {
                    dem = 100; num = (uint)(value * dem);
                }
                else
                {
                    dem = 1; num = (uint)(value);
                }
            } 
        }
    }
}

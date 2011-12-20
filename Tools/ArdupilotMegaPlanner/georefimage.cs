using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Windows.Forms;
using com.drew.imaging.jpg;
using com.drew.metadata;

using SharpKml.Base;
using SharpKml.Dom;
using SharpKml.Dom.GX;

namespace ArdupilotMega
{
    class georefimage : Form
    {
        private OpenFileDialog openFileDialog1;
        private MyButton BUT_browselog;
        private MyButton BUT_browsedir;
        private TextBox TXT_logfile;
        private TextBox TXT_jpgdir;
        private TextBox TXT_offsetseconds;
        private MyButton BUT_doit;
        private FolderBrowserDialog folderBrowserDialog1;
        private Label label1;
        private TextBox TXT_outputlog;
        private MyButton BUT_estoffset;

        int latpos = 5, lngpos = 4, altpos = 7;

        internal georefimage() {
            InitializeComponent();
        }

        DateTime getPhotoTime(string fn)
        {
            DateTime dtaken = DateTime.MinValue;

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
                    Console.Error.WriteLine(e.Message);
                    return dtaken;
                }

                foreach (AbstractDirectory lcDirectory in lcMetadata)
                {

                    if (lcDirectory.ContainsTag(0x9003))
                    {
                        dtaken = lcDirectory.GetDate(0x9003);
                        Console.WriteLine("does " + lcDirectory.GetTagName(0x9003) + " " + dtaken);
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
            List<string[]> list = new List<string[]>();

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

            return list;
        }

        public void dowork(string logFile, string dirWithImages, float offsetseconds, bool dooffset)
        {
            DateTime localmin = DateTime.MaxValue;
            DateTime localmax = DateTime.MinValue;

            DateTime startTime = DateTime.MinValue;

            //logFile = @"C:\Users\hog\Pictures\farm 1-10-2011\100SSCAM\2011-10-01 11-48 1.log";

            List<string[]> list = readLog(logFile);

            //dirWithImages = @"C:\Users\hog\Pictures\farm 1-10-2011\100SSCAM";

            string[] files = Directory.GetFiles(dirWithImages);

            Document kml = new Document();

            StreamWriter sw3 = new StreamWriter(dirWithImages + Path.DirectorySeparatorChar + "location.kml");

            StreamWriter sw2 = new StreamWriter(dirWithImages + Path.DirectorySeparatorChar + "location.txt");

            StreamWriter sw = new StreamWriter(dirWithImages + Path.DirectorySeparatorChar + "location.tel");
            sw.WriteLine("version=1");
            sw.WriteLine("#longitude and latitude - in degrees");
            sw.WriteLine("#name	utc	longitude	latitude	height");

            int first = 0;
            int matchs = 0;

            foreach (string file in files)
            {
                if (file.ToLower().EndsWith(".jpg"))
                {
                    DateTime dt = getPhotoTime(file);

                    if (startTime == DateTime.MinValue)
                    {
                        startTime = new DateTime(dt.Year, dt.Month, dt.Day, 0, 0, 0, 0, DateTimeKind.Utc).ToLocalTime();

                        foreach (string[] arr in list)
                        {
                            DateTime crap = startTime.AddMilliseconds(int.Parse(arr[1])).AddSeconds(offsetseconds);

                            if (localmin > crap)
                                localmin = crap;
                            if (localmax < crap)
                                localmax = crap;
                        }

                        Console.WriteLine("min " + localmin + " max " + localmax);
                        TXT_outputlog.AppendText("Log min " + localmin + " max " + localmax + "\r\n");
                    }

                    TXT_outputlog.AppendText("Photo  " + Path.GetFileNameWithoutExtension(file) + " time  " + dt + "\r\n");

                    foreach (string[] arr in list)
                    {
                        //Application.DoEvents();

                        DateTime crap = startTime.AddMilliseconds(int.Parse(arr[1])).AddSeconds(offsetseconds);


                        if (first == 0)
                        {
                            TXT_outputlog.AppendText("Photo " + Path.GetFileNameWithoutExtension(file) + " " + dt + " vs Log " + crap + "\r\n");

                            TXT_outputlog.AppendText("offset should be about " + (dt - crap).TotalSeconds + "\r\n");

                            if (dooffset)
                                return;

                            first++;
                        }

                        //Console.Write("ph " + dt + " log " + crap + "         \r");

                        if (dt.Equals(crap))
                        {
                            TXT_outputlog.AppendText("MATCH Photo " + Path.GetFileNameWithoutExtension(file) + " " + dt + "\r\n");

                            matchs++;

                            kml.AddFeature(
                                new Placemark()
                                {
                                    Name = Path.GetFileNameWithoutExtension(file),
                                    Geometry = new SharpKml.Dom.Point()
                                    {
                                        Coordinate = new Vector(double.Parse(arr[lngpos]), double.Parse(arr[latpos]), double.Parse(arr[altpos]))
                                    }
                                }
                            );

                            sw2.WriteLine(Path.GetFileNameWithoutExtension(file) + " " + arr[lngpos] + " " + arr[latpos] + " " + arr[altpos]);
                            sw.WriteLine(Path.GetFileNameWithoutExtension(file) + "\t" + crap.ToString("yyyy:MM:dd HH:mm:ss") + "\t" + arr[lngpos] + "\t" + arr[latpos] + "\t" + arr[altpos]);
                            sw.Flush();
                            sw2.Flush();
                            Console.WriteLine(Path.GetFileNameWithoutExtension(file) + " " + arr[lngpos] + " " + arr[latpos] + " " + arr[altpos] + "           ");
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


            sw2.Close();
            sw.Close();

            MessageBox.Show("Done " + matchs + " matchs");
        }

        private void InitializeComponent()
        {
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.BUT_browselog = new ArdupilotMega.MyButton();
            this.BUT_browsedir = new ArdupilotMega.MyButton();
            this.TXT_logfile = new System.Windows.Forms.TextBox();
            this.TXT_jpgdir = new System.Windows.Forms.TextBox();
            this.TXT_offsetseconds = new System.Windows.Forms.TextBox();
            this.BUT_doit = new ArdupilotMega.MyButton();
            this.folderBrowserDialog1 = new System.Windows.Forms.FolderBrowserDialog();
            this.TXT_outputlog = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.BUT_estoffset = new ArdupilotMega.MyButton();
            this.SuspendLayout();
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
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
            // TXT_logfile
            // 
            this.TXT_logfile.Location = new System.Drawing.Point(28, 14);
            this.TXT_logfile.Name = "TXT_logfile";
            this.TXT_logfile.Size = new System.Drawing.Size(317, 20);
            this.TXT_logfile.TabIndex = 2;
            this.TXT_logfile.Text = "C:\\Users\\hog\\Pictures\\farm 1-10-2011\\100SSCAM\\2011-10-01 11-48 1.log";
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
            this.TXT_offsetseconds.Text = "0";
            // 
            // BUT_doit
            // 
            this.BUT_doit.Location = new System.Drawing.Point(194, 105);
            this.BUT_doit.Name = "BUT_doit";
            this.BUT_doit.Size = new System.Drawing.Size(75, 23);
            this.BUT_doit.TabIndex = 5;
            this.BUT_doit.Text = "Do It";
            this.BUT_doit.UseVisualStyleBackColor = true;
            this.BUT_doit.Click += new System.EventHandler(this.BUT_doit_Click);
            // 
            // TXT_outputlog
            // 
            this.TXT_outputlog.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.TXT_outputlog.Location = new System.Drawing.Point(28, 144);
            this.TXT_outputlog.Multiline = true;
            this.TXT_outputlog.Name = "TXT_outputlog";
            this.TXT_outputlog.ReadOnly = true;
            this.TXT_outputlog.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.TXT_outputlog.Size = new System.Drawing.Size(398, 143);
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
            // georefimage
            // 
            this.ClientSize = new System.Drawing.Size(453, 299);
            this.Controls.Add(this.BUT_estoffset);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.TXT_outputlog);
            this.Controls.Add(this.BUT_doit);
            this.Controls.Add(this.TXT_offsetseconds);
            this.Controls.Add(this.TXT_jpgdir);
            this.Controls.Add(this.TXT_logfile);
            this.Controls.Add(this.BUT_browsedir);
            this.Controls.Add(this.BUT_browselog);
            this.Name = "georefimage";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        private void BUT_browselog_Click(object sender, EventArgs e)
        {
            openFileDialog1.Filter = "Logs|*.log";
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
        }

        private void BUT_estoffset_Click(object sender, EventArgs e)
        {
            dowork(TXT_logfile.Text, TXT_jpgdir.Text, 0, true);
        }
    }
}

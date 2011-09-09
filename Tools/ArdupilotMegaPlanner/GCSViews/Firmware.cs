using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Text.RegularExpressions;
using System.IO.Ports;
using System.IO;
using System.Runtime.InteropServices;
using System.Xml;
using System.Net;

namespace ArdupilotMega.GCSViews
{
    class Firmware : MyUserControl
    {
        private System.Windows.Forms.PictureBox pictureBoxAPM;
        private System.Windows.Forms.PictureBox pictureBoxAPMHIL;
        private System.Windows.Forms.PictureBox pictureBoxQuad;
        private System.Windows.Forms.PictureBox pictureBoxHexa;
        private System.Windows.Forms.PictureBox pictureBoxTri;
        private System.Windows.Forms.PictureBox pictureBoxY6;
        private System.Windows.Forms.Label lbl_status;
        private System.Windows.Forms.ProgressBar progress;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private Label label8;
        private PictureBox pictureBoxHeli;
        private MyButton BUT_setup;
        private Label label9;
        private PictureBox pictureBoxQuadHil;
        private System.Windows.Forms.Label label7;

        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Firmware));
            this.pictureBoxAPM = new System.Windows.Forms.PictureBox();
            this.pictureBoxAPMHIL = new System.Windows.Forms.PictureBox();
            this.pictureBoxQuad = new System.Windows.Forms.PictureBox();
            this.pictureBoxHexa = new System.Windows.Forms.PictureBox();
            this.pictureBoxTri = new System.Windows.Forms.PictureBox();
            this.pictureBoxY6 = new System.Windows.Forms.PictureBox();
            this.lbl_status = new System.Windows.Forms.Label();
            this.progress = new System.Windows.Forms.ProgressBar();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.pictureBoxHeli = new System.Windows.Forms.PictureBox();
            this.label9 = new System.Windows.Forms.Label();
            this.pictureBoxQuadHil = new System.Windows.Forms.PictureBox();
            this.BUT_setup = new ArdupilotMega.MyButton();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxAPM)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxAPMHIL)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxQuad)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxHexa)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxTri)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxY6)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxHeli)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxQuadHil)).BeginInit();
            this.SuspendLayout();
            // 
            // pictureBoxAPM
            // 
            this.pictureBoxAPM.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pictureBoxAPM.Image = global::ArdupilotMega.Properties.Resources.APM_airframes_001;
            resources.ApplyResources(this.pictureBoxAPM, "pictureBoxAPM");
            this.pictureBoxAPM.Name = "pictureBoxAPM";
            this.pictureBoxAPM.TabStop = false;
            this.pictureBoxAPM.Click += new System.EventHandler(this.pictureBoxAPM_Click);
            // 
            // pictureBoxAPMHIL
            // 
            this.pictureBoxAPMHIL.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pictureBoxAPMHIL.Image = global::ArdupilotMega.Properties.Resources.APM_airframes_002;
            resources.ApplyResources(this.pictureBoxAPMHIL, "pictureBoxAPMHIL");
            this.pictureBoxAPMHIL.Name = "pictureBoxAPMHIL";
            this.pictureBoxAPMHIL.TabStop = false;
            this.pictureBoxAPMHIL.Click += new System.EventHandler(this.pictureBoxAPMHIL_Click);
            // 
            // pictureBoxQuad
            // 
            this.pictureBoxQuad.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pictureBoxQuad.Image = global::ArdupilotMega.Properties.Resources.frames_03;
            resources.ApplyResources(this.pictureBoxQuad, "pictureBoxQuad");
            this.pictureBoxQuad.Name = "pictureBoxQuad";
            this.pictureBoxQuad.TabStop = false;
            this.pictureBoxQuad.Click += new System.EventHandler(this.pictureBoxQuad_Click);
            // 
            // pictureBoxHexa
            // 
            this.pictureBoxHexa.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pictureBoxHexa.Image = global::ArdupilotMega.Properties.Resources.frames_07;
            resources.ApplyResources(this.pictureBoxHexa, "pictureBoxHexa");
            this.pictureBoxHexa.Name = "pictureBoxHexa";
            this.pictureBoxHexa.TabStop = false;
            this.pictureBoxHexa.Click += new System.EventHandler(this.pictureBoxHexa_Click);
            // 
            // pictureBoxTri
            // 
            this.pictureBoxTri.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pictureBoxTri.Image = global::ArdupilotMega.Properties.Resources.frames_05;
            resources.ApplyResources(this.pictureBoxTri, "pictureBoxTri");
            this.pictureBoxTri.Name = "pictureBoxTri";
            this.pictureBoxTri.TabStop = false;
            this.pictureBoxTri.Click += new System.EventHandler(this.pictureBoxTri_Click);
            // 
            // pictureBoxY6
            // 
            this.pictureBoxY6.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pictureBoxY6.Image = global::ArdupilotMega.Properties.Resources.frames_08;
            resources.ApplyResources(this.pictureBoxY6, "pictureBoxY6");
            this.pictureBoxY6.Name = "pictureBoxY6";
            this.pictureBoxY6.TabStop = false;
            this.pictureBoxY6.Click += new System.EventHandler(this.pictureBoxY6_Click);
            // 
            // lbl_status
            // 
            resources.ApplyResources(this.lbl_status, "lbl_status");
            this.lbl_status.Name = "lbl_status";
            // 
            // progress
            // 
            resources.ApplyResources(this.progress, "progress");
            this.progress.Name = "progress";
            this.progress.Step = 1;
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // label3
            // 
            resources.ApplyResources(this.label3, "label3");
            this.label3.Name = "label3";
            // 
            // label4
            // 
            resources.ApplyResources(this.label4, "label4");
            this.label4.Name = "label4";
            // 
            // label5
            // 
            resources.ApplyResources(this.label5, "label5");
            this.label5.Name = "label5";
            // 
            // label6
            // 
            resources.ApplyResources(this.label6, "label6");
            this.label6.Name = "label6";
            // 
            // label7
            // 
            resources.ApplyResources(this.label7, "label7");
            this.label7.Name = "label7";
            // 
            // label8
            // 
            resources.ApplyResources(this.label8, "label8");
            this.label8.Name = "label8";
            // 
            // pictureBoxHeli
            // 
            this.pictureBoxHeli.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pictureBoxHeli.Image = global::ArdupilotMega.Properties.Resources.APM_airframes_08;
            resources.ApplyResources(this.pictureBoxHeli, "pictureBoxHeli");
            this.pictureBoxHeli.Name = "pictureBoxHeli";
            this.pictureBoxHeli.TabStop = false;
            this.pictureBoxHeli.Click += new System.EventHandler(this.pictureBoxHeli_Click);
            // 
            // label9
            // 
            resources.ApplyResources(this.label9, "label9");
            this.label9.Name = "label9";
            // 
            // pictureBoxQuadHil
            // 
            this.pictureBoxQuadHil.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pictureBoxQuadHil.Image = global::ArdupilotMega.Properties.Resources.new_frames_09;
            resources.ApplyResources(this.pictureBoxQuadHil, "pictureBoxQuadHil");
            this.pictureBoxQuadHil.Name = "pictureBoxQuadHil";
            this.pictureBoxQuadHil.TabStop = false;
            this.pictureBoxQuadHil.Click += new System.EventHandler(this.pictureBoxQuadHil_Click);
            // 
            // BUT_setup
            // 
            resources.ApplyResources(this.BUT_setup, "BUT_setup");
            this.BUT_setup.Name = "BUT_setup";
            this.BUT_setup.UseVisualStyleBackColor = true;
            this.BUT_setup.Click += new System.EventHandler(this.BUT_setup_Click);
            // 
            // Firmware
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.label9);
            this.Controls.Add(this.pictureBoxQuadHil);
            this.Controls.Add(this.BUT_setup);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.pictureBoxHeli);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.lbl_status);
            this.Controls.Add(this.progress);
            this.Controls.Add(this.pictureBoxY6);
            this.Controls.Add(this.pictureBoxTri);
            this.Controls.Add(this.pictureBoxHexa);
            this.Controls.Add(this.pictureBoxQuad);
            this.Controls.Add(this.pictureBoxAPMHIL);
            this.Controls.Add(this.pictureBoxAPM);
            this.MinimumSize = new System.Drawing.Size(1008, 461);
            this.Name = "Firmware";
            this.Load += new System.EventHandler(this.FirmwareVisual_Load);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxAPM)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxAPMHIL)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxQuad)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxHexa)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxTri)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxY6)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxHeli)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxQuadHil)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            if (keyData == (Keys.Control | Keys.B))
            {
                findfirmware("APM-trunk");
                return true;
            }
            if (keyData == (Keys.Control | Keys.A))
            {
                findfirmware("AC2-QUADHIL");
                return true;
            }
            return base.ProcessCmdKey(ref msg, keyData);
        }

        List<software> softwares = new List<software>();
        bool flashing = false;

        public struct software
        {
            public string url;
            public string url2560;
            public string name;
            public string desc;
            public int k_format_version;
        }

        FRAMETYPES currentframe = FRAMETYPES.NONE;

        public enum FRAMETYPES
        {
            NONE,
            TRI,
            QUAD,
            HEXA,
            Y6,
            APM,
            APMHIL,
            HELI
        }

        public Firmware()
        {
            InitializeComponent();
            WebRequest.DefaultWebProxy.Credentials = System.Net.CredentialCache.DefaultCredentials;
        }

        private void FirmwareVisual_Load(object sender, EventArgs e)
        {
            string url = "";
            string url2560 = "";
            string name = "";
            string desc = "";
            int k_format_version = 0;

            software temp = new software();

            try
            {

                using (XmlTextReader xmlreader = new XmlTextReader("http://ardupilot-mega.googlecode.com/svn/Tools/trunk/ArdupilotMegaPlanner/Firmware/firmware2.xml"))
                {
                    while (xmlreader.Read())
                    {
                        xmlreader.MoveToElement();
                        switch (xmlreader.Name)
                        {
                            case "url":
                                url = xmlreader.ReadString();
                                break;
                            case "url2560":
                                url2560 = xmlreader.ReadString();
                                break;
                            case "name":
                                name = xmlreader.ReadString();
                                break;
                            case "format_version":
                                k_format_version = int.Parse(xmlreader.ReadString());
                                break;
                            case "desc":
                                desc = xmlreader.ReadString();
                                break;
                            case "Firmware":
                                if (!url.Equals("") && !name.Equals("") && !desc.Equals("Please Update"))
                                {
                                    temp.desc = desc;
                                    temp.name = name;
                                    temp.url = url;
                                    temp.url2560 = url2560;
                                    temp.k_format_version = k_format_version;

                                    softwares.Add(temp);
                                }
                                url = "";
                                url2560 = "";
                                name = "";
                                desc = "";
                                k_format_version = 0;
                                temp = new software();
                                break;
                            default:
                                break;
                        }
                    }
                }

                List<string> list = new List<string>();

            }
            catch (Exception ex) { MessageBox.Show("Failed to get Firmware List : " + ex.Message); }
        }

        void findfirmware(string findwhat)
        {
            foreach (software temp in softwares)
            {
                if (temp.url.ToLower().Contains(findwhat.ToLower()))
                {
                    DialogResult dr = MessageBox.Show("Are you sure you want to upload " + temp.name + "?", "Continue", MessageBoxButtons.YesNo);
                    if (dr == System.Windows.Forms.DialogResult.Yes)
                    {
                        update(temp);
                    }
                    return;
                }
            }

            MessageBox.Show("The requested firmware was not found.");
        }

        private void pictureBoxAPM_Click(object sender, EventArgs e)
        {
            currentframe = FRAMETYPES.APM;
            findfirmware("APM2-");
        }

        private void pictureBoxAPMHIL_Click(object sender, EventArgs e)
        {
            currentframe = FRAMETYPES.APMHIL;
            findfirmware("APM2HIL-");
        }

        private void pictureBoxQuad_Click(object sender, EventArgs e)
        {
            currentframe = FRAMETYPES.QUAD;
            findfirmware("AC2-Quad-");
        }

        private void pictureBoxHexa_Click(object sender, EventArgs e)
        {
            currentframe = FRAMETYPES.HEXA;
            findfirmware("AC2-Hexa-");
        }

        private void pictureBoxTri_Click(object sender, EventArgs e)
        {
            currentframe = FRAMETYPES.TRI;
            findfirmware("AC2-Tri-");
        }

        private void pictureBoxY6_Click(object sender, EventArgs e)
        {
            currentframe = FRAMETYPES.Y6;
            findfirmware("AC2-Y6-");
        }

        private void pictureBoxHeli_Click(object sender, EventArgs e)
        {
            currentframe = FRAMETYPES.HELI;
            findfirmware("AC2-Heli-");
        }

        private void pictureBoxQuadHil_Click(object sender, EventArgs e)
        {
            currentframe = FRAMETYPES.QUAD;
            findfirmware("AC2-QUADHIL");
        }

        private void update(software temp)
        {
            string board = "";
            MainV2.comPort.BaseStream.DtrEnable = false;
            MainV2.comPort.Close();
            System.Threading.Thread.Sleep(100);
            MainV2.givecomport = true;

            try
            {
                if (softwares.Count == 0)
                {
                    MessageBox.Show("No valid options");
                    return;
                }

                lbl_status.Text = "Detecting APM Version";

                this.Refresh();

                board = ArduinoDetect.DetectVersion(MainV2.comportname);

                if (board == "")
                {
                    MessageBox.Show("Cant detect your APM version. Please check your cabling");
                    return;
                }

                int apmformat_version = ArduinoDetect.decodeApVar(MainV2.comportname, board);

                if (apmformat_version != -1 && apmformat_version != temp.k_format_version)
                {
                    if (DialogResult.No == MessageBox.Show("Epprom changed, all your setting will be lost during the update,\nDo you wish to continue?", "Epprom format changed (" + apmformat_version + " vs " + temp.k_format_version + ")", MessageBoxButtons.YesNo))
                    {
                        MessageBox.Show("Please connect and backup your config in the configuration tab.");
                        return;
                    }
                }



                Console.WriteLine("Detected a " + board);

                string baseurl = "";
                if (board == "2560")
                {
                    baseurl = temp.url2560.ToString();
                }
                else
                {
                    baseurl = temp.url.ToString();
                }

                // Create a request using a URL that can receive a post. 
                WebRequest request = WebRequest.Create(baseurl);
                request.Timeout = 10000;
                // Set the Method property of the request to POST.
                request.Method = "GET";
                // Get the request stream.
                Stream dataStream; //= request.GetRequestStream();
                // Get the response.
                WebResponse response = request.GetResponse();
                // Display the status.
                Console.WriteLine(((HttpWebResponse)response).StatusDescription);
                // Get the stream containing content returned by the server.
                dataStream = response.GetResponseStream();

                long bytes = response.ContentLength;
                long contlen = bytes;

                byte[] buf1 = new byte[1024];

                FileStream fs = new FileStream(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"firmware.hex", FileMode.Create);

                lbl_status.Text = "Downloading from Internet";

                this.Refresh();

                while (dataStream.CanRead && bytes > 0)
                {
                    try
                    {
                        progress.Value = (int)(((float)(response.ContentLength - bytes) / (float)response.ContentLength) * 100);
                        this.progress.Refresh();
                    }
                    catch { }
                    int len = dataStream.Read(buf1, 0, 1024);
                    bytes -= len;
                    fs.Write(buf1, 0, len);
                }

                fs.Close();
                dataStream.Close();
                response.Close();

                progress.Value = 100;
                this.Refresh();
                Console.WriteLine("Downloaded");
            }
            catch (Exception ex) { lbl_status.Text = "Failed download"; MessageBox.Show("Failed to download new firmware : " + ex.Message); return; }

            byte[] FLASH = new byte[1];
            StreamReader sr = null;
            try
            {
                lbl_status.Text = "Reading Hex File";
                this.Refresh();
                sr = new StreamReader(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"firmware.hex");
                FLASH = readIntelHEXv2(sr);
                sr.Close();
                Console.WriteLine("\n\nSize: {0}\n\n", FLASH.Length);
            }
            catch (Exception ex) { if (sr != null) { sr.Dispose(); } lbl_status.Text = "Failed read HEX"; MessageBox.Show("Failed to read firmware.hex : " + ex.Message); return; }
            ArduinoComms port = new ArduinoSTK();

            if (board == "1280")
            {
                //port = new ArduinoSTK();
                port.BaudRate = 57600;
            }
            else if (board == "2560")
            {
                port = new ArduinoSTKv2();
                port.BaudRate = 115200;
            }
            port.DataBits = 8;
            port.StopBits = StopBits.One;
            port.Parity = Parity.None;
            port.DtrEnable = true;

            try
            {
                port.PortName = MainV2.comportname;

                port.Open();

                flashing = true;

                if (port.connectAP())
                {
                    Console.WriteLine("starting");
                    lbl_status.Text = "Uploading " + FLASH.Length + " bytes to APM";
                    progress.Value = 0;
                    this.Refresh();

                    // this is enough to make ap_var reset
                    //port.upload(new byte[256], 0, 2, 0);

                    port.Progress += new ProgressEventHandler(port_Progress);

                    if (!port.uploadflash(FLASH, 0, FLASH.Length, 0))
                    {
                        flashing = false;
                        if (port.IsOpen)
                            port.Close();
                        throw new Exception("Upload failed. Lost sync. Try Arduino!!");
                    }

                    port.Progress -= new ProgressEventHandler(port_Progress);

                    progress.Value = 100;

                    Console.WriteLine("Uploaded");

                    this.Refresh();

                    int start = 0;
                    short length = 0x100;

                    byte[] flashverify = new byte[FLASH.Length + 256];

                    lbl_status.Text = "Verify APM";
                    progress.Value = 0;
                    this.Refresh();

                    while (start < FLASH.Length)
                    {
                        progress.Value = (int)((start / (float)FLASH.Length) * 100);
                        progress.Refresh();
                        port.setaddress(start);
                        Console.WriteLine("Downloading " + length + " at " + start);
                        port.downloadflash(length).CopyTo(flashverify, start);
                        start += length;
                    }

                    progress.Value = 100;

                    for (int s = 0; s < FLASH.Length; s++)
                    {
                        if (FLASH[s] != flashverify[s])
                        {
                            MessageBox.Show("Upload succeeded, but verify failed: exp " + FLASH[s].ToString("X") + " got " + flashverify[s].ToString("X") + " at " + s);
                            break;
                        }
                    }

                    lbl_status.Text = "Done";
                }
                else
                {
                    lbl_status.Text = "Failed upload";
                    MessageBox.Show("Communication Error - no connection");
                }
                port.Close();

                flashing = false;

                Application.DoEvents();

                System.Threading.Thread.Sleep(5000); // 5 seconds - new apvar erases eeprom on new format version, this should buy us some time.

            }
            catch (Exception ex) { lbl_status.Text = "Failed upload"; MessageBox.Show("Check port settings or Port in use? " + ex.ToString()); port.Close(); }
            flashing = false;
            MainV2.givecomport = false;
        }

        void port_Progress(int progress)
        {
            Console.WriteLine("Progress {0} ", progress);
            this.progress.Value = progress;
            this.progress.Refresh();
        }

        byte[] readIntelHEXv2(StreamReader sr)
        {
            byte[] FLASH = new byte[sr.BaseStream.Length / 2];

            int optionoffset = 0;
            int total = 0;

            while (!sr.EndOfStream)
            {
                progress.Value = (int)(((float)sr.BaseStream.Position / (float)sr.BaseStream.Length) * 100);
                progress.Refresh();

                string line = sr.ReadLine();

                if (line.StartsWith(":"))
                {
                    int length = Convert.ToInt32(line.Substring(1, 2), 16);
                    int address = Convert.ToInt32(line.Substring(3, 4), 16);
                    int option = Convert.ToInt32(line.Substring(7, 2), 16);
                    Console.WriteLine("len {0} add {1} opt {2}", length, address, option);

                    if (option == 0)
                    {
                        string data = line.Substring(9, length * 2);
                        for (int i = 0; i < length; i++)
                        {
                            byte byte1 = Convert.ToByte(data.Substring(i * 2, 2), 16);
                            FLASH[optionoffset + address] = byte1;
                            address++;
                            if ((optionoffset + address) > total)
                                total = optionoffset + address;
                        }
                    }
                    else if (option == 2)
                    {
                        optionoffset += (int)Convert.ToUInt16(line.Substring(9, 4), 16) << 4;
                    }
                    int checksum = Convert.ToInt32(line.Substring(line.Length - 2, 2), 16);
                }
                //Regex regex = new Regex(@"^:(..)(....)(..)(.*)(..)$"); // length - address - option - data - checksum
            }

            Array.Resize<byte>(ref FLASH, total);

            return FLASH;
        }

        private void FirmwareVisual_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (flashing == true)
            {
                e.Cancel = true;
                MessageBox.Show("Cant exit while updating");
            }
        }

        private void ACSetup_Click(object sender, EventArgs e)
        {
            MainV2.givecomport = true;

            MessageBox.Show("Please make sure you are in CLI/Setup mode");

            ICommsSerial comPortT = MainV2.comPort.BaseStream;

            if (comPortT.IsOpen)
                comPortT.Close();

            comPortT.DtrEnable = true;
            if (MainV2.comportname == null)
            {
                MessageBox.Show("Please select a valid comport! look in the Options menu");
                MainV2.givecomport = false;
                return;
            }
            try
            {
                comPortT.Open();

            }
            catch (Exception ex) { MainV2.givecomport = false; MessageBox.Show("Invalid Comport Settings : " + ex.Message); return; }

            lbl_status.Text = "Comport Opened";
            this.Refresh();

            comPortT.DtrEnable = false;
            System.Threading.Thread.Sleep(100);
            comPortT.DtrEnable = true;
            System.Threading.Thread.Sleep(3000);
            comPortT.DtrEnable = false;
            System.Threading.Thread.Sleep(100);
            comPortT.DtrEnable = true;

            string data = "";

            DateTime timeout = DateTime.Now;
            ///////////////////////// FIX ME////////////////////////////////////////////////////////
            int step = 0;
            //System.Threading.Thread.Sleep(2000);
            //comPortT.Write("IMU\r");

            comPortT.ReadTimeout = -1;

            while (comPortT.IsOpen)
            {
                string line;
                try
                {
                    line = comPortT.ReadLine();
                }
                catch
                {
                    try { line = comPortT.ReadExisting(); }
                    catch { MessageBox.Show("Can not read from serial port - existing"); return; }
                }
                this.Refresh();
                Console.Write(line + "\n");
                switch (step)
                {
                    case 0:
                        if (line.Contains("interactive"))
                        {
                            lbl_status.Text = "Erasing EEPROM.. (20 seconds)";
                            this.Refresh();

                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("setup\r");
                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("erase\r");
                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("y\r");
                            step = 0;
                        }
                        if (line.Contains("done"))
                        {
                            lbl_status.Text = "Rebooting APM..";
                            this.Refresh();

                            comPortT.DtrEnable = false;
                            System.Threading.Thread.Sleep(100);
                            comPortT.DtrEnable = true;
                            System.Threading.Thread.Sleep(3000);
                            comPortT.DtrEnable = false;
                            Console.WriteLine(comPortT.ReadExisting());
                            System.Threading.Thread.Sleep(100);
                            comPortT.DtrEnable = true;
                            step = 1;
                        }
                        break;
                    case 1:
                        if (line.Contains("interactive")) // becuase we rebooted
                        {
                            lbl_status.Text = "Setup Radio..";
                            this.Refresh();

                            Console.WriteLine(comPortT.ReadExisting());

                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("setup\r");
                            System.Threading.Thread.Sleep(200);
                            MessageBox.Show("Ensure that your RC transmitter is on, and that you have your ArduCopter battery plugged in or are otherwise powering APM's RC pins (USB power does NOT power the RC receiver)", "Radio Setup");
                            comPortT.Write("radio\r");
                            MessageBox.Show("Move all your radio controls to each extreme. Hit OK when done.", "Radio Setup");
                            //comPortT.DiscardInBuffer();
                            comPortT.Write("\r\r");
                            step = 2;
                        }
                        break;
                    case 2:
                        if (data.Contains("----"))
                            data = "";
                        data += line;

                        if (line.Contains("CH7")) // 
                        {
                            MessageBox.Show("Here are the detected radio options\nNOTE Channels not connected are displayed as 1500\nNormal values are around 1100 | 1900\nChannel:Min | Max \n" + data, "Radio");

                            lbl_status.Text = "Setup Accel Offsets..";
                            this.Refresh();

                            MessageBox.Show("Ensure your quad is level, and click OK to continue", "Offset Setup");

                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("setup\r");
                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("level\r");
                            System.Threading.Thread.Sleep(1000);
                            step = 3;
                        }
                        break;
                    case 3:
                        if (line.Contains("IMU")) // 
                        {
                            lbl_status.Text = "Setup Options";
                            this.Refresh();

                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("setup\r");

                            DialogResult dr;
                            /*
                            dr = MessageBox.Show("Do you have a Current sensor attached?","Current Sensor",MessageBoxButtons.YesNo);
                            if (dr == System.Windows.Forms.DialogResult.Yes)
                            {
                                comPortT.Write("current on\r");
                                System.Threading.Thread.Sleep(100);
                            }*/
                            dr = MessageBox.Show("Do you have a Sonar sensor attached?", "Sonar Sensor", MessageBoxButtons.YesNo);
                            if (dr == System.Windows.Forms.DialogResult.Yes)
                            {
                                comPortT.Write("sonar on\r");
                                System.Threading.Thread.Sleep(100);
                            }
                            dr = MessageBox.Show("Do you have a Compass sensor attached?", "Compass Sensor", MessageBoxButtons.YesNo);
                            if (dr == System.Windows.Forms.DialogResult.Yes)
                            {
                                comPortT.Write("compass on\r");
                                System.Threading.Thread.Sleep(100);

                                MessageBox.Show("Next a webpage will appear to get your magnetic declination,\nenter it in the box that appears next", "Mag Dec");

                                try
                                {
                                    //System.Diagnostics.Process.Start("http://www.ngdc.noaa.gov/geomagmodels/Declination.jsp");
                                    System.Diagnostics.Process.Start("http://www.magnetic-declination.com/");
                                }
                                catch { MessageBox.Show("Webpage open failed... do you have a virus?\nhttp://www.magnetic-declination.com/"); }
                                //This can be taken from 

                                try
                                {
                                    string declination = "0";
                                    Common.InputBox("Declination", "Magnetic Declination (-20.0 to 20.0) eg 2° 3' W is -2.3", ref declination);
                                    float dec = 0.0f;
                                    float.TryParse(declination, out dec);
                                    float deg = (float)((int)dec);
                                    float mins = (dec - deg);
                                    if (dec > 0)
                                    {
                                        dec += ((mins) / 60.0f);
                                    }
                                    else
                                    {
                                        dec -= ((mins) / 60.0f);
                                    }
                                    comPortT.Write("exit\rsetup\rdeclination " + dec.ToString("0.00") + "\r");
                                }
                                catch { MessageBox.Show("Invalid input!"); }
                            }

                            if (currentframe == FRAMETYPES.Y6 || currentframe == FRAMETYPES.Y6)
                            {

                            }
                            else
                            {

                                string frame = "+";
                                Common.InputBox("Frame", "Enter Frame type (options +, x)", ref frame);
                                System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();
                                byte[] data2 = encoding.GetBytes("exit\rsetup\rframe " + frame.ToLower() + "\r");
                                comPortT.Write(data2, 0, data2.Length);
                            }
                            Console.WriteLine(comPortT.ReadExisting());

                            MessageBox.Show("NOTE: this setup has defaulted all modes to stabilize.\n To change your flight modes, please use the CLI menu via the Terminal tab");

                            comPortT.Close();
                            MainV2.givecomport = false;
                            return;
                            //step = 4;
                        }
                        break;
                    //
                }
            }

        }

        private void APMSetup_Click(object sender, EventArgs e)
        {
            MainV2.givecomport = true;

            MessageBox.Show("Please make sure you are in CLI/Setup mode");

            ICommsSerial comPortT = MainV2.comPort.BaseStream;

            if (comPortT.IsOpen)
                comPortT.Close();

            comPortT.DtrEnable = true;
            if (MainV2.comportname == null)
            {
                MessageBox.Show("Please select a valid comport! look in the Options menu");
                MainV2.givecomport = false;
                return;
            }
            try
            {
                comPortT.Open();

            }
            catch (Exception ex) { MainV2.givecomport = false; MessageBox.Show("Invalid Comport Settings : " + ex.Message); return; }

            lbl_status.Text = "Comport Opened";
            this.Refresh();

            comPortT.DtrEnable = false;
            System.Threading.Thread.Sleep(100);
            comPortT.DtrEnable = true;
            System.Threading.Thread.Sleep(3000);
            comPortT.DtrEnable = false;
            System.Threading.Thread.Sleep(100);
            comPortT.DtrEnable = true;

            string data = "";

            DateTime timeout = DateTime.Now;
            ///////////////////////// FIX ME////////////////////////////////////////////////////////
            int step = 0;
            //System.Threading.Thread.Sleep(2000);
            //comPortT.Write("CH7\r");

            comPortT.ReadTimeout = -1;

            while (comPortT.IsOpen)
            {
                string line;
                try
                {
                    line = comPortT.ReadLine();
                }
                catch
                {
                    try { line = comPortT.ReadExisting(); }
                    catch { MessageBox.Show("Can not read from serial port - existing"); return; }
                }
                this.Refresh();
                Console.Write(line + "\n");
                switch (step)
                {
                    case 0:
                        if (line.Contains("interactive"))
                        {
                            lbl_status.Text = "Erasing EEPROM.. (20 seconds)";
                            this.Refresh();

                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("setup\r");
                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("erase\r");
                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("y\r");
                            step = 0;
                        }
                        if (line.Contains("done"))
                        {
                            lbl_status.Text = "Rebooting APM..";
                            this.Refresh();

                            comPortT.DtrEnable = false;
                            System.Threading.Thread.Sleep(100);
                            comPortT.DtrEnable = true;
                            System.Threading.Thread.Sleep(3000);
                            comPortT.DtrEnable = false;
                            System.Threading.Thread.Sleep(100);
                            comPortT.DtrEnable = true;
                            step = 1;
                        }
                        break;
                    case 1:
                        if (line.Contains("interactive")) // becuase we rebooted
                        {
                            lbl_status.Text = "Setup Radio..";
                            this.Refresh();

                            System.Threading.Thread.Sleep(1000);
                            Console.WriteLine(comPortT.ReadExisting());

                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("setup\r");
                            System.Threading.Thread.Sleep(200);
                            MessageBox.Show("Ensure that your RC transmitter is on, and that you have your ArduPilot battery plugged in or are otherwise powering APM's RC pins (USB power does NOT power the RC receiver)", "Radio Setup");
                            comPortT.Write("radio\r");
                            Console.WriteLine(comPortT.ReadExisting());
                            MessageBox.Show("Move all your radio controls to each extreme. Hit OK when done.", "Radio Setup");
                            comPortT.Write("\r\r");
                            step = 2;
                        }
                        break;
                    case 2:
                        if (data.Contains("----"))
                            data = "";
                        data += line;

                        if (line.Contains("CH7")) // 
                        {
                            MessageBox.Show("Here are the detected radio options\nNOTE Channels not connected are displayed as 1500\nNormal values are around 1100 | 1900\nChannel:Min | Max \n" + data, "Radio");

                            lbl_status.Text = "Clearing Log Dataflash (this may take a minute)";
                            this.Refresh();

                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("exit\rlogs\rerase\r");
                            System.Threading.Thread.Sleep(200);
                            Console.WriteLine(comPortT.ReadExisting());
                            step = 3;
                        }
                        break;
                    case 3:
                        if (line.Contains("Log erased"))
                        {
                            lbl_status.Text = "Setup Options";
                            this.Refresh();

                            Console.WriteLine(comPortT.ReadExisting());

                            System.Threading.Thread.Sleep(50);
                            comPortT.Write("exit\rsetup\r");

                            DialogResult dr;
                            dr = MessageBox.Show("Do you have a Compass sensor attached?", "Compass Sensor", MessageBoxButtons.YesNo);
                            if (dr == System.Windows.Forms.DialogResult.Yes)
                            {
                                comPortT.Write("compass on\r");
                                System.Threading.Thread.Sleep(100);

                                MessageBox.Show("Next a webpage will appear to get your magnetic declination,\nenter it in the box that appears next", "Mag Dec");

                                try
                                {
                                    //System.Diagnostics.Process.Start("http://www.ngdc.noaa.gov/geomagmodels/Declination.jsp");
                                    System.Diagnostics.Process.Start("http://www.magnetic-declination.com/");
                                }
                                catch { MessageBox.Show("Webpage open failed... do you have a virus?\nhttp://www.magnetic-declination.com/"); }
                                //This can be taken from 

                                string declination = "0";
                                Common.InputBox("Declination", "Magnetic Declination (-20.0 to 20.0) eg 2° 3' W is -2.3", ref declination);
                                float dec = 0.0f;
                                float.TryParse(declination, out dec);
                                float deg = (float)((int)dec);
                                float mins = (dec - deg);
                                if (dec > 0)
                                {
                                    dec += ((mins) / 60.0f);
                                }
                                else
                                {
                                    dec -= ((mins) / 60.0f);
                                }
                                comPortT.Write("exit\rsetup\rdeclination " + dec.ToString() + "\r");
                            }

                            MessageBox.Show("NOTE: this setup has defaulted all modes to there default.\n As you are new these are the safest.\n To change this option please use the modes option in the CLI");

                            Console.WriteLine(comPortT.ReadExisting());

                            comPortT.Close();
                            MainV2.givecomport = false;
                            lbl_status.Text = "Setup Done";
                            this.Refresh();
                            return;
                            //step = 4;
                        }
                        break;
                    //
                }
            }
        }

        private void BUT_setup_Click(object sender, EventArgs e)
        {
            Form temp = new Setup.Setup();
            MainV2.fixtheme(temp);
            temp.ShowDialog();
        }
    }
}
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
        private System.Windows.Forms.Label lbl_AP;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label lbl_APHil;
        private System.Windows.Forms.Label lbl_ACQuad;
        private System.Windows.Forms.Label lbl_ACHexa;
        private System.Windows.Forms.Label lbl_ACTri;
        private Label lbl_Heli;
        private PictureBox pictureBoxHeli;
        private MyButton BUT_setup;
        private Label lbl_ACHil;
        private PictureBox pictureBoxQuadHil;
        private System.Windows.Forms.Label lbl_ACY6;

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
            this.lbl_AP = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.lbl_APHil = new System.Windows.Forms.Label();
            this.lbl_ACQuad = new System.Windows.Forms.Label();
            this.lbl_ACHexa = new System.Windows.Forms.Label();
            this.lbl_ACTri = new System.Windows.Forms.Label();
            this.lbl_ACY6 = new System.Windows.Forms.Label();
            this.lbl_Heli = new System.Windows.Forms.Label();
            this.pictureBoxHeli = new System.Windows.Forms.PictureBox();
            this.lbl_ACHil = new System.Windows.Forms.Label();
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
            // lbl_AP
            // 
            resources.ApplyResources(this.lbl_AP, "lbl_AP");
            this.lbl_AP.Name = "lbl_AP";
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // lbl_APHil
            // 
            resources.ApplyResources(this.lbl_APHil, "lbl_APHil");
            this.lbl_APHil.Name = "lbl_APHil";
            // 
            // lbl_ACQuad
            // 
            resources.ApplyResources(this.lbl_ACQuad, "lbl_ACQuad");
            this.lbl_ACQuad.Name = "lbl_ACQuad";
            // 
            // lbl_ACHexa
            // 
            resources.ApplyResources(this.lbl_ACHexa, "lbl_ACHexa");
            this.lbl_ACHexa.Name = "lbl_ACHexa";
            // 
            // lbl_ACTri
            // 
            resources.ApplyResources(this.lbl_ACTri, "lbl_ACTri");
            this.lbl_ACTri.Name = "lbl_ACTri";
            // 
            // lbl_ACY6
            // 
            resources.ApplyResources(this.lbl_ACY6, "lbl_ACY6");
            this.lbl_ACY6.Name = "lbl_ACY6";
            // 
            // lbl_Heli
            // 
            resources.ApplyResources(this.lbl_Heli, "lbl_Heli");
            this.lbl_Heli.Name = "lbl_Heli";
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
            // lbl_ACHil
            // 
            resources.ApplyResources(this.lbl_ACHil, "lbl_ACHil");
            this.lbl_ACHil.Name = "lbl_ACHil";
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
            this.Controls.Add(this.lbl_ACHil);
            this.Controls.Add(this.pictureBoxQuadHil);
            this.Controls.Add(this.BUT_setup);
            this.Controls.Add(this.lbl_Heli);
            this.Controls.Add(this.pictureBoxHeli);
            this.Controls.Add(this.lbl_ACY6);
            this.Controls.Add(this.lbl_ACTri);
            this.Controls.Add(this.lbl_ACHexa);
            this.Controls.Add(this.lbl_ACQuad);
            this.Controls.Add(this.lbl_APHil);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.lbl_AP);
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
                findfirmware("AP-trunk");
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

                using (XmlTextReader xmlreader = new XmlTextReader("http://ardupilot-mega.googlecode.com/git/Tools/ArdupilotMegaPlanner/Firmware/firmware2.xml"))
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

                                    try
                                    {
                                        updateDisplayName(temp);
                                    }
                                    catch { } // just in case

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

        void updateDisplayName(software temp)
        {
            if (temp.url.ToLower().Contains("firmware/AP-1".ToLower()))
            {
                lbl_AP.Text = temp.name;
            }
            else if (temp.url.ToLower().Contains("firmware/APHIL-".ToLower()))
            {
                lbl_APHil.Text = temp.name;
            }
            else if (temp.url.ToLower().Contains("firmware/ac2-quad-".ToLower()))
            {
                lbl_ACQuad.Text = temp.name;
            }
            else if (temp.url.ToLower().Contains("firmware/ac2-tri".ToLower()))
            {
                lbl_ACTri.Text = temp.name;
            }
            else if (temp.url.ToLower().Contains("firmware/ac2-hexa".ToLower()))
            {
                lbl_ACHexa.Text = temp.name;
            }
            else if (temp.url.ToLower().Contains("firmware/ac2-y6".ToLower()))
            {
                lbl_ACY6.Text = temp.name;
            }
            else if (temp.url.ToLower().Contains("firmware/ac2-heli-1".ToLower()))
            {
                lbl_Heli.Text = temp.name;
            }
            else if (temp.url.ToLower().Contains("firmware/ac2-quadhil".ToLower()))
            {
                lbl_ACHil.Text = temp.name;
            }
            else
            {
                Console.WriteLine("No Home "+ temp.name + " " + temp.url);
            }
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
            findfirmware("firmware/AP-1");
        }

        private void pictureBoxAPMHIL_Click(object sender, EventArgs e)
        {
            findfirmware("firmware/APHIL-");
        }

        private void pictureBoxQuad_Click(object sender, EventArgs e)
        {
            findfirmware("AC2-Quad-");
        }

        private void pictureBoxHexa_Click(object sender, EventArgs e)
        {
            findfirmware("AC2-Hexa-");
        }

        private void pictureBoxTri_Click(object sender, EventArgs e)
        {
            findfirmware("AC2-Tri-");
        }

        private void pictureBoxY6_Click(object sender, EventArgs e)
        {
            findfirmware("AC2-Y6-");
        }

        private void pictureBoxHeli_Click(object sender, EventArgs e)
        {
            findfirmware("AC2-Heli-");
        }

        private void pictureBoxQuadHil_Click(object sender, EventArgs e)
        {
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

                dataStream.ReadTimeout = 30000;

                while (dataStream.CanRead)
                {
                    try
                    {
                        progress.Value = 50;// (int)(((float)(response.ContentLength - bytes) / (float)response.ContentLength) * 100);
                        this.progress.Refresh();
                    }
                    catch { }
                    int len = dataStream.Read(buf1, 0, 1024);
                    if (len == 0)
                        break;
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

            UploadFlash(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"firmware.hex", board);
        }

        void UploadFlash(string filename, string board)
        {
            byte[] FLASH = new byte[1];
            StreamReader sr = null;
            try
            {
                lbl_status.Text = "Reading Hex File";
                this.Refresh();
                sr = new StreamReader(filename);
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
            bool hitend = false;

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
                    else if (option == 1)
                    {
                        hitend = true;
                    }
                    int checksum = Convert.ToInt32(line.Substring(line.Length - 2, 2), 16);

                    byte checksumact = 0;
                    for (int z = 0; z < ((line.Length - 1 - 2) / 2) ; z++) // minus 1 for : then mins 2 for checksum itself
                    { 
                        checksumact += Convert.ToByte(line.Substring(z * 2 + 1, 2), 16);
                    }
                    checksumact = (byte)(0x100 - checksumact);

                    if (checksumact != checksum)
                    {
                        MessageBox.Show("The hex file loaded is invalid, please try again.");
                        throw new Exception("Checksum Failed - Invalid Hex");
                    }
                }
                //Regex regex = new Regex(@"^:(..)(....)(..)(.*)(..)$"); // length - address - option - data - checksum
            }

            if (!hitend)
            {
                MessageBox.Show("The hex file did no contain an end flag. aborting");
                throw new Exception("No end flag in file");
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

        private void BUT_setup_Click(object sender, EventArgs e)
        {
            Form temp = new Setup.Setup();
            MainV2.fixtheme(temp);
            temp.ShowDialog();
        }
    }
}
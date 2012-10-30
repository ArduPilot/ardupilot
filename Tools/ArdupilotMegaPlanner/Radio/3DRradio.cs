using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Net;
using System.IO;
using ArdupilotMega.Controls.BackstageView;
using ArdupilotMega.Arduino;
using ArdupilotMega.Comms;
using log4net;
using System.Reflection;
using System.Text.RegularExpressions;

namespace ArdupilotMega
{
    public partial class _3DRradio : UserControl
    {
        public delegate void LogEventHandler(string message, int level = 0);

        public delegate void ProgressEventHandler(double completed);

        string firmwarefile = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "radio.hex";

        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        public _3DRradio()
        {
            InitializeComponent();

            // hide advanced view
            //SPLIT_local.Panel2Collapsed = true;
            //SPLIT_remote.Panel2Collapsed = true;

            // setup netid
            S3.DataSource = Enumerable.Range(0, 500).ToArray();
            RS3.DataSource = Enumerable.Range(0, 500).ToArray();
        }

        bool getFirmware(uploader.Uploader.Code device)
        {
            // was https://raw.github.com/tridge/SiK/master/Firmware/dst/radio.hm_trp.hex
            // now http://www.samba.org/tridge/UAV/3DR/radio.hm_trp.hex

            if (device == uploader.Uploader.Code.DEVICE_ID_HM_TRP)
            {
                return Common.getFilefromNet("http://www.samba.org/tridge/UAV/3DR/radio.hm_trp.hex", firmwarefile);
            }
            else if (device == uploader.Uploader.Code.DEVICE_ID_RFD900)
            {
                return Common.getFilefromNet("http://rfdesign.com.au/firmware/radio.rfd900.hex", firmwarefile);
            }
            else if (device == uploader.Uploader.Code.DEVICE_ID_RFD900A)
            {
                return Common.getFilefromNet("http://rfdesign.com.au/firmware/radio.rfd900a.hex", firmwarefile);
            }
            else
            {
                return false;
            }
        }

        void Sleep(int mstimeout)
        {
            DateTime endtime = DateTime.Now.AddMilliseconds(mstimeout);

            while (DateTime.Now < endtime)
            {
                System.Threading.Thread.Sleep(1);
                Application.DoEvents();
            }
        }

        private void BUT_upload_Click(object sender, EventArgs e)
        {
            ArduinoSTK comPort = new ArduinoSTK();

            uploader.Uploader uploader = new uploader.Uploader();

            try
            {
                comPort.PortName = MainV2.comPort.BaseStream.PortName;
                comPort.BaudRate = 115200;

                comPort.Open();

            }
            catch { CustomMessageBox.Show("Invalid ComPort or in use"); return; }

            bool bootloadermode = false;

            // attempt bootloader mode
            try
            {
                uploader_ProgressEvent(0);
                uploader_LogEvent("Trying Bootloader Mode");
                uploader.port = comPort;
                uploader.connect_and_sync();

                uploader.ProgressEvent += new ProgressEventHandler(uploader_ProgressEvent);
                uploader.LogEvent += new LogEventHandler(uploader_LogEvent);

                uploader_LogEvent("In Bootloader Mode");
                bootloadermode = true;
            }
            catch
            {
                // cleanup bootloader mode fail, and try firmware mode
                comPort.Close();
                comPort.BaudRate = MainV2.comPort.BaseStream.BaudRate;
                comPort.Open();

                uploader.ProgressEvent += new ProgressEventHandler(uploader_ProgressEvent);
                uploader.LogEvent += new LogEventHandler(uploader_LogEvent);

                uploader_LogEvent("Trying Firmware Mode");
                bootloadermode = false;
            }

            // check for either already bootloadermode, or if we can do a ATI to ID the firmware 
            if (bootloadermode || doConnect(comPort))
            {

                uploader.IHex iHex = new uploader.IHex();

                iHex.LogEvent += new LogEventHandler(iHex_LogEvent);

                iHex.ProgressEvent += new ProgressEventHandler(iHex_ProgressEvent);

                // put into bootloader mode/udpate mode
                if (!bootloadermode)
                {
                    try
                    {
                        comPort.Write("AT&UPDATE\r\n");
                        string left = comPort.ReadExisting();
                        log.Info(left);
                        Sleep(700);
                        comPort.BaudRate = 115200;
                    }
                    catch { }
                }

                global::uploader.Uploader.Code device = global::uploader.Uploader.Code.FAILED;
                global::uploader.Uploader.Code freq = global::uploader.Uploader.Code.FAILED;

                // get the device type and frequency in the bootloader
                uploader.getDevice(ref device, ref freq);

                // get firmware for this device
                if (getFirmware(device))
                {
                    // load the hex
                    try
                    {
                        iHex.load(firmwarefile);
                    }
                    catch { CustomMessageBox.Show("Bad Firmware File"); goto exit; }

                    // upload the hex and verify
                    try
                    {
                        uploader.upload(comPort, iHex);
                    }
                    catch (Exception ex) { CustomMessageBox.Show("Upload Failed " + ex.Message); }

                }
                else
                {
                    CustomMessageBox.Show("Failed to download new firmware");
                }

            }
            else
            {
                CustomMessageBox.Show("Failed to identify Radio");
            }

        exit:
            if (comPort.IsOpen)
                comPort.Close();
        }

        void iHex_ProgressEvent(double completed)
        {
            try
            {
                Progressbar.Value = (int)(completed * 100);
                Application.DoEvents();
            }
            catch { }
        }

        void uploader_LogEvent(string message, int level = 0)
        {
            try
            {
                if (level == 0)
                {
                    Console.Write(message);
                    lbl_status.Text = message;
                    log.Info(message);
                    Application.DoEvents();
                }
                else if (level < 5) // 5 = byte data
                {
                    log.Debug(message);
                }
            }
            catch { }
        }

        void iHex_LogEvent(string message, int level = 0)
        {
            try
            {
                if (level == 0)
                {
                    lbl_status.Text = message;
                    Console.WriteLine(message);
                    log.Info(message);
                    Application.DoEvents();
                }
            }
            catch { }
        }

        void uploader_ProgressEvent(double completed)
        {
            try
            {
                Progressbar.Value = (int)(completed * 100);
                Application.DoEvents();
            }
            catch { }
        }

        private void BUT_savesettings_Click(object sender, EventArgs e)
        {
            ArdupilotMega.Comms.ICommsSerial comPort = new SerialPort();

            try
            {
                comPort.PortName = MainV2.comPort.BaseStream.PortName;
                comPort.BaudRate = MainV2.comPort.BaseStream.BaudRate;

                comPort.ReadTimeout = 4000;

                comPort.Open();


            }
            catch { CustomMessageBox.Show("Invalid ComPort or in use"); return; }

            lbl_status.Text = "Connecting";

            if (doConnect(comPort))
            {
                // cleanup
                doCommand(comPort, "AT&T");

                comPort.DiscardInBuffer();

                lbl_status.Text = "Doing Command";

                if (RTI.Text != "")
                {

                    // remote
                    string answer = doCommand(comPort, "RTI5");

                    string[] items = answer.Split('\n');

                    foreach (string item in items)
                    {
                        if (item.StartsWith("S"))
                        {
                            string[] values = item.Split(':', '=');

                            if (values.Length == 3)
                            {
                                Control[] controls = this.Controls.Find("R" + values[0].Trim(), true);

                                if (controls.Length > 0)
                                {
                                    if (controls[0].GetType() == typeof(CheckBox))
                                    {
                                        string value = ((CheckBox)controls[0]).Checked ? "1" : "0";

                                        if (value != values[2].Trim())
                                        {
                                            string cmdanswer = doCommand(comPort, "RT" + values[0].Trim() + "=" + value + "\r");

                                            if (cmdanswer.Contains("OK"))
                                            {

                                            }
                                            else
                                            {
                                                CustomMessageBox.Show("Set Command error");
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if (controls[0].Text != values[2].Trim() && controls[0].Text != "")
                                        {
                                            string cmdanswer = doCommand(comPort, "RT" + values[0].Trim() + "=" + controls[0].Text + "\r");

                                            if (cmdanswer.Contains("OK"))
                                            {

                                            }
                                            else
                                            {
                                                CustomMessageBox.Show("Set Command error");
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // write it
                    doCommand(comPort, "RT&W");

                    // return to normal mode
                    doCommand(comPort, "RTZ");

                    Sleep(100);
                }

                comPort.DiscardInBuffer();
                {
                    //local
                    string answer = doCommand(comPort, "ATI5");

                    string[] items = answer.Split('\n');

                    foreach (string item in items)
                    {
                        if (item.StartsWith("S"))
                        {
                            string[] values = item.Split(':', '=');

                            if (values.Length == 3)
                            {
                                Control[] controls = this.Controls.Find(values[0].Trim(), true);

                                if (controls.Length > 0)
                                {
                                    if (controls[0].GetType() == typeof(CheckBox))
                                    {
                                        string value = ((CheckBox)controls[0]).Checked ? "1" : "0";

                                        if (value != values[2].Trim())
                                        {
                                            string cmdanswer = doCommand(comPort, "AT" + values[0].Trim() + "=" + value + "\r");

                                            if (cmdanswer.Contains("OK"))
                                            {

                                            }
                                            else
                                            {
                                                CustomMessageBox.Show("Set Command error");
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if (controls[0].Text != values[2].Trim())
                                        {
                                            string cmdanswer = doCommand(comPort, "AT" + values[0].Trim() + "=" + controls[0].Text + "\r");

                                            if (cmdanswer.Contains("OK"))
                                            {

                                            }
                                            else
                                            {
                                                CustomMessageBox.Show("Set Command error");
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // write it
                    doCommand(comPort, "AT&W");


                    // return to normal mode
                    doCommand(comPort, "ATZ");
                }

                lbl_status.Text = "Done";
            }
            else
            {

                // return to normal mode
                doCommand(comPort, "ATZ");

                lbl_status.Text = "Fail";
                CustomMessageBox.Show("Failed to enter command mode");
            }


            comPort.Close();
        }

        public static IEnumerable<int> Range(int start,int step, int end)
        {
            List<int> list = new List<int>();

            for (int a = start; a <= end; a += step)
            {
                list.Add(a);
            }

            return list;
        }


        private void BUT_getcurrent_Click(object sender, EventArgs e)
        {
            ArdupilotMega.Comms.ICommsSerial comPort = new SerialPort();

            try
            {
                comPort.PortName = MainV2.comPort.BaseStream.PortName;
                comPort.BaudRate = MainV2.comPort.BaseStream.BaudRate;

                comPort.ReadTimeout = 4000;

                comPort.Open();


            }
            catch { CustomMessageBox.Show("Invalid ComPort or in use"); return; }

            lbl_status.Text = "Connecting";

            if (doConnect(comPort))
            {
                // cleanup
                doCommand(comPort, "AT&T");

                comPort.DiscardInBuffer();

                lbl_status.Text = "Doing Command ATI & RTI";

                ATI.Text = doCommand(comPort, "ATI");

                RTI.Text = doCommand(comPort, "RTI");

                    uploader.Uploader.Code freq = (uploader.Uploader.Code)Enum.Parse(typeof(uploader.Uploader.Code), doCommand(comPort, "ATI3"));
                    uploader.Uploader.Code board = (uploader.Uploader.Code)Enum.Parse(typeof(uploader.Uploader.Code), doCommand(comPort, "ATI2"));

                    ATI3.Text = freq.ToString();
                // 8 and 9
                    if (freq == uploader.Uploader.Code.FREQ_915) {
                        S8.DataSource = Range(895000, 1000, 935000);
                        RS8.DataSource = Range(895000, 1000, 935000);

                        S9.DataSource = Range(895000, 1000, 935000);
                        RS9.DataSource = Range(895000, 1000, 935000);
                    }
                    else if (freq == uploader.Uploader.Code.FREQ_433)
                    {
                        S8.DataSource = Range(414000, 100, 454000);
                        RS8.DataSource = Range(414000, 100, 454000);

                        S9.DataSource = Range(414000, 100, 454000);
                        RS9.DataSource = Range(414000, 100, 454000);
                    }

                    if (board == uploader.Uploader.Code.DEVICE_ID_RFD900 || board == uploader.Uploader.Code.DEVICE_ID_RFD900A)
                    {
                        S4.DataSource = Range(1, 1, 30);
                        RS4.DataSource = Range(1, 1, 30);
                    }


                RSSI.Text = doCommand(comPort, "ATI7").Trim();

                lbl_status.Text = "Doing Command ATI5";

                string answer = doCommand(comPort, "ATI5");

                string[] items = answer.Split('\n');

                foreach (string item in items)
                {
                    if (item.StartsWith("S"))
                    {
                        string[] values = item.Split(':', '=');

                        if (values.Length == 3)
                        {
                            Control[] controls = this.Controls.Find(values[0].Trim(), true);

                            if (controls.Length > 0)
                            {
                                if (controls[0].GetType() == typeof(CheckBox))
                                {
                                    ((CheckBox)controls[0]).Checked = values[2].Trim() == "1";
                                }
                                else
                                {
                                    controls[0].Text = values[2].Trim();
                                }
                            }
                        }
                    }
                }

                // remote
                foreach (Control ctl in groupBox2.Controls)
                {
                    if (ctl.Name.StartsWith("RS") && ctl.Name != "RSSI")
                        ctl.ResetText();
                }


                comPort.DiscardInBuffer();

                lbl_status.Text = "Doing Command RTI5";

                answer = doCommand(comPort, "RTI5");

                items = answer.Split('\n');

                foreach (string item in items)
                {
                    if (item.StartsWith("S"))
                    {
                        string[] values = item.Split(':', '=');

                        if (values.Length == 3)
                        {
                            Control[] controls = this.Controls.Find("R" + values[0].Trim(), true);

                            if (controls.Length == 0)
                                continue;

                            if (controls[0].GetType() == typeof(CheckBox))
                            {
                                ((CheckBox)controls[0]).Checked = values[2].Trim() == "1";
                            }
                            else if (controls[0].GetType() == typeof(TextBox))
                            {
                                ((TextBox)controls[0]).Text = values[2].Trim();
                            }
                            else if (controls[0].GetType() == typeof(ComboBox))
                            {
                                ((ComboBox)controls[0]).Text = values[2].Trim();
                            }
                        }
                        else
                        {
                            log.Info("Odd config line :" + item);
                        }
                    }
                }

                // off hook
                doCommand(comPort, "ATO");

                lbl_status.Text = "Done";
            }
            else
            {

                // off hook
                doCommand(comPort, "ATO");

                lbl_status.Text = "Fail";
                CustomMessageBox.Show("Failed to enter command mode");
            }

            comPort.Close();

            BUT_Syncoptions.Enabled = true;

            BUT_savesettings.Enabled = true;
        }

        string Serial_ReadLine(ArdupilotMega.Comms.ICommsSerial comPort)
        {
            StringBuilder sb = new StringBuilder();
            DateTime Deadline = DateTime.Now.AddMilliseconds(comPort.ReadTimeout);

            while (DateTime.Now < Deadline)
            {
                if (comPort.BytesToRead > 0)
                {
                    byte data = (byte)comPort.ReadByte();
                    sb.Append((char)data);
                    if (data == '\n')
                        break;
                }
            }

            return sb.ToString();
        }

        public string doCommand(ArdupilotMega.Comms.ICommsSerial comPort, string cmd, int level = 0)
        {
            if (!comPort.IsOpen)
                return "";

            comPort.ReadTimeout = 1000;
            // setup to known state
            comPort.Write("\r\n");
            // alow some time to gather thoughts
            Sleep(100);
            // ignore all existing data
            comPort.DiscardInBuffer();
            lbl_status.Text = "Doing Command " + cmd;
            log.Info("Doing Command " + cmd);
            // write command
            comPort.Write(cmd + "\r\n");
            // read echoed line or existing data
            string temp;
            try
            {
                temp = Serial_ReadLine(comPort);
            }
            catch { temp = comPort.ReadExisting(); }
            log.Info("cmd " + cmd + " echo " + temp);
            // delay for command
            Sleep(500);
            // get responce
            string ans = "";
            while (comPort.BytesToRead > 0)
            {
                try
                {
                    ans = ans + Serial_ReadLine(comPort) + "\n";
                }
                catch { ans = ans + comPort.ReadExisting() + "\n"; }
                Sleep(50);

                if (ans.Length > 500)
                {
                    break;
                }
            }

            log.Info("responce " + level + " " + ans.Replace('\0', ' '));

            // try again
            if (ans == "" && level == 0)
                return doCommand(comPort, cmd, 1);

            return ans;
        }

        public bool doConnect(ArdupilotMega.Comms.ICommsSerial comPort)
        {
            // clear buffer
            comPort.DiscardInBuffer();
            // setup a known enviroment
            comPort.Write("\r\n");
            // wait
            Sleep(1100);
            // send config string
            comPort.Write("+++");
            // wait
            Sleep(1100);
            // check for config responce "OK"
            log.Info("Connect btr " + comPort.BytesToRead + " baud " + comPort.BaudRate);
            string conn = comPort.ReadExisting();
            log.Info("Connect first responce " + conn.Replace('\0', ' ') + " " + conn.Length);
            if (conn.Contains("OK"))
            {
                //return true;
            }
            else
            {
                // cleanup incase we are already in cmd mode
                comPort.Write("\r\n");
            }



            doCommand(comPort, "AT&T");

            string version = doCommand(comPort, "ATI");

            log.Info("Connect Version: " + version.Trim() + "\n");

            Regex regex = new Regex(@"SiK\s+(.*)\s+on\s+(.*)");

            if (regex.IsMatch(version))
            {
                return true;
            }

            return false;
        }

        private void BUT_Syncoptions_Click(object sender, EventArgs e)
        {
            RS2.Text = S2.Text;
            RS3.Text = S3.Text;
            RS5.Checked = S5.Checked;
            RS8.Text = S8.Text;
            RS9.Text = S9.Text;
            RS10.Text = S10.Text;
        }

        private void CHK_advanced_CheckedChanged(object sender, EventArgs e)
        {
            SPLIT_local.Panel2Collapsed = !CHK_advanced.Checked;
            SPLIT_remote.Panel2Collapsed = !CHK_advanced.Checked;
        }

        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            CustomMessageBox.Show(@"The 3DR Radios have 2 status LEDs, one red and one green.
green LED blinking - searching for another radio 
green LED solid - link is established with another radio 
red LED flashing - transmitting data 
red LED solid - in firmware update mode");
        }

        private void BUT_resettodefault_Click(object sender, EventArgs e)
        {
             ArdupilotMega.Comms.ICommsSerial comPort = new SerialPort();

            try
            {
                comPort.PortName = MainV2.comPort.BaseStream.PortName;
                comPort.BaudRate = MainV2.comPort.BaseStream.BaudRate;

                comPort.ReadTimeout = 4000;

                comPort.Open();


            }
            catch { CustomMessageBox.Show("Invalid ComPort or in use"); return; }

            lbl_status.Text = "Connecting";

            if (doConnect(comPort))
            {
                // cleanup
                doCommand(comPort, "AT&T");

                comPort.DiscardInBuffer();

                lbl_status.Text = "Doing Command ATI & AT&F";

                doCommand(comPort, "AT&F");

                doCommand(comPort, "AT&W");

                lbl_status.Text = "Reset";

                doCommand(comPort, "ATZ");

                comPort.Close();

                BUT_getcurrent_Click(sender, e);
            }
            else
            {

                // off hook
                doCommand(comPort, "ATO");

                lbl_status.Text = "Fail";
                CustomMessageBox.Show("Failed to enter command mode");
            }

            if (comPort.IsOpen)
                comPort.Close();
        }
    }
}
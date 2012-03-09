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

namespace ArdupilotMega
{
    public partial class _3DRradio : Form
    {
        public delegate void LogEventHandler(string message, int level = 0);

        public delegate void ProgressEventHandler(double completed);

        string firmwarefile = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "radio.hm_trp.hex";

        public _3DRradio()
        {
            InitializeComponent();

            S3.DataSource = Enumerable.Range(0, 500).ToArray();
            RS3.DataSource = S3.DataSource;
        }

        bool getFirmware()
        {
            //https://raw.github.com/tridge/SiK/master/Firmware/dst/radio.hm_trp.hex

            return Common.getFilefromNet("https://raw.github.com/tridge/SiK/master/Firmware/dst/radio.hm_trp.hex", firmwarefile);
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

            uploader.ProgressEvent += new ProgressEventHandler(uploader_ProgressEvent);
            uploader.LogEvent += new LogEventHandler(uploader_LogEvent);

            try
            {
                uploader_ProgressEvent(0);
                uploader_LogEvent("Trying Bootloader Mode");
                uploader.port = comPort;
                uploader.connect_and_sync();
                uploader_LogEvent("In Bootloader Mode");
                bootloadermode = true;
            }
            catch {
                comPort.Close();
                comPort.BaudRate = MainV2.comPort.BaseStream.BaudRate;
                comPort.Open();
                uploader_LogEvent("Trying Firmware Mode");
                bootloadermode = false;
            }



            if (bootloadermode || doConnect(comPort))
            {
                if (getFirmware())
                {
                    uploader.IHex iHex = new uploader.IHex();

                    iHex.LogEvent += new LogEventHandler(iHex_LogEvent);

                    iHex.ProgressEvent += new ProgressEventHandler(iHex_ProgressEvent);

                    try
                    {
                        iHex.load(firmwarefile);
                    }
                    catch { CustomMessageBox.Show("Bad Firmware File"); goto exit; }

                    if (!bootloadermode)
                    {
                        try
                        {
                            comPort.Write("AT&UPDATE\r\n");
                            string left = comPort.ReadExisting();
                            Console.WriteLine(left);
                            Sleep(700);
                            comPort.BaudRate = 115200;
                        }
                        catch { }
                    }

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
                    Application.DoEvents();
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
            ArdupilotMega.ICommsSerial comPort = new SerialPort();

            try {
            comPort.PortName = MainV2.comPort.BaseStream.PortName;
            comPort.BaudRate = MainV2.comPort.BaseStream.BaudRate;

            comPort.ReadTimeout = 4000;

            comPort.Open();


            }
            catch { CustomMessageBox.Show("Invalid ComPort or in use"); return; }

            lbl_status.Text = "Connecting";

            if (doConnect(comPort))
            {
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
                                Control[] controls = this.Controls.Find("R" + values[0].Trim(), false);

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
                                Control[] controls = this.Controls.Find(values[0].Trim(), false);

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


        private void BUT_getcurrent_Click(object sender, EventArgs e)
        {
            ArdupilotMega.ICommsSerial comPort = new SerialPort();

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
                comPort.DiscardInBuffer();

                lbl_status.Text = "Doing Command ATI & RTI";

                ATI.Text = doCommand(comPort, "ATI1").Trim();

                RTI.Text = doCommand(comPort, "RTI1").Trim();

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
                            Control[] controls = this.Controls.Find(values[0].Trim(), false);

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
                foreach (Control ctl in this.Controls)
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
                            Control[] controls = this.Controls.Find("R" + values[0].Trim(), false);

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
                                ((ComboBox)controls[0]).SelectedText = values[2].Trim();
                            }
                        }
                        else
                        {
                            Console.WriteLine("Odd config line :" + item);
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
        }

        string Serial_ReadLine(ArdupilotMega.ICommsSerial comPort)
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

        string doCommand(ArdupilotMega.ICommsSerial comPort, string cmd, int level = 0)
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
            Console.WriteLine("Doing Command " + cmd);
            // write command
            comPort.Write(cmd + "\r\n");
            // read echoed line or existing data
            string temp;
            try
            {
                temp = Serial_ReadLine(comPort);
            }
            catch { temp = comPort.ReadExisting(); }
            Console.WriteLine("cmd " + cmd + " echo " + temp);
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

            Console.WriteLine("responce " + level + " " + ans.Replace('\0',' '));

            // try again
            if (ans == "" && level == 0)
                return doCommand(comPort, cmd, 1);

            return ans;
        }

        bool doConnect(ArdupilotMega.ICommsSerial comPort)
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
            Console.WriteLine("Connect btr " + comPort.BytesToRead + " baud " + comPort.BaudRate);
            string conn = comPort.ReadExisting();
            Console.WriteLine("Connect first responce " + conn.Replace('\0',' ') + " " + conn.Length);
            if (conn.Contains("OK"))
            {
                //return true;
            }
            else
            {
                // cleanup incase we are already in cmd mode
                comPort.Write("\r\n");
            }

            string version = doCommand(comPort, "ATI");

            Console.Write("Connect Version: " + version.Trim() + "\n");

            if (version.Contains("on HM-TRP"))
            {
                return true;
            }

            return false;
        }

        private void BUT_syncS2_Click(object sender, EventArgs e)
        {
            RS2.Text = S2.Text;
        }

        private void BUT_syncS3_Click(object sender, EventArgs e)
        {
            RS3.Text = S3.Text;
        }

        private void BUT_syncS5_Click(object sender, EventArgs e)
        {
            RS5.Checked = S5.Checked;
        }
    }
}
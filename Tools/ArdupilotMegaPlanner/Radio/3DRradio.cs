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

            string version = "";

            uploader.Uploader uploader = new uploader.Uploader();

            comPort.PortName = MainV2.comPort.BaseStream.PortName;
            comPort.BaudRate = 115200;

            comPort.Open();

            bool bootloadermode = false;

            try
            {
                uploader_ProgressEvent(0);
                uploader_LogEvent("Trying Bootloader Mode");
                uploader.port = comPort;
                uploader.connect_and_sync();
                uploader_LogEvent("In Bootloader Mode");
                bootloadermode = true;
            }
            catch { uploader_LogEvent("Trying Firmware Mode"); bootloadermode = false; }

            uploader.ProgressEvent += new ProgressEventHandler(uploader_ProgressEvent);
            uploader.LogEvent += new LogEventHandler(uploader_LogEvent);

            if (!bootloadermode)
            {
                comPort.BaudRate = 57600;
                // clear buffer
                comPort.DiscardInBuffer();
                // setup a known enviroment
                comPort.Write("\r\n");
                // wait
                Sleep(1000);
                // send config string
                comPort.Write("+++");
                // wait
                Sleep(1100);
                // check for config responce "OK"
                if (comPort.ReadExisting().Contains("OK"))
                {

                }

                comPort.Write("\r\nATI\r\n");

                Sleep(100);

                version = comPort.ReadExisting();
            }


            if (version.Contains("on HM-TRP") || bootloadermode)
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
                    catch { MessageBox.Show("Bad Firmware File"); goto exit; }

                    if (!bootloadermode)
                    {

                        comPort.Write("AT&UPDATE\r\n");
                        string left = comPort.ReadExisting();
                        Console.WriteLine(left);
                        Sleep(700);
                        comPort.BaudRate = 115200;

                    }

                    try
                    {
                        uploader.upload(comPort, iHex);
                    }
                    catch (Exception ex) { MessageBox.Show("Upload Failed " + ex.Message); goto exit; }
                }
                else
                {
                    MessageBox.Show("Failed to download new firmware");
                }
            }
            else
            {
                MessageBox.Show("Failed to identify Radio");
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

        private void BUT_getcurrent_Click(object sender, EventArgs e)
        {
            SerialPort comPort = new SerialPort();

            comPort.PortName = MainV2.comPort.BaseStream.PortName;
            comPort.BaudRate = 57600;

            comPort.ReadTimeout = 4000;

            comPort.Open();

            lbl_status.Text = "Connecting";

            if (doConnect(comPort))
            {
                comPort.DiscardInBuffer();

                lbl_status.Text = "Doing Command ATI & RTI";

                 ATI.Text = doCommand(comPort, "ATI1");

                 RTI.Text = doCommand(comPort, "RTI1");

                 RSSI.Text = doCommand(comPort, "ATI7");

                lbl_status.Text = "Doing Command ATI5";

                string answer = doCommand(comPort, "ATI5");

                Console.Write("Local\n" + answer);

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

                comPort.DiscardInBuffer();

                lbl_status.Text = "Doing Command RTI5";

                answer = doCommand(comPort, "RTI5");

                Console.Write("Remote\n" + answer);

                items = answer.Split('\n');

                foreach (string item in items)
                {
                    if (item.StartsWith("S"))
                    {
                        string[] values = item.Split(':', '=');

                        if (values.Length == 3)
                        {
                            Control[] controls = this.Controls.Find("R"+values[0].Trim(), false);

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

                lbl_status.Text = "Done";
            }
            else
            {
                lbl_status.Text = "Fail";
                MessageBox.Show("Failed to enter command mode");
            }

            comPort.WriteLine("ATZ");

            comPort.Close();

        }

        string doCommand(SerialPort comPort, string cmd)
        {
            Sleep(100);
            comPort.DiscardInBuffer();
            comPort.Write(cmd + "\r\n");
            string temp = comPort.ReadLine(); // echo
            Sleep(500);
            string ans = "";
            while (comPort.BytesToRead > 0)
            {
                ans = ans + comPort.ReadLine() + "\n";
                Sleep(50);

                if (ans.Length > 500)
                    return "";
            }

            return ans;
        }

        bool doConnect(SerialPort comPort)
        {
            // clear buffer
            comPort.DiscardOutBuffer();
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
            if (comPort.ReadExisting().Contains("OK"))
            {
                //return true;
            }
            else
            {
                // cleanup incase we are already in cmd mode
                comPort.Write("\r\n");
            }

            string version = doCommand(comPort, "ATI");

            Console.Write("Connect Version: "+version);

            if (version.Contains("on HM-TRP"))
            {
                return true;
            }

            return false;
        }

        private void BUT_savesettings_Click(object sender, EventArgs e)
        {
            SerialPort comPort = new SerialPort();

            comPort.PortName = MainV2.comPort.BaseStream.PortName;
            comPort.BaudRate = 57600;

            comPort.ReadTimeout = 4000;

            comPort.Open();

            lbl_status.Text = "Connecting";

            if (doConnect(comPort))
            {
                comPort.DiscardInBuffer();

                lbl_status.Text = "Doing Command";

                // remote
                string answer = doCommand(comPort, "RTI5");

                Console.Write("Remote\n"+answer);

                string[] items = answer.Split('\n');

                foreach (string item in items)
                {
                    if (item.StartsWith("S"))
                    {
                        string[] values = item.Split(':', '=');

                        if (values.Length == 3)
                        {
                            Control[] controls = this.Controls.Find("R"+values[0].Trim(), false);

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
                                            MessageBox.Show("Set Command error");
                                        }
                                    }
                                }
                                else
                                {
                                    if (controls[0].Text != values[2].Trim())
                                    {
                                        string cmdanswer = doCommand(comPort, "RT" + values[0].Trim() + "=" + controls[0].Text + "\r");

                                        if (cmdanswer.Contains("OK"))
                                        {

                                        }
                                        else
                                        {
                                            MessageBox.Show("Set Command error");
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
                comPort.WriteLine("RTZ");

                comPort.Write("\r\n");

                Sleep(100);

                comPort.DiscardInBuffer();
                //local
                answer = doCommand(comPort, "ATI5");

                Console.Write("Local\n" + answer);
                
                items = answer.Split('\n');

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
                                            MessageBox.Show("Set Command error");
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
                                            MessageBox.Show("Set Command error");
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // write it
                doCommand(comPort, "AT&W");

                lbl_status.Text = "Done";
            }
            else
            {
                lbl_status.Text = "Fail";
                MessageBox.Show("Failed to enter command mode");
            }

            // return to normal mode
            comPort.WriteLine("ATZ");

            comPort.Close();
        }
    }
}
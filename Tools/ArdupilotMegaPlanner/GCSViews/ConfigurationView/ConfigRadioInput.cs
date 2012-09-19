using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;
using ArdupilotMega.Controls;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigRadioInput : UserControl, IActivate, IDeactivate
    {
        bool startup = false;
        bool run = false;

        float[] rcmin = new float[8];
        float[] rcmax = new float[8];
        float[] rctrim = new float[8];

        Timer timer = new Timer();

        public ConfigRadioInput()
        {
            InitializeComponent();

            // setup rc calib extents
            for (int a = 0; a < rcmin.Length; a++)
            {
                rcmin[a] = 3000;
                rcmax[a] = 0;
                rctrim[a] = 1500;
            }

            // setup rc update
            timer.Tick += new EventHandler(timer_Tick);
        }

        public void Deactivate()
        {
            timer.Stop();
        }

        void timer_Tick(object sender, EventArgs e)
        {
            // update all linked controls - 10hz
            try
            {
                MainV2.cs.UpdateCurrentSettings(currentStateBindingSource);
            }
            catch { }
        }

        public void Activate()
        {
            timer.Enabled = true;
            timer.Interval = 100;
            timer.Start();

            startup = true;

            if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
                try
                {
                    CHK_mixmode.Checked = MainV2.comPort.param["ELEVON_MIXING"].ToString() == "1";
                    CHK_elevonrev.Checked = MainV2.comPort.param["ELEVON_REVERSE"].ToString() == "1";
                    CHK_elevonch1rev.Checked = MainV2.comPort.param["ELEVON_CH1_REV"].ToString() == "1";
                    CHK_elevonch2rev.Checked = MainV2.comPort.param["ELEVON_CH2_REV"].ToString() == "1";
                }
                catch { } // this will fail on arducopter
            }
            else
            {
                groupBoxElevons.Visible = false;
            }
            try
            {
                CHK_revch1.Checked = MainV2.comPort.param["RC1_REV"].ToString() == "-1";
                CHK_revch2.Checked = MainV2.comPort.param["RC2_REV"].ToString() == "-1";
                CHK_revch3.Checked = MainV2.comPort.param["RC3_REV"].ToString() == "-1";
                CHK_revch4.Checked = MainV2.comPort.param["RC4_REV"].ToString() == "-1";
            }
            catch (Exception ex) { /*CustomMessageBox.Show("Missing RC rev Param " + ex.ToString());*/ }
            startup = false;
        }

        private void BUT_Calibrateradio_Click(object sender, EventArgs e)
        {
            if (run)
            {
                BUT_Calibrateradio.Text = "Completed";
                run = false;
                return;
            }

            CustomMessageBox.Show("Ensure your transmitter is on and receiver is powered and connected\nEnsure your motor does not have power/no props!!!");

            byte oldrc = MainV2.cs.raterc;
            byte oldatt = MainV2.cs.rateattitude;
            byte oldpos = MainV2.cs.rateposition;
            byte oldstatus = MainV2.cs.ratestatus;

            MainV2.cs.raterc = 10;
            MainV2.cs.rateattitude = 0;
            MainV2.cs.rateposition = 0;
            MainV2.cs.ratestatus = 0;

            try
            {

                MainV2.comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.RC_CHANNELS, 10);

            }
            catch { }

            BUT_Calibrateradio.Text = "Click when Done";

            run = true;


            while (run)
            {
                Application.DoEvents();

                System.Threading.Thread.Sleep(5);

                MainV2.cs.UpdateCurrentSettings(currentStateBindingSource, true, MainV2.comPort);

                // check for non 0 values
                if (MainV2.cs.ch1in > 800 && MainV2.cs.ch1in < 2200)
                {
                    rcmin[0] = Math.Min(rcmin[0], MainV2.cs.ch1in);
                    rcmax[0] = Math.Max(rcmax[0], MainV2.cs.ch1in);

                    rcmin[1] = Math.Min(rcmin[1], MainV2.cs.ch2in);
                    rcmax[1] = Math.Max(rcmax[1], MainV2.cs.ch2in);

                    rcmin[2] = Math.Min(rcmin[2], MainV2.cs.ch3in);
                    rcmax[2] = Math.Max(rcmax[2], MainV2.cs.ch3in);

                    rcmin[3] = Math.Min(rcmin[3], MainV2.cs.ch4in);
                    rcmax[3] = Math.Max(rcmax[3], MainV2.cs.ch4in);

                    rcmin[4] = Math.Min(rcmin[4], MainV2.cs.ch5in);
                    rcmax[4] = Math.Max(rcmax[4], MainV2.cs.ch5in);

                    rcmin[5] = Math.Min(rcmin[5], MainV2.cs.ch6in);
                    rcmax[5] = Math.Max(rcmax[5], MainV2.cs.ch6in);

                    rcmin[6] = Math.Min(rcmin[6], MainV2.cs.ch7in);
                    rcmax[6] = Math.Max(rcmax[6], MainV2.cs.ch7in);

                    rcmin[7] = Math.Min(rcmin[7], MainV2.cs.ch8in);
                    rcmax[7] = Math.Max(rcmax[7], MainV2.cs.ch8in);

                    BARroll.minline = (int)rcmin[0];
                    BARroll.maxline = (int)rcmax[0];

                    BARpitch.minline = (int)rcmin[1];
                    BARpitch.maxline = (int)rcmax[1];

                    BARthrottle.minline = (int)rcmin[2];
                    BARthrottle.maxline = (int)rcmax[2];

                    BARyaw.minline = (int)rcmin[3];
                    BARyaw.maxline = (int)rcmax[3];

                    BAR5.minline = (int)rcmin[4];
                    BAR5.maxline = (int)rcmax[4];

                    BAR6.minline = (int)rcmin[5];
                    BAR6.maxline = (int)rcmax[5];

                    BAR7.minline = (int)rcmin[6];
                    BAR7.maxline = (int)rcmax[6];

                    BAR8.minline = (int)rcmin[7];
                    BAR8.maxline = (int)rcmax[7];

                }
            }

            CustomMessageBox.Show("Ensure all your sticks are centered and throttle is down, and click ok to continue");

            MainV2.cs.UpdateCurrentSettings(currentStateBindingSource, true, MainV2.comPort);

            rctrim[0] = MainV2.cs.ch1in;
            rctrim[1] = MainV2.cs.ch2in;
            rctrim[2] = MainV2.cs.ch3in;
            rctrim[3] = MainV2.cs.ch4in;
            rctrim[4] = MainV2.cs.ch5in;
            rctrim[5] = MainV2.cs.ch6in;
            rctrim[6] = MainV2.cs.ch7in;
            rctrim[7] = MainV2.cs.ch8in;

            string data = "---------------\n";

            for (int a = 0; a < 8; a++)
            {
                // we want these to save no matter what
                BUT_Calibrateradio.Text = "Saving";
                try
                {
                    if (rcmin[a] != rcmax[a])
                    {
                        MainV2.comPort.setParam("RC" + (a + 1).ToString("0") + "_MIN", rcmin[a]);
                        MainV2.comPort.setParam("RC" + (a + 1).ToString("0") + "_MAX", rcmax[a]);
                    }
                    if (rctrim[a] < 1195 || rctrim[a] > 1205)
                        MainV2.comPort.setParam("RC" + (a + 1).ToString("0") + "_TRIM", rctrim[a]);
                }
                catch { CustomMessageBox.Show("Failed to set Channel " + (a + 1).ToString()); }

                data = data + "CH" + (a + 1) + " " + rcmin[a] + " | " + rcmax[a] + "\n";
            }

            MainV2.cs.raterc = oldrc;
            MainV2.cs.rateattitude = oldatt;
            MainV2.cs.rateposition = oldpos;
            MainV2.cs.ratestatus = oldstatus;

            try
            {

                MainV2.comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.RC_CHANNELS, oldrc);

            }
            catch { }

            CustomMessageBox.Show("Here are the detected radio options\nNOTE Channels not connected are displayed as 1500 +-2\nNormal values are around 1100 | 1900\nChannel:Min | Max \n" + data, "Radio");

            BUT_Calibrateradio.Text = "Completed";
        }

        private void CHK_mixmode_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["ELEVON_MIXING"] == null)
                {
                    CustomMessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("ELEVON_MIXING", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { CustomMessageBox.Show("Set ELEVON_MIXING Failed"); }
        }

        private void CHK_elevonrev_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["ELEVON_REVERSE"] == null)
                {
                    CustomMessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("ELEVON_REVERSE", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { CustomMessageBox.Show("Set ELEVON_REVERSE Failed"); }
        }

        private void CHK_elevonch1rev_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["ELEVON_CH1_REV"] == null)
                {
                    CustomMessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("ELEVON_CH1_REV", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { CustomMessageBox.Show("Set ELEVON_CH1_REV Failed"); }
        }

        private void CHK_elevonch2rev_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["ELEVON_CH2_REV"] == null)
                {
                    CustomMessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("ELEVON_CH2_REV", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { CustomMessageBox.Show("Set ELEVON_CH2_REV Failed"); }
        }

        private void CHK_revch1_CheckedChanged(object sender, EventArgs e)
        {
            reverseChannel("RC1_REV", ((CheckBox)sender).Checked, BARroll);
        }

        private void CHK_revch2_CheckedChanged(object sender, EventArgs e)
        {
            reverseChannel("RC2_REV", ((CheckBox)sender).Checked, BARpitch);
        }

        private void CHK_revch3_CheckedChanged(object sender, EventArgs e)
        {
            reverseChannel("RC3_REV", ((CheckBox)sender).Checked, BARthrottle);
        }

        private void CHK_revch4_CheckedChanged(object sender, EventArgs e)
        {
            reverseChannel("RC4_REV", ((CheckBox)sender).Checked, BARyaw);
        }

        void reverseChannel(string name, bool normalreverse, Control progressbar)
        {
            if (normalreverse == true)
            {
                ((HorizontalProgressBar2)progressbar).reverse = true;
                ((HorizontalProgressBar2)progressbar).BackgroundColor = Color.FromArgb(148, 193, 31);
                ((HorizontalProgressBar2)progressbar).ValueColor = Color.FromArgb(0x43, 0x44, 0x45);
            }
            else
            {
                ((HorizontalProgressBar2)progressbar).reverse = false;
                ((HorizontalProgressBar2)progressbar).BackgroundColor = Color.FromArgb(0x43, 0x44, 0x45);
                ((HorizontalProgressBar2)progressbar).ValueColor = Color.FromArgb(148, 193, 31);
            }

            if (startup)
                return;
            if (MainV2.comPort.param["SWITCH_ENABLE"] != null && (float)MainV2.comPort.param["SWITCH_ENABLE"] == 1)
            {
                try
                {
                    MainV2.comPort.setParam("SWITCH_ENABLE", 0);
                    CustomMessageBox.Show("Disabled Dip Switchs");
                }
                catch { CustomMessageBox.Show("Error Disableing Dip Switch"); }
            }
            try
            {
                int i = normalreverse == false ? 1 : -1;
                MainV2.comPort.setParam(name, i);
            }
            catch { CustomMessageBox.Show("Error Reversing"); }
        }
    }
}

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega.Setup
{
    public partial class Setup : Form
    {
        internal GCSViews.Configuration Configuration;
        bool run = false;
        bool startup = false;

        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        float[] rcmin = new float[8];
        float[] rcmax = new float[8];
        float[] rctrim = new float[8];

        Timer timer = new Timer();

        public Setup()
        {
            InitializeComponent();

            for (int a = 0; a < rcmin.Length; a++)
            {
                rcmin[a] = 3000;
                rcmax[a] = 0;
                rctrim[a] = 1500;
            }

            timer.Tick += new EventHandler(timer_Tick);

            timer.Enabled = true;
            timer.Interval = 100;
            timer.Start();
        }

        void timer_Tick(object sender, EventArgs e)
        {
            MainV2.cs.UpdateCurrentSettings(currentStateBindingSource);

            float pwm = 0;

            if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane) // APM 
            {
                pwm = MainV2.cs.ch8in;
                LBL_flightmodepwm.Text = "8: " + MainV2.cs.ch8in.ToString();
            }

            if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2) // ac2
            {
                pwm = MainV2.cs.ch5in;
                LBL_flightmodepwm.Text = "5: " + MainV2.cs.ch5in.ToString();
            }

            Control[] fmodelist = new Control[] { CMB_fmode1, CMB_fmode2, CMB_fmode3, CMB_fmode4, CMB_fmode5, CMB_fmode6 };

            foreach (Control ctl in fmodelist)
            {
                ctl.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
            }

            byte no = readSwitch(pwm);

            fmodelist[no].BackColor = Color.Green;

            if (tabControl1.SelectedTab == tabHeli)
            {
                if (HS3.minline == 0)
                    HS3.minline = 2200;

                if (HS4.minline == 0)
                    HS4.minline = 2200;

                HS3.minline = Math.Min(HS3.minline, (int)MainV2.cs.ch3in);
                HS3.maxline = Math.Max(HS3.maxline, (int)MainV2.cs.ch3in);

                HS4.minline = Math.Min(HS4.minline, (int)MainV2.cs.ch4in);
                HS4.maxline = Math.Max(HS4.maxline, (int)MainV2.cs.ch4in);
            }

        }

        // from arducopter code
        byte readSwitch(float inpwm)
        {
            int pulsewidth = (int)inpwm;			// default for Arducopter

            if (pulsewidth > 1230 && pulsewidth <= 1360) return 1;
            if (pulsewidth > 1360 && pulsewidth <= 1490) return 2;
            if (pulsewidth > 1490 && pulsewidth <= 1620) return 3;
            if (pulsewidth > 1620 && pulsewidth <= 1749) return 4;	// Software Manual
            if (pulsewidth >= 1750) return 5;	// Hardware Manual
            return 0;
        }

        private void BUT_Calibrateradio_Click(object sender, EventArgs e)
        {
            if (run)
            {
                BUT_Calibrateradio.Text = "Please goto the next tab";
                run = false;
                return;
            }

            MessageBox.Show("Ensure your transmitter is on and receiver is powered and connected\nEnsure your motor does not have power/no props!!!");

            byte oldrc = MainV2.cs.raterc;
            byte oldatt = MainV2.cs.rateattitude;
            byte oldpos = MainV2.cs.rateposition;
            byte oldstatus = MainV2.cs.ratestatus;

            MainV2.cs.raterc = 10;
            MainV2.cs.rateattitude = 0;
            MainV2.cs.rateposition = 0;
            MainV2.cs.ratestatus = 0;

            MainV2.comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RC_CHANNELS, 10);

            BUT_Calibrateradio.Text = "Click when Done";

            run = true;


            while (run)
            {
                Application.DoEvents();

                System.Threading.Thread.Sleep(5);

                MainV2.cs.UpdateCurrentSettings(currentStateBindingSource, true);

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

            MessageBox.Show("Ensure all your sticks are centered, and click ok to continue");

            MainV2.cs.UpdateCurrentSettings(currentStateBindingSource, true);

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
                    if (rctrim[a] < 1195 && rctrim[a] > 1205)
                        MainV2.comPort.setParam("RC" + (a + 1).ToString("0") + "_TRIM", rctrim[a]);
                }
                catch { MessageBox.Show("Failed to set Channel " + (a + 1).ToString()); }

                data = data +"CH" + (a+1) +  " " + rcmin[a] + " | " + rcmax[a] + "\n";
            }

            MainV2.cs.raterc = oldrc;
            MainV2.cs.rateattitude = oldatt;
            MainV2.cs.rateposition = oldpos;
            MainV2.cs.ratestatus = oldstatus;

            MainV2.comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RC_CHANNELS, oldrc);

            if (Configuration != null)
            {
                Configuration.startup = true;
                Configuration.processToScreen();
                Configuration.startup = false;
            }

            MessageBox.Show("Here are the detected radio options\nNOTE Channels not connected are displayed as 1500 +-2\nNormal values are around 1100 | 1900\nChannel:Min | Max \n" + data, "Radio");

            BUT_Calibrateradio.Text = "Please goto the next tab";
        }




        private void tabControl1_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (tabControl1.SelectedTab == tabModes)
            {
                if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane) // APM
                {
                    CB_simple1.Visible = false;
                    CB_simple2.Visible = false;
                    CB_simple3.Visible = false;
                    CB_simple4.Visible = false;
                    CB_simple5.Visible = false;
                    CB_simple6.Visible = false;

                    CMB_fmode1.Items.Clear();
                    CMB_fmode2.Items.Clear();
                    CMB_fmode3.Items.Clear();
                    CMB_fmode4.Items.Clear();
                    CMB_fmode5.Items.Clear();
                    CMB_fmode6.Items.Clear();

                    CMB_fmode1.Items.AddRange(Enum.GetNames(typeof(Common.apmmodes)));
                    CMB_fmode2.Items.AddRange(Enum.GetNames(typeof(Common.apmmodes)));
                    CMB_fmode3.Items.AddRange(Enum.GetNames(typeof(Common.apmmodes)));
                    CMB_fmode4.Items.AddRange(Enum.GetNames(typeof(Common.apmmodes)));
                    CMB_fmode5.Items.AddRange(Enum.GetNames(typeof(Common.apmmodes)));
                    CMB_fmode6.Items.AddRange(Enum.GetNames(typeof(Common.apmmodes)));

                    try
                    {
                        CMB_fmode1.Text = Enum.Parse(typeof(Common.apmmodes), MainV2.comPort.param["FLTMODE1"].ToString()).ToString();
                        CMB_fmode2.Text = Enum.Parse(typeof(Common.apmmodes), MainV2.comPort.param["FLTMODE2"].ToString()).ToString();
                        CMB_fmode3.Text = Enum.Parse(typeof(Common.apmmodes), MainV2.comPort.param["FLTMODE3"].ToString()).ToString();
                        CMB_fmode4.Text = Enum.Parse(typeof(Common.apmmodes), MainV2.comPort.param["FLTMODE4"].ToString()).ToString();
                        CMB_fmode5.Text = Enum.Parse(typeof(Common.apmmodes), MainV2.comPort.param["FLTMODE5"].ToString()).ToString();
                        CMB_fmode6.Text = Enum.Parse(typeof(Common.apmmodes), MainV2.comPort.param["FLTMODE6"].ToString()).ToString();
                    }
                    catch { }
                }
                if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2) // ac2
                {
                    CMB_fmode1.Items.Clear();
                    CMB_fmode2.Items.Clear();
                    CMB_fmode3.Items.Clear();
                    CMB_fmode4.Items.Clear();
                    CMB_fmode5.Items.Clear();
                    CMB_fmode6.Items.Clear();

                    CMB_fmode1.Items.AddRange(Enum.GetNames(typeof(Common.ac2modes)));
                    CMB_fmode2.Items.AddRange(Enum.GetNames(typeof(Common.ac2modes)));
                    CMB_fmode3.Items.AddRange(Enum.GetNames(typeof(Common.ac2modes)));
                    CMB_fmode4.Items.AddRange(Enum.GetNames(typeof(Common.ac2modes)));
                    CMB_fmode5.Items.AddRange(Enum.GetNames(typeof(Common.ac2modes)));
                    CMB_fmode6.Items.AddRange(Enum.GetNames(typeof(Common.ac2modes)));

                    try
                    {
                        CMB_fmode1.Text = Enum.Parse(typeof(Common.ac2modes), MainV2.comPort.param["FLTMODE1"].ToString()).ToString();
                        CMB_fmode2.Text = Enum.Parse(typeof(Common.ac2modes), MainV2.comPort.param["FLTMODE2"].ToString()).ToString();
                        CMB_fmode3.Text = Enum.Parse(typeof(Common.ac2modes), MainV2.comPort.param["FLTMODE3"].ToString()).ToString();
                        CMB_fmode4.Text = Enum.Parse(typeof(Common.ac2modes), MainV2.comPort.param["FLTMODE4"].ToString()).ToString();
                        CMB_fmode5.Text = Enum.Parse(typeof(Common.ac2modes), MainV2.comPort.param["FLTMODE5"].ToString()).ToString();
                        CMB_fmode6.Text = Enum.Parse(typeof(Common.ac2modes), MainV2.comPort.param["FLTMODE6"].ToString()).ToString();
                    }
                    catch { }
                }
            }

            if (tabControl1.SelectedTab == tabHardware)
            {
                startup = true;
                if (MainV2.comPort.param["ARSPD_ENABLE"] != null)
                    CHK_enableairspeed.Checked = MainV2.comPort.param["ARSPD_ENABLE"].ToString() == "1" ? true : false;

                if (MainV2.comPort.param["SONAR_ENABLE"] != null)
                    CHK_enablesonar.Checked = MainV2.comPort.param["SONAR_ENABLE"].ToString() == "1" ? true : false;

                if (MainV2.comPort.param["MAG_ENABLE"] != null)
                    CHK_enablecompass.Checked = MainV2.comPort.param["MAG_ENABLE"].ToString() == "1" ? true : false;

                if (MainV2.comPort.param["BATT_MONITOR"] != null)
                {
                    if (MainV2.comPort.param["BATT_MONITOR"].ToString() != "0")
                    {
                        CHK_enablebattmon.Checked = true;
                        CMB_batmontype.SelectedIndex = (int)float.Parse(MainV2.comPort.param["BATT_MONITOR"].ToString());
                    }
                }

                if (MainV2.comPort.param["COMPASS_DEC"] != null)
                    TXT_declination.Text = (float.Parse(MainV2.comPort.param["COMPASS_DEC"].ToString()) * rad2deg).ToString();

                if (MainV2.comPort.param["BATT_CAPACITY"] != null)
                    TXT_battcapacity.Text = MainV2.comPort.param["BATT_CAPACITY"].ToString();

                startup = false;
            }

            if (tabControl1.SelectedTab == tabArducopter)
            {
                if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
                {
                    tabArducopter.Enabled = false;
                    return;
                }
            }

            if (tabControl1.SelectedTab == tabHeli)
            {
                if (MainV2.comPort.param["GYR_ENABLE_"] == null)
                {
                    tabHeli.Enabled = false;
                    return;
                }
                startup = true;
                try
                {
                    foreach (string value in MainV2.comPort.param.Keys)
                    {
                        if (value == "")
                            continue;

                        Control[] control = tabHeli.Controls.Find(value, true);
                        if (control.Length > 0)
                        {
                            if (control[0].GetType() == typeof(TextBox))
                            {
                                TextBox temp = (TextBox)control[0];
                                string option = MainV2.comPort.param[value].ToString();
                                temp.Text = option;
                            }
                            if (control[0].GetType() == typeof(CheckBox))
                            {
                                CheckBox temp = (CheckBox)control[0];
                                string option = MainV2.comPort.param[value].ToString();
                                temp.Checked = option == "1" ? true : false;
                            }
                            if (control[0].GetType() == typeof(MyTrackBar))
                            {
                                MyTrackBar temp = (MyTrackBar)control[0];
                                string option = MainV2.comPort.param[value].ToString();
                                temp.Value = int.Parse(option);
                            }
                        }
                    }
                }
                catch { }
                startup = false;
            }
        }

        private void BUT_SaveModes_Click(object sender, EventArgs e)
        {
            try
            {
                if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane) // APM
                {
                    MainV2.comPort.setParam("FLTMODE1", (float)(int)Enum.Parse(typeof(Common.apmmodes), CMB_fmode1.Text));
                    MainV2.comPort.setParam("FLTMODE2", (float)(int)Enum.Parse(typeof(Common.apmmodes), CMB_fmode2.Text));
                    MainV2.comPort.setParam("FLTMODE3", (float)(int)Enum.Parse(typeof(Common.apmmodes), CMB_fmode3.Text));
                    MainV2.comPort.setParam("FLTMODE4", (float)(int)Enum.Parse(typeof(Common.apmmodes), CMB_fmode4.Text));
                    MainV2.comPort.setParam("FLTMODE5", (float)(int)Enum.Parse(typeof(Common.apmmodes), CMB_fmode5.Text));
                    MainV2.comPort.setParam("FLTMODE6", (float)(int)Enum.Parse(typeof(Common.apmmodes), CMB_fmode6.Text));
                }
                if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2) // ac2
                {
                    MainV2.comPort.setParam("FLTMODE1", (float)(int)Enum.Parse(typeof(Common.ac2modes), CMB_fmode1.Text));
                    MainV2.comPort.setParam("FLTMODE2", (float)(int)Enum.Parse(typeof(Common.ac2modes), CMB_fmode2.Text));
                    MainV2.comPort.setParam("FLTMODE3", (float)(int)Enum.Parse(typeof(Common.ac2modes), CMB_fmode3.Text));
                    MainV2.comPort.setParam("FLTMODE4", (float)(int)Enum.Parse(typeof(Common.ac2modes), CMB_fmode4.Text));
                    MainV2.comPort.setParam("FLTMODE5", (float)(int)Enum.Parse(typeof(Common.ac2modes), CMB_fmode5.Text));
                    MainV2.comPort.setParam("FLTMODE6", (float)(int)Enum.Parse(typeof(Common.ac2modes), CMB_fmode6.Text));

                    float value = (float)(CB_simple1.Checked ? 1 : 0) + (CB_simple2.Checked ? 1 << 1 : 0) + (CB_simple3.Checked ? 1 <<2 : 0)
                        + (CB_simple4.Checked ? 1 << 3 : 0) + (CB_simple5.Checked ? 1 << 4 : 0) + (CB_simple6.Checked ? 1 << 5 : 0);
                    MainV2.comPort.setParam("SIMPLE", value);
                }
            }
            catch { MessageBox.Show("Failed to set Flight modes"); }
        }
        private void TXT_declination_Validating(object sender, CancelEventArgs e)
        {
            float ans = 0;
            e.Cancel = !float.TryParse(TXT_declination.Text, out ans);
        }

        private void TXT_battcapacity_Validating(object sender, CancelEventArgs e)
        {
            float ans = 0;
            e.Cancel = !float.TryParse(TXT_declination.Text, out ans);
        }

        private void CMB_batmontype_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["BATT_MONITOR"] == null)
                {
                    MessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("BATT_MONITOR", CMB_batmontype.SelectedIndex);
                    if (CMB_batmontype.SelectedIndex != 0)
                    {
                        CHK_enablebattmon.Checked = true;
                    }
                    else
                    {
                        CHK_enablebattmon.Checked = false;
                    }
                }
            }
            catch { MessageBox.Show("Set BATT_MONITOR Failed"); }
        }

        private void TXT_declination_Validated(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["COMPASS_DEC"] == null)
                {
                    MessageBox.Show("Not Available");
                }
                else
                {
                    float dec = 0.0f;
                    try
                    {
                        string declination = TXT_declination.Text;
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
                    }
                    catch { MessageBox.Show("Invalid input!"); return; }

                    MainV2.comPort.setParam("COMPASS_DEC", dec * deg2rad);
                }
            }
            catch { MessageBox.Show("Set COMPASS_DEC Failed"); }
        }

        private void TXT_battcapacity_Validated(object sender, EventArgs e)
        {
            if (startup || ((TextBox)sender).Enabled == false)
                return;
            try
            {
                if (MainV2.comPort.param["BATT_CAPACITY"] == null)
                {
                    MessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("BATT_CAPACITY", float.Parse(TXT_battcapacity.Text));
                }
            }
            catch { MessageBox.Show("Set BATT_CAPACITY Failed"); }
        }

        private void CHK_enablecompass_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["MAG_ENABLE"] == null)
                {
                    MessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("MAG_ENABLE", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { MessageBox.Show("Set MAG_ENABLE Failed"); }
        }

        //((CheckBox)sender).Checked = !((CheckBox)sender).Checked;

        private void CHK_enablebattmon_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {                    
                if (((CheckBox)sender).Checked == false)
                {
                    CMB_batmontype.SelectedIndex = 0;
                }
                else
                {
                    if (CMB_batmontype.SelectedIndex <= 0)
                        CMB_batmontype.SelectedIndex = 1;
                }
            }
            catch { MessageBox.Show("Set BATT_MONITOR Failed"); }
        }

        private void CHK_enablesonar_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["SONAR_ENABLE"] == null)
                {
                    MessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("SONAR_ENABLE", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { MessageBox.Show("Set SONAR_ENABLE Failed"); }
        }

        private void CHK_enableairspeed_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["ARSPD_ENABLE"] == null)
                {
                    MessageBox.Show("Not Available on "+ MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("ARSPD_ENABLE", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { MessageBox.Show("Set ARSPD_ENABLE Failed"); }
        }

        private void BUT_reset_Click(object sender, EventArgs e)
        {
            try
            {
                MainV2.comPort.setParam("SYSID_SW_MREV", UInt16.MaxValue);
            }
            catch { MessageBox.Show("Set SYSID_SW_MREV Failed"); return; }

            MainV2.givecomport = true;

            ICommsSerial comPortT = MainV2.comPort.BaseStream;

            comPortT.DtrEnable = false;

            if (comPortT.IsOpen)
                comPortT.Close();

            System.Threading.Thread.Sleep(200);

            try
            {
                comPortT.DtrEnable = true;
                comPortT.Open();
            }
            catch (Exception ex) { MainV2.givecomport = false; MessageBox.Show("Invalid Comport Settings : " + ex.Message); return; }

            BUT_reset.Text = "Rebooting (20 sec)";
            BUT_reset.Refresh();
            Application.DoEvents();

            Sleep(20000, comPortT); // wait for boot/reset

            comPortT.DtrEnable = false;

            Sleep(200, comPortT);

            comPortT.DtrEnable = true;

            Sleep(200, comPortT);

            comPortT.DtrEnable = false;

            comPortT.Close();

            MainV2.givecomport = false;
            try
            {
                MainV2.comPort.Open(true);
            }
            catch
            {
                MessageBox.Show("Failed to re-connect : Please try again");
                this.Close();
            }

            BUT_reset.Text = "Please goto next tab";
        }

        void Sleep(int ms, ICommsSerial comPortT)
        {
            DateTime start = DateTime.Now;
            Console.WriteLine("sleep in");
            while (start.AddMilliseconds(ms) > DateTime.Now)
            {
                while (comPortT.BytesToRead > 0)
                {
                    Console.Write((char)comPortT.ReadByte());
                }
                System.Threading.Thread.Sleep(1);
            }
            Console.WriteLine("sleep out");
        } 

        private void pictureBoxQuad_Click(object sender, EventArgs e)
        {
            try
            {
                MainV2.comPort.setParam("FRAME", 0f);
                MessageBox.Show("Set to +");
            }
            catch { MessageBox.Show("Set frame failed"); }
        }

        private void pictureBoxQuadX_Click(object sender, EventArgs e)
        {
            try
            {
                MainV2.comPort.setParam("FRAME", 1f);
                MessageBox.Show("Set to x");
            }
            catch { MessageBox.Show("Set frame failed"); }
        }

        private void Setup_Load(object sender, EventArgs e)
        {
            if (!MainV2.comPort.BaseStream.IsOpen)
            {
                MessageBox.Show("Please Connect First");
                this.Close();
            }
        }

        private void TXT_srvpos1_Validating(object sender, CancelEventArgs e)
        {
            if (startup || this.Disposing)
                return;
            int test = 0;
            if (!int.TryParse(((TextBox)sender).Text, out test))
            {
                e.Cancel = true;
            }

            Gservoloc.Value0 = test;

            try
            {
                MainV2.comPort.setParam(((TextBox)sender).Name, test);

            }
            catch { MessageBox.Show("Set " + ((TextBox)sender).Name + " failed"); }
        }

        private void TXT_srvpos2_Validating(object sender, CancelEventArgs e)
        {
            if (startup || this.Disposing)
                return;
            int test = 0;
            if (!int.TryParse(((TextBox)sender).Text, out test))
            {
                e.Cancel = true;
            }

            Gservoloc.Value1 = test;

            try
            {
                MainV2.comPort.setParam(((TextBox)sender).Name, test);
            }
            catch { MessageBox.Show("Set " + ((TextBox)sender).Name + " failed"); }
        }

        private void TXT_srvpos3_Validating(object sender, CancelEventArgs e)
        {
            if (startup || this.Disposing)
                return;
            int test = 0;
            if (!int.TryParse(((TextBox)sender).Text, out test))
            {
                e.Cancel = true;
            }

            Gservoloc.Value2 = test;

            try
            {
                MainV2.comPort.setParam(((TextBox)sender).Name, test);
            }
            catch { MessageBox.Show("Set " + ((TextBox)sender).Name + " failed"); }
        }

        private void BUT_0collective_Click(object sender, EventArgs e)
        {
            MessageBox.Show("Make sure your blades are at 0 degrees");

            try
            {

                MainV2.comPort.setParam("COL_MID_", MainV2.cs.ch3in);

                COL_MID_.Text = MainV2.comPort.param["COL_MID_"].ToString();
            }
            catch { MessageBox.Show("Set COL_MID_ failed"); }
        }

        private void HS1_REV_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((CheckBox)sender).Name, ((CheckBox)sender).Checked == true ? 1.0f : -1.0f);
        }

        private void HS2_REV_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((CheckBox)sender).Name, ((CheckBox)sender).Checked == true ? 1.0f : -1.0f);
        }

        private void HS3_REV_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((CheckBox)sender).Name, ((CheckBox)sender).Checked == true ? 1.0f : -1.0f);
        }

        private void HS4_REV_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((CheckBox)sender).Name, ((CheckBox)sender).Checked == true ? 1.0f : -1.0f);
        }

        private void HS1_TRIM_Scroll(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((MyTrackBar)sender).Name, (float)((MyTrackBar)sender).Value);
        }

        private void HS2_TRIM_Scroll(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((MyTrackBar)sender).Name, (float)((MyTrackBar)sender).Value);
        }

        private void HS3_TRIM_Scroll(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((MyTrackBar)sender).Name, (float)((MyTrackBar)sender).Value);
        }

        private void HS4_TRIM_Scroll(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((MyTrackBar)sender).Name, (float)((MyTrackBar)sender).Value);
        }

        private void ROL_MAX__Validating(object sender, CancelEventArgs e)
        {
            if (startup || this.Disposing)
                return;
            int test = 0;
            if (!int.TryParse(((TextBox)sender).Text, out test))
            {
                e.Cancel = true;
            }

            MainV2.comPort.setParam(((TextBox)sender).Name, test);
        }

        private void PIT_MAX__Validating(object sender, CancelEventArgs e)
        {
            if (startup || this.Disposing)
                return;
            int test = 0;
            if (!int.TryParse(((TextBox)sender).Text, out test))
            {
                e.Cancel = true;
            }

            MainV2.comPort.setParam(((TextBox)sender).Name, test);
        }

        private void GYR_GAIN__Validating(object sender, CancelEventArgs e)
        {
            if (startup || this.Disposing || ((TextBox)sender).Enabled == false)
                return;
            int test = 0;
            if (!int.TryParse(((TextBox)sender).Text, out test))
            {
                e.Cancel = true;
            }

            try
            {
                MainV2.comPort.setParam(((TextBox)sender).Name, test);
            }
            catch { MessageBox.Show("Failed to set Gyro Gain"); }
        }

        private void GYR_ENABLE__CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((CheckBox)sender).Name, ((CheckBox)sender).Checked == true ? 1.0f : 0.0f);
        }

        private void BUT_saveheliconfig_Click(object sender, EventArgs e)
        {
            try
            {
                MainV2.comPort.setParam("COL_MIN_", HS3.minline);
                MainV2.comPort.setParam("COL_MAX_", HS3.maxline);

                MainV2.comPort.setParam("HS4_MIN", HS4.minline);
                MainV2.comPort.setParam("HS4_MAX", HS4.maxline);
            }
            catch { MessageBox.Show("Failed to set min/max");  }
        }

        private void BUT_levelac2_Click(object sender, EventArgs e)
        {
            try
            {
                MainV2.comPort.doAction(MAVLink.MAV_ACTION.MAV_ACTION_CALIBRATE_ACC);

                BUT_levelac2.Text = "Complete";
            }
            catch
            {
                MessageBox.Show("Failed to level : ac2 2.0.37+ is required");
            }
        }

        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            try
            {
                //System.Diagnostics.Process.Start("http://www.ngdc.noaa.gov/geomagmodels/Declination.jsp");
                System.Diagnostics.Process.Start("http://www.magnetic-declination.com/");
            }
            catch { MessageBox.Show("Webpage open failed... do you have a virus?\nhttp://www.magnetic-declination.com/"); }
        }

        void reverseChannel(string name,bool normalreverse,Control progressbar)
        {
            if (startup)
                return;
            if (MainV2.comPort.param["SWITCH_ENABLE"] != null && (float)MainV2.comPort.param["SWITCH_ENABLE"] == 1)
            {
                try
                {
                    MainV2.comPort.setParam("SWITCH_ENABLE", 0);
                    MessageBox.Show("Disabled Dip Switchs");
                }
                catch { MessageBox.Show("Error Disableing Dip Switch"); }
            }
            try
            {
                int i = normalreverse == false ? 1 : -1;
                MainV2.comPort.setParam(name, i);

                if (normalreverse == true)
                {
                    ((HorizontalProgressBar2)progressbar).BackgroundColor = Color.FromArgb(148, 193, 31);
                    ((HorizontalProgressBar2)progressbar).ValueColor = Color.FromArgb(0x43, 0x44, 0x45); 
                }
                else
                {
                    ((HorizontalProgressBar2)progressbar).BackgroundColor = Color.FromArgb(0x43, 0x44, 0x45);
                    ((HorizontalProgressBar2)progressbar).ValueColor = Color.FromArgb(148, 193, 31);
                }
            }
            catch { MessageBox.Show("Error Reversing"); }
        }

        private void CHK_revch1_CheckedChanged(object sender, EventArgs e)
        {
            reverseChannel("RC1_REV", ((CheckBox)sender).Checked,BARroll);
        }

        private void CHK_revch2_CheckedChanged(object sender, EventArgs e)
        {
            reverseChannel("RC2_REV", ((CheckBox)sender).Checked,BARpitch);
        }

        private void CHK_revch3_CheckedChanged(object sender, EventArgs e)
        {
            reverseChannel("RC3_REV", ((CheckBox)sender).Checked,BARthrottle);
        }

        private void CHK_revch4_CheckedChanged(object sender, EventArgs e)
        {
            reverseChannel("RC4_REV", ((CheckBox)sender).Checked,BARyaw);
        }
    }
}

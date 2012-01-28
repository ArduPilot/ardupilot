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

        bool inpwmdetect = false;

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

            MainV2.comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RC_CHANNELS, MainV2.cs.raterc);

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
                if (MainV2.comPort.param["HSV_MAN"] == null || MainV2.comPort.param["HSV_MAN"].ToString() == "0")
                    return;

                if (HS3.minline == 0)
                    HS3.minline = 2200;

                if (HS4.minline == 0)
                    HS4.minline = 2200;

                HS3.minline = Math.Min(HS3.minline, (int)MainV2.cs.ch3in);
                HS3.maxline = Math.Max(HS3.maxline, (int)MainV2.cs.ch3in);

                HS4.minline = Math.Min(HS4.minline, (int)MainV2.cs.ch4in);
                HS4.maxline = Math.Max(HS4.maxline, (int)MainV2.cs.ch4in);

                if (!inpwmdetect)
                {
                    HS3_Paint(null, null);
                    HS4_Paint(null, null);
                }
                else
                {
                    try
                    {
                        HS3.minline = int.Parse(COL_MIN_.Text);
                        HS3.maxline = int.Parse(COL_MAX_.Text);
                        HS4.maxline = int.Parse(HS4_MIN.Text);
                        HS4.minline = int.Parse(HS4_MAX.Text);
                    }
                    catch { }
                }
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

            try
            {

                MainV2.comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RC_CHANNELS, 10);

            }
            catch {  }

            BUT_Calibrateradio.Text = "Click when Done";

            run = true;


            while (run)
            {
                Application.DoEvents();

                System.Threading.Thread.Sleep(5);

                MainV2.cs.UpdateCurrentSettings(currentStateBindingSource, true, MainV2.comPort);

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
                catch { MessageBox.Show("Failed to set Channel " + (a + 1).ToString()); }

                data = data + "CH" + (a + 1) + " " + rcmin[a] + " | " + rcmax[a] + "\n";
            }

            MainV2.cs.raterc = oldrc;
            MainV2.cs.rateattitude = oldatt;
            MainV2.cs.rateposition = oldpos;
            MainV2.cs.ratestatus = oldstatus;

            try
            {

                MainV2.comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RC_CHANNELS, oldrc);

            }
            catch { }

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
            int monosux = 0;
            monosux *= 5;

            if (tabControl1.SelectedTab == tabRadioIn)
            {
                startup = true;

                if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2)
                {
                    groupBoxElevons.Visible = false;
                }

                try
                {
                    CHK_mixmode.Checked = MainV2.comPort.param["ELEVON_MIXING"].ToString() == "1";
                    CHK_elevonrev.Checked = MainV2.comPort.param["ELEVON_REVERSE"].ToString() == "1";
                    CHK_elevonch1rev.Checked = MainV2.comPort.param["ELEVON_CH1_REV"].ToString() == "1";
                    CHK_elevonch2rev.Checked = MainV2.comPort.param["ELEVON_CH2_REV"].ToString() == "1";
                }
                catch {  } // this will fail on arducopter
                try
                {
                    CHK_revch1.Checked = MainV2.comPort.param["RC1_REV"].ToString() == "-1";
                    CHK_revch2.Checked = MainV2.comPort.param["RC2_REV"].ToString() == "-1";
                    CHK_revch3.Checked = MainV2.comPort.param["RC3_REV"].ToString() == "-1";
                    CHK_revch4.Checked = MainV2.comPort.param["RC4_REV"].ToString() == "-1";
                }
                catch (Exception ex) { MessageBox.Show("Missing RC rev Param "+ex.ToString()); }
                startup = false;
            }

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
                        CMB_fmode6.Text = Common.apmmodes.MANUAL.ToString();
                        CMB_fmode6.Enabled = false;
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
                        CMB_fmode6.Enabled = true;

                        int simple = int.Parse(MainV2.comPort.param["SIMPLE"].ToString());

                        CB_simple1.Checked = ((simple >> 0 & 1) == 1);
                        CB_simple2.Checked = ((simple >> 1 & 1) == 1);
                        CB_simple3.Checked = ((simple >> 2 & 1) == 1);
                        CB_simple4.Checked = ((simple >> 3 & 1) == 1);
                        CB_simple5.Checked = ((simple >> 4 & 1) == 1);
                        CB_simple6.Checked = ((simple >> 5 & 1) == 1);
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

                if (MainV2.comPort.param["COMPASS_DEC"] != null)
                    TXT_declination.Text = (float.Parse(MainV2.comPort.param["COMPASS_DEC"].ToString()) * rad2deg).ToString();

                if (MainV2.comPort.param["SONAR_TYPE"] != null)
                    CMB_sonartype.SelectedIndex = int.Parse(MainV2.comPort.param["SONAR_TYPE"].ToString());

                if (MainV2.comPort.param["FLOW_ENABLE"] != null)
                    CHK_enableoptflow.Checked = MainV2.comPort.param["FLOW_ENABLE"].ToString() == "1" ? true : false;
                

                startup = false;
            }

            if (tabControl1.SelectedTab == tabBattery)
            {
                startup = true;
                bool not_supported = false;
                if (MainV2.comPort.param["BATT_MONITOR"] != null)
                {
                    if (MainV2.comPort.param["BATT_MONITOR"].ToString() != "0.0")
                    {
                        CMB_batmontype.SelectedIndex = getIndex(CMB_batmontype,(int)float.Parse(MainV2.comPort.param["BATT_MONITOR"].ToString()));
                    }

                    if (TXT_ampspervolt.Text == "13.6612")
                    {
                        CMB_batmonsensortype.SelectedIndex = 1;
                    }
                    else if (TXT_ampspervolt.Text == "27.3224")
                    {
                        CMB_batmonsensortype.SelectedIndex = 2;
                    }
                    else if (TXT_ampspervolt.Text == "54.64481")
                    {
                        CMB_batmonsensortype.SelectedIndex = 3;
                    }
                    else
                    {
                        CMB_batmonsensortype.SelectedIndex = 0;
                    }
                    
                }

                if (MainV2.comPort.param["BATT_CAPACITY"] != null)
                    TXT_battcapacity.Text = MainV2.comPort.param["BATT_CAPACITY"].ToString();
                if (MainV2.comPort.param["INPUT_VOLTS"] != null)
                    TXT_inputvoltage.Text = MainV2.comPort.param["INPUT_VOLTS"].ToString();
                else
                    not_supported = true;
                TXT_voltage.Text = MainV2.cs.battery_voltage.ToString();
                TXT_measuredvoltage.Text = TXT_voltage.Text;
                if (MainV2.comPort.param["VOLT_DIVIDER"] != null)
                    TXT_divider.Text = MainV2.comPort.param["VOLT_DIVIDER"].ToString();
                else
                    not_supported = true;
                if (MainV2.comPort.param["AMP_PER_VOLT"] != null)
                    TXT_ampspervolt.Text = MainV2.comPort.param["AMP_PER_VOLT"].ToString();
                else
                    not_supported = true;
                if (not_supported)
                {
                    TXT_inputvoltage.Enabled = false;
                    TXT_measuredvoltage.Enabled = false;
                    TXT_divider.Enabled = false;
                    TXT_ampspervolt.Enabled = false;
                }

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
                            if (control[0].GetType() == typeof(NumericUpDown))
                            {
                                NumericUpDown temp = (NumericUpDown)control[0];
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

                    HS1_REV.Checked = MainV2.comPort.param["HS1_REV"].ToString() == "-1";
                    HS2_REV.Checked = MainV2.comPort.param["HS2_REV"].ToString() == "-1";
                    HS3_REV.Checked = MainV2.comPort.param["HS3_REV"].ToString() == "-1";
                    HS4_REV.Checked = MainV2.comPort.param["HS4_REV"].ToString() == "-1";

                }
                catch { }
                startup = false;
            }
        }

        int getIndex(ComboBox ctl, int no)
        {
            foreach (var item in ctl.Items)
            {
                int ans = int.Parse(item.ToString().Substring(0, 1));

                if (ans == no)
                    return ctl.Items.IndexOf(item);
            }

            return -1;
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

                    float value = (float)(CB_simple1.Checked ? 1 : 0) + (CB_simple2.Checked ? 1 << 1 : 0) + (CB_simple3.Checked ? 1 << 2 : 0)
                        + (CB_simple4.Checked ? 1 << 3 : 0) + (CB_simple5.Checked ? 1 << 4 : 0) + (CB_simple6.Checked ? 1 << 5 : 0);
                    if (MainV2.comPort.param.ContainsKey("SIMPLE"))
                        MainV2.comPort.setParam("SIMPLE", value);
                }
            }
            catch { MessageBox.Show("Failed to set Flight modes"); }
            BUT_SaveModes.Text = "Complete";
        }

        private void TXT_declination_Validating(object sender, CancelEventArgs e)
        {
            float ans = 0;
            e.Cancel = !float.TryParse(TXT_declination.Text, out ans);
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

                    TXT_declination.Text = dec.ToString();

                    MainV2.comPort.setParam("COMPASS_DEC", dec * deg2rad);
                }
            }
            catch { MessageBox.Show("Set COMPASS_DEC Failed"); }
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
                    MessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("ARSPD_ENABLE", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { MessageBox.Show("Set ARSPD_ENABLE Failed"); }
        }
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
        private void TXT_battcapacity_Validating(object sender, CancelEventArgs e)
        {
            float ans = 0;
            e.Cancel = !float.TryParse(TXT_battcapacity.Text, out ans);
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
                    int selection = int.Parse(CMB_batmontype.Text.Substring(0,1));

                    CMB_batmonsensortype.Enabled = true;

                    TXT_voltage.Enabled = false;

                    if (selection == 0)
                    {
                        CMB_batmonsensortype.Enabled = false;
                        groupBox4.Enabled = false;
                    }
                    else if (selection == 4)
                    {
                        CMB_batmonsensortype.Enabled = true;
                        groupBox4.Enabled = true;
                        TXT_ampspervolt.Enabled = true;
                    }
                    else if (selection == 3)
                    {
                        groupBox4.Enabled = true;
                        CMB_batmonsensortype.Enabled = false;
                        TXT_ampspervolt.Enabled = false;
                        TXT_inputvoltage.Enabled = true;
                        TXT_measuredvoltage.Enabled = true;
                        TXT_divider.Enabled = true;
                    }

                    MainV2.comPort.setParam("BATT_MONITOR", selection);
                }
            }
            catch { MessageBox.Show("Set BATT_MONITOR Failed"); }
        }
        private void TXT_inputvoltage_Validating(object sender, CancelEventArgs e)
        {
            float ans = 0;
            e.Cancel = !float.TryParse(TXT_inputvoltage.Text, out ans);
        }
        private void TXT_inputvoltage_Validated(object sender, EventArgs e)
        {
            if (startup || ((TextBox)sender).Enabled == false)
                return;
            try
            {
                if (MainV2.comPort.param["INPUT_VOLTS"] == null)
                {
                    MessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("INPUT_VOLTS", float.Parse(TXT_inputvoltage.Text));
                }
            }
            catch { MessageBox.Show("Set INPUT_VOLTS Failed"); }
        }
        private void TXT_measuredvoltage_Validating(object sender, CancelEventArgs e)
        {
            float ans = 0;
            e.Cancel = !float.TryParse(TXT_measuredvoltage.Text, out ans);
        }
        private void TXT_measuredvoltage_Validated(object sender, EventArgs e)
        {
            if (startup || ((TextBox)sender).Enabled == false)
                return;
            float measuredvoltage = float.Parse(TXT_measuredvoltage.Text);
            float voltage = float.Parse(TXT_voltage.Text);
            float divider = float.Parse(TXT_divider.Text);
            if (voltage == 0)
                return;
            float new_divider = (measuredvoltage * divider) / voltage;
            TXT_divider.Text = new_divider.ToString();
            try
            {
                if (MainV2.comPort.param["VOLT_DIVIDER"] == null)
                {
                    MessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("VOLT_DIVIDER", float.Parse(TXT_divider.Text));
                }
            }
            catch { MessageBox.Show("Set VOLT_DIVIDER Failed"); }
        }
        private void TXT_divider_Validating(object sender, CancelEventArgs e)
        {
            float ans = 0;
            e.Cancel = !float.TryParse(TXT_divider.Text, out ans);
        }
        private void TXT_divider_Validated(object sender, EventArgs e)
        {
            if (startup || ((TextBox)sender).Enabled == false)
                return;
            try
            {
                if (MainV2.comPort.param["VOLT_DIVIDER"] == null)
                {
                    MessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("VOLT_DIVIDER", float.Parse(TXT_divider.Text));
                }
            }
            catch { MessageBox.Show("Set VOLT_DIVIDER Failed"); }
        }
        private void TXT_ampspervolt_Validating(object sender, CancelEventArgs e)
        {
            float ans = 0;
            e.Cancel = !float.TryParse(TXT_ampspervolt.Text, out ans);
        }
        private void TXT_ampspervolt_Validated(object sender, EventArgs e)
        {
            if (startup || ((TextBox)sender).Enabled == false)
                return;
            try
            {
                if (MainV2.comPort.param["AMP_PER_VOLT"] == null)
                {
                    MessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("AMP_PER_VOLT", float.Parse(TXT_ampspervolt.Text));
                }
            }
            catch { MessageBox.Show("Set AMP_PER_VOLT Failed"); }
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

            BUT_reset.Text = "Rebooting (17 sec)";
            BUT_reset.Refresh();
            Application.DoEvents();

            Sleep(17000, comPortT); // wait for boot/reset

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
                MainV2.comPort.setParam("HSV_MAN", 1); // randy request
                MainV2.comPort.setParam(((TextBox)sender).Name, test);
                System.Threading.Thread.Sleep(100);
                MainV2.comPort.setParam("HSV_MAN", 0); // randy request - last

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
                MainV2.comPort.setParam("HSV_MAN", 1); // randy request
                MainV2.comPort.setParam(((TextBox)sender).Name, test);
                System.Threading.Thread.Sleep(100);
                MainV2.comPort.setParam("HSV_MAN", 0); // randy request - last
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
                MainV2.comPort.setParam("HSV_MAN", 1); // randy request
                MainV2.comPort.setParam(((TextBox)sender).Name, test);
                System.Threading.Thread.Sleep(100);
                MainV2.comPort.setParam("HSV_MAN", 0); // randy request - last
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
            MainV2.comPort.setParam(((CheckBox)sender).Name, ((CheckBox)sender).Checked == false ? 1.0f : -1.0f);
        }

        private void HS2_REV_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((CheckBox)sender).Name, ((CheckBox)sender).Checked == false ? 1.0f : -1.0f);
        }

        private void HS3_REV_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((CheckBox)sender).Name, ((CheckBox)sender).Checked == false ? 1.0f : -1.0f);
        }

        private void HS4_REV_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((CheckBox)sender).Name, ((CheckBox)sender).Checked == false ? 1.0f : -1.0f);
            HS4.reverse = !HS4.reverse;
        }

        private void HS1_TRIM_ValueChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((NumericUpDown)sender).Name, (float)((NumericUpDown)sender).Value);
        }

        private void HS2_TRIM_ValueChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((NumericUpDown)sender).Name, (float)((NumericUpDown)sender).Value);
        }

        private void HS3_TRIM_ValueChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((NumericUpDown)sender).Name, (float)((NumericUpDown)sender).Value);
        }

        private void HS4_TRIM_ValueChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((NumericUpDown)sender).Name, (float)((NumericUpDown)sender).Value);
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

        private void BUT_levelac2_Click(object sender, EventArgs e)
        {
            try
            {
#if MAVLINK10
                int fixme; // needs to be accel only
                MainV2.comPort.doCommand(MAVLink.MAV_CMD.PREFLIGHT_CALIBRATION,1,1,1,1,1,1,1);
#else
                MainV2.comPort.doAction(MAVLink.MAV_ACTION.MAV_ACTION_CALIBRATE_ACC);
#endif

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
                    MessageBox.Show("Disabled Dip Switchs");
                }
                catch { MessageBox.Show("Error Disableing Dip Switch"); }
            }
            try
            {
                int i = normalreverse == false ? 1 : -1;
                MainV2.comPort.setParam(name, i);
            }
            catch { MessageBox.Show("Error Reversing"); }
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

        private void BUT_swash_manual_Click(object sender, EventArgs e)
        {
            try
            {
                if (MainV2.comPort.param["HSV_MAN"].ToString() == "1")
                {
                    MainV2.comPort.setParam("COL_MIN_", int.Parse(COL_MIN_.Text));
                    MainV2.comPort.setParam("COL_MAX_", int.Parse(COL_MAX_.Text));
                    MainV2.comPort.setParam("HSV_MAN", 0); // randy request - last
                    BUT_swash_manual.Text = "Manual";

                    COL_MAX_.Enabled = false;
                    COL_MID_.Enabled = false;
                    COL_MIN_.Enabled = false;
                    BUT_0collective.Enabled = false;
                }
                else
                {
                    COL_MAX_.Text = "1500";
                    COL_MIN_.Text = "1500";
                    MainV2.comPort.setParam("HSV_MAN", 1); // randy request
                    BUT_swash_manual.Text = "Save";

                    COL_MAX_.Enabled = true;
                    COL_MID_.Enabled = true;
                    COL_MIN_.Enabled = true;
                    BUT_0collective.Enabled = true;
                }
            }
            catch { MessageBox.Show("Failed to set HSV_MAN"); }
        }

        private void BUT_HS4save_Click(object sender, EventArgs e)
        {
            try
            {
                if (MainV2.comPort.param["HSV_MAN"].ToString() == "1")
                {
                    MainV2.comPort.setParam("HS4_MIN", int.Parse(HS4_MIN.Text));
                    MainV2.comPort.setParam("HS4_MAX", int.Parse(HS4_MAX.Text));
                    MainV2.comPort.setParam("HSV_MAN", 0); // randy request - last
                    BUT_HS4save.Text = "Manual";

                    HS4_MAX.Enabled = false;
                    HS4_MIN.Enabled = false;
                }
                else
                {
                    HS4_MIN.Text = "1500";
                    HS4_MAX.Text = "1500";
                    MainV2.comPort.setParam("HSV_MAN", 1); // randy request
                    BUT_HS4save.Text = "Save";


                    HS4_MAX.Enabled = true;
                    HS4_MIN.Enabled = true;
                }
            }
            catch { MessageBox.Show("Failed to set HSV_MAN"); }
        }

        private void tabHeli_Click(object sender, EventArgs e)
        {

        }

        private void HS4_Paint(object sender, PaintEventArgs e)
        {
            try
            {
                if (int.Parse(HS4_MIN.Text) > HS4.minline)
                    HS4_MIN.Text = HS4.minline.ToString();
                if (int.Parse(HS4_MAX.Text) < HS4.maxline)
                    HS4_MAX.Text = HS4.maxline.ToString();
            }
            catch { }
        }

        private void HS3_Paint(object sender, PaintEventArgs e)
        {
            try
            {
                if (int.Parse(COL_MIN_.Text) > HS3.minline)
                    COL_MIN_.Text = HS3.minline.ToString();
                if (int.Parse(COL_MAX_.Text) < HS3.maxline)
                    COL_MAX_.Text = HS3.maxline.ToString();
            }
            catch { }
        }

        private void COL_MAX__Enter(object sender, EventArgs e)
        {
            inpwmdetect = true;
        }

        private void COL_MIN__Enter(object sender, EventArgs e)
        {
            inpwmdetect = true;
        }

        private void COL_MAX__Leave(object sender, EventArgs e)
        {
            inpwmdetect = false;
        }

        private void COL_MIN__Leave(object sender, EventArgs e)
        {
            inpwmdetect = false;
        }

        private void HS4_MIN_Enter(object sender, EventArgs e)
        {
            inpwmdetect = true;
        }

        private void HS4_MIN_Leave(object sender, EventArgs e)
        {
            inpwmdetect = false;
        }

        private void HS4_MAX_Enter(object sender, EventArgs e)
        {
            inpwmdetect = true;
        }

        private void HS4_MAX_Leave(object sender, EventArgs e)
        {
            inpwmdetect = false;
        }

        private void PWM_Validating(object sender, CancelEventArgs e)
        {
            Control temp = (Control)(sender);

            if (int.Parse(temp.Text) < 900)
                temp.Text = "900";
            if (int.Parse(temp.Text) > 2100)
                temp.Text = "2100";
        }

        private void Setup_FormClosing(object sender, FormClosingEventArgs e)
        {
            timer.Stop();
            timer.Dispose();
        }

        private void CHK_enableoptflow_CheckedChanged(object sender, EventArgs e)
        {

            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["FLOW_ENABLE"] == null)
                {
                    MessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("FLOW_ENABLE", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { MessageBox.Show("Set FLOW_ENABLE Failed"); }
        }

        private void CMB_sonartype_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["SONAR_TYPE"] == null)
                {
                    MessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("SONAR_TYPE", ((ComboBox)sender).SelectedIndex);
                }
            }
            catch { MessageBox.Show("Set SONAR_TYPE Failed"); }
        }

        private void CHK_mixmode_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["ELEVON_MIXING"] == null)
                {
                    MessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("ELEVON_MIXING", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { MessageBox.Show("Set ELEVON_MIXING Failed"); }
        }

        private void CHK_elevonrev_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["ELEVON_REVERSE"] == null)
                {
                    MessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("ELEVON_REVERSE", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { MessageBox.Show("Set ELEVON_REVERSE Failed"); }
        }

        private void CHK_elevonch1rev_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["ELEVON_CH1_REV"] == null)
                {
                    MessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("ELEVON_CH1_REV", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { MessageBox.Show("Set ELEVON_CH1_REV Failed"); }
        }

        private void CHK_elevonch2rev_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["ELEVON_CH2_REV"] == null)
                {
                    MessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("ELEVON_CH2_REV", ((CheckBox)sender).Checked == true ? 1 : 0);
                }
            }
            catch { MessageBox.Show("Set ELEVON_CH2_REV Failed"); }
        }

        private void CMB_batmonsensortype_SelectedIndexChanged(object sender, EventArgs e)
        {
            int selection = int.Parse(CMB_batmonsensortype.Text.Substring(0,1));


            if (selection == 1) // atto 45
            {
                float maxvolt = 13.6f;
                float maxamps = 44.7f;
                float mvpervolt = 242.3f;
                float mvperamp = 73.20f;

                // ~ 3.295v
                float topvolt = (maxvolt * mvpervolt) / 1000;
                // ~ 3.294v
                float topamps = (maxamps * mvperamp) / 1000;

                TXT_divider.Text = (maxvolt / topvolt).ToString();
                TXT_ampspervolt.Text = (maxamps / topamps).ToString();
            }
            else if (selection == 2) // atto 90
            {
                float maxvolt = 50f;
                float maxamps = 89.4f;
                float mvpervolt = 63.69f;
                float mvperamp = 36.60f;

                float topvolt = (maxvolt * mvpervolt) / 1000;
                float topamps = (maxamps * mvperamp) / 1000;

                TXT_divider.Text = (maxvolt / topvolt).ToString();
                TXT_ampspervolt.Text = (maxamps / topamps).ToString();
            }
            else if (selection == 3) // atto 180
            {
                float maxvolt = 50f;
                float maxamps = 178.8f;
                float mvpervolt = 63.69f;
                float mvperamp = 18.30f;

                float topvolt = (maxvolt * mvpervolt) / 1000;
                float topamps = (maxamps * mvperamp) / 1000;

                TXT_divider.Text = (maxvolt / topvolt).ToString();
                TXT_ampspervolt.Text = (maxamps / topamps).ToString();
            }

            // enable to update
            TXT_divider.Enabled = true;
            TXT_ampspervolt.Enabled = true;
            TXT_measuredvoltage.Enabled = true;
            TXT_inputvoltage.Enabled = true;

            // update
            TXT_ampspervolt_Validated(TXT_ampspervolt, null);

            TXT_divider_Validated(TXT_divider, null);

            // disable
            TXT_divider.Enabled = false;
            TXT_ampspervolt.Enabled = false;
            TXT_measuredvoltage.Enabled = false;

            //reenable if needed
            if (selection == 0)
            {
                TXT_divider.Enabled = true;
                TXT_ampspervolt.Enabled = true;
                TXT_measuredvoltage.Enabled = true;
                TXT_inputvoltage.Enabled = true;
            }
        }
    }
}
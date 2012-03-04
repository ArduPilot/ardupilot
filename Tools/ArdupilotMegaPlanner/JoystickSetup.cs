using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using Microsoft.DirectX.DirectInput;



namespace ArdupilotMega
{
    public partial class JoystickSetup : Form
    {
        bool startup = true;

        int noButtons = 0;

        public JoystickSetup()
        {
            InitializeComponent();
        }

        private void Joystick_Load(object sender, EventArgs e)
        {
            try
            {
                DeviceList joysticklist = Joystick.getDevices();

                foreach (DeviceInstance device in joysticklist)
                {
                    CMB_joysticks.Items.Add(device.ProductName);
                }
            }
            catch { MessageBox.Show("Error geting joystick list: do you have the directx redist installed?"); this.Close(); return; }

            if (CMB_joysticks.Items.Count > 0)
                CMB_joysticks.SelectedIndex = 0;

            CMB_CH1.DataSource = (Enum.GetValues(typeof(Joystick.joystickaxis)));
            CMB_CH2.DataSource = (Enum.GetValues(typeof(Joystick.joystickaxis)));
            CMB_CH3.DataSource = (Enum.GetValues(typeof(Joystick.joystickaxis)));
            CMB_CH4.DataSource = (Enum.GetValues(typeof(Joystick.joystickaxis)));
            CMB_CH5.DataSource = (Enum.GetValues(typeof(Joystick.joystickaxis)));
            CMB_CH6.DataSource = (Enum.GetValues(typeof(Joystick.joystickaxis)));
            CMB_CH7.DataSource = (Enum.GetValues(typeof(Joystick.joystickaxis)));
            CMB_CH8.DataSource = (Enum.GetValues(typeof(Joystick.joystickaxis)));

            try
            {
                //CMB_CH1
                CMB_CH1.Text = MainV2.config["CMB_CH1"].ToString();
                CMB_CH2.Text = MainV2.config["CMB_CH2"].ToString();
                CMB_CH3.Text = MainV2.config["CMB_CH3"].ToString();
                CMB_CH4.Text = MainV2.config["CMB_CH4"].ToString();
                CMB_CH5.Text = MainV2.config["CMB_CH5"].ToString();
                CMB_CH6.Text = MainV2.config["CMB_CH6"].ToString();
                CMB_CH7.Text = MainV2.config["CMB_CH7"].ToString();
                CMB_CH8.Text = MainV2.config["CMB_CH8"].ToString();

                //revCH1
                revCH1.Checked = bool.Parse(MainV2.config["revCH1"].ToString());
                revCH2.Checked = bool.Parse(MainV2.config["revCH2"].ToString());
                revCH3.Checked = bool.Parse(MainV2.config["revCH3"].ToString());
                revCH4.Checked = bool.Parse(MainV2.config["revCH4"].ToString());
                revCH5.Checked = bool.Parse(MainV2.config["revCH5"].ToString());
                revCH6.Checked = bool.Parse(MainV2.config["revCH6"].ToString());
                revCH7.Checked = bool.Parse(MainV2.config["revCH7"].ToString());
                revCH8.Checked = bool.Parse(MainV2.config["revCH8"].ToString());

                //expo_ch1
                expo_ch1.Text = MainV2.config["expo_ch1"].ToString();
                expo_ch2.Text = MainV2.config["expo_ch2"].ToString();
                expo_ch3.Text = MainV2.config["expo_ch3"].ToString();
                expo_ch4.Text = MainV2.config["expo_ch4"].ToString();
                expo_ch5.Text = MainV2.config["expo_ch5"].ToString();
                expo_ch6.Text = MainV2.config["expo_ch6"].ToString();
                expo_ch7.Text = MainV2.config["expo_ch7"].ToString();
                expo_ch8.Text = MainV2.config["expo_ch8"].ToString();

                CHK_elevons.Checked = bool.Parse(MainV2.config["joy_elevons"].ToString());
            }
            catch { } // IF 1 DOESNT EXIST NONE WILL

            if (MainV2.joystick != null && MainV2.joystick.enabled)
            {
                timer1.Start();
                BUT_enable.Text = "Disable";
            }

            startup = false;
        }

        int[] getButtonNumbers()
        {
            int[] temp = new int[128];
            temp[0] = -1;
            for (int a = 0; a < temp.Length - 1; a++)
            {
                temp[a + 1] = a;
            }
            return temp;
        }

        private void BUT_enable_Click(object sender, EventArgs e)
        {
            if (MainV2.joystick == null || MainV2.joystick.enabled == false)
            {
                try
                {
                    if (MainV2.joystick != null)
                        MainV2.joystick.UnAcquireJoyStick();
                }
                catch { }

                Joystick joy = new Joystick();
                joy.setChannel(1, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH1.Text), revCH1.Checked, int.Parse(expo_ch1.Text));
                joy.setChannel(2, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH2.Text), revCH2.Checked, int.Parse(expo_ch2.Text));
                joy.setChannel(3, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH3.Text), revCH3.Checked, int.Parse(expo_ch3.Text));
                joy.setChannel(4, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH4.Text), revCH4.Checked, int.Parse(expo_ch4.Text));
                joy.setChannel(5, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH5.Text), revCH5.Checked, int.Parse(expo_ch5.Text));
                joy.setChannel(6, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH6.Text), revCH6.Checked, int.Parse(expo_ch6.Text));
                joy.setChannel(7, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH7.Text), revCH7.Checked, int.Parse(expo_ch7.Text));
                joy.setChannel(8, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH8.Text), revCH8.Checked, int.Parse(expo_ch8.Text));

                joy.elevons = CHK_elevons.Checked;

                for (int f = 0; f < noButtons; f++)
                {
                    string name = (f + 1).ToString();
                    try
                    {
                        joy.setButton(f, int.Parse(this.Controls.Find("cmbbutton" + name, false)[0].Text), this.Controls.Find("cmbaction" + name, false)[0].Text);
                    }
                    catch { MessageBox.Show("Set Button "+ name + " Failed"); }
                }

                joy.start(CMB_joysticks.Text);

                MainV2.joystick = joy;
                MainV2.joystick.enabled = true;

                BUT_enable.Text = "Disable";

                //timer1.Start();
            }
            else
            {
                MAVLink.__mavlink_rc_channels_override_t rc = new MAVLink.__mavlink_rc_channels_override_t();

                rc.target_component = MainV2.comPort.compid;
                rc.target_system = MainV2.comPort.sysid;

                rc.chan1_raw = 0;
                rc.chan2_raw = 0;
                rc.chan3_raw = 0;
                rc.chan4_raw = 0;
                rc.chan5_raw = 0;
                rc.chan6_raw = 0;
                rc.chan7_raw = 0;
                rc.chan8_raw = 0;

                MainV2.comPort.sendPacket(rc);
                System.Threading.Thread.Sleep(20);
                MainV2.comPort.sendPacket(rc);
                System.Threading.Thread.Sleep(20);
                MainV2.comPort.sendPacket(rc);

                //timer1.Stop();
                MainV2.joystick.enabled = false;
                MainV2.joystick = null;
                BUT_enable.Text = "Enable";
            }
        }

        private void BUT_save_Click(object sender, EventArgs e)
        {
            //CMB_CH1
            MainV2.config["CMB_CH1"] = CMB_CH1.Text;
            MainV2.config["CMB_CH2"] = CMB_CH2.Text;
            MainV2.config["CMB_CH3"] = CMB_CH3.Text;
            MainV2.config["CMB_CH4"] = CMB_CH4.Text;
            MainV2.config["CMB_CH5"] = CMB_CH5.Text;
            MainV2.config["CMB_CH6"] = CMB_CH6.Text;
            MainV2.config["CMB_CH7"] = CMB_CH7.Text;
            MainV2.config["CMB_CH8"] = CMB_CH8.Text;

            //revCH1
            MainV2.config["revCH1"] = revCH1.Checked;
            MainV2.config["revCH2"] = revCH2.Checked;
            MainV2.config["revCH3"] = revCH3.Checked;
            MainV2.config["revCH4"] = revCH4.Checked;
            MainV2.config["revCH5"] = revCH5.Checked;
            MainV2.config["revCH6"] = revCH6.Checked;
            MainV2.config["revCH7"] = revCH7.Checked;
            MainV2.config["revCH8"] = revCH8.Checked;

            //expo_ch1
            MainV2.config["expo_ch1"] = expo_ch1.Text;
            MainV2.config["expo_ch2"] = expo_ch2.Text;
            MainV2.config["expo_ch3"] = expo_ch3.Text;
            MainV2.config["expo_ch4"] = expo_ch4.Text;
            MainV2.config["expo_ch5"] = expo_ch5.Text;
            MainV2.config["expo_ch6"] = expo_ch6.Text;
            MainV2.config["expo_ch7"] = expo_ch7.Text;
            MainV2.config["expo_ch8"] = expo_ch8.Text;

            MainV2.config["joy_elevons"] = CHK_elevons.Checked;

            for (int f = 0; f < noButtons; f++)
            {
                string name = (f + 1).ToString();
                MainV2.config["butno" + name] = this.Controls.Find("cmbbutton" + name, false)[0].Text;
                MainV2.config["butaction" + name] = this.Controls.Find("cmbaction" + name, false)[0].Text;
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            try
            {
                if (MainV2.joystick == null || MainV2.joystick.enabled == false)
                {
                    //Console.WriteLine(DateTime.Now.Millisecond + " start ");
                    Joystick joy = MainV2.joystick;
                    if (joy == null)
                    {
                        joy = new Joystick();
                        joy.setChannel(1, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH1.Text), revCH1.Checked, int.Parse(expo_ch1.Text));
                        joy.setChannel(2, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH2.Text), revCH2.Checked, int.Parse(expo_ch2.Text));
                        joy.setChannel(3, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH3.Text), revCH3.Checked, int.Parse(expo_ch3.Text));
                        joy.setChannel(4, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH4.Text), revCH4.Checked, int.Parse(expo_ch4.Text));
                        joy.setChannel(5, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH5.Text), revCH5.Checked, int.Parse(expo_ch5.Text));
                        joy.setChannel(6, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH6.Text), revCH6.Checked, int.Parse(expo_ch6.Text));
                        joy.setChannel(7, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH7.Text), revCH7.Checked, int.Parse(expo_ch7.Text));
                        joy.setChannel(8, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), CMB_CH8.Text), revCH8.Checked, int.Parse(expo_ch8.Text));

                        joy.elevons = CHK_elevons.Checked;

                        joy.AcquireJoystick(CMB_joysticks.Text);

                        joy.name = CMB_joysticks.Text;

                        noButtons = joy.getNumButtons();

                        for (int f = 0; f < noButtons; f++)
                        {
                            string name = (f + 1).ToString();

                            doButtontoUI(name, 10, 290 + f * 25);

                            joy.setButton(f, int.Parse(this.Controls.Find("cmbbutton" + name, false)[0].Text), this.Controls.Find("cmbaction" + name, false)[0].Text);
                        }

                        MainV2.joystick = joy;

                        ThemeManager.ApplyThemeTo(this);

                        CMB_joysticks.SelectedIndex = CMB_joysticks.Items.IndexOf(joy.name);
                    }

                    MainV2.joystick.elevons = CHK_elevons.Checked;

                    MainV2.cs.rcoverridech1 = joy.getValueForChannel(1, CMB_joysticks.Text);
                    MainV2.cs.rcoverridech2 = joy.getValueForChannel(2, CMB_joysticks.Text);
                    MainV2.cs.rcoverridech3 = joy.getValueForChannel(3, CMB_joysticks.Text);
                    MainV2.cs.rcoverridech4 = joy.getValueForChannel(4, CMB_joysticks.Text);
                    MainV2.cs.rcoverridech5 = joy.getValueForChannel(5, CMB_joysticks.Text);
                    MainV2.cs.rcoverridech6 = joy.getValueForChannel(6, CMB_joysticks.Text);
                    MainV2.cs.rcoverridech7 = joy.getValueForChannel(7, CMB_joysticks.Text);
                    MainV2.cs.rcoverridech8 = joy.getValueForChannel(8, CMB_joysticks.Text);

                    //Console.WriteLine(DateTime.Now.Millisecond + " end ");
                }
            }
            catch { }

            progressBar1.Value = MainV2.cs.rcoverridech1;
            progressBar2.Value = MainV2.cs.rcoverridech2;
            progressBar3.Value = MainV2.cs.rcoverridech3;
            progressBar4.Value = MainV2.cs.rcoverridech4;
            horizontalProgressBar1.Value = MainV2.cs.rcoverridech5;
            horizontalProgressBar2.Value = MainV2.cs.rcoverridech6;
            horizontalProgressBar3.Value = MainV2.cs.rcoverridech7;
            horizontalProgressBar4.Value = MainV2.cs.rcoverridech8;

            try
            {
                for (int f = 0; f < noButtons; f++)
                {
                    string name = (f + 1).ToString();

                        ((HorizontalProgressBar)this.Controls.Find("hbar" + name, false)[0]).Value = MainV2.joystick.isButtonPressed(f) ? 100 : 0;
                }
            }
            catch {  } // this is for buttons - silent fail

        }

        private void CMB_joysticks_Click(object sender, EventArgs e)
        {
            CMB_joysticks.Items.Clear();

            DeviceList joysticklist = Joystick.getDevices();

            foreach (DeviceInstance device in joysticklist)
            {
                CMB_joysticks.Items.Add(device.ProductName);
            }

            if (CMB_joysticks.Items.Count > 0)
                CMB_joysticks.SelectedIndex = 0;
        }

        private void revCH1_CheckedChanged(object sender, EventArgs e)
        {
            if (MainV2.joystick != null)
                MainV2.joystick.setReverse(1, ((CheckBox)sender).Checked);
        }

        private void revCH2_CheckedChanged(object sender, EventArgs e)
        {
            if (MainV2.joystick != null)
                MainV2.joystick.setReverse(2, ((CheckBox)sender).Checked);
        }

        private void revCH3_CheckedChanged(object sender, EventArgs e)
        {
            if (MainV2.joystick != null)
                MainV2.joystick.setReverse(3, ((CheckBox)sender).Checked);
        }

        private void revCH4_CheckedChanged(object sender, EventArgs e)
        {
            if (MainV2.joystick != null)
                MainV2.joystick.setReverse(4, ((CheckBox)sender).Checked);
        }

        private void BUT_detch1_Click(object sender, EventArgs e)
        {
            CMB_CH1.Text = Joystick.getMovingAxis(CMB_joysticks.Text, 16000).ToString();
        }

        private void BUT_detch2_Click(object sender, EventArgs e)
        {
            CMB_CH2.Text = Joystick.getMovingAxis(CMB_joysticks.Text, 16000).ToString();
        }

        private void BUT_detch3_Click(object sender, EventArgs e)
        {
            CMB_CH3.Text = Joystick.getMovingAxis(CMB_joysticks.Text, 16000).ToString();
        }

        private void BUT_detch4_Click(object sender, EventArgs e)
        {
            CMB_CH4.Text = Joystick.getMovingAxis(CMB_joysticks.Text, 16000).ToString();
        }

        private void CMB_CH1_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup || MainV2.joystick == null)
                return;
            MainV2.joystick.setAxis(1, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), ((ComboBox)sender).Text));
        }

        private void CMB_CH2_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup || MainV2.joystick == null)
                return;
            MainV2.joystick.setAxis(2, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), ((ComboBox)sender).Text));
        }

        private void CMB_CH3_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup || MainV2.joystick == null)
                return;
            MainV2.joystick.setAxis(3, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), ((ComboBox)sender).Text));
        }

        private void CMB_CH4_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup || MainV2.joystick == null)
                return;
            MainV2.joystick.setAxis(4, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), ((ComboBox)sender).Text));
        }

        private void cmbbutton_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup)
                return;

            string name = ((ComboBox)sender).Name.Replace("cmbbutton", "");

            MainV2.joystick.changeButton((int.Parse(name) - 1), int.Parse(((ComboBox)sender).Text));
        }

        private void BUT_detbutton_Click(object sender, EventArgs e)
        {
            string name = ((MyButton)sender).Name.Replace("mybut", "");

            ComboBox cmb = (ComboBox)(this.Controls.Find("cmbbutton" + name, false)[0]);
            cmb.Text = Joystick.getPressedButton(CMB_joysticks.Text).ToString();
        }

        void doButtontoUI(string name, int x, int y)
        {
            MyLabel lbl = new MyLabel();
            ComboBox cmbbutton = new ComboBox();
            MyButton mybut = new MyButton();
            HorizontalProgressBar hbar = new HorizontalProgressBar();
            ComboBox cmbaction = new ComboBox();

            // do this here so putting in text works
            this.Controls.AddRange(new Control[] { lbl, cmbbutton, mybut, hbar, cmbaction });

            lbl.Location = new Point(x, y);
            lbl.Size = new Size(47, 13);
            lbl.Text = "Button " + name;

            cmbbutton.Location = new Point(72, y);
            cmbbutton.Size = new Size(70, 21);
            cmbbutton.DataSource = getButtonNumbers();
            cmbbutton.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbbutton.Name = "cmbbutton" + name;
            if (MainV2.config["butno" + name] != null)
                cmbbutton.Text = (MainV2.config["butno" + name].ToString());
            cmbbutton.SelectedIndexChanged += new EventHandler(cmbbutton_SelectedIndexChanged);

            mybut.Location = new Point(BUT_detch1.Left, y);
            mybut.Size = BUT_detch1.Size;
            mybut.Text = BUT_detch1.Text;
            mybut.Name = "mybut" + name;
            mybut.Click += new EventHandler(BUT_detbutton_Click);

            hbar.Location = new Point(progressBar1.Left, y);
            hbar.Size = progressBar1.Size;
            hbar.Name = "hbar" + name;

            cmbaction.Location = new Point(hbar.Right + 5, y);
            cmbaction.Size = new Size(100, 21);
            cmbaction.DataSource = (Enum.GetValues(Common.getModes()));
            cmbaction.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbaction.Name = "cmbaction" + name;
            if (MainV2.config["butaction" + name] != null)
                cmbaction.Text = MainV2.config["butaction" + name].ToString();

            this.Height += 25;
        }

        private void CMB_joysticks_SelectedIndexChanged(object sender, EventArgs e)
        {
            try
            {
                if (MainV2.joystick != null && MainV2.joystick.enabled == false)
                    MainV2.joystick.UnAcquireJoyStick();
            }
            catch { }
        }

        private void JoystickSetup_FormClosed(object sender, FormClosedEventArgs e)
        {
            if (MainV2.joystick != null && MainV2.joystick.enabled == false)
            {
                MainV2.joystick.UnAcquireJoyStick();
                MainV2.joystick = null;
            }
        }

        private void CHK_elevons_CheckedChanged(object sender, EventArgs e)
        {
            if (MainV2.joystick == null)
            {
                return;
            }
            MainV2.joystick.elevons = CHK_elevons.Checked;
        }

        private void CMB_CH5_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup || MainV2.joystick == null)
                return;
            MainV2.joystick.setAxis(5, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), ((ComboBox)sender).Text));
        }

        private void CMB_CH6_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup || MainV2.joystick == null)
                return;
            MainV2.joystick.setAxis(6, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), ((ComboBox)sender).Text));
        }

        private void CMB_CH7_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup || MainV2.joystick == null)
                return;
            MainV2.joystick.setAxis(7, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), ((ComboBox)sender).Text));
        }

        private void CMB_CH8_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup || MainV2.joystick == null)
                return;
            MainV2.joystick.setAxis(8, (Joystick.joystickaxis)Enum.Parse(typeof(Joystick.joystickaxis), ((ComboBox)sender).Text));
        }

        private void BUT_detch5_Click(object sender, EventArgs e)
        {
            CMB_CH5.Text = Joystick.getMovingAxis(CMB_joysticks.Text, 16000).ToString();
        }

        private void BUT_detch6_Click(object sender, EventArgs e)
        {
            CMB_CH6.Text = Joystick.getMovingAxis(CMB_joysticks.Text, 16000).ToString();
        }

        private void BUT_detch7_Click(object sender, EventArgs e)
        {
            CMB_CH7.Text = Joystick.getMovingAxis(CMB_joysticks.Text, 16000).ToString();
        }

        private void BUT_detch8_Click(object sender, EventArgs e)
        {
            CMB_CH8.Text = Joystick.getMovingAxis(CMB_joysticks.Text, 16000).ToString();
        }

        private void revCH5_CheckedChanged(object sender, EventArgs e)
        {
            if (MainV2.joystick != null)
                MainV2.joystick.setReverse(5, ((CheckBox)sender).Checked);
        }

        private void revCH6_CheckedChanged(object sender, EventArgs e)
        {
            if (MainV2.joystick != null)
                MainV2.joystick.setReverse(6, ((CheckBox)sender).Checked);
        }

        private void revCH7_CheckedChanged(object sender, EventArgs e)
        {
            if (MainV2.joystick != null)
                MainV2.joystick.setReverse(7, ((CheckBox)sender).Checked);
        }

        private void revCH8_CheckedChanged(object sender, EventArgs e)
        {
            if (MainV2.joystick != null)
                MainV2.joystick.setReverse(8, ((CheckBox)sender).Checked);
        }
    }
}

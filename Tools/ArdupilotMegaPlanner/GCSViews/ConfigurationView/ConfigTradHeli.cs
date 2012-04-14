using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigTradHeli : BackStageViewContentPanel
    {
        bool startup = false;
        bool inpwmdetect = false;

        Timer timer = new Timer();

        public ConfigTradHeli()
        {
            InitializeComponent();
        }

        private void H1_ENABLE_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["H1_ENABLE"] == null)
                {
                    CustomMessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("H1_ENABLE", ((RadioButton)sender).Checked == true ? 1 : 0);
                }
            }
            catch { CustomMessageBox.Show("Set H1_ENABLE Failed"); }
        }

        private void BUT_swash_manual_Click(object sender, EventArgs e)
        {
            try
            {
                if (MainV2.comPort.param["HSV_MAN"].ToString() == "1")
                {
                    MainV2.comPort.setParam("COL_MIN", int.Parse(COL_MIN.Text));
                    MainV2.comPort.setParam("COL_MAX", int.Parse(COL_MAX.Text));
                    MainV2.comPort.setParam("HSV_MAN", 0); // randy request - last
                    BUT_swash_manual.Text = "Manual";

                    COL_MAX.Enabled = false;
                    COL_MID.Enabled = false;
                    COL_MIN.Enabled = false;
                    BUT_0collective.Enabled = false;
                }
                else
                {
                    COL_MAX.Text = "1500";
                    COL_MIN.Text = "1500";
                    MainV2.comPort.setParam("HSV_MAN", 1); // randy request
                    BUT_swash_manual.Text = "Save";

                    COL_MAX.Enabled = true;
                    COL_MID.Enabled = true;
                    COL_MIN.Enabled = true;
                    BUT_0collective.Enabled = true;
                }
            }
            catch { CustomMessageBox.Show("Failed to set HSV_MAN"); }
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
            catch { CustomMessageBox.Show("Failed to set HSV_MAN"); }
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
                if (int.Parse(COL_MIN.Text) > HS3.minline)
                    COL_MIN.Text = HS3.minline.ToString();
                if (int.Parse(COL_MAX.Text) < HS3.maxline)
                    COL_MAX.Text = HS3.maxline.ToString();
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
            catch { CustomMessageBox.Show("Set " + ((TextBox)sender).Name + " failed"); }
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
            catch { CustomMessageBox.Show("Set " + ((TextBox)sender).Name + " failed"); }
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
            catch { CustomMessageBox.Show("Set " + ((TextBox)sender).Name + " failed"); }
        }

        private void BUT_0collective_Click(object sender, EventArgs e)
        {
            CustomMessageBox.Show("Make sure your blades are at 0 degrees");

            try
            {

                MainV2.comPort.setParam("COL_MID", MainV2.cs.ch3in);

                COL_MID.Text = MainV2.comPort.param["COL_MID"].ToString();
            }
            catch { CustomMessageBox.Show("Set COL_MID_ failed"); }
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
            if (startup || this.Disposing || !this.Enabled)
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
            if (startup || this.Disposing || !this.Enabled)
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
            if (startup || this.Disposing || !this.Enabled)
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
            catch { CustomMessageBox.Show("Failed to set Gyro Gain"); }
        }

        private void GYR_ENABLE__CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.comPort.setParam(((CheckBox)sender).Name, ((CheckBox)sender).Checked == true ? 1.0f : 0.0f);
        }

        private void ConfigTradHeli_Load(object sender, EventArgs e)
        {
            if (!MainV2.comPort.BaseStream.IsOpen)
            {
                this.Enabled = false;
                return;
            }
            else
            {
                this.Enabled = true;
            }

            if (MainV2.comPort.param["GYR_ENABLE"] == null)
            {
                this.Enabled = false;
                return;
            }

            timer.Tick += new EventHandler(timer_Tick);

            timer.Enabled = true;
            timer.Interval = 100;
            timer.Start();

            startup = true;
            try
            {
                if (MainV2.comPort.param.ContainsKey("H1_ENABLE"))
                {
                    CCPM.Checked = MainV2.comPort.param["H1_ENABLE"].ToString() == "0" ? true : false;
                    H1_ENABLE.Checked = !CCPM.Checked;
                }

                foreach (string value in MainV2.comPort.param.Keys)
                {
                    if (value == "")
                        continue;

                    Control[] control = this.Controls.Find(value, true);
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

        void timer_Tick(object sender, EventArgs e)
        {
            try
            {
                MainV2.cs.UpdateCurrentSettings(currentStateBindingSource);
            }
            catch { }

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
                    HS3.minline = int.Parse(COL_MIN.Text);
                    HS3.maxline = int.Parse(COL_MAX.Text);
                    HS4.maxline = int.Parse(HS4_MIN.Text);
                    HS4.minline = int.Parse(HS4_MAX.Text);
                }
                catch { }
            }
        }

        private void ConfigTradHeli_FormClosing(object sender, FormClosingEventArgs e)
        {
            startup = true;
        }
    }
}

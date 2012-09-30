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
    public partial class ConfigTradHeli : UserControl, IActivate, IDeactivate
    {
        bool startup = false;
        bool inpwmdetect = false;

        Timer timer = new Timer();

        public ConfigTradHeli()
        {
            InitializeComponent();
        }

        private void H_SWASH_TYPE_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.param["H_SWASH_TYPE"] == null)
                {
                    CustomMessageBox.Show("Not Available on " + MainV2.cs.firmware.ToString());
                }
                else
                {
                    MainV2.comPort.setParam("H_SWASH_TYPE", ((RadioButton)sender).Checked == true ? 1 : 0);
                }
            }
            catch { CustomMessageBox.Show("Set H_SWASH_TYPE Failed"); }
        }

        private void BUT_swash_manual_Click(object sender, EventArgs e)
        {
            try
            {
                if (MainV2.comPort.param["H_SV_MAN"].ToString() == "1")
                {
                    MainV2.comPort.setParam("H_COL_MIN", int.Parse(H_COL_MIN.Text));
                    MainV2.comPort.setParam("H_COL_MAX", int.Parse(H_COL_MAX.Text));
                    MainV2.comPort.setParam("H_SV_MAN", 0); // randy request - last
                    BUT_swash_manual.Text = "Manual";

                    H_COL_MAX.Enabled = false;
                    H_COL_MID.Enabled = false;
                    H_COL_MIN.Enabled = false;
                    BUT_0collective.Enabled = false;
                }
                else
                {
                    H_COL_MAX.Text = "1500";
                    H_COL_MIN.Text = "1500";
                    MainV2.comPort.setParam("H_SV_MAN", 1); // randy request
                    BUT_swash_manual.Text = "Save";

                    H_COL_MAX.Enabled = true;
                    H_COL_MID.Enabled = true;
                    H_COL_MIN.Enabled = true;
                    BUT_0collective.Enabled = true;
                }
            }
            catch { CustomMessageBox.Show("Failed to set H_SV_MAN"); }
        }

        private void BUT_HS4save_Click(object sender, EventArgs e)
        {
            try
            {
                if (MainV2.comPort.param["H_SV_MAN"].ToString() == "1")
                {
                    MainV2.comPort.setParam("HS4_MIN", int.Parse(HS4_MIN.Text));
                    MainV2.comPort.setParam("HS4_MAX", int.Parse(HS4_MAX.Text));
                    MainV2.comPort.setParam("H_SV_MAN", 0); // randy request - last
                    BUT_HS4save.Text = "Manual";

                    HS4_MAX.Enabled = false;
                    HS4_MIN.Enabled = false;
                }
                else
                {
                    HS4_MIN.Text = "1500";
                    HS4_MAX.Text = "1500";
                    MainV2.comPort.setParam("H_SV_MAN", 1); // randy request
                    BUT_HS4save.Text = "Save";


                    HS4_MAX.Enabled = true;
                    HS4_MIN.Enabled = true;
                }
            }
            catch { CustomMessageBox.Show("Failed to set H_SV_MAN"); }
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
                if (int.Parse(H_COL_MIN.Text) > HS3.minline)
                    H_COL_MIN.Text = HS3.minline.ToString();
                if (int.Parse(H_COL_MAX.Text) < HS3.maxline)
                    H_COL_MAX.Text = HS3.maxline.ToString();
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
                MainV2.comPort.setParam("H_SV_MAN", 1); // randy request
                MainV2.comPort.setParam(((TextBox)sender).Name, test);
                System.Threading.Thread.Sleep(100);
                MainV2.comPort.setParam("H_SV_MAN", 0); // randy request - last

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
                MainV2.comPort.setParam("H_SV_MAN", 1); // randy request
                MainV2.comPort.setParam(((TextBox)sender).Name, test);
                System.Threading.Thread.Sleep(100);
                MainV2.comPort.setParam("H_SV_MAN", 0); // randy request - last
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
                MainV2.comPort.setParam("H_SV_MAN", 1); // randy request
                MainV2.comPort.setParam(((TextBox)sender).Name, test);
                System.Threading.Thread.Sleep(100);
                MainV2.comPort.setParam("H_SV_MAN", 0); // randy request - last
            }
            catch { CustomMessageBox.Show("Set " + ((TextBox)sender).Name + " failed"); }
        }

        private void BUT_0collective_Click(object sender, EventArgs e)
        {
            CustomMessageBox.Show("Make sure your blades are at 0 degrees");

            try
            {

                MainV2.comPort.setParam("H_COL_MID", MainV2.cs.ch3in);

                H_COL_MID.Text = MainV2.comPort.param["H_COL_MID"].ToString();
            }
            catch { CustomMessageBox.Show("Set H_COL_MID failed"); }
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

        public void Activate()
        {

            if (MainV2.comPort.param["H_GYR_ENABLE"] == null)
            {
                this.Enabled = false;
                return;
            }

            timer.Tick += new EventHandler(timer_Tick);

            timer.Enabled = true;
            timer.Interval = 100;
            timer.Start();

            mavlinkNumericUpDown1min.setup(800,1400,1,1,"HS1_MIN",MainV2.comPort.param);
            mavlinkNumericUpDown1max.setup(1600,2200,1,1,"HS1_MAX",MainV2.comPort.param);
            mavlinkNumericUpDown2min.setup(800, 1400, 1, 1, "HS2_MIN", MainV2.comPort.param);
            mavlinkNumericUpDown2max.setup(1600, 2200, 1, 1, "HS2_MAX", MainV2.comPort.param);
            mavlinkNumericUpDown3min.setup(800, 1400, 1, 1, "HS3_MIN", MainV2.comPort.param);
            mavlinkNumericUpDown3max.setup(1600, 2200, 1, 1, "HS3_MAX", MainV2.comPort.param);

            mavlinkNumericUpDownpitchmax.setup(10, 65, 100, 1, "H_PIT_MAX", MainV2.comPort.param);
            mavlinkNumericUpDownrollmax.setup(10, 65, 100, 1, "H_ROL_MAX", MainV2.comPort.param);

            startup = true;
            try
            {
                if (MainV2.comPort.param.ContainsKey("H_SWASH_TYPE"))
                {
                    CCPM.Checked = MainV2.comPort.param["H_SWASH_TYPE"].ToString() == "0" ? true : false;
                    H_SWASH_TYPE.Checked = !CCPM.Checked;
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
                            ArdupilotMega.Controls.MyTrackBar temp = (MyTrackBar)control[0];
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

            if (MainV2.comPort.param["H_SV_MAN"] == null || MainV2.comPort.param["H_SV_MAN"].ToString() == "0")
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
                    HS3.minline = int.Parse(H_COL_MIN.Text);
                    HS3.maxline = int.Parse(H_COL_MAX.Text);
                    HS4.maxline = int.Parse(HS4_MIN.Text);
                    HS4.minline = int.Parse(HS4_MAX.Text);
                }
                catch { }
            }
        }

        public void Deactivate()
        {
            timer.Stop();

            startup = true;
        }
    }
}

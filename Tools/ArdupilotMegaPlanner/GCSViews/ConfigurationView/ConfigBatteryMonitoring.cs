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
    public partial class ConfigBatteryMonitoring : UserControl, IActivate, IDeactivate
    {
        bool startup = false;

        public ConfigBatteryMonitoring()
        {
            InitializeComponent();
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
            catch { CustomMessageBox.Show("Set BATT_MONITOR Failed"); }
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
                if (MainV2.comPort.MAV.param["BATT_CAPACITY"] == null)
                {
                    CustomMessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("BATT_CAPACITY", float.Parse(TXT_battcapacity.Text));
                }
            }
            catch { CustomMessageBox.Show("Set BATT_CAPACITY Failed"); }
        }
        private void CMB_batmontype_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                if (MainV2.comPort.MAV.param["BATT_MONITOR"] == null)
                {
                    CustomMessageBox.Show("Not Available");
                }
                else
                {
                    int selection = int.Parse(CMB_batmontype.Text.Substring(0, 1));

                    CMB_batmonsensortype.Enabled = true;

                    TXT_voltage.Enabled = false;

                    if (selection == 0)
                    {
                        CMB_batmonsensortype.Enabled = false;
                        groupBox4.Enabled = false;
                        MainV2.comPort.setParam("BATT_VOLT_PIN", -1);
                        MainV2.comPort.setParam("BATT_CURR_PIN", -1);
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
            catch { CustomMessageBox.Show("Set BATT_MONITOR,BATT_VOLT_PIN,BATT_CURR_PIN Failed"); }
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
                if (MainV2.comPort.MAV.param["INPUT_VOLTS"] == null)
                {
                    CustomMessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("INPUT_VOLTS", float.Parse(TXT_inputvoltage.Text));
                }
            }
            catch { CustomMessageBox.Show("Set INPUT_VOLTS Failed"); }
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
            try
            {
                float measuredvoltage = float.Parse(TXT_measuredvoltage.Text);
                float voltage = float.Parse(TXT_voltage.Text);
                float divider = float.Parse(TXT_divider.Text);
                if (voltage == 0)
                    return;
                float new_divider = (measuredvoltage * divider) / voltage;
                TXT_divider.Text = new_divider.ToString();
            }
            catch { CustomMessageBox.Show("Invalid number entered"); return; }

            try
            {
                if (MainV2.comPort.MAV.param["VOLT_DIVIDER"] == null)
                {
                    CustomMessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("VOLT_DIVIDER", float.Parse(TXT_divider.Text));
                }
            }
            catch { CustomMessageBox.Show("Set VOLT_DIVIDER Failed"); }
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
                if (MainV2.comPort.MAV.param["VOLT_DIVIDER"] == null)
                {
                    CustomMessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("VOLT_DIVIDER", float.Parse(TXT_divider.Text));
                }
            }
            catch { CustomMessageBox.Show("Set VOLT_DIVIDER Failed"); }
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
                if (MainV2.comPort.MAV.param["AMP_PER_VOLT"] == null)
                {
                    CustomMessageBox.Show("Not Available");
                }
                else
                {
                    MainV2.comPort.setParam("AMP_PER_VOLT", float.Parse(TXT_ampspervolt.Text));
                }
            }
            catch { CustomMessageBox.Show("Set AMP_PER_VOLT Failed"); }
        }

        private void CMB_batmonsensortype_SelectedIndexChanged(object sender, EventArgs e)
        {
            int selection = int.Parse(CMB_batmonsensortype.Text.Substring(0, 1));


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
            else if (selection == 4) // 3dr iv
            {
                float maxvolt = 50f;
                float maxamps = 90f;
                float mvpervolt = 100f;
                float mvperamp = 55.55f;

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

            disableinstructionbox();
        }

        public void Deactivate()
        {
            timer1.Stop();
        }

        public void Activate()
        {
            startup = true;
            bool not_supported = false;
            if (MainV2.comPort.MAV.param["BATT_MONITOR"] != null)
            {
                if (MainV2.comPort.MAV.param["BATT_MONITOR"].ToString() != "0.0")
                {
                    CMB_batmontype.SelectedIndex = getIndex(CMB_batmontype, (int)float.Parse(MainV2.comPort.MAV.param["BATT_MONITOR"].ToString()));
                }
            }

            if (MainV2.comPort.MAV.param["BATT_CAPACITY"] != null)
                TXT_battcapacity.Text = MainV2.comPort.MAV.param["BATT_CAPACITY"].ToString();
            if (MainV2.comPort.MAV.param["INPUT_VOLTS"] != null)
                TXT_inputvoltage.Text = MainV2.comPort.MAV.param["INPUT_VOLTS"].ToString();
            else
                not_supported = true;
            TXT_voltage.Text = MainV2.comPort.MAV.cs.battery_voltage.ToString();
            TXT_measuredvoltage.Text = TXT_voltage.Text;
            if (MainV2.comPort.MAV.param["VOLT_DIVIDER"] != null)
                TXT_divider.Text = MainV2.comPort.MAV.param["VOLT_DIVIDER"].ToString();
            else
                not_supported = true;
            if (MainV2.comPort.MAV.param["AMP_PER_VOLT"] != null)
                TXT_ampspervolt.Text = MainV2.comPort.MAV.param["AMP_PER_VOLT"].ToString();
            else
                not_supported = true;
            if (not_supported)
            {
                TXT_inputvoltage.Enabled = false;
                TXT_measuredvoltage.Enabled = false;
                TXT_divider.Enabled = false;
                TXT_ampspervolt.Enabled = false;
            }

            // ignore language re . vs ,

            if (TXT_ampspervolt.Text == (13.6612).ToString())
            {
                CMB_batmonsensortype.SelectedIndex = 1;
            }
            else if (TXT_ampspervolt.Text == (27.3224).ToString())
            {
                CMB_batmonsensortype.SelectedIndex = 2;
            }
            else if (TXT_ampspervolt.Text == (54.64481).ToString())
            {
                CMB_batmonsensortype.SelectedIndex = 3;
            }
            else if (TXT_ampspervolt.Text == (18.0018).ToString())
            {
                CMB_batmonsensortype.SelectedIndex = 4;
            }
            else
            {
                CMB_batmonsensortype.SelectedIndex = 0;
            }

            if (MainV2.comPort.MAV.param["BATT_VOLT_PIN"] != null)
            {
                CMB_apmversion.Enabled = true;

                float value = (float)MainV2.comPort.MAV.param["BATT_VOLT_PIN"];
                if (value == 0) // apm1
                {
                    CMB_apmversion.SelectedIndex = 0;
                }
                else if (value == 1) // apm2
                {
                    CMB_apmversion.SelectedIndex = 1;
                }
                else if (value == 2) // apm2.5
                {
                    CMB_apmversion.SelectedIndex = 2;
                }
            }
            else
            {
                CMB_apmversion.Enabled = false;
            }

            startup = false;

            timer1.Start();
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

        private void timer1_Tick(object sender, EventArgs e)
        {
            TXT_voltage.Text = MainV2.comPort.MAV.cs.battery_voltage.ToString();
        }

        private void CMB_apmversion_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup)
                return;

            int selection = int.Parse(CMB_apmversion.Text.Substring(0, 1));

            try
            {
                if (selection == 0)
                {
                    // apm1
                    MainV2.comPort.setParam("BATT_VOLT_PIN", 0);
                    MainV2.comPort.setParam("BATT_CURR_PIN", 1);
                }
                else if (selection == 1)
                {
                    // apm2
                    MainV2.comPort.setParam("BATT_VOLT_PIN", 1);
                    MainV2.comPort.setParam("BATT_CURR_PIN", 2);
                }
                else if (selection == 2)
                {
                    //apm2.5
                    MainV2.comPort.setParam("BATT_VOLT_PIN", 13);
                    MainV2.comPort.setParam("BATT_CURR_PIN", 12);
                }
            }
            catch { CustomMessageBox.Show("Set BATT_????_PIN Failed"); }

            disableinstructionbox();
        }

        void disableinstructionbox()
        {
            try
            {
                if (int.Parse(CMB_apmversion.Text.Substring(0, 1)) == 2)
                {
                    if (int.Parse(CMB_batmonsensortype.Text.Substring(0, 1)) == 4)
                    {
                        textBox3.Visible = false;
                    }
                    else
                    {
                        textBox3.Visible = true;
                    }
                }
            }
            catch { }
        }
    }
}

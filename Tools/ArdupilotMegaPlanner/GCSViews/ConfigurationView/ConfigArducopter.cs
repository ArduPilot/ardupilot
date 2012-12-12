using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;
using System.Collections;
using ArdupilotMega.Controls;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigArducopter : UserControl, IActivate
    {
        Hashtable changes = new Hashtable();
        static Hashtable tooltips = new Hashtable();
        internal bool startup = true;


        public ConfigArducopter()
        {
            InitializeComponent();
        }

        public struct paramsettings // hk's
        {
            public string name;
            public float minvalue;
            public float maxvalue;
            public float normalvalue;
            public float scale;
            public string desc;
        }

        public void Activate()
        {
            if (!MainV2.comPort.BaseStream.IsOpen)
            {
                this.Enabled = false;
                return;
            }
            else
            {
                if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduCopter2)
                {
                    this.Enabled = true;
                }
                else
                {
                    this.Enabled = false;
                    return;
                }
            }

            startup = true;

            changes.Clear();

            // read tooltips
            if (tooltips.Count == 0)
                readToolTips();

            // ensure the fields are populated before setting them
            CH7_OPT.DataSource = Utilities.EnumTranslator.Translate<Common.ac2ch7modes>().ToList();
            CH7_OPT.DisplayMember = "Value";
            CH7_OPT.ValueMember = "Key";

            TUNE.DataSource = Utilities.EnumTranslator.Translate<Common.ac2ch6modes>().ToList();
            TUNE.DisplayMember = "Value";
            TUNE.ValueMember = "Key";

            // prefill all fields
            processToScreen();

            MVCHK_thr_Acc_enable.setup(1, 0, "THR_ACC_ENABLE", MainV2.comPort.MAV.param, groupBox2);

            startup = false;
        }

        void readToolTips()
        {
            string data = global::ArdupilotMega.Properties.Resources.MAVParam;

            string[] tips = data.Split(new char[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

            foreach (var tip in tips)
            {
                if (!tip.StartsWith("||"))
                    continue;

                string[] cols = tip.Split(new string[] { "||" }, 9, StringSplitOptions.None);

                if (cols.Length >= 8)
                {
                    paramsettings param = new paramsettings();
                    try
                    {
                        param.name = cols[1];
                        param.desc = AddNewLinesForTooltip(cols[7]);
                        param.scale = float.Parse(cols[5]);
                        param.minvalue = float.Parse(cols[2]);
                        param.maxvalue = float.Parse(cols[3]);
                        param.normalvalue = float.Parse(cols[4]);
                    }
                    catch { }
                    tooltips[cols[1]] = param;
                }

            }
        }

        // from http://stackoverflow.com/questions/2512781/winforms-big-paragraph-tooltip/2512895#2512895
        private const int maximumSingleLineTooltipLength = 50;

        private static string AddNewLinesForTooltip(string text)
        {
            if (text.Length < maximumSingleLineTooltipLength)
                return text;
            int lineLength = (int)Math.Sqrt((double)text.Length) * 2;
            StringBuilder sb = new StringBuilder();
            int currentLinePosition = 0;
            for (int textIndex = 0; textIndex < text.Length; textIndex++)
            {
                // If we have reached the target line length and the next      
                // character is whitespace then begin a new line.   
                if (currentLinePosition >= lineLength &&
                    char.IsWhiteSpace(text[textIndex]))
                {
                    sb.Append(Environment.NewLine);
                    currentLinePosition = 0;
                }
                // If we have just started a new line, skip all the whitespace.    
                if (currentLinePosition == 0)
                    while (textIndex < text.Length && char.IsWhiteSpace(text[textIndex]))
                        textIndex++;
                // Append the next character.     
                if (textIndex < text.Length) sb.Append(text[textIndex]);
                currentLinePosition++;
            }
            return sb.ToString();
        }

        void disableNumericUpDownControls(Control inctl)
        {
            foreach (Control ctl in inctl.Controls)
            {
                if (ctl.Controls.Count > 0)
                {
                    disableNumericUpDownControls(ctl);
                }
                if (ctl.GetType() == typeof(NumericUpDown))
                {
                    ctl.Enabled = false;
                }
            }
        }

        internal void processToScreen()
        {
            toolTip1.RemoveAll();

            disableNumericUpDownControls(this);


            // process hashdefines and update display
            foreach (string value in MainV2.comPort.MAV.param.Keys)
            {
                if (value == null || value == "")
                    continue;

                //System.Diagnostics.Debug.WriteLine("Doing: " + value);


                string name = value;
                Control[] text = this.Controls.Find(name, true);
                foreach (Control ctl in text)
                {
                    try
                    {
                        if (ctl.GetType() == typeof(NumericUpDown))
                        {

                            float numbervalue = (float)MainV2.comPort.MAV.param[value];

                            MAVLink.modifyParamForDisplay(true, value, ref numbervalue);

                            NumericUpDown thisctl = ((NumericUpDown)ctl);
                            thisctl.Maximum = 9000;
                            thisctl.Minimum = -9000;
                            thisctl.Value = (decimal)numbervalue;
                            thisctl.Increment = (decimal)0.0001;
                            if (thisctl.Name.EndsWith("_P") || thisctl.Name.EndsWith("_I") || thisctl.Name.EndsWith("_D")
                                || thisctl.Name.EndsWith("_LOW") || thisctl.Name.EndsWith("_HIGH") || thisctl.Value == 0
                                || thisctl.Value.ToString("0.####", new System.Globalization.CultureInfo("en-US")).Contains("."))
                            {
                                thisctl.DecimalPlaces = 4;
                            }
                            else
                            {
                                thisctl.Increment = (decimal)1;
                                thisctl.DecimalPlaces = 1;
                            }

                            if (thisctl.Name.ToUpper().EndsWith("THR_RATE_IMAX"))
                            {
                                thisctl.Maximum = 1000; // is a pwm
                                thisctl.Minimum = 0;
                            } else if (thisctl.Name.EndsWith("_IMAX"))
                            {
                                thisctl.Maximum = 180;
                                thisctl.Minimum = -180;
                            }

                            thisctl.Enabled = true;

                            thisctl.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                            thisctl.Validated += null;
                            if (tooltips[value] != null)
                            {
                                try
                                {
                                    toolTip1.SetToolTip(ctl, ((paramsettings)tooltips[value]).desc);
                                }
                                catch { }
                            }
                            thisctl.Validated += new EventHandler(EEPROM_View_float_TextChanged);

                        }
                        else if (ctl.GetType() == typeof(ComboBox))
                        {

                            ComboBox thisctl = ((ComboBox)ctl);

                            thisctl.SelectedValue = (int)(float)MainV2.comPort.MAV.param[value];

                            thisctl.Validated += new EventHandler(ComboBox_Validated);

                            thisctl.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                        }
                    }
                    catch { }

                }
                if (text.Length == 0)
                {
                    //Console.WriteLine(name + " not found");
                }

            }
        }

               void ComboBox_Validated(object sender, EventArgs e)
        {
            EEPROM_View_float_TextChanged(sender, e);
        }

        void Configuration_Validating(object sender, CancelEventArgs e)
        {
            EEPROM_View_float_TextChanged(sender, e);
        }

        internal void EEPROM_View_float_TextChanged(object sender, EventArgs e)
        {
            if (startup == true)
                return;

            float value = 0;
            string name = ((Control)sender).Name;

            // do domainupdown state check
            try
            {
                if (sender.GetType() == typeof(NumericUpDown))
                {
                    value = float.Parse(((Control)sender).Text);
                    MAVLink.modifyParamForDisplay(false, ((Control)sender).Name, ref value);
                    changes[name] = value;
                }
                else if (sender.GetType() == typeof(ComboBox))
                {
                    value = (int)((ComboBox)sender).SelectedValue;
                    changes[name] = value;
                }
                ((Control)sender).BackColor = Color.Green;
            }
            catch (Exception)
            {
                ((Control)sender).BackColor = Color.Red;
            }

            try
            {
                // enable roll and pitch pairing for ac2
                if (CHK_lockrollpitch.Checked)
                {
                    if (name.StartsWith("RATE_") || name.StartsWith("STB_") || name.StartsWith("ACRO_"))
                    {
                        if (name.Contains("_RLL_"))
                        {
                            string newname = name.Replace("_RLL_", "_PIT_");
                            Control[] arr = this.Controls.Find(newname, true);
                            changes[newname] = value;

                            if (arr.Length > 0)
                            {
                                arr[0].Text = ((Control)sender).Text;
                                arr[0].BackColor = Color.Green;
                            }

                        }
                        else if (name.Contains("_PIT_"))
                        {
                            string newname = name.Replace("_PIT_", "_RLL_");
                            Control[] arr = this.Controls.Find(newname, true);
                            changes[newname] = value;

                            if (arr.Length > 0)
                            {
                                arr[0].Text = ((Control)sender).Text;
                                arr[0].BackColor = Color.Green;
                            }
                        }
                    }
                }
                // keep nav_lat and nav_lon paired
                if (name.Contains("NAV_LAT_"))
                {
                    string newname = name.Replace("NAV_LAT_", "NAV_LON_");
                    Control[] arr = this.Controls.Find(newname, true);
                    changes[newname] = value;

                    if (arr.Length > 0)
                    {
                        arr[0].Text = ((Control)sender).Text;
                        arr[0].BackColor = Color.Green;
                    }
                }
                // keep loiter_lat and loiter_lon paired
                if (name.Contains("LOITER_LAT_"))
                {
                    string newname = name.Replace("LOITER_LAT_", "LOITER_LON_");
                    Control[] arr = this.Controls.Find(newname, true);
                    changes[newname] = value;

                    if (arr.Length > 0)
                    {
                        arr[0].Text = ((Control)sender).Text;
                        arr[0].BackColor = Color.Green;
                    }
                }
                // keep nav_lat and nav_lon paired
                if (name.Contains("HLD_LAT_"))
                {
                    string newname = name.Replace("HLD_LAT_", "HLD_LON_");
                    Control[] arr = this.Controls.Find(newname, true);
                    changes[newname] = value;

                    if (arr.Length > 0)
                    {
                        arr[0].Text = ((Control)sender).Text;
                        arr[0].BackColor = Color.Green;
                    }
                }
            }
            catch { }
        }

        private void BUT_writePIDS_Click(object sender, EventArgs e)
        {
            var temp = (Hashtable)changes.Clone();

            foreach (string value in temp.Keys)
            {
                try
                {
                    MainV2.comPort.setParam(value, (float)changes[value]);

                    try
                    {
                        // set control as well
                        var textControls = this.Controls.Find(value, true);
                        if (textControls.Length > 0)
                        {
                            textControls[0].BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                        }
                    }
                    catch
                    {

                    }

                }
                catch
                {
                    CustomMessageBox.Show("Set " + value + " Failed");
                }
            }
        }

        /// <summary>
        /// Handles the Click event of the BUT_rerequestparams control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="System.EventArgs"/> instance containing the event data.</param>
        protected void BUT_rerequestparams_Click(object sender, EventArgs e)
        {
            if (!MainV2.comPort.BaseStream.IsOpen)
                return;

            ((Control)sender).Enabled = false;

            try
            {
                MainV2.comPort.getParamList();
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show("Error: getting param list " + ex.ToString());
            }


            ((Control)sender).Enabled = true;


            this.Activate();
        }

        private void groupBox1_Enter(object sender, EventArgs e)
        {

        }

        private void groupBox4_Enter(object sender, EventArgs e)
        {

        }

        private void groupBox6_Enter(object sender, EventArgs e)
        {

        }

        private void groupBox7_Enter(object sender, EventArgs e)
        {

        }

        private void groupBox19_Enter(object sender, EventArgs e)
        {

        }

        private void groupBox20_Enter(object sender, EventArgs e)
        {

        }

        private void groupBox21_Enter(object sender, EventArgs e)
        {

        }

        private void groupBox23_Enter(object sender, EventArgs e)
        {

        }

        private void groupBox24_Enter(object sender, EventArgs e)
        {

        }
      
        
    }
}

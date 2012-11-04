using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Collections;

namespace ArdupilotMega.Controls
{
    public class MavlinkNumericUpDown : NumericUpDown
    {
        [System.ComponentModel.Browsable(true)]
        public float Min { get; set; }

        [System.ComponentModel.Browsable(true)]
        public float Max { get; set; }

        [System.ComponentModel.Browsable(true)]
        public string ParamName { get; set; }

        [System.ComponentModel.Browsable(true)]
        public Hashtable param { get; set; }

        float _scale = 1;


        public MavlinkNumericUpDown()
        {
            Min = 0;
            Max = 1;

            this.Enabled = false;
        }

        public void setup(float Min, float Max, float Scale, float Increment, string paramname, Hashtable paramlist)
        {
            this.ValueChanged -= MavlinkNumericUpDown_ValueChanged;

            _scale = Scale;
            this.Minimum = (decimal)(Min);
            this.Maximum = (decimal)(Max);
            this.Increment = (decimal)(Increment);
            this.ParamName = paramname;
            this.param = paramlist;

            if (paramlist.ContainsKey(paramname))
            {
                this.Enabled = true;

                decimal value = (decimal)((float)paramlist[paramname] / _scale);

                if (value < this.Minimum)
                    this.Minimum = value;
                if (value > this.Maximum)
                    this.Maximum = value;

                this.Value = value;

            }
            else
            {
                this.Enabled = false;
            }

            this.ValueChanged += new EventHandler(MavlinkNumericUpDown_ValueChanged);
        }

        void MavlinkNumericUpDown_ValueChanged(object sender, EventArgs e)
        {
            try
            {
                bool ans = MainV2.comPort.setParam(ParamName, (float)this.Value * _scale);
                if (ans == false)
                    CustomMessageBox.Show("Set " + ParamName + " Failed 1!");
            }
            catch { CustomMessageBox.Show("Set " + ParamName + " Failed 2!"); }
        }

    }
}

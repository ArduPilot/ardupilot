using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Collections;

namespace ArdupilotMega.Controls
{
    public class MavlinkCheckBox : CheckBox
    {
        [System.ComponentModel.Browsable(true)]
        public float OnValue { get; set; }

        [System.ComponentModel.Browsable(true)]
        public float OffValue { get; set; }

        [System.ComponentModel.Browsable(true)]
        public string ParamName { get; set; }

        [System.ComponentModel.Browsable(true)]
        public Hashtable param { get; set; }

        Control _control;

        public MavlinkCheckBox()
        {
            OnValue = 1;
            OffValue = 0;

            this.Enabled = false;
        }

        public void setup(float OnValue, float OffValue, string paramname, Hashtable paramlist, Control enabledisable = null)
        {
            base.CheckedChanged -= MavlinkCheckBox_CheckedChanged;

            this.OnValue = OnValue;
            this.OffValue = OffValue;
            this.ParamName = paramname;
            this.param = paramlist;
            this._control = enabledisable;

            if (paramlist.ContainsKey(paramname))
            {
                this.Enabled = true;

                if ((float)paramlist[paramname] == OnValue)
                {
                    this.Checked = true;
                    enableControl(true);
                }
                else if ((float)paramlist[paramname] == OffValue)
                {
                    this.Checked = false;
                    enableControl(false);
                }
                else
                {
                    this.CheckState = System.Windows.Forms.CheckState.Indeterminate;
                    enableControl(false);
                }
            }
            else
            {
                this.Enabled = false;
            }

            base.CheckedChanged += new EventHandler(MavlinkCheckBox_CheckedChanged);
        }

        void enableControl(bool enable)
        {
            if (_control != null)
                _control.Enabled = enable;
        }

        void MavlinkCheckBox_CheckedChanged(object sender, EventArgs e)
        {
            if (this.Checked)
            {
                enableControl(true);
                if (!MainV2.comPort.setParam(ParamName, OnValue))
                {
                    CustomMessageBox.Show("Set "+ParamName + " Failed!");
                }
            }
            else
            {
                enableControl(false);
                if (!MainV2.comPort.setParam(ParamName, OffValue))
                {
                    CustomMessageBox.Show("Set " + ParamName + " Failed!");
                }
            }
        }

        
    }
}

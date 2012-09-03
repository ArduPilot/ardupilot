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
    public partial class ConfigAccelerometerCalibrationPlane : UserControl, IActivate
    {
        bool startup = false;

        public ConfigAccelerometerCalibrationPlane()
        {
            InitializeComponent();
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
                if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
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

            if (MainV2.comPort.param["MANUAL_LEVEL"] != null)
                CHK_manuallevel.Checked = MainV2.comPort.param["MANUAL_LEVEL"].ToString() == "1" ? true : false;

            startup = false;
        }

        private void CHK_manuallevel_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            try
            {
                MainV2.comPort.setParam("MANUAL_LEVEL", ((CheckBox)sender).Checked == true ? 1 : 0);
            }
            catch
            {
                CustomMessageBox.Show("Failed to level : AP 2.32+ is required");
            }
        }

        private void BUT_levelplane_Click(object sender, EventArgs e)
        {
            try
            {
#if MAVLINK10              
                            MainV2.comPort.doCommand(MAVLink.MAV_CMD.PREFLIGHT_CALIBRATION,1,0,1,0,0,0,0);
#else
                MainV2.comPort.doAction(MAVLink.MAV_ACTION.MAV_ACTION_CALIBRATE_ACC);
#endif
                BUT_levelplane.Text = "Complete";
            }
            catch
            {
                CustomMessageBox.Show("Failed to level : AP 2.32+ is required");
            }
        }
    }
}

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
    public partial class ConfigAccelerometerCalibrationQuad : BackStageViewContentPanel
    {
        public ConfigAccelerometerCalibrationQuad()
        {
            InitializeComponent();
        }

        private void pictureBoxQuadX_Click(object sender, EventArgs e)
        {
            try
            {
                MainV2.comPort.setParam("FRAME", 1f);
                CustomMessageBox.Show("Set to x");

                lbl_frame.Text = "X";
            }
            catch { CustomMessageBox.Show("Set frame failed"); }
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
                CustomMessageBox.Show("Failed to level : ac2 2.0.37+ is required");
            }
        }

        private void pictureBoxQuad_Click(object sender, EventArgs e)
        {
            try
            {
                MainV2.comPort.setParam("FRAME", 0f);
                CustomMessageBox.Show("Set to +");
                lbl_frame.Text = "+";
            }
            catch { CustomMessageBox.Show("Set frame failed"); }
        }

        private void ConfigAccelerometerCalibration_Load(object sender, EventArgs e)
        {
            if (!MainV2.comPort.BaseStream.IsOpen)
            {
                this.Enabled = false;
                return;
            }
            else
            {
                if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2)
                {
                    this.Enabled = true;
                }
                else
                {
                    this.Enabled = false;
                    return;
                }
            }

            try
            {
                lbl_frame.Text = ((float)MainV2.comPort.param["FRAME"] == 0) ? "+" : "X";
            }
            catch { lbl_frame.Text = "Invalid Frame"; }
        }
    }
}

using System;
using System.Reflection;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;
using log4net;
using Transitions;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigAccelerometerCalibrationQuad : UserControl, IActivate, IDeactivate
    {
        private static readonly ILog Log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        private const float DisabledOpacity = 0.2F;
        private const float EnabledOpacity = 1.0F;

        public ConfigAccelerometerCalibrationQuad()
        {
            InitializeComponent();
        }

        private void BUT_levelac2_Click(object sender, EventArgs e)
        {
            try
            {
                Log.Info("Sending level command (mavlink 1.0)");                
                MainV2.comPort.doCommand(MAVLink.MAV_CMD.PREFLIGHT_CALIBRATION,1,0,0,0,0,0,0);

                BUT_levelac2.Text = "Complete";
            }
            catch(Exception ex)
            {
                Log.Error("Exception on level", ex);
                CustomMessageBox.Show("Failed to level : ac2 2.0.37+ is required");
            }
        }

        private void pictureBox_Click(object sender, EventArgs e)
        {
            if (sender == pictureBoxPlus)
                radioButton_Plus.Checked = true;
            else
                radioButton_X.Checked = true;
        }

        private void SetPlus()
        {
            FadePicBoxes(DisabledOpacity, EnabledOpacity);
            SetFrameParam(true);
        }

        private void SetX()
        {
            FadePicBoxes(EnabledOpacity, DisabledOpacity);
            SetFrameParam(false);
        }

        private void SetFrameParam(bool isPlus)
        {
            var f = isPlus ? 0f : 1f;

            try
            {
                MainV2.comPort.setParam("FRAME", f);
            }
            catch
            {
                CustomMessageBox.Show("Set frame failed", "Error", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }

        private void FadePicBoxes(float xOpacity, float plusOpacity)
        {
            var fade = new Transition(new TransitionType_Linear(400));
            fade.add(pictureBoxX, "Opacity", xOpacity);
            fade.add(pictureBoxPlus, "Opacity", plusOpacity);
            fade.run();
        }

        public void Activate()
        {
            if (!MainV2.comPort.param.ContainsKey("FRAME"))
            {
                this.Enabled = false;
                return;
            }

            if ((float)MainV2.comPort.param["FRAME"] == 0)
            {
                this.radioButton_Plus.Checked = true;
                pictureBoxX.Opacity = DisabledOpacity;
                pictureBoxPlus.Opacity = EnabledOpacity;
            }
            else
            {
                this.radioButton_X.Checked = true;
                pictureBoxX.Opacity = EnabledOpacity;
                pictureBoxPlus.Opacity = DisabledOpacity;
            }

            radioButton_Plus.CheckedChanged += RadioButtonPlusCheckedChanged;
        }

        public void Deactivate()
        {
            MainV2.giveComport = false;

            radioButton_Plus.CheckedChanged -= RadioButtonPlusCheckedChanged;
        }

        void RadioButtonPlusCheckedChanged(object sender, EventArgs e)
        {
            if (radioButton_X.Checked)
                SetX();
            else
                SetPlus();
        }

        private void BUT_calib_accell_Click(object sender, EventArgs e)
        {
            if (MainV2.giveComport == true)
            {
                MainV2.comPort.BaseStream.WriteLine("");
                return;
            }

            try
            {
                Log.Info("Sending accel command (mavlink 1.0)");
                MainV2.giveComport = true;
                MainV2.comPort.doCommand(MAVLink.MAV_CMD.PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0);
                //MainV2.giveComport = false;

                System.Threading.ThreadPool.QueueUserWorkItem(readmessage,this);

                BUT_calib_accell.Text = "Click When Done";
            }
            catch (Exception ex)
            {
                MainV2.giveComport = false;
                Log.Error("Exception on level", ex);
                CustomMessageBox.Show("Failed to level : ac2 2.0.37+ is required");
            }
        }

        static void readmessage(object item)
        {
            ConfigAccelerometerCalibrationQuad local = (ConfigAccelerometerCalibrationQuad)item;

            while (!(MainV2.cs.message.Contains("Calibration successful") || MainV2.cs.message.Contains("Calibration failed")))
            {
                System.Threading.Thread.Sleep(10);
                // read the message
                MainV2.comPort.readPacket();
                // update cs with the message
                MainV2.cs.UpdateCurrentSettings(null);
                // update user display
                local.UpdateUserMessage();
            }

            MainV2.giveComport = false;

            local.Invoke((MethodInvoker)delegate()
            {
                local.BUT_calib_accell.Text = "Done";
            });
        }

        public void UpdateUserMessage()
        {
            this.Invoke((MethodInvoker)delegate()
            {
                 lbl_Accel_user.Text = MainV2.cs.message;
            });
        }
    }
}

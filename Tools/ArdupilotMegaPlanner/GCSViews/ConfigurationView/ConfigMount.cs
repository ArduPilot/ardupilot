using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;
using ArdupilotMega.Presenter;
using Transitions;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigMount : UserControl, IActivate
    {
        private Transition[] _ErrorTransition;
        private Transition _NoErrorTransition;

        public ConfigMount()
        {
            InitializeComponent();
            PBOX_WarningIcon.Opacity = 0.0F;
            LBL_Error.Opacity = 0.0F;




            var delay = new Transition(new TransitionType_Linear(2000));
            var fadeIn = new Transition(new TransitionType_Linear(800));
            fadeIn.add(PBOX_WarningIcon, "Opacity", 1.0F);
            fadeIn.add(LBL_Error, "Opacity", 1.0F);

            _ErrorTransition = new[] { delay, fadeIn };

            _NoErrorTransition = new Transition(new TransitionType_Linear(10));
            _NoErrorTransition.add(PBOX_WarningIcon, "Opacity", 0.0F);
            _NoErrorTransition.add(LBL_Error, "Opacity", 0.0F);

            //setup button actions
            foreach (var btn in Controls.Cast<Control>().OfType<Button>())
                btn.Click += HandleButtonClick;

            LNK_wiki.MouseEnter += (s, e) => FadeLinkTo((LinkLabel)s, Color.CornflowerBlue);
            LNK_wiki.MouseLeave += (s, e) => FadeLinkTo((LinkLabel)s, Color.WhiteSmoke);

            SetErrorMessageOpacity();
        }

        // 0 = disabled 1 = enabled
        enum Channelap
        {
            Disable = 0,
            CH_5 = 1,
            CH_6 = 1,
            CH_7 = 1,
            CH_8 = 1
        }

        // 0 = disabled 1 = enabled
        enum Channelac
        {
            Disable = 0,
            CAM_P = 7,
            CAM_R = 8,
            CAM_Y = 6
        }

        public void Activate()
        {
            if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
                mavlinkComboBoxTilt.setup(typeof(Channelap), "MNT_STAB_PITCH", MainV2.comPort.param);
                mavlinkComboBoxRoll.setup(typeof(Channelap), "MNT_STAB_ROLL", MainV2.comPort.param);
                mavlinkComboBoxPan.setup(typeof(Channelap), "MNT_STAB_YAW", MainV2.comPort.param);
            }
            else
            {
                mavlinkComboBoxTilt.setup(typeof(Channelac), "CAM_P_FUNCTION", MainV2.comPort.param, "MNT_STAB_PITCH");
                mavlinkComboBoxRoll.setup(typeof(Channelac), "CAM_R_FUNCTION", MainV2.comPort.param, "MNT_STAB_ROLL");
                mavlinkComboBoxPan.setup(typeof(Channelac), "CAM_Y_FUNCTION", MainV2.comPort.param, "MNT_STAB_YAW");
            }

            updatePitch();
            updateRoll();
            updateYaw();

        }

        void updatePitch()
        {
            // pitch
            mavlinkNumericUpDown11.setup(800, 2200, 1, 1, mavlinkComboBoxTilt.Text +"_MIN", MainV2.comPort.param);
            mavlinkNumericUpDown12.setup(800, 2200, 1, 1, mavlinkComboBoxTilt.Text + "_MAX", MainV2.comPort.param);
            mavlinkNumericUpDown1.setup(-90, 0, 100, 1, mavlinkComboBoxTilt.Text + "_ANGLE_MIN", MainV2.comPort.param);
            mavlinkNumericUpDown2.setup(0, 90, 100, 1, mavlinkComboBoxTilt.Text + "_ANGLE_MAX", MainV2.comPort.param);
            mavlinkCheckBox1.setup(-1, 1, mavlinkComboBoxTilt.Text + "_REV", MainV2.comPort.param);
        }

        void updateRoll()
        {
            // roll
            mavlinkNumericUpDown5.setup(800, 2200, 1, 1, mavlinkComboBoxRoll.Text +"_MIN", MainV2.comPort.param);
            mavlinkNumericUpDown6.setup(800, 2200, 1, 1, mavlinkComboBoxRoll.Text + "_MAX", MainV2.comPort.param);
            mavlinkNumericUpDown3.setup(-90, 0, 100, 1, mavlinkComboBoxRoll.Text + "_ANGLE_MIN", MainV2.comPort.param);
            mavlinkNumericUpDown4.setup(0, 90, 100, 1, mavlinkComboBoxRoll.Text + "_ANGLE_MAX", MainV2.comPort.param);
            mavlinkCheckBox2.setup(-1, 1, mavlinkComboBoxRoll.Text + "_REV", MainV2.comPort.param);
        }

        void updateYaw()
        {
            // yaw
            mavlinkNumericUpDown9.setup(800, 2200, 1, 1, mavlinkComboBoxPan.Text + "_MIN", MainV2.comPort.param);
            mavlinkNumericUpDown10.setup(800, 2200, 1, 1, mavlinkComboBoxPan.Text + "_MAX", MainV2.comPort.param);
            mavlinkNumericUpDown7.setup(-90, 0, 100, 1, mavlinkComboBoxPan.Text + "_ANGLE_MIN", MainV2.comPort.param);
            mavlinkNumericUpDown8.setup(0, 90, 100, 1, mavlinkComboBoxPan.Text + "_ANGLE_MAX", MainV2.comPort.param);
            mavlinkCheckBox3.setup(-1, 1, mavlinkComboBoxPan.Text + "_REV", MainV2.comPort.param);
        }

        private void SetErrorMessageOpacity()
        {
           /* if (_presenter.HasError)
            {
                // Todo - is this the prob? maybe single log trasition
                var t = new Transition(new TransitionType_Acceleration(1000));
                t.add(PBOX_WarningIcon, "Opacity", 1.0F);
                t.add(LBL_Error, "Opacity", 1.0F);
                t.run();

                //Transition.runChain(_ErrorTransition);
            }
            else*/
            {
                _NoErrorTransition.run();
            }
        }

        private static void FadeLinkTo(LinkLabel l, Color c)
        {
            var changeColorTransition = new Transition(new TransitionType_Linear(300));
            changeColorTransition.add(l, "LinkColor", c);
            changeColorTransition.run();
        }

        // Common handler for all buttons
        // Will execute an ICommand if one is found on the button Tag
        private static void HandleButtonClick(object sender, EventArgs e)
        {
            if (sender is Button)
            {
                var cmd = (sender as Button).Tag as ICommand;

                if (cmd != null)
                    if (cmd.CanExecute(null))
                        cmd.Execute(null);
            }
        }

        // Something has changed on the presenter - This may be an Icommand
        // enabled state, so update the buttons as appropriate
        void CheckCommandStates(object sender, PropertyChangedEventArgs propertyChangedEventArgs)
        {
            foreach (var btn in Controls.Cast<Control>().OfType<Button>())
            {
                var cmd = btn.Tag as ICommand;
                if (cmd != null)
                    btn.Enabled = cmd.CanExecute(null);
            }
        }

        private void LNK_Wiki_Clicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            Process.Start(new ProcessStartInfo("http://code.google.com/p/arducopter/wiki/AC2_Camera"));
        }

        private void mavlinkComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            // enable 3 axis stabilize
            if (MainV2.comPort.param.ContainsKey("MNT_MODE"))
                MainV2.comPort.setParam("MNT_MODE",3);

            updatePitch();
            updateRoll();
            updateYaw();
        }
    }
}
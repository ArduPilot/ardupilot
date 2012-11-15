using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;
using ArdupilotMega.Presenter;
using Transitions;
using System.Collections;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigMount : UserControl, IActivate
    {
        private Transition[] _ErrorTransition;
        private Transition _NoErrorTransition;
        bool startup = true;

        public string ParamHead = "MNT_";

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

            if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
                mavlinkComboBoxTilt.Items.AddRange(Enum.GetNames(typeof(Channelap)));
                mavlinkComboBoxRoll.Items.AddRange(Enum.GetNames(typeof(Channelap)));
                mavlinkComboBoxPan.Items.AddRange(Enum.GetNames(typeof(Channelap)));
            }
            else
            {
                mavlinkComboBoxTilt.Items.AddRange(Enum.GetNames(typeof(Channelac)));
                mavlinkComboBoxRoll.Items.AddRange(Enum.GetNames(typeof(Channelac)));
                mavlinkComboBoxPan.Items.AddRange(Enum.GetNames(typeof(Channelac)));
            }
        }

        // 0 = disabled 1 = enabled
        enum Channelap
        {
            Disable = 0,
            RC5 = 1,
            RC6 = 1,
            RC7 = 1,
            RC8 = 1,
            RC9 = 1,
            RC10 = 1,
            RC11 = 1
        }

        // 0 = disabled 1 = enabled
        enum Channelac
        {
            Disable = 0,
            RC5 = 1,
            RC6 = 1,
            RC7 = 1,
            RC8 = 1,
            RC10 = 1,
            RC11 = 1
        }

        enum Channelinput
        {
            Disable = 0,
            RC5 = 5,
            RC6 = 6,
            RC7 = 7,
            RC8 = 8
        }

        public void Activate()
        {
            Hashtable copy = new Hashtable(MainV2.comPort.param);

            foreach (string item in copy.Keys)
            {
                if (item.EndsWith("_FUNCTION"))
                {
                    switch (MainV2.comPort.param[item].ToString())
                    {
                        case "6":
                            mavlinkComboBoxPan.Text = item.Replace("_FUNCTION", "");
                            break;
                        case "7":
                            mavlinkComboBoxTilt.Text = item.Replace("_FUNCTION", "");
                            break;
                        case "8":
                            mavlinkComboBoxRoll.Text = item.Replace("_FUNCTION", "");
                            break;
                        default:
                            break;
                    }
                }
            }

            startup = false;

            try
            {
                updatePitch();
                updateRoll();
                updateYaw();

                CHK_stab_tilt.setup(1, 0, ParamHead+"STAB_TILT", MainV2.comPort.param);
                CHK_stab_roll.setup(1, 0, ParamHead+"STAB_ROLL", MainV2.comPort.param);
                CHK_stab_pan.setup(1, 0, ParamHead+"STAB_PAN", MainV2.comPort.param);

                NUD_CONTROL_x.setup(-180, 180, 100, 1, ParamHead+"CONTROL_X",MainV2.comPort.param);
                NUD_CONTROL_y.setup(-180, 180, 100, 1, ParamHead+"CONTROL_Y", MainV2.comPort.param);
                NUD_CONTROL_z.setup(-180, 180, 100, 1, ParamHead+"CONTROL_Z", MainV2.comPort.param);

                NUD_NEUTRAL_x.setup(-180, 180, 100, 1, ParamHead+"NEUTRAL_X", MainV2.comPort.param);
                NUD_NEUTRAL_y.setup(-180, 180, 100, 1, ParamHead+"NEUTRAL_Y", MainV2.comPort.param);
                NUD_NEUTRAL_z.setup(-180, 180, 100, 1, ParamHead+"NEUTRAL_Z", MainV2.comPort.param);

                NUD_RETRACT_x.setup(-180, 180, 100, 1, ParamHead+"RETRACT_X", MainV2.comPort.param);
                NUD_RETRACT_y.setup(-180, 180, 100, 1, ParamHead+"RETRACT_Y", MainV2.comPort.param);
                NUD_RETRACT_z.setup(-180, 180, 100, 1, ParamHead+"RETRACT_Z", MainV2.comPort.param);
            }
            catch (Exception ex) { CustomMessageBox.Show("Failed to set Param\n" + ex.ToString()); this.Enabled = false; return; }
        }

        void ensureDisabled(ComboBox cmb, int number, string exclude = "")
        {
            foreach (string item in cmb.Items) 
            {
                if (MainV2.comPort.param.ContainsKey(item+"_FUNCTION")) {
                    float ans = (float)MainV2.comPort.param[item+"_FUNCTION"];

                    if (item == exclude)
                        continue;

                    if (ans == number)
                    {
                        MainV2.comPort.setParam(item + "_FUNCTION",0);
                    }
                }
            }
        }

        void updatePitch()
        {
            // pitch
            if (mavlinkComboBoxTilt.Text == "")
                return;

            if (mavlinkComboBoxTilt.Text != "Disable")
            {
                MainV2.comPort.setParam(mavlinkComboBoxTilt.Text + "_FUNCTION", 7);
                //MainV2.MainV2.comPort.setParam(ParamHead+"STAB_TILT", 1);
            }
            else
            {
                //MainV2.comPort.setParam(ParamHead+"STAB_TILT", 0);
                ensureDisabled(mavlinkComboBoxTilt, 7);    
            }
            

            mavlinkNumericUpDownTSM.setup(800, 2200, 1, 1, mavlinkComboBoxTilt.Text +"_MIN", MainV2.comPort.param);
            mavlinkNumericUpDownTSMX.setup(800, 2200, 1, 1, mavlinkComboBoxTilt.Text + "_MAX", MainV2.comPort.param);
            mavlinkNumericUpDownTAM.setup(-90, 0, 100, 1, ParamHead+"ANGMIN_TIL", MainV2.comPort.param);
            mavlinkNumericUpDownTAMX.setup(0, 90, 100, 1, ParamHead+"ANGMAX_TIL", MainV2.comPort.param);
            mavlinkCheckBoxTR.setup(-1, 1, mavlinkComboBoxTilt.Text + "_REV", MainV2.comPort.param);
            CMB_inputch_tilt.setup(typeof(Channelinput), ParamHead+"RC_IN_TILT", MainV2.comPort.param);
        }

        void updateRoll()
        {
            // roll
            if (mavlinkComboBoxRoll.Text == "")
                return;

            if (mavlinkComboBoxRoll.Text != "Disable")
            {
                MainV2.comPort.setParam(mavlinkComboBoxRoll.Text + "_FUNCTION", 8);
                //MainV2.comPort.setParam(ParamHead+"STAB_ROLL", 1);
            }
            else
            {
                //MainV2.comPort.setParam(ParamHead+"STAB_ROLL", 0);
                ensureDisabled(mavlinkComboBoxRoll,8);
            }

            mavlinkNumericUpDownRSM.setup(800, 2200, 1, 1, mavlinkComboBoxRoll.Text +"_MIN", MainV2.comPort.param);
            mavlinkNumericUpDownRSMX.setup(800, 2200, 1, 1, mavlinkComboBoxRoll.Text + "_MAX", MainV2.comPort.param);
            mavlinkNumericUpDownRAM.setup(-90, 0, 100, 1, ParamHead+"ANGMIN_ROL", MainV2.comPort.param);
            mavlinkNumericUpDownRAMX.setup(0, 90, 100, 1, ParamHead+"ANGMAX_ROL", MainV2.comPort.param);
            mavlinkCheckBoxRR.setup(-1, 1, mavlinkComboBoxRoll.Text + "_REV", MainV2.comPort.param);
            CMB_inputch_roll.setup(typeof(Channelinput), ParamHead+"RC_IN_ROLL", MainV2.comPort.param);
        }

        void updateYaw()
        {
            // yaw
            if (mavlinkComboBoxPan.Text == "")
                return;

            if (mavlinkComboBoxPan.Text != "Disable")
            {
                MainV2.comPort.setParam(mavlinkComboBoxPan.Text + "_FUNCTION", 6);
                //MainV2.comPort.setParam(ParamHead+"STAB_PAN", 1);
            }
            else
            {
                //MainV2.comPort.setParam(ParamHead+"STAB_PAN", 0);
                ensureDisabled(mavlinkComboBoxPan,6);
            }

            mavlinkNumericUpDownPSM.setup(800, 2200, 1, 1, mavlinkComboBoxPan.Text + "_MIN", MainV2.comPort.param);
            mavlinkNumericUpDownPSMX.setup(800, 2200, 1, 1, mavlinkComboBoxPan.Text + "_MAX", MainV2.comPort.param);
            mavlinkNumericUpDownPAM.setup(-90, 0, 100, 1, ParamHead+"ANGMIN_PAN", MainV2.comPort.param);
            mavlinkNumericUpDownPAMX.setup(0, 90, 100, 1, ParamHead+"ANGMAX_PAN", MainV2.comPort.param);
            mavlinkCheckBoxPR.setup(-1, 1, mavlinkComboBoxPan.Text + "_REV", MainV2.comPort.param);
            CMB_inputch_pan.setup(typeof(Channelinput), ParamHead+"RC_IN_PAN", MainV2.comPort.param);
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
            if (startup == true)
                return;

            ComboBox cmb = sender as ComboBox;

            try
            {

                // cleanup all others - disableing any previous selection
                ensureDisabled(cmb, 6, mavlinkComboBoxPan.Text);
                ensureDisabled(cmb, 7, mavlinkComboBoxTilt.Text);
                ensureDisabled(cmb, 8, mavlinkComboBoxRoll.Text);

                // enable 3 axis stabilize
                if (MainV2.comPort.param.ContainsKey(ParamHead+"MODE"))
                    MainV2.comPort.setParam(ParamHead+"MODE", 3);

                updatePitch();
                updateRoll();
                updateYaw();
            }
            catch (Exception ex) { CustomMessageBox.Show("Failed to set Param\n" + ex.ToString()); return; }
        }
    }
}
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;
using System.Reflection;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigSignalization : UserControl, IActivate
    {
        [Flags]
        public enum BeeperMode : byte
        {
            Startup = 1,
            Arm = 2,
            Disarm = 4,
            Armed = 8,
            GPS = 16,
            Voltage = 32,
            CH7 = 64,
            Landing = 128
        }

        public enum LedStyle : byte
        {
            None = 0,
            Legacy = 1,
            Features = 2,
            Disco = 3
        }

        [Flags]
        public enum LedMode : byte
        {
            [Description("Motor, TODO: add description")]
            Motor = 1,
            [Description("GPS, TODO: add description")]
            GPS = 2,
            [Description("Aux, TODO: add description")]
            Aux = 4,
            [Description("Assign AN5 to LED (0) or beeper (1)")]
            Beeper = 8,
            [Description("Low battery - fast flash (0) / oscillate (1)")]
            Voltage = 16,
            [Description("Nav blink - motors")]
            Nav_blink_motor = 32,
            [Description("Nav blink - GPS")]
            Nav_blink_GPS = 64
        }

        public enum LedModeX : byte
        {
            Off = 0,
            On = 1,
            Armed = 2,
            GPS = 3,
            Aux = 4,
            [Description("Low battery - fast flash")]
            Low_batt_fast = 5,
            [Description("Low battery - oscillate")]
            Low_batt_oscillate = 6,
            [Description("Nav blink - motors")]
            Nav_blink_motor = 7,
            [Description("Nav blink - GPS")]
            Nav_blink_GPS = 8
        }

        public ConfigSignalization()
        {
            InitializeComponent();
        }

        public void Activate()
        {
            if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
            }
            else if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduRover)
            {
            }
            else if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduCopter2)
            {
                Object value;
                //read BEEPER_MODE
                try
                {
                    value = MainV2.comPort.param["BEEPER_MODE"];
                    if (value == null)
                    {
                        Setup.flashMessage.FadeInOut("Reading BEEPER_MODE failed", false);
                    }
                    else
                    {
                        BeeperMode beeper_mode = (BeeperMode)System.Convert.ToSByte(value);
                        CB_beeper_0.Checked = (beeper_mode & BeeperMode.Startup) == BeeperMode.Startup;
                        CB_beeper_1.Checked = (beeper_mode & BeeperMode.Arm) == BeeperMode.Arm;
                        CB_beeper_2.Checked = (beeper_mode & BeeperMode.Disarm) == BeeperMode.Disarm;
                        CB_beeper_3.Checked = (beeper_mode & BeeperMode.Armed) == BeeperMode.Armed;
                        CB_beeper_4.Checked = (beeper_mode & BeeperMode.GPS) == BeeperMode.GPS;
                        CB_beeper_5.Checked = (beeper_mode & BeeperMode.Voltage) == BeeperMode.Voltage;
                        CB_beeper_6.Checked = (beeper_mode & BeeperMode.CH7) == BeeperMode.CH7;
                        CB_beeper_7.Checked = (beeper_mode & BeeperMode.Landing) == BeeperMode.Landing;
                        Setup.flashMessage.FadeInOut("Reading BEEPER_MODE succeed", true);
                    }
                }
                catch {
                    Setup.flashMessage.FadeInOut("Reading BEEPER_MODE failed", false);
                }

                //read LED_STYLE
                try
                {
                    value = MainV2.comPort.param["LED_STYLE"];
                    if (value == null)
                    {
                        Setup.flashMessage.FadeInOut("Reading LED_STYLE failed", false);
                    }
                    else
                    {
                        LedStyle led_style = (LedStyle)System.Convert.ToSByte(value);
                        CMB_leds_style.DataSource = Enum.GetValues(typeof(LedStyle));
                        CMB_leds_style.SelectedItem = led_style;
                        Setup.flashMessage.FadeInOut("Reading LED_STYLE succeed", true);
                    }
                }
                catch
                {
                    Setup.flashMessage.FadeInOut("Reading LED_STYLE failed", false);
                }
                CMB_led_1.DataSource = Enum.GetValues(typeof(LedMode));

                //read LED_MODE
                try
                {
                    value = MainV2.comPort.param["LED_MODE"];
                    if (value == null)
                    {
                        Setup.flashMessage.FadeInOut("Reading LED_MODE failed", false);
                    }
                    else
                    {
                        LedMode led_mode = (LedMode)System.Convert.ToSByte(value);
                        CB_leds_legacy_1.Checked = (led_mode & LedMode.Motor) == LedMode.Motor;
                        CB_leds_legacy_2.Checked = (led_mode & LedMode.GPS) == LedMode.GPS;
                        CB_leds_legacy_3.Checked = (led_mode & LedMode.Aux) == LedMode.Aux;
                        CB_leds_legacy_4.Checked = (led_mode & LedMode.Beeper) == LedMode.Beeper;
                        CB_leds_legacy_5.Checked = (led_mode & LedMode.Voltage) == LedMode.Voltage;
                        CB_leds_legacy_6.Checked = (led_mode & LedMode.Nav_blink_motor) == LedMode.Nav_blink_motor;
                        CB_leds_legacy_7.Checked = (led_mode & LedMode.Nav_blink_GPS) == LedMode.Nav_blink_GPS;
                        Setup.flashMessage.FadeInOut("Reading LED_MODE succeed", true);
                    }
                }
                catch
                {
                    Setup.flashMessage.FadeInOut("Reading LED_MODE failed", false);
                }

            }
        }

        private void B_beeper_write_Click(object sender, EventArgs e)
        {
            try
            {
                BeeperMode beeper_mode =
                    (CB_beeper_0.Checked ? BeeperMode.Startup : 0) |
                    (CB_beeper_1.Checked ? BeeperMode.Arm : 0) |
                    (CB_beeper_2.Checked ? BeeperMode.Disarm : 0) |
                    (CB_beeper_3.Checked ? BeeperMode.Armed : 0) |
                    (CB_beeper_4.Checked ? BeeperMode.GPS : 0) |
                    (CB_beeper_5.Checked ? BeeperMode.Voltage : 0) |
                    (CB_beeper_6.Checked ? BeeperMode.CH7 : 0) |
                    (CB_beeper_7.Checked ? BeeperMode.Landing : 0);
                if (MainV2.comPort.setParam("BEEPER_MODE", (byte)beeper_mode))
                {
                    Setup.flashMessage.FadeInOut("Writing BEEPER_MODE succeed", true);
                }
                else
                {
                    Setup.flashMessage.FadeInOut("Writing BEEPER_MODE failed", false);
                }
            }
            catch
            {
                Setup.flashMessage.FadeInOut("Writing BEEPER_MODE failed", false);
            }
        }

        private void B_leds_write_Click(object sender, EventArgs e)
        {
            //write LED_STYLE
            try
            {
                LedStyle led_style = (LedStyle)Enum.Parse(typeof(LedStyle), CMB_leds_style.SelectedItem.ToString());
                if (MainV2.comPort.setParam("LED_STYLE", (byte)led_style))
                {
                    Setup.flashMessage.FadeInOut("Writing LED_STYLE succeed", true);
                }
                else
                {
                    Setup.flashMessage.FadeInOut("Writing LED_STYLE failed", false);
                }
            }
            catch
            {
                Setup.flashMessage.FadeInOut("Writing LED_STYLE failed", false);
            }

            //write LED_MODE
            try
            {
                LedMode led_mode =
                    (CB_leds_legacy_1.Checked ? LedMode.Motor : 0) |
                    (CB_leds_legacy_2.Checked ? LedMode.GPS : 0) |
                    (CB_leds_legacy_3.Checked ? LedMode.Aux : 0) |
                    (CB_leds_legacy_4.Checked ? LedMode.Beeper : 0) |
                    (CB_leds_legacy_5.Checked ? LedMode.Voltage : 0) |
                    (CB_leds_legacy_6.Checked ? LedMode.Nav_blink_motor : 0) |
                    (CB_leds_legacy_7.Checked ? LedMode.Nav_blink_GPS : 0);
                if (MainV2.comPort.setParam("LED_MODE", (byte)led_mode))
                {
                    Setup.flashMessage.FadeInOut("Writing LED_MODE succeed", true);
                }
                else
                {
                    Setup.flashMessage.FadeInOut("Writing LED_MODE failed", false);
                }
            }
            catch
            {
                Setup.flashMessage.FadeInOut("Writing LED_MODE failed", false);
            }

        }

        private void CMB_leds_style_SelectedIndexChanged(object sender, EventArgs e)
        {
            LedStyle led_style = (LedStyle)Enum.Parse(typeof(LedStyle), ((ComboBox)sender).SelectedItem.ToString());
            switch (led_style)
	        {
                case LedStyle.None:
                    P_leds_mode_combo.Visible = false;
                    P_leds_mode_legacy.Visible = false;
                    break;
                case LedStyle.Legacy:
                    P_leds_mode_combo.Visible = false;
                    P_leds_mode_legacy.Visible = true;
                    break;
                case LedStyle.Features:
                    P_leds_mode_combo.Visible = true;
                    P_leds_mode_legacy.Visible = false;
                    break;
                case LedStyle.Disco:
                    P_leds_mode_combo.Visible = false;
                    P_leds_mode_legacy.Visible = false;
                    break;
            }
        }

        private void CMB_Format(object sender, ListControlConvertEventArgs e)
        {
            LedMode item = (LedMode)e.ListItem;
            e.Value = GetEnumDescription((LedMode)item);
        }

        public static string GetEnumDescription(Enum value)
        {
            FieldInfo fi = value.GetType().GetField(value.ToString());

            DescriptionAttribute[] attributes = (DescriptionAttribute[])fi.GetCustomAttributes(typeof(DescriptionAttribute), false);

            if (attributes != null && attributes.Length > 0)
            {
                return attributes[0].Description;
            }
            else
            {
                return value.ToString();
            }
        }
    }
}

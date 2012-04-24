using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Globalization;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Windows.Forms;
using DirectShowLib;
using ArdupilotMega.Controls.BackstageView;
using ArdupilotMega.Controls;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigPlanner : BackStageViewContentPanel
    {
        // AR todo: replicate this functionality
        private bool startup = false;

        public ConfigPlanner()
        {
            InitializeComponent();
        }


        private void BUT_videostart_Click(object sender, EventArgs e)
        {
            // stop first
            BUT_videostop_Click(sender, e);

            var bmp = (GCSViews.Configuration.GCSBitmapInfo)CMB_videoresolutions.SelectedItem;

            try
            {
                MainV2.cam = new WebCamService.Capture(CMB_videosources.SelectedIndex, bmp.Media);

                MainV2.cam.showhud = CHK_hudshow.Checked;

                MainV2.cam.Start();

                MainV2.config["video_options"] = CMB_videoresolutions.SelectedIndex;

                BUT_videostart.Enabled = false;
            }
            catch (Exception ex) { CustomMessageBox.Show("Camera Fail: " + ex.Message); }

        }

        private void BUT_videostop_Click(object sender, EventArgs e)
        {
            BUT_videostart.Enabled = true;
            if (MainV2.cam != null)
            {
                MainV2.cam.Dispose();
                MainV2.cam = null;
            }
        }

        private void CMB_videosources_MouseClick(object sender, MouseEventArgs e)
        {
            // the reason why i dont populate this list is because on linux/mac this call will fail.
            WebCamService.Capture capt = new WebCamService.Capture();

            List<string> devices = WebCamService.Capture.getDevices();

            CMB_videosources.DataSource = devices;

            capt.Dispose();
        }

        private void CMB_videosources_SelectedIndexChanged(object sender, EventArgs e)
        {
            int hr;
            int count;
            int size;
            object o;
            IBaseFilter capFilter = null;
            ICaptureGraphBuilder2 capGraph = null;
            AMMediaType media = null;
            VideoInfoHeader v;
            VideoStreamConfigCaps c;
            List<GCSViews.Configuration.GCSBitmapInfo> modes = new List<GCSViews.Configuration.GCSBitmapInfo>();

            // Get the ICaptureGraphBuilder2
            capGraph = (ICaptureGraphBuilder2)new CaptureGraphBuilder2();
            IFilterGraph2 m_FilterGraph = (IFilterGraph2)new FilterGraph();

            DsDevice[] capDevices;
            capDevices = DsDevice.GetDevicesOfCat(FilterCategory.VideoInputDevice);

            // Add the video device
            hr = m_FilterGraph.AddSourceFilterForMoniker(capDevices[CMB_videosources.SelectedIndex].Mon, null, "Video input", out capFilter);
            try
            {
                DsError.ThrowExceptionForHR(hr);
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show("Can not add video source\n" + ex.ToString());
                return;
            }

            // Find the stream config interface
            hr = capGraph.FindInterface(PinCategory.Capture, MediaType.Video, capFilter, typeof(IAMStreamConfig).GUID, out o);
            DsError.ThrowExceptionForHR(hr);

            IAMStreamConfig videoStreamConfig = o as IAMStreamConfig;
            if (videoStreamConfig == null)
            {
                throw new Exception("Failed to get IAMStreamConfig");
            }

            hr = videoStreamConfig.GetNumberOfCapabilities(out count, out size);
            DsError.ThrowExceptionForHR(hr);
            IntPtr TaskMemPointer = Marshal.AllocCoTaskMem(size);
            for (int i = 0; i < count; i++)
            {
                IntPtr ptr = IntPtr.Zero;

                hr = videoStreamConfig.GetStreamCaps(i, out media, TaskMemPointer);
                v = (VideoInfoHeader)Marshal.PtrToStructure(media.formatPtr, typeof(VideoInfoHeader));
                c = (VideoStreamConfigCaps)Marshal.PtrToStructure(TaskMemPointer, typeof(VideoStreamConfigCaps));
                modes.Add(new GCSViews.Configuration.GCSBitmapInfo(v.BmiHeader.Width, v.BmiHeader.Height, c.MaxFrameInterval, c.VideoStandard.ToString(), media));
            }
            Marshal.FreeCoTaskMem(TaskMemPointer);
            DsUtils.FreeAMMediaType(media);

            CMB_videoresolutions.DataSource = modes;

            if (MainV2.getConfig("video_options") != "" && CMB_videosources.Text != "")
            {
                CMB_videoresolutions.SelectedIndex = int.Parse(MainV2.getConfig("video_options"));
            }
        }

        private void CHK_hudshow_CheckedChanged(object sender, EventArgs e)
        {
            GCSViews.FlightData.myhud.hudon = CHK_hudshow.Checked;
        }

        private void CHK_enablespeech_CheckedChanged(object sender, EventArgs e)
        {
            MainV2.speechEnable = CHK_enablespeech.Checked;
            MainV2.config["speechenable"] = CHK_enablespeech.Checked;
            if (MainV2.speechEngine != null)
                MainV2.speechEngine.SpeakAsyncCancelAll();
        }

        private void CMB_language_SelectedIndexChanged(object sender, EventArgs e)
        {
            MainV2.instance.changelanguage((CultureInfo)CMB_language.SelectedItem);

#if !DEBUG
                MessageBox.Show("Please Restart the Planner");

                Application.Exit();
#endif
        }

        private void CMB_osdcolor_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            if (CMB_osdcolor.Text != "")
            {
                MainV2.config["hudcolor"] = CMB_osdcolor.Text;
                GCSViews.FlightData.myhud.hudcolor = Color.FromKnownColor((KnownColor)Enum.Parse(typeof(KnownColor), CMB_osdcolor.Text));
            }
        }

        private void CHK_speechwaypoint_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.config["speechwaypointenabled"] = ((CheckBox)sender).Checked.ToString();

            if (((CheckBox)sender).Checked)
            {
                string speechstring = "Heading to Waypoint {wpn}";
                if (MainV2.config["speechwaypoint"] != null)
                    speechstring = MainV2.config["speechwaypoint"].ToString();
                Common.InputBox("Notification", "What do you want it to say?", ref speechstring);
                MainV2.config["speechwaypoint"] = speechstring;
            }
        }

        private void CHK_speechmode_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.config["speechmodeenabled"] = ((CheckBox)sender).Checked.ToString();

            if (((CheckBox)sender).Checked)
            {
                string speechstring = "Mode changed to {mode}";
                if (MainV2.config["speechmode"] != null)
                    speechstring = MainV2.config["speechmode"].ToString();
                Common.InputBox("Notification", "What do you want it to say?", ref speechstring);
                MainV2.config["speechmode"] = speechstring;
            }
        }

        private void CHK_speechcustom_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.config["speechcustomenabled"] = ((CheckBox)sender).Checked.ToString();

            if (((CheckBox)sender).Checked)
            {
                string speechstring = "Heading to Waypoint {wpn}, altitude is {alt}, Ground speed is {gsp} ";
                if (MainV2.config["speechcustom"] != null)
                    speechstring = MainV2.config["speechcustom"].ToString();
                Common.InputBox("Notification", "What do you want it to say?", ref speechstring);
                MainV2.config["speechcustom"] = speechstring;
            }
        }

        private void BUT_rerequestparams_Click(object sender, EventArgs e)
        {
            if (!MainV2.comPort.BaseStream.IsOpen)
                return;
            ((MyButton)sender).Enabled = false;
            try
            {

                MainV2.comPort.getParamList();




            }
            catch { CustomMessageBox.Show("Error: getting param list"); }


            ((MyButton)sender).Enabled = true;
            startup = true;

            

            startup = false;
        }

        private void CHK_speechbattery_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.config["speechbatteryenabled"] = ((CheckBox)sender).Checked.ToString();

            if (((CheckBox)sender).Checked)
            {
                string speechstring = "WARNING, Battery at {batv} Volt";
                if (MainV2.config["speechbattery"] != null)
                    speechstring = MainV2.config["speechbattery"].ToString();
                Common.InputBox("Notification", "What do you want it to say?", ref speechstring);
                MainV2.config["speechbattery"] = speechstring;

                speechstring = "9.6";
                if (MainV2.config["speechbatteryvolt"] != null)
                    speechstring = MainV2.config["speechbatteryvolt"].ToString();
                Common.InputBox("Battery Level", "What Voltage do you want to warn at?", ref speechstring);
                MainV2.config["speechbatteryvolt"] = speechstring;

            }
        }

        private void BUT_Joystick_Click(object sender, EventArgs e)
        {
            Form joy = new JoystickSetup();
            ThemeManager.ApplyThemeTo(joy);
            joy.Show();
        }

        private void CMB_distunits_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.config["distunits"] = CMB_distunits.Text;
            MainV2.instance.ChangeUnits();
        }

        private void CMB_speedunits_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.config["speedunits"] = CMB_speedunits.Text;
            MainV2.instance.ChangeUnits();
        }

        private void CMB_rateattitude_SelectedIndexChanged(object sender, EventArgs e)
        {
            MainV2.config[((ComboBox)sender).Name] = ((ComboBox)sender).Text;
            MainV2.cs.rateattitude = byte.Parse(((ComboBox)sender).Text);
        }

        private void CMB_rateposition_SelectedIndexChanged(object sender, EventArgs e)
        {
            MainV2.config[((ComboBox)sender).Name] = ((ComboBox)sender).Text;
            MainV2.cs.rateposition = byte.Parse(((ComboBox)sender).Text);
        }

        private void CMB_ratestatus_SelectedIndexChanged(object sender, EventArgs e)
        {
            MainV2.config[((ComboBox)sender).Name] = ((ComboBox)sender).Text;
            MainV2.cs.ratestatus = byte.Parse(((ComboBox)sender).Text);
        }

        private void CMB_raterc_SelectedIndexChanged(object sender, EventArgs e)
        {
            MainV2.config[((ComboBox)sender).Name] = ((ComboBox)sender).Text;
            MainV2.cs.raterc = byte.Parse(((ComboBox)sender).Text);
        }

        private void CMB_ratesensors_SelectedIndexChanged(object sender, EventArgs e)
        {
            MainV2.config[((ComboBox)sender).Name] = ((ComboBox)sender).Text;
            MainV2.cs.ratesensors = byte.Parse(((ComboBox)sender).Text);
        }

        private void CHK_mavdebug_CheckedChanged(object sender, EventArgs e)
        {
            MainV2.comPort.debugmavlink = CHK_mavdebug.Checked;
        }

        private void CHK_resetapmonconnect_CheckedChanged(object sender, EventArgs e)
        {
            MainV2.config[((CheckBox)sender).Name] = ((CheckBox)sender).Checked.ToString();
        }

        private void CHK_speechaltwarning_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.config["speechaltenabled"] = ((CheckBox)sender).Checked.ToString();

            if (((CheckBox)sender).Checked)
            {
                string speechstring = "WARNING, low altitude {alt}";
                if (MainV2.config["speechalt"] != null)
                    speechstring = MainV2.config["speechalt"].ToString();
                Common.InputBox("Notification", "What do you want it to say?", ref speechstring);
                MainV2.config["speechalt"] = speechstring;

                speechstring = "2";
                if (MainV2.config["speechaltheight"] != null)
                    speechstring = MainV2.config["speechaltheight"].ToString();
                Common.InputBox("Min Alt", "What altitude do you want to warn at? (relative to home)", ref speechstring);
                MainV2.config["speechaltheight"] = (double.Parse(speechstring) / MainV2.cs.multiplierdist).ToString(); // save as m

            }
        }

        private void NUM_tracklength_ValueChanged(object sender, EventArgs e)
        {
            MainV2.config["NUM_tracklength"] = NUM_tracklength.Value.ToString();

        }

        private void CHK_loadwponconnect_CheckedChanged(object sender, EventArgs e)
        {
            MainV2.config["loadwpsonconnect"] = CHK_loadwponconnect.Checked.ToString();
        }

        private void CHK_GDIPlus_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            CustomMessageBox.Show("You need to restart the planner for this to take effect");
            MainV2.config["CHK_GDIPlus"] = CHK_GDIPlus.Checked.ToString();
        }

    }
}

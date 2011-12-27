using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.IO;
using System.IO.Ports;
using System.Text.RegularExpressions;
using System.Collections;
using System.Globalization;
using System.Threading;
using DirectShowLib;
using System.Runtime.InteropServices;

namespace ArdupilotMega.GCSViews
{
    public partial class Configuration : MyUserControl
    {
        Hashtable param = new Hashtable();
        Hashtable changes = new Hashtable();
        static Hashtable tooltips = new Hashtable();
        internal bool startup = true;
        List<CultureInfo> languages = new List<CultureInfo>();

        public class GCSBitmapInfo
        {
            public int Width { get; set; }
            public int Height { get; set; }
            public long Fps { get; set; }
            public string Standard { get; set; }
            public AMMediaType Media { get; set; }

            public GCSBitmapInfo(int width, int height, long fps, string standard, AMMediaType media)
            {
                Width = width;
                Height = height;
                Fps = fps;
                Standard = standard;
                Media = media;
            }

            public override string ToString()
            {
                return Width.ToString() + " x " + Height.ToString() + String.Format(" {0:0.00} fps ", 10000000.0 / Fps) + Standard;
            }
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

        public Configuration()
        {
            InitializeComponent();

            //this.Width = this.Parent.Width;
            //this.Height = this.Parent.Height;

            // fix for dup name
            //XTRK_ANGLE_CD1.Name = "XTRK_ANGLE_CD";
            XTRK_GAIN_SC1.Name = "XTRK_GAIN_SC";
        }

        private void Configuration_Load(object sender, EventArgs e)
        {
            startup = true;

            // enable disable relevbant hardware tabs
            if (MainV2.APMFirmware == MainV2.Firmwares.ArduPlane)
            {
                ConfigTabs.SelectedIndex = 0;
                TabAPM2.Enabled = true;
                TabAC2.Enabled = false;
            }
            else
            {
                ConfigTabs.SelectedIndex = 1;
                TabAPM2.Enabled = false;
                TabAC2.Enabled = true;
            }

            // read tooltips
            if (tooltips.Count == 0)
                readToolTips();

            // prefill all fields
            param = MainV2.comPort.param;
            processToScreen();



            // setup up camera button states
            if (MainV2.cam != null)
            {
                BUT_videostart.Enabled = false;
                CHK_hudshow.Checked = GCSViews.FlightData.myhud.hudon;
            }
            else
            {
                BUT_videostart.Enabled = true;
            }

            // setup speech states
            if (MainV2.config["speechenable"] != null)
                CHK_enablespeech.Checked = bool.Parse(MainV2.config["speechenable"].ToString());
            if (MainV2.config["speechwaypointenabled"] != null)
                CHK_speechwaypoint.Checked = bool.Parse(MainV2.config["speechwaypointenabled"].ToString());
            if (MainV2.config["speechmodeenabled"] != null)
                CHK_speechmode.Checked = bool.Parse(MainV2.config["speechmodeenabled"].ToString());
            if (MainV2.config["speechcustomenabled"] != null)
                CHK_speechcustom.Checked = bool.Parse(MainV2.config["speechcustomenabled"].ToString());
            if (MainV2.config["speechbatteryenabled"] != null)
                CHK_speechbattery.Checked = bool.Parse(MainV2.config["speechbatteryenabled"].ToString());
            if (MainV2.config["speechaltenabled"] != null)
                CHK_speechaltwarning.Checked = bool.Parse(MainV2.config["speechaltenabled"].ToString());

            // this can't fail because it set at startup
            NUM_tracklength.Value = int.Parse(MainV2.config["NUM_tracklength"].ToString());

            // get wps on connect
            if (MainV2.config["loadwpsonconnect"] != null)
                CHK_loadwponconnect.Checked = bool.Parse(MainV2.config["loadwpsonconnect"].ToString());

            // setup other config state
            if (MainV2.config["CHK_resetapmonconnect"] != null)
                CHK_resetapmonconnect.Checked = bool.Parse(MainV2.config["CHK_resetapmonconnect"].ToString());

            CMB_rateattitude.Text = MainV2.cs.rateattitude.ToString();
            CMB_rateposition.Text = MainV2.cs.rateposition.ToString();
            CMB_raterc.Text = MainV2.cs.raterc.ToString();
            CMB_ratestatus.Text = MainV2.cs.ratestatus.ToString();


            if (MainV2.config["CHK_GDIPlus"] != null)
                CHK_GDIPlus.Checked = bool.Parse(MainV2.config["CHK_GDIPlus"].ToString());

            //set hud color state
            string hudcolor = (string)MainV2.config["hudcolor"];

            CMB_osdcolor.DataSource = Enum.GetNames(typeof(KnownColor));
            if (hudcolor != null)
            {
                int index = CMB_osdcolor.Items.IndexOf(hudcolor);
                CMB_osdcolor.SelectedIndex = index;
            }
            else
            {
                int index = CMB_osdcolor.Items.IndexOf("White");
                CMB_osdcolor.SelectedIndex = index;
            }

            // set distance/speed unit states
            CMB_distunits.DataSource = Enum.GetNames(typeof(Common.distances));
            CMB_speedunits.DataSource = Enum.GetNames(typeof(Common.speeds));
            if (MainV2.config["distunits"] != null)
                CMB_distunits.Text = MainV2.config["distunits"].ToString();
            if (MainV2.config["speedunits"] != null)
                CMB_speedunits.Text = MainV2.config["speedunits"].ToString();

            // setup language selection
            CultureInfo ci = null;
            foreach (string name in new string[] { "en-US", "zh-Hans", "ru-RU" })
            {
                ci = MainV2.getcultureinfo(name);
                if (ci != null)
                    languages.Add(ci);
            }

            CMB_language.DisplayMember = "DisplayName";
            CMB_language.DataSource = languages;
            bool match = false;
            for (int i = 0; i < languages.Count && !match; i++)
            {
                ci = Thread.CurrentThread.CurrentUICulture;
                while (!ci.Equals(CultureInfo.InvariantCulture))
                {
                    if (ci.Equals(languages[i]))
                    {
                        CMB_language.SelectedIndex = i;
                        match = true;
                        break;
                    }
                    ci = ci.Parent;
                }
            }
            CMB_language.SelectedIndexChanged += CMB_language_SelectedIndexChanged;

            startup = false;
        }

        string[] genpids()
        {
            List<string> temp = new List<string>();
            // pids
            for (double a = 8.00; a >= -0.001; a -= 0.001)
            {
                temp.Add(a.ToString("0.0##"));
            }

            // Nav angles + throttle
            for (int a = 100; a >= -90; a -= 1)
            {
                temp.Add(a.ToString("0.0#"));
            }

            // imax
            for (int a = 8000; a >= -4500; a -= 100)
            {
                temp.Add(a.ToString("0.0#"));
            }

            // FS pulse
            for (int a = 1200; a >= 900; a -= 1)
            {
                temp.Add(a.ToString("0.0#"));
            }

            return temp.ToArray();
        }

        void readToolTips()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Configuration));

            string data = resources.GetString("MAVParam");

            if (data == null)
                return;

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
            Params.Rows.Clear();

            disableNumericUpDownControls(TabAC2);
            disableNumericUpDownControls(TabAPM2);

            // process hashdefines and update display
            foreach (string value in param.Keys)
            {
                if (value == null || value == "")
                    continue;

                //System.Diagnostics.Debug.WriteLine("Doing: " + value);

                Params.Rows.Add();
                Params.Rows[Params.RowCount - 1].Cells[Command.Index].Value = value;
                Params.Rows[Params.RowCount - 1].Cells[Value.Index].Value = ((float)param[value]).ToString("0.###");
                try
                {
                    if (tooltips[value] != null)
                    {
                        Params.Rows[Params.RowCount - 1].Cells[Command.Index].ToolTipText = ((paramsettings)tooltips[value]).desc;
                        //Params.Rows[Params.RowCount - 1].Cells[RawValue.Index].ToolTipText = ((paramsettings)tooltips[value]).desc;
                        Params.Rows[Params.RowCount - 1].Cells[Value.Index].ToolTipText = ((paramsettings)tooltips[value]).desc;

                        //Params.Rows[Params.RowCount - 1].Cells[Default.Index].Value = ((paramsettings)tooltips[value]).normalvalue;
                        //Params.Rows[Params.RowCount - 1].Cells[mavScale.Index].Value = ((paramsettings)tooltips[value]).scale;
                        //Params.Rows[Params.RowCount - 1].Cells[Value.Index].Value = float.Parse(Params.Rows[Params.RowCount - 1].Cells[RawValue.Index].Value.ToString()) / float.Parse(Params.Rows[Params.RowCount - 1].Cells[mavScale.Index].Value.ToString());
                    }
                }
                catch { }

                string name = value;
                Control[] text = this.Controls.Find(name, true);
                foreach (Control ctl in text)
                {
                    try
                    {
                        if (ctl.GetType() == typeof(NumericUpDown))
                        {

                            NumericUpDown thisctl = ((NumericUpDown)ctl);
                            thisctl.Maximum = 9000;
                            thisctl.Minimum = -9000;
                            thisctl.Value = (decimal)(float)param[value];
                            thisctl.Increment = (decimal)0.001;
                            if (thisctl.Name.EndsWith("_P") || thisctl.Name.EndsWith("_I") || thisctl.Name.EndsWith("_D") || thisctl.Value == 0 || thisctl.Value.ToString("0.###", new System.Globalization.CultureInfo("en-US")).Contains("."))
                            {
                                thisctl.DecimalPlaces = 3;
                            }
                            else
                            {
                                thisctl.Increment = (decimal)1;
                                thisctl.DecimalPlaces = 1;
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

                            thisctl.SelectedIndex = (int)(float)param[value];

                            thisctl.Validated += new EventHandler(ComboBox_Validated);
                        }
                    }
                    catch { }

                }
                if (text.Length == 0)
                {
                    Console.WriteLine(name + " not found");
                }

            }
            Params.Sort(Params.Columns[0], ListSortDirection.Ascending);
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
            float value = 0;
            string name = ((Control)sender).Name;

            // do domainupdown state check
            try
            {
                if (sender.GetType() == typeof(NumericUpDown))
                {
                    value = float.Parse(((Control)sender).Text);
                    changes[name] = value;
                }
                else if (sender.GetType() == typeof(ComboBox))
                {
                    value = ((ComboBox)sender).SelectedIndex;
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
                            foreach (DataGridViewRow row in Params.Rows)
                            {
                                if (row.Cells[0].Value.ToString() == newname)
                                {
                                    row.Cells[1].Value = float.Parse(((Control)sender).Text);
                                    break;
                                }
                            }
                        }
                        else if (name.Contains("_PIT_"))
                        {
                            string newname = name.Replace("_PIT_", "_RLL_");
                            foreach (DataGridViewRow row in Params.Rows)
                            {
                                if (row.Cells[0].Value.ToString() == newname)
                                {
                                    row.Cells[1].Value = float.Parse(((Control)sender).Text);
                                    break;
                                }
                            }
                        }
                    }
                }
                // keep nav_lat and nav_lon paired
                if (name.Contains("NAV_LAT_"))
                {
                    string newname = name.Replace("NAV_LAT_", "NAV_LON_");
                    foreach (DataGridViewRow row in Params.Rows)
                    {
                        if (row.Cells[0].Value.ToString() == newname)
                        {
                            row.Cells[1].Value = float.Parse(((Control)sender).Text);
                            break;
                        }
                    }
                }
                // keep nav_lat and nav_lon paired
                if (name.Contains("HLD_LAT_"))
                {
                    string newname = name.Replace("HLD_LAT_", "HLD_LON_");
                    foreach (DataGridViewRow row in Params.Rows)
                    {
                        if (row.Cells[0].Value.ToString() == newname)
                        {
                            row.Cells[1].Value = float.Parse(((Control)sender).Text);
                            break;
                        }
                    }
                }
            }
            catch { }

            try
            {
                // set param table as well
                foreach (DataGridViewRow row in Params.Rows)
                {
                    if (row.Cells[0].Value.ToString() == name)
                    {
                        if (sender.GetType() == typeof(NumericUpDown))
                        {
                            row.Cells[1].Value = float.Parse(((Control)sender).Text);
                        }
                        else if (sender.GetType() == typeof(ComboBox))
                        {
                            row.Cells[1].Value = ((ComboBox)sender).SelectedIndex;
                        }
                        break;
                    }
                }
            }
            catch { }
            //((Control)sender).Focus();
        }

        void Params_CellValueChanged(object sender, DataGridViewCellEventArgs e)
        {
            if (e.RowIndex == -1 || e.ColumnIndex == -1 || startup == true || e.ColumnIndex != 1)
                return;
            try
            {
                if (Params[Command.Index, e.RowIndex].Value.ToString().EndsWith("_REV") && (Params[Command.Index, e.RowIndex].Value.ToString().StartsWith("RC") || Params[Command.Index, e.RowIndex].Value.ToString().StartsWith("HS")))
                {
                    if (Params[e.ColumnIndex, e.RowIndex].Value.ToString() == "0")
                        Params[e.ColumnIndex, e.RowIndex].Value = "-1";
                }

                Params[e.ColumnIndex, e.RowIndex].Style.BackColor = Color.Green;
                changes[Params[0, e.RowIndex].Value] = float.Parse(Params[e.ColumnIndex, e.RowIndex].Value.ToString());
            }
            catch (Exception)
            {
                Params[e.ColumnIndex, e.RowIndex].Style.BackColor = Color.Red;
            }

            // set control as well
            Control[] text = this.Controls.Find(Params[0, e.RowIndex].Value.ToString(), true);
            try
            {
                if (text.Length > 0)
                {
                        if (sender.GetType() == typeof(NumericUpDown))
                        {
                            decimal option = (decimal)(float.Parse(Params[e.ColumnIndex, e.RowIndex].Value.ToString()));
                            ((NumericUpDown)text[0]).Value = option;
                            ((NumericUpDown)text[0]).BackColor = Color.Green;
                        }
                        else if (sender.GetType() == typeof(ComboBox))
                        {
                            int option = (int)(float.Parse(Params[e.ColumnIndex, e.RowIndex].Value.ToString()));
                            ((ComboBox)text[0]).SelectedIndex = option;
                            ((ComboBox)text[0]).BackColor = Color.Green;
                        }
                }
            }
            catch { ((Control)text[0]).BackColor = Color.Red; }

            Params.Focus();
        }        

        private void BUT_load_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.AddExtension = true;
            ofd.DefaultExt = ".param";
            ofd.RestoreDirectory = true;
            ofd.Filter = "Param List|*.param;*.parm";
            DialogResult dr = ofd.ShowDialog();
            if (dr == DialogResult.OK)
            {
                StreamReader sr = new StreamReader(ofd.OpenFile());
                while (!sr.EndOfStream)
                {
                    string line = sr.ReadLine();

                    if (line.Contains("NOTE:"))
                        MessageBox.Show(line, "Saved Note");

                    int index = line.IndexOf(',');

                    int index2 = line.IndexOf(',', index + 1);

                    if (index == -1)
                        continue;

                    if (index2 != -1)
                        line = line.Replace(',','.');

                    string name = line.Substring(0, index);
                    float value = float.Parse(line.Substring(index + 1), new System.Globalization.CultureInfo("en-US"));

                    MAVLink.modifyParamForDisplay(true,name,ref value);

                    // set param table as well
                    foreach (DataGridViewRow row in Params.Rows)
                    {
                        if (name == "SYSID_SW_MREV")
                            continue;
                        if (name == "WP_TOTAL")
                            continue;
                        if (name == "CMD_TOTAL")
                            continue;
                        if (name == "FENCE_TOTAL")
                            continue;
                        if (name == "SYS_NUM_RESETS")
                            continue;
                        if (name == "ARSPD_OFFSET")
                            continue;
                        if (name == "GND_ABS_PRESS")
                            continue;
                        if (name == "GND_TEMP")
                            continue;
                        if (name == "CMD_INDEX")
                            continue;
                        if (name == "LOG_LASTFILE")
                            continue;
                        if (row.Cells[0].Value.ToString() == name)
                        {
                            if (row.Cells[1].Value.ToString() != value.ToString())
                                row.Cells[1].Value = value;
                            break;
                        }
                    }
                }
                sr.Close();
            }
        }

        private void BUT_save_Click(object sender, EventArgs e)
        {
            SaveFileDialog sfd = new SaveFileDialog();
            sfd.AddExtension = true;
            sfd.DefaultExt = ".param";
            sfd.RestoreDirectory = true;
            sfd.Filter = "Param List|*.param;*.parm";
            DialogResult dr = sfd.ShowDialog();
            if (dr == DialogResult.OK)
            {
                StreamWriter sw = new StreamWriter(sfd.OpenFile());
                string input = DateTime.Now + " Frame : + | Arducopter Kit | Kit motors";
                if (MainV2.APMFirmware == MainV2.Firmwares.ArduPlane)
                {
                    input = DateTime.Now + " Plane: Skywalker";
                }
                Common.InputBox("Custom Note", "Enter your Notes/Frame Type etc", ref input);
                if (input != "")
                    sw.WriteLine("NOTE: " + input.Replace(',', '|'));
                foreach (DataGridViewRow row in Params.Rows)
                {
                    float value = float.Parse(row.Cells[1].Value.ToString());

                    MAVLink.modifyParamForDisplay(false, row.Cells[0].Value.ToString(), ref value);

                    sw.WriteLine(row.Cells[0].Value.ToString() + "," + value.ToString(new System.Globalization.CultureInfo("en-US")));
                }
                sw.Close();
            }
        }

        private void BUT_writePIDS_Click(object sender, EventArgs e)
        {

            Hashtable temp = (Hashtable)changes.Clone();

            foreach (string value in temp.Keys)
            {
                try
                {
                    MainV2.comPort.setParam(value, (float)changes[value]);

                    try
                    {
                        // set control as well
                        Control[] text = this.Controls.Find(value, true);
                        if (text.Length > 0)
                        {
                            ((NumericUpDown)text[0]).BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                        }
                    }
                    catch { }

                    try
                    {
                        // set param table as well
                        foreach (DataGridViewRow row in Params.Rows)
                        {
                            if (row.Cells[0].Value.ToString() == value)
                            {
                                row.Cells[1].Style.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                                changes.Remove(value);
                                break;
                            }
                        }
                    }
                    catch { }

                }
                catch { MessageBox.Show("Set " + value + " Failed"); }
            }
        }

        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        private void Planner_TabIndexChanged(object sender, EventArgs e)
        {
            if (ConfigTabs.SelectedTab == TabSetup)
            {
                if (!MainV2.comPort.BaseStream.IsOpen)
                {
                    MessageBox.Show("Please Connect First");
                    ConfigTabs.SelectedIndex = 0;
                }
                else
                {

                    Setup.Setup temp = new Setup.Setup();

                    temp.Configuration = this;

                    MainV2.fixtheme(temp);

                    temp.ShowDialog();

                    startup = true;
                    processToScreen();
                    startup = false;
                }
            }
        }

        private void BUT_videostart_Click(object sender, EventArgs e)
        {
            // stop first
            BUT_videostop_Click(sender, e);

            GCSBitmapInfo bmp = (GCSBitmapInfo)CMB_videoresolutions.SelectedItem;

            try
            {
                MainV2.cam = new WebCamService.Capture(CMB_videosources.SelectedIndex, bmp.Media);

                MainV2.cam.showhud = CHK_hudshow.Checked;

                MainV2.cam.Start();

                BUT_videostart.Enabled = false;
            }
            catch (Exception ex) { MessageBox.Show("Camera Fail: " + ex.Message); }

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
            List<GCSBitmapInfo> modes = new List<GCSBitmapInfo>();

            // Get the ICaptureGraphBuilder2
            capGraph = (ICaptureGraphBuilder2)new CaptureGraphBuilder2();
            IFilterGraph2 m_FilterGraph = (IFilterGraph2)new FilterGraph();

            DsDevice[] capDevices;
            capDevices = DsDevice.GetDevicesOfCat(FilterCategory.VideoInputDevice);

            // Add the video device
            hr = m_FilterGraph.AddSourceFilterForMoniker(capDevices[CMB_videosources.SelectedIndex].Mon, null, "Video input", out capFilter);
            DsError.ThrowExceptionForHR(hr);

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
                modes.Add(new GCSBitmapInfo(v.BmiHeader.Width, v.BmiHeader.Height, c.MaxFrameInterval, c.VideoStandard.ToString(), media));
            }
            Marshal.FreeCoTaskMem(TaskMemPointer);
            DsUtils.FreeAMMediaType(media);

            CMB_videoresolutions.DataSource = modes;
        }

        private void CHK_hudshow_CheckedChanged(object sender, EventArgs e)
        {
            GCSViews.FlightData.myhud.hudon = CHK_hudshow.Checked;
        }

        private void CHK_enablespeech_CheckedChanged(object sender, EventArgs e)
        {
            MainV2.speechenable = CHK_enablespeech.Checked;
            MainV2.config["speechenable"] = CHK_enablespeech.Checked;
            if (MainV2.talk != null)
                MainV2.talk.SpeakAsyncCancelAll();
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
            catch { MessageBox.Show("Error: getting param list"); }

            ((MyButton)sender).Enabled = true;
            startup = true;
            Configuration_Load(null, null);
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
            MainV2.fixtheme(joy);
            joy.Show();
        }

        private void CMB_distunits_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.config["distunits"] = CMB_distunits.Text;
            MainV2.instance.changeunits();
        }

        private void CMB_speedunits_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MainV2.config["speedunits"] = CMB_speedunits.Text;
            MainV2.instance.changeunits();
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

        private void CMB_osdcolor_DrawItem(object sender, DrawItemEventArgs e)
        {
            if (e.Index < 0)
                return;

            Graphics g = e.Graphics;
            Rectangle rect = e.Bounds;
            Brush brush = null;

            if ((e.State & DrawItemState.Selected) == 0)
                brush = new SolidBrush(CMB_osdcolor.BackColor);
            else
                brush = SystemBrushes.Highlight;

            g.FillRectangle(brush, rect);

            brush = new SolidBrush(Color.FromName((string)CMB_osdcolor.Items[e.Index]));

            g.FillRectangle(brush, rect.X + 2, rect.Y + 2, 30, rect.Height - 4);
            g.DrawRectangle(Pens.Black, rect.X + 2, rect.Y + 2, 30, rect.Height - 4);

            if ((e.State & DrawItemState.Selected) == 0)
                brush = new SolidBrush(CMB_osdcolor.ForeColor);
            else
                brush = SystemBrushes.HighlightText;
            g.DrawString(CMB_osdcolor.Items[e.Index].ToString(),
                CMB_osdcolor.Font, brush, rect.X + 35, rect.Top + rect.Height - CMB_osdcolor.Font.Height);
        }

        private void NUM_tracklength_ValueChanged(object sender, EventArgs e)
        {
            MainV2.config["NUM_tracklength"] = NUM_tracklength.Value.ToString();

        }

        private void CHK_loadwponconnect_CheckedChanged(object sender, EventArgs e)
        {
            MainV2.config["loadwpsonconnect"] = CHK_loadwponconnect.Checked.ToString();
        }

        private void BUT_compare_Click(object sender, EventArgs e)
        {
            Hashtable param2 = new Hashtable();

            OpenFileDialog ofd = new OpenFileDialog();
            ofd.AddExtension = true;
            ofd.DefaultExt = ".param";
            ofd.RestoreDirectory = true;
            ofd.Filter = "Param List|*.param;*.parm";
            DialogResult dr = ofd.ShowDialog();
            if (dr == DialogResult.OK)
            {
                StreamReader sr = new StreamReader(ofd.OpenFile());
                while (!sr.EndOfStream)
                {
                    string line = sr.ReadLine();

                    if (line.Contains("NOTE:"))
                        MessageBox.Show(line, "Saved Note");

                    int index = line.IndexOf(',');

                    if (index == -1)
                        continue;

                    string name = line.Substring(0, index);
                    float value = float.Parse(line.Substring(index + 1), new System.Globalization.CultureInfo("en-US"));

                    MAVLink.modifyParamForDisplay(true, name, ref value);

                    if (name == "SYSID_SW_MREV")
                        continue;
                    if (name == "WP_TOTAL")
                        continue;
                    if (name == "CMD_TOTAL")
                        continue;

                    param2[name] = value;
                }
                sr.Close();

                ParamCompare temp = new ParamCompare(this, param, param2);
                MainV2.fixtheme(temp);
                temp.ShowDialog();
            }
        }

        private void CHK_GDIPlus_CheckedChanged(object sender, EventArgs e)
        {
            if (startup)
                return;
            MessageBox.Show("You need to restart the planner for this to take effect");
            MainV2.config["CHK_GDIPlus"] = CHK_GDIPlus.Checked.ToString();
        }

        private void CHK_lockrollpitch_CheckedChanged(object sender, EventArgs e)
        {

        }
    }
}
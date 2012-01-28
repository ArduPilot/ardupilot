using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;
using System.Xml;
using System.Collections;
using System.Net;
using System.Text.RegularExpressions;
using System.Web;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Speech.Synthesis;
using System.Globalization;
using System.Threading;
using System.Net.Sockets;
using IronPython.Hosting;

namespace ArdupilotMega
{
    public partial class MainV2 : Form
    {
        [DllImport("user32.dll")]
        public static extern int FindWindow(string szClass, string szTitle);
        [DllImport("user32.dll")]
        public static extern int ShowWindow(int Handle, int showState);

        const int SW_SHOWNORMAL = 1;
        const int SW_HIDE = 0;

        /// <summary>
        /// Home Location
        /// </summary>
        public static PointLatLngAlt HomeLocation = new PointLatLngAlt();

        public static MAVLink comPort = new MAVLink();
        public static string comportname = "";
        public static Hashtable config = new Hashtable();
        public static bool givecomport = false;
        public static Firmwares APMFirmware = Firmwares.ArduPlane;
        public static bool MONO = false;

        public static bool speechenable = false;
        public static SpeechSynthesizer talk = new SpeechSynthesizer();

        public static Joystick joystick = null;
        DateTime lastjoystick = DateTime.Now;

        public static WebCamService.Capture cam = null;

        public static CurrentState cs = new CurrentState();

        bool serialthread = false;

        TcpListener listener;

        DateTime heatbeatsend = DateTime.Now;

        public static List<System.Threading.Thread> threads = new List<System.Threading.Thread>();
        public static MainV2 instance = null;

        /*
         * "PITCH_KP",
"PITCH_KI",
"PITCH_LIM",

         */

        public enum Firmwares
        {
            ArduPlane,
            ArduCopter2,
        }

        GCSViews.FlightData FlightData;
        GCSViews.FlightPlanner FlightPlanner;
        GCSViews.Configuration Configuration;
        GCSViews.Simulation Simulation;
        GCSViews.Firmware Firmware;
        GCSViews.Terminal Terminal;

        public MainV2()
        {
            Form splash = new Splash();
            splash.Show();

            string strVersion = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version.ToString();
            strVersion = "";
            splash.Text = "APM Planner " + Application.ProductVersion + " " + strVersion + " By Michael Oborne";

            splash.Refresh();

            Application.DoEvents();

            instance = this;

            InitializeComponent();

            srtm.datadirectory = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "srtm";

            var t = Type.GetType("Mono.Runtime");
            MONO = (t != null);

            //talk.SpeakAsync("Welcome to APM Planner");

            MyRenderer.currentpressed = MenuFlightData;

            MainMenu.Renderer = new MyRenderer();

            List<object> list = new List<object>();
            foreach (object obj in Enum.GetValues(typeof(Firmwares)))
            {
                TOOL_APMFirmware.Items.Add(obj);
            }

            if (TOOL_APMFirmware.Items.Count > 0)
                TOOL_APMFirmware.SelectedIndex = 0;

            this.Text = splash.Text;

            comPort.BaseStream.BaudRate = 115200;

            CMB_serialport.Items.AddRange(GetPortNames());
            CMB_serialport.Items.Add("TCP");
            CMB_serialport.Items.Add("UDP");
            if (CMB_serialport.Items.Count > 0)
            {
                CMB_serialport.SelectedIndex = 0;
                CMB_baudrate.SelectedIndex = 7;
            }

            splash.Refresh();
            Application.DoEvents();

            // set this before we reset it
            MainV2.config["NUM_tracklength"] = "200";

            xmlconfig(false);

            if (config.ContainsKey("language") && !string.IsNullOrEmpty((string)config["language"]))
                changelanguage(getcultureinfo((string)config["language"]));

            if (!MONO) // windows only
            {
                if (MainV2.config["showconsole"] != null && MainV2.config["showconsole"].ToString() == "True")
                {
                }
                else
                {
                    int win = FindWindow("ConsoleWindowClass", null);
                    ShowWindow(win, SW_HIDE); // hide window
                }
            }

            try
            {
                FlightData = new GCSViews.FlightData();
                FlightPlanner = new GCSViews.FlightPlanner();
                //Configuration = new GCSViews.Configuration();
                Simulation = new GCSViews.Simulation();
                Firmware = new GCSViews.Firmware();
                //Terminal = new GCSViews.Terminal();

                // preload
                Python.CreateEngine();
            }
            catch (Exception e) { MessageBox.Show("A Major error has occured : " + e.ToString()); this.Close(); }

            if (MainV2.config["CHK_GDIPlus"] != null)
                GCSViews.FlightData.myhud.UseOpenGL = !bool.Parse(MainV2.config["CHK_GDIPlus"].ToString());

            changeunits();

            try
            {
                if (config["MainLocX"] != null && config["MainLocY"] != null)
                {
                    this.StartPosition = FormStartPosition.Manual;
                    Point startpos = new Point(int.Parse(config["MainLocX"].ToString()), int.Parse(config["MainLocY"].ToString()));
                    this.Location = startpos;
                }

                if (config["MainHeight"] != null)
                    this.Height = int.Parse(config["MainHeight"].ToString());
                if (config["MainWidth"] != null)
                    this.Width = int.Parse(config["MainWidth"].ToString());
                if (config["MainMaximised"] != null)
                    this.WindowState = (FormWindowState)Enum.Parse(typeof(FormWindowState), config["MainMaximised"].ToString());

                if (config["CMB_rateattitude"] != null)
                    MainV2.cs.rateattitude = byte.Parse(config["CMB_rateattitude"].ToString());
                if (config["CMB_rateattitude"] != null)
                    MainV2.cs.rateposition = byte.Parse(config["CMB_rateposition"].ToString());
                if (config["CMB_rateattitude"] != null)
                    MainV2.cs.ratestatus = byte.Parse(config["CMB_ratestatus"].ToString());
                if (config["CMB_rateattitude"] != null)
                    MainV2.cs.raterc = byte.Parse(config["CMB_raterc"].ToString());

                if (config["speechenable"] != null)
                    MainV2.speechenable = bool.Parse(config["speechenable"].ToString());


                try
                {
                    if (config["TXT_homelat"] != null)
                        HomeLocation.Lat = double.Parse(config["TXT_homelat"].ToString());

                    if (config["TXT_homelng"] != null)
                        HomeLocation.Lng = double.Parse(config["TXT_homelng"].ToString());

                    if (config["TXT_homealt"] != null)
                        HomeLocation.Alt = double.Parse(config["TXT_homealt"].ToString());
                }
                catch { }

            }
            catch { }

            if (cs.rateattitude == 0) // initilised to 10, configured above from save
            {
                MessageBox.Show("NOTE: your attitude rate is 0, the hud will not work\nChange in Configuration > Planner > Telemetry Rates");
            }


            //System.Threading.Thread.Sleep(2000);

            // make sure new enough .net framework is installed
            if (!MONO)
            {
                Microsoft.Win32.RegistryKey installed_versions = Microsoft.Win32.Registry.LocalMachine.OpenSubKey(@"SOFTWARE\Microsoft\NET Framework Setup\NDP");
                string[] version_names = installed_versions.GetSubKeyNames();
                //version names start with 'v', eg, 'v3.5' which needs to be trimmed off before conversion
                double Framework = Convert.ToDouble(version_names[version_names.Length - 1].Remove(0, 1), CultureInfo.InvariantCulture);
                int SP = Convert.ToInt32(installed_versions.OpenSubKey(version_names[version_names.Length - 1]).GetValue("SP", 0));

                if (Framework < 4.0)
                {
                    MessageBox.Show("This program requires .NET Framework 4.0 +. You currently have " + Framework);
                }
            }

            Application.DoEvents();


            splash.Close();
        }

        private string[] GetPortNames()
        {
            string[] devs = new string[0];

            if (MONO)
            {
                if (Directory.Exists("/dev/"))
                    devs = Directory.GetFiles("/dev/", "*ACM*");
            }

            string[] ports = SerialPort.GetPortNames();

            string[] all = new string[devs.Length + ports.Length];

            devs.CopyTo(all, 0);
            ports.CopyTo(all, devs.Length);

            return all;
        }

        internal void ScreenShot()
        {
            Rectangle bounds = Screen.GetBounds(Point.Empty);
            using (Bitmap bitmap = new Bitmap(bounds.Width, bounds.Height))
            {
                using (Graphics g = Graphics.FromImage(bitmap))
                {
                    g.CopyFromScreen(Point.Empty, Point.Empty, bounds.Size);
                }
                string name = "ss" + DateTime.Now.ToString("hhmmss") + ".jpg";
                bitmap.Save(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + name, System.Drawing.Imaging.ImageFormat.Jpeg);
                MessageBox.Show("Screenshot saved to " + name);
            }

        }

        private void CMB_serialport_Click(object sender, EventArgs e)
        {
            string oldport = CMB_serialport.Text;
            CMB_serialport.Items.Clear();
            CMB_serialport.Items.AddRange(GetPortNames());
            CMB_serialport.Items.Add("TCP");
            CMB_serialport.Items.Add("UDP");
            if (CMB_serialport.Items.Contains(oldport))
                CMB_serialport.Text = oldport;
        }

        public static void fixtheme(Control temp)
        {
            fixtheme(temp, 0);
        }

        public static void fixtheme(Control temp, int level)
        {
            if (level == 0)
            {
                temp.BackColor = Color.FromArgb(0x26, 0x27, 0x28);
                temp.ForeColor = Color.White;// Color.FromArgb(0xe6, 0xe8, 0xea);
            }
            //Console.WriteLine(temp.GetType());

            //temp.Font = new Font("Lucida Console", 8.25f);

            foreach (Control ctl in temp.Controls)
            {
                if (((Type)ctl.GetType()) == typeof(System.Windows.Forms.Button))
                {
                    ctl.ForeColor = Color.Black;
                    System.Windows.Forms.Button but = (System.Windows.Forms.Button)ctl;
                }
                else if (((Type)ctl.GetType()) == typeof(TextBox))
                {
                    ctl.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                    ctl.ForeColor = Color.White;// Color.FromArgb(0xe6, 0xe8, 0xea);
                    TextBox txt = (TextBox)ctl;
                    txt.BorderStyle = BorderStyle.None;
                }
                else if (((Type)ctl.GetType()) == typeof(DomainUpDown))
                {
                    ctl.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                    ctl.ForeColor = Color.White;// Color.FromArgb(0xe6, 0xe8, 0xea);
                    DomainUpDown txt = (DomainUpDown)ctl;
                    txt.BorderStyle = BorderStyle.None;
                }
                else if (((Type)ctl.GetType()) == typeof(GroupBox))
                {
                    ctl.BackColor = Color.FromArgb(0x26, 0x27, 0x28);
                    ctl.ForeColor = Color.White;// Color.FromArgb(0xe6, 0xe8, 0xea);
                }
                else if (((Type)ctl.GetType()) == typeof(ZedGraph.ZedGraphControl))
                {
                    ZedGraph.ZedGraphControl zg1 = (ZedGraph.ZedGraphControl)ctl;
                    zg1.GraphPane.Chart.Fill = new ZedGraph.Fill(Color.FromArgb(0x1f, 0x1f, 0x20));
                    zg1.GraphPane.Fill = new ZedGraph.Fill(Color.FromArgb(0x37, 0x37, 0x38));

                    foreach (ZedGraph.LineItem li in zg1.GraphPane.CurveList)
                    {
                        li.Line.Width = 4;
                    }

                    zg1.GraphPane.Title.FontSpec.FontColor = Color.White;

                    zg1.GraphPane.XAxis.MajorTic.Color = Color.White;
                    zg1.GraphPane.XAxis.MinorTic.Color = Color.White;
                    zg1.GraphPane.YAxis.MajorTic.Color = Color.White;
                    zg1.GraphPane.YAxis.MinorTic.Color = Color.White;

                    zg1.GraphPane.XAxis.MajorGrid.Color = Color.White;
                    zg1.GraphPane.YAxis.MajorGrid.Color = Color.White;

                    zg1.GraphPane.YAxis.Scale.FontSpec.FontColor = Color.White;
                    zg1.GraphPane.YAxis.Title.FontSpec.FontColor = Color.White;

                    zg1.GraphPane.XAxis.Scale.FontSpec.FontColor = Color.White;
                    zg1.GraphPane.XAxis.Title.FontSpec.FontColor = Color.White;

                    zg1.GraphPane.Legend.Fill = new ZedGraph.Fill(Color.FromArgb(0x85, 0x84, 0x83));
                    zg1.GraphPane.Legend.FontSpec.FontColor = Color.White;
                }
                else if (((Type)ctl.GetType()) == typeof(BSE.Windows.Forms.Panel) || ((Type)ctl.GetType()) == typeof(System.Windows.Forms.SplitterPanel))
                {
                    ctl.BackColor = Color.FromArgb(0x26, 0x27, 0x28);
                    ctl.ForeColor = Color.White;// Color.FromArgb(0xe6, 0xe8, 0xea);
                }
                else if (((Type)ctl.GetType()) == typeof(Form))
                {
                    ctl.BackColor = Color.FromArgb(0x26, 0x27, 0x28);
                    ctl.ForeColor = Color.White;// Color.FromArgb(0xe6, 0xe8, 0xea);
                }
                else if (((Type)ctl.GetType()) == typeof(RichTextBox))
                {
                    ctl.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                    ctl.ForeColor = Color.White;
                    RichTextBox txtr = (RichTextBox)ctl;
                    txtr.BorderStyle = BorderStyle.None;
                }
                else if (((Type)ctl.GetType()) == typeof(CheckedListBox))
                {
                    ctl.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                    ctl.ForeColor = Color.White;
                    CheckedListBox txtr = (CheckedListBox)ctl;
                    txtr.BorderStyle = BorderStyle.None;
                }
                else if (((Type)ctl.GetType()) == typeof(TabPage))
                {
                    ctl.BackColor = Color.FromArgb(0x26, 0x27, 0x28);  //Color.FromArgb(0x43, 0x44, 0x45);
                    ctl.ForeColor = Color.White;
                    TabPage txtr = (TabPage)ctl;
                    txtr.BorderStyle = BorderStyle.None;
                }
                else if (((Type)ctl.GetType()) == typeof(TabControl))
                {
                    ctl.BackColor = Color.FromArgb(0x26, 0x27, 0x28);  //Color.FromArgb(0x43, 0x44, 0x45);
                    ctl.ForeColor = Color.White;
                    TabControl txtr = (TabControl)ctl;

                }
                else if (((Type)ctl.GetType()) == typeof(DataGridView))
                {
                    ctl.ForeColor = Color.White;
                    DataGridView dgv = (DataGridView)ctl;
                    dgv.EnableHeadersVisualStyles = false;
                    dgv.BorderStyle = BorderStyle.None;
                    dgv.BackgroundColor = Color.FromArgb(0x26, 0x27, 0x28);
                    DataGridViewCellStyle rs = new DataGridViewCellStyle();
                    rs.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                    rs.ForeColor = Color.White;
                    dgv.RowsDefaultCellStyle = rs;

                    DataGridViewCellStyle hs = new DataGridViewCellStyle(dgv.ColumnHeadersDefaultCellStyle);
                    hs.BackColor = Color.FromArgb(0x26, 0x27, 0x28);
                    hs.ForeColor = Color.White;

                    dgv.ColumnHeadersDefaultCellStyle = hs;

                    dgv.RowHeadersDefaultCellStyle = hs;
                }
                else if (((Type)ctl.GetType()) == typeof(ComboBox))
                {
                    ctl.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                    ctl.ForeColor = Color.White;
                    ComboBox CMB = (ComboBox)ctl;
                    CMB.FlatStyle = FlatStyle.Flat;
                }
                else if (((Type)ctl.GetType()) == typeof(NumericUpDown))
                {
                    ctl.BackColor = Color.FromArgb(0x43, 0x44, 0x45);
                    ctl.ForeColor = Color.White;
                }
                else if (((Type)ctl.GetType()) == typeof(TrackBar))
                {
                    ctl.BackColor = Color.FromArgb(0x26, 0x27, 0x28);
                    ctl.ForeColor = Color.White;
                }
                else if (((Type)ctl.GetType()) == typeof(LinkLabel))
                {
                    ctl.BackColor = Color.FromArgb(0x26, 0x27, 0x28);
                    ctl.ForeColor = Color.White;
                    LinkLabel LNK = (LinkLabel)ctl;
                    LNK.ActiveLinkColor = Color.White;
                    LNK.LinkColor = Color.White;
                    LNK.VisitedLinkColor = Color.White;

                }
                else if (((Type)ctl.GetType()) == typeof(HorizontalProgressBar2) ||
                  ((Type)ctl.GetType()) == typeof(VerticalProgressBar2))
                {
                    ((HorizontalProgressBar2)ctl).BackgroundColor = Color.FromArgb(0x43, 0x44, 0x45);
                    ((HorizontalProgressBar2)ctl).ValueColor = Color.FromArgb(148, 193, 31);
                }

                if (ctl.Controls.Count > 0)
                    fixtheme(ctl, 1);
            }
        }

        private void MenuFlightData_Click(object sender, EventArgs e)
        {
            MyView.Controls.Clear();

            GCSViews.Terminal.threadrun = false;

            UserControl temp = FlightData;

            fixtheme(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Location = new Point(0, MainMenu.Height);

            temp.Dock = DockStyle.Fill;

            //temp.ForeColor = Color.White;

            //temp.BackColor = Color.FromArgb(0x26, 0x27, 0x28);

            MyView.Controls.Add(temp);

            if (MainV2.config["FlightSplitter"] != null)
                ((GCSViews.FlightData)temp).MainHcopy.SplitterDistance = int.Parse(MainV2.config["FlightSplitter"].ToString());
        }

        private void MenuFlightPlanner_Click(object sender, EventArgs e)
        {
            MyView.Controls.Clear();

            GCSViews.Terminal.threadrun = false;

            UserControl temp = FlightPlanner;

            fixtheme(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Location = new Point(0, MainMenu.Height);

            temp.Dock = DockStyle.Fill;

            temp.ForeColor = Color.White;

            temp.BackColor = Color.FromArgb(0x26, 0x27, 0x28);

            MyView.Controls.Add(temp);
        }

        private void MenuConfiguration_Click(object sender, EventArgs e)
        {
            MyView.Controls.Clear();

            GCSViews.Terminal.threadrun = false;

            // dispose of old else memory leak
            if (Configuration != null)
            {
                try
                {
                    Configuration.Dispose();
                }
                catch { }
            }

            Configuration = new GCSViews.Configuration();

            UserControl temp = Configuration;

            fixtheme(temp);

            //temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Location = new Point(0, 0);

            temp.Dock = DockStyle.Fill;

            temp.ForeColor = Color.White;

            temp.BackColor = Color.FromArgb(0x26, 0x27, 0x28);

            temp.Size = MyView.Size;

            //temp.Parent = MyView;

            MyView.Controls.Add(temp);
        }

        private void MenuSimulation_Click(object sender, EventArgs e)
        {
            MyView.Controls.Clear();

            GCSViews.Terminal.threadrun = false;

            UserControl temp = Simulation;

            fixtheme(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Location = new Point(0, MainMenu.Height);

            temp.Dock = DockStyle.Fill;

            temp.ForeColor = Color.White;

            temp.BackColor = Color.FromArgb(0x26, 0x27, 0x28);

            MyView.Controls.Add(temp);
        }

        private void MenuFirmware_Click(object sender, EventArgs e)
        {
            MyView.Controls.Clear();

            GCSViews.Terminal.threadrun = false;

            UserControl temp = Firmware;

            fixtheme(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Dock = DockStyle.Fill;

            temp.ForeColor = Color.White;

            temp.BackColor = Color.FromArgb(0x26, 0x27, 0x28);

            MyView.Controls.Add(temp);
        }

        private void MenuTerminal_Click(object sender, EventArgs e)
        {
            if (comPort.BaseStream.IsOpen)
            {
                MenuConnect_Click(sender, e);
            }

            givecomport = true;

            MyView.Controls.Clear();

            this.MenuConnect.BackgroundImage = global::ArdupilotMega.Properties.Resources.disconnect;

            // dispose of old else memory leak
            if (Terminal != null)
            {
                try
                {
                    Terminal.Dispose();
                }
                catch { }
            }

            Terminal = new GCSViews.Terminal();

            UserControl temp = Terminal;

            fixtheme(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Dock = DockStyle.Fill;

            MyView.Controls.Add(temp);

            temp.ForeColor = Color.White;

            temp.BackColor = Color.FromArgb(0x26, 0x27, 0x28);

        }

        private void MenuConnect_Click(object sender, EventArgs e)
        {
            givecomport = false;

            if (comPort.BaseStream.IsOpen && cs.groundspeed > 4)
            {
                if (DialogResult.No == MessageBox.Show("Your model is still moving are you sure you want to disconnect?", "Disconnect", MessageBoxButtons.YesNo))
                {
                    return;
                }
            }

            if (comPort.BaseStream.IsOpen)
            {
                try
                {
                    if (talk != null) // cancel all pending speech
                        talk.SpeakAsyncCancelAll();

                    if (comPort.logfile != null)
                        comPort.logfile.Close();

                    comPort.BaseStream.DtrEnable = false;
                    comPort.Close();
                }
                catch { }

                this.MenuConnect.BackgroundImage = global::ArdupilotMega.Properties.Resources.connect;
            }
            else
            {
                if (CMB_serialport.Text == "TCP")
                {
                    comPort.BaseStream = new TcpSerial();
                }
                else
                    if (CMB_serialport.Text == "UDP")
                    {
                        comPort.BaseStream = new UdpSerial();
                    }
                    else
                    {
                        comPort.BaseStream = new SerialPort();
                    }
                try
                {
                    comPort.BaseStream.BaudRate = int.Parse(CMB_baudrate.Text);
                }
                catch { }
                comPort.BaseStream.DataBits = 8;
                comPort.BaseStream.StopBits = (StopBits)Enum.Parse(typeof(StopBits), "1");
                comPort.BaseStream.Parity = (Parity)Enum.Parse(typeof(Parity), "None");

                comPort.BaseStream.DtrEnable = false;

                if (config["CHK_resetapmonconnect"] == null || bool.Parse(config["CHK_resetapmonconnect"].ToString()) == true)
                    comPort.BaseStream.toggleDTR();

                try
                {
                    if (comPort.logfile != null)
                        comPort.logfile.Close();
                    try
                    {
                        Directory.CreateDirectory(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs");
                        comPort.logfile = new BinaryWriter(File.Open(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs" + Path.DirectorySeparatorChar + DateTime.Now.ToString("yyyy-MM-dd hh-mm-ss") + ".tlog", FileMode.CreateNew));
                    }
                    catch { MessageBox.Show("Failed to create log - wont log this session"); } // soft fail

                    comPort.BaseStream.PortName = CMB_serialport.Text;
                    comPort.Open(true);

                    if (comPort.param["SYSID_SW_TYPE"] != null)
                    {
                        if (float.Parse(comPort.param["SYSID_SW_TYPE"].ToString()) == 10)
                        {
                            TOOL_APMFirmware.SelectedIndex = TOOL_APMFirmware.Items.IndexOf(Firmwares.ArduCopter2);
                        }
                        else if (float.Parse(comPort.param["SYSID_SW_TYPE"].ToString()) == 0)
                        {
                            TOOL_APMFirmware.SelectedIndex = TOOL_APMFirmware.Items.IndexOf(Firmwares.ArduPlane);
                        }
                    }

                    cs.firmware = APMFirmware;

                    config[CMB_serialport.Text + "_BAUD"] = CMB_baudrate.Text;

                    if (config["loadwpsonconnect"] != null && bool.Parse(config["loadwpsonconnect"].ToString()) == true)
                    {
                        MenuFlightPlanner_Click(null, null);
                        FlightPlanner.BUT_read_Click(null, null);
                    }

                    this.MenuConnect.BackgroundImage = global::ArdupilotMega.Properties.Resources.disconnect;
                }
                catch (Exception ex)
                {
                    try
                    {
                        comPort.Close();
                    }
                    catch { }
                    try
                    {
                        string version = ArduinoDetect.DetectVersion(comPort.BaseStream.PortName);
                        ArduinoComms port = new ArduinoSTK();
                        if (version == "1280")
                        {
                            port = new ArduinoSTK();
                            port.BaudRate = 57600;
                        }
                        else if (version == "2560")
                        {
                            port = new ArduinoSTKv2();
                            port.BaudRate = 115200;
                        }
                        else { throw new Exception("Can not determine APM board type"); }
                        port.PortName = comPort.BaseStream.PortName;
                        port.DtrEnable = true;
                        port.Open();
                        if (port.connectAP())
                        {
                            byte[] buffer = port.download(20);
                            port.Close();

                            if (buffer[0] != 'A' || buffer[1] != 'P') // this is the apvar header
                            {
                                MessageBox.Show("You dont appear to have uploaded a firmware yet,\n\nPlease goto the firmware page and upload one.");
                                return;
                            }
                            else
                            {
                                Console.WriteLine("Valid eeprom contents");
                            }
                        }
                    }
                    catch { }
                    MessageBox.Show("Can not establish a connection\n\n" + ex.ToString());
                    return;
                }
            }
        }

        private void CMB_serialport_SelectedIndexChanged(object sender, EventArgs e)
        {
            comportname = CMB_serialport.Text;
            if (comportname == "UDP" || comportname == "TCP")
            {
                CMB_baudrate.Enabled = false;
                if (comportname == "TCP")
                    MainV2.comPort.BaseStream = new TcpSerial();
                if (comportname == "UDP")
                    MainV2.comPort.BaseStream = new UdpSerial();
            }
            else
            {
                CMB_baudrate.Enabled = true;
                MainV2.comPort.BaseStream = new ArdupilotMega.SerialPort();
            }

            try
            {
                comPort.BaseStream.PortName = CMB_serialport.Text;

                MainV2.comPort.BaseStream.BaudRate = int.Parse(CMB_baudrate.Text);

                if (config[CMB_serialport.Text + "_BAUD"] != null)
                {
                    CMB_baudrate.Text = config[CMB_serialport.Text + "_BAUD"].ToString();
                }
            }
            catch { }
        }

        private void toolStripMenuItem2_Click(object sender, EventArgs e)
        {
            //Form temp = new Main();
            //temp.Show();
        }

        private void MainV2_FormClosed(object sender, FormClosedEventArgs e)
        {
            GCSViews.FlightData.threadrun = 0;
            GCSViews.Simulation.threadrun = 0;

            serialthread = false;

            try
            {
                if (comPort.BaseStream.IsOpen)
                    comPort.Close();
            }
            catch { } // i get alot of these errors, the port is still open, but not valid - user has unpluged usb
            try
            {
                FlightData.Dispose();
            }
            catch { }
            try
            {
                FlightPlanner.Dispose();
            }
            catch { }
            try
            {
                Simulation.Dispose();
            }
            catch { }

            xmlconfig(true);
        }


        private void xmlconfig(bool write)
        {
            if (write || !File.Exists(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"config.xml"))
            {
                try
                {
                    //System.Configuration.Configuration appconfig = System.Configuration.ConfigurationManager.OpenExeConfiguration(System.Configuration.ConfigurationUserLevel.None);

                    XmlTextWriter xmlwriter = new XmlTextWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"config.xml", Encoding.ASCII);
                    xmlwriter.Formatting = Formatting.Indented;

                    xmlwriter.WriteStartDocument();

                    xmlwriter.WriteStartElement("Config");

                    xmlwriter.WriteElementString("comport", comportname);

                    xmlwriter.WriteElementString("baudrate", CMB_baudrate.Text);

                    xmlwriter.WriteElementString("APMFirmware", APMFirmware.ToString());

                    //appconfig.AppSettings.Settings.Add("comport", comportname);
                    //appconfig.AppSettings.Settings.Add("baudrate", CMB_baudrate.Text);
                    //appconfig.AppSettings.Settings.Add("APMFirmware", APMFirmware.ToString());

                    foreach (string key in config.Keys)
                    {
                        try
                        {
                            if (key == "" || key.Contains("/")) // "/dev/blah"
                                continue;
                            xmlwriter.WriteElementString(key, config[key].ToString());

                            //appconfig.AppSettings.Settings.Add(key, config[key].ToString());
                        }
                        catch { }
                    }

                    xmlwriter.WriteEndElement();

                    xmlwriter.WriteEndDocument();
                    xmlwriter.Close();

                    //appconfig.Save();
                }
                catch (Exception ex) { MessageBox.Show(ex.ToString()); }
            }
            else
            {
                try
                {
                    using (XmlTextReader xmlreader = new XmlTextReader(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"config.xml"))
                    {
                        while (xmlreader.Read())
                        {
                            xmlreader.MoveToElement();
                            try
                            {
                                switch (xmlreader.Name)
                                {
                                    case "comport":
                                        string temp = xmlreader.ReadString();

                                        CMB_serialport.SelectedIndex = CMB_serialport.FindString(temp);
                                        if (CMB_serialport.SelectedIndex == -1)
                                        {
                                            CMB_serialport.Text = temp; // allows ports that dont exist - yet
                                        }
                                        comPort.BaseStream.PortName = temp;
                                        comportname = temp;
                                        break;
                                    case "baudrate":
                                        string temp2 = xmlreader.ReadString();

                                        CMB_baudrate.SelectedIndex = CMB_baudrate.FindString(temp2);
                                        if (CMB_baudrate.SelectedIndex == -1)
                                        {
                                            CMB_baudrate.Text = temp2;
                                            //CMB_baudrate.SelectedIndex = CMB_baudrate.FindString("57600"); ; // must exist
                                        }
                                        //bau = int.Parse(CMB_baudrate.Text);
                                        break;
                                    case "APMFirmware":
                                        string temp3 = xmlreader.ReadString();
                                        TOOL_APMFirmware.SelectedIndex = TOOL_APMFirmware.FindStringExact(temp3);
                                        if (TOOL_APMFirmware.SelectedIndex == -1)
                                            TOOL_APMFirmware.SelectedIndex = 0;
                                        APMFirmware = (MainV2.Firmwares)Enum.Parse(typeof(MainV2.Firmwares), TOOL_APMFirmware.Text);
                                        break;
                                    case "Config":
                                        break;
                                    case "xml":
                                        break;
                                    default:
                                        if (xmlreader.Name == "") // line feeds
                                            break;
                                        config[xmlreader.Name] = xmlreader.ReadString();
                                        break;
                                }
                            }
                            catch (Exception ee) { Console.WriteLine(ee.Message); } // silent fail on bad entry
                        }
                    }
                }
                catch (Exception ex) { Console.WriteLine("Bad Config File: " + ex.ToString()); } // bad config file
            }
        }

        private void CMB_baudrate_SelectedIndexChanged(object sender, EventArgs e)
        {
            try
            {
                comPort.BaseStream.BaudRate = int.Parse(CMB_baudrate.Text);
            }
            catch { }
        }

        private void joysticksend()
        {
            while (true)
            {
                try
                {
                    if (!MONO)
                    {
                        //joystick stuff

                        if (joystick != null && joystick.enabled)
                        {
                            MAVLink.__mavlink_rc_channels_override_t rc = new MAVLink.__mavlink_rc_channels_override_t();

                            rc.target_component = comPort.compid;
                            rc.target_system = comPort.sysid;

                            if (joystick.getJoystickAxis(1) != Joystick.joystickaxis.None)
                                rc.chan1_raw = cs.rcoverridech1;//(ushort)(((int)state.Rz / 65.535) + 1000);
                            if (joystick.getJoystickAxis(2) != Joystick.joystickaxis.None)
                                rc.chan2_raw = cs.rcoverridech2;//(ushort)(((int)state.Y / 65.535) + 1000);
                            if (joystick.getJoystickAxis(3) != Joystick.joystickaxis.None)
                                rc.chan3_raw = cs.rcoverridech3;//(ushort)(1000 - ((int)slider[0] / 65.535 ) + 1000);
                            if (joystick.getJoystickAxis(4) != Joystick.joystickaxis.None)
                                rc.chan4_raw = cs.rcoverridech4;//(ushort)(((int)state.X / 65.535) + 1000);
                            if (joystick.getJoystickAxis(5) != Joystick.joystickaxis.None)
                                rc.chan5_raw = cs.rcoverridech5;
                            if (joystick.getJoystickAxis(6) != Joystick.joystickaxis.None)
                                rc.chan6_raw = cs.rcoverridech6;
                            if (joystick.getJoystickAxis(7) != Joystick.joystickaxis.None)
                                rc.chan7_raw = cs.rcoverridech7;
                            if (joystick.getJoystickAxis(8) != Joystick.joystickaxis.None)
                                rc.chan8_raw = cs.rcoverridech8;

                            if (lastjoystick.AddMilliseconds(50) < DateTime.Now)
                            {
                                //                                Console.WriteLine(DateTime.Now.Millisecond + " {0} {1} {2} {3} ", rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw);
                                comPort.sendPacket(rc);
                                lastjoystick = DateTime.Now;
                            }

                        }
                    }
                    System.Threading.Thread.Sleep(50);
                }
                catch { } // cant fall out
            }
        }




        private void SerialReader()
        {
            if (serialthread == true)
                return;
            serialthread = true;

            int minbytes = 10;

            if (MONO)
                minbytes = 0;

            DateTime menuupdate = DateTime.Now;

            DateTime speechcustomtime = DateTime.Now;

            DateTime linkqualitytime = DateTime.Now;

            while (serialthread)
            {
                try
                {
                    System.Threading.Thread.Sleep(5);
                    if ((DateTime.Now - menuupdate).Milliseconds > 500)
                    {
                        //                        Console.WriteLine(DateTime.Now.Millisecond);
                        if (comPort.BaseStream.IsOpen)
                        {
                            if ((string)this.MenuConnect.BackgroundImage.Tag != "Disconnect")
                            {
                                this.Invoke((MethodInvoker)delegate
                                {
                                    this.MenuConnect.BackgroundImage = global::ArdupilotMega.Properties.Resources.disconnect;
                                    this.MenuConnect.BackgroundImage.Tag = "Disconnect";
                                    CMB_baudrate.Enabled = false;
                                    CMB_serialport.Enabled = false;
                                });
                            }
                        }
                        else
                        {
                            if ((string)this.MenuConnect.BackgroundImage.Tag != "Connect")
                            {
                                this.Invoke((MethodInvoker)delegate
                                {
                                    this.MenuConnect.BackgroundImage = global::ArdupilotMega.Properties.Resources.connect;
                                    this.MenuConnect.BackgroundImage.Tag = "Connect";
                                    CMB_baudrate.Enabled = true;
                                    CMB_serialport.Enabled = true;
                                });
                            }
                        }
                        menuupdate = DateTime.Now;
                    }

                    if (speechenable && talk != null && (DateTime.Now - speechcustomtime).TotalSeconds > 30 && MainV2.cs.lat != 0 && (MainV2.comPort.logreadmode || comPort.BaseStream.IsOpen))
                    {
                        //speechbatteryvolt
                        float warnvolt = 0;
                        float.TryParse(MainV2.getConfig("speechbatteryvolt"), out warnvolt);

                        if (MainV2.getConfig("speechbatteryenabled") == "True" && MainV2.cs.battery_voltage <= warnvolt)
                        {
                            MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechbattery")));
                        }

                        if (MainV2.getConfig("speechcustomenabled") == "True")
                        {
                            MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechcustom")));
                        }

                        speechcustomtime = DateTime.Now;
                    }

                    if ((DateTime.Now - comPort.lastvalidpacket).TotalSeconds > 10)
                    {
                        MainV2.cs.linkqualitygcs = 0;
                    }

                    if ((DateTime.Now - comPort.lastvalidpacket).TotalSeconds >= 1)
                    {
                        if (linkqualitytime.Second != DateTime.Now.Second)
                        {
                            MainV2.cs.linkqualitygcs = (ushort)(MainV2.cs.linkqualitygcs * 0.8f);
                            linkqualitytime = DateTime.Now;

                            GCSViews.FlightData.myhud.Invalidate();
                        }

                        GC.Collect();
                    }

                    if (speechenable && talk != null && (MainV2.comPort.logreadmode || comPort.BaseStream.IsOpen))
                    {
                        float warnalt = float.MaxValue;
                        float.TryParse(MainV2.getConfig("speechaltheight"), out warnalt);
                        try
                        {
                            if (MainV2.getConfig("speechaltenabled") == "True" && (MainV2.cs.alt - (int)double.Parse(MainV2.getConfig("TXT_homealt"))) <= warnalt)
                            {
                                if (MainV2.talk.State == SynthesizerState.Ready)
                                    MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechalt")));
                            }
                        }
                        catch { } // silent fail
                    }

                    if (!comPort.BaseStream.IsOpen || givecomport == true)
                    {
                        System.Threading.Thread.Sleep(100);
                        continue;
                    }

                    if (heatbeatsend.Second != DateTime.Now.Second)
                    {
                        //                        Console.WriteLine("remote lost {0}", cs.packetdropremote);

                        MAVLink.__mavlink_heartbeat_t htb = new MAVLink.__mavlink_heartbeat_t();

#if MAVLINK10
                        htb.type = (byte)MAVLink.MAV_TYPE.MAV_TYPE_GCS;
                        htb.autopilot = (byte)MAVLink.MAV_AUTOPILOT.MAV_AUTOPILOT_ARDUPILOTMEGA;
                        htb.mavlink_version = 3;
#else
                        htb.type = (byte)MAVLink.MAV_TYPE.MAV_GENERIC;
                        htb.autopilot = (byte)MAVLink.MAV_AUTOPILOT_TYPE.MAV_AUTOPILOT_ARDUPILOTMEGA;
                        htb.mavlink_version = 2;
#endif

                        comPort.sendPacket(htb);
                        heatbeatsend = DateTime.Now;
                    }

                    // data loss warning
                    if ((DateTime.Now - comPort.lastvalidpacket).TotalSeconds > 10)
                    {
                        if (speechenable && talk != null)
                        {
                            if (MainV2.talk.State == SynthesizerState.Ready)
                                MainV2.talk.SpeakAsync("WARNING No Data for " + (int)(DateTime.Now - comPort.lastvalidpacket).TotalSeconds + " Seconds");
                        }
                    }

                    //Console.WriteLine(comPort.BaseStream.BytesToRead);

                    while (comPort.BaseStream.BytesToRead > minbytes && givecomport == false)
                        comPort.readPacket();
                }
                catch (Exception e)
                {
                    Console.WriteLine("Serial Reader fail :" + e.Message);
                    try
                    {
                        comPort.Close();
                    }
                    catch { }
                }
            }
        }

        private class MyRenderer : ToolStripProfessionalRenderer
        {
            public static ToolStripItem currentpressed;
            protected override void OnRenderButtonBackground(ToolStripItemRenderEventArgs e)
            {
                //BackgroundImage
                if (e.Item.BackgroundImage == null) base.OnRenderButtonBackground(e);
                else
                {
                    Rectangle bounds = new Rectangle(Point.Empty, e.Item.Size);
                    e.Graphics.DrawImage(e.Item.BackgroundImage, bounds);
                    if (e.Item.Pressed || e.Item == currentpressed)
                    {
                        SolidBrush brush = new SolidBrush(Color.FromArgb(73, 0x2b, 0x3a, 0x03));
                        e.Graphics.FillRectangle(brush, bounds);
                        if (e.Item.Name != "MenuConnect")
                        {
                            //Console.WriteLine("new " + e.Item.Name + " old " + currentpressed.Name );
                            //e.Item.GetCurrentParent().Invalidate();
                            if (currentpressed != e.Item)
                                currentpressed.Invalidate();
                            currentpressed = e.Item;
                        }

                        // Something...
                    }
                    else if (e.Item.Selected) // mouse over
                    {
                        SolidBrush brush = new SolidBrush(Color.FromArgb(73, 0x2b, 0x3a, 0x03));
                        e.Graphics.FillRectangle(brush, bounds);
                        // Something...
                    }
                    using (Pen pen = new Pen(Color.Black))
                    {
                        //e.Graphics.DrawRectangle(pen, bounds.X, bounds.Y, bounds.Width - 1, bounds.Height - 1);
                    }
                }
            }

            protected override void OnRenderItemImage(ToolStripItemImageRenderEventArgs e)
            {
                //base.OnRenderItemImage(e);
            }
        }

        private void MainV2_Load(object sender, EventArgs e)
        {
            // generate requestall for jani and sandro
            comPort.sysid = 7;
            comPort.compid = 1;
            //comPort.requestDatastream((byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_ALL,3);
            //

            MenuFlightData_Click(sender, e);

            try
            {
                listener = new TcpListener(IPAddress.Any, 56781);
                System.Threading.Thread t13 = new System.Threading.Thread(new System.Threading.ThreadStart(listernforclients))
                {
                    Name = "motion jpg stream",
                    IsBackground = true
                };
                // wait for tcp connections               
                t13.Start();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }

            System.Threading.Thread t12 = new System.Threading.Thread(new ThreadStart(joysticksend))
            {
                IsBackground = true,
                Priority = ThreadPriority.AboveNormal,
                Name = "Main joystick sender"
            };
            t12.Start();

            System.Threading.Thread t11 = new System.Threading.Thread(new ThreadStart(SerialReader))
            {
                IsBackground = true,
                Name = "Main Serial reader"
            };
            t11.Start();

            try
            {
                checkForUpdate();
            }
            catch { Console.WriteLine("update check failed"); }
        }

        public static String ComputeWebSocketHandshakeSecurityHash09(String secWebSocketKey)
        {
            const String MagicKEY = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
            String secWebSocketAccept = String.Empty;

            // 1. Combine the request Sec-WebSocket-Key with magic key.
            String ret = secWebSocketKey + MagicKEY;

            // 2. Compute the SHA1 hash
            System.Security.Cryptography.SHA1 sha = new System.Security.Cryptography.SHA1CryptoServiceProvider();
            byte[] sha1Hash = sha.ComputeHash(Encoding.UTF8.GetBytes(ret));

            // 3. Base64 encode the hash
            secWebSocketAccept = Convert.ToBase64String(sha1Hash);

            return secWebSocketAccept;
        }

        /// <summary>          
        /// little web server for sending network link kml's          
        /// </summary>          

        void listernforclients()
        {
            try
            {
                listener.Start();
            }
            catch { Console.WriteLine("do you have the planner open already"); return; } // in use
            // Enter the listening loop.               
            while (true)
            {
                // Perform a blocking call to accept requests.           
                // You could also user server.AcceptSocket() here.               
                try
                {
                    Console.WriteLine("Listening for client - 1 client at a time");
                    TcpClient client = listener.AcceptTcpClient();
                    // Get a stream object for reading and writing          
                    Console.WriteLine("Accepted Client " + client.Client.RemoteEndPoint.ToString());
                    //client.SendBufferSize = 100 * 1024; // 100kb
                    //client.LingerState.Enabled = true;
                    //client.NoDelay = true;

                    // makesure we have valid image
                    GCSViews.FlightData.mymap.streamjpgenable = true;
                    GCSViews.FlightData.myhud.streamjpgenable = true;

                    MethodInvoker m = delegate()
                    {
                        GCSViews.FlightData.mymap.Refresh();
                    };
                    this.Invoke(m);

                    NetworkStream stream = client.GetStream();

                    System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();

                    byte[] request = new byte[1024];

                    int len = stream.Read(request, 0, request.Length);
                    string head = System.Text.ASCIIEncoding.ASCII.GetString(request, 0, len);
                    Console.WriteLine(head);

                    int index = head.IndexOf('\n');

                    string url = head.Substring(0, index - 1);
                    //url = url.Replace("\r", "");
                    //url = url.Replace("GET ","");
                    //url = url.Replace(" HTTP/1.0", "");
                    //url = url.Replace(" HTTP/1.1", "");

                    if (url.Contains("websocket"))
                    {
                        using (var writer = new StreamWriter(stream, Encoding.Default))
                        {
                            writer.WriteLine("HTTP/1.1 101 WebSocket Protocol Handshake");
                            writer.WriteLine("Upgrade: WebSocket");
                            writer.WriteLine("Connection: Upgrade");
                            writer.WriteLine("WebSocket-Location: ws://localhost:56781/websocket/server");

                            int start = head.IndexOf("Sec-WebSocket-Key:") + 19;
                            int end = head.IndexOf('\r', start);
                            if (end == -1)
                                end = head.IndexOf('\n', start);
                            string accept = ComputeWebSocketHandshakeSecurityHash09(head.Substring(start, end - start));

                            writer.WriteLine("Sec-WebSocket-Accept: " + accept);

                            writer.WriteLine("Server: APM Planner");

                            writer.WriteLine("");

                            writer.Flush();

                            while (client.Connected)
                            {
                                System.Threading.Thread.Sleep(200);
                                Console.WriteLine(stream.DataAvailable + " " + client.Available);

                                while (client.Available > 0)
                                {
                                    Console.Write(stream.ReadByte());
                                }

                                byte[] packet = new byte[256];

                                string sendme = cs.roll + "," + cs.pitch + "," + cs.yaw;

                                packet[0] = 0x81; // fin - binary
                                packet[1] = (byte)sendme.Length;

                                int i = 2;
                                foreach (char ch in sendme)
                                {
                                    packet[i++] = (byte)ch;
                                }

                                stream.Write(packet, 0, i);

                                //break;
                            }
                        }
                    }
                    else if (url.Contains("network.kml"))
                    {
                        string header = "HTTP/1.1 200 OK\r\nContent-Type: application/vnd.google-earth.kml+xml\n\n";
                        byte[] temp = encoding.GetBytes(header);
                        stream.Write(temp, 0, temp.Length);

                        SharpKml.Dom.Document kml = new SharpKml.Dom.Document();

                        SharpKml.Dom.Placemark pmplane = new SharpKml.Dom.Placemark();
                        pmplane.Name = "P/Q ";

                        pmplane.Visibility = true;

                        SharpKml.Dom.Location loc = new SharpKml.Dom.Location();
                        loc.Latitude = cs.lat;
                        loc.Longitude = cs.lng;
                        loc.Altitude = cs.alt;

                        if (loc.Altitude < 0)
                            loc.Altitude = 0.01;

                        SharpKml.Dom.Orientation ori = new SharpKml.Dom.Orientation();
                        ori.Heading = cs.yaw;
                        ori.Roll = -cs.roll;
                        ori.Tilt = -cs.pitch;

                        SharpKml.Dom.Scale sca = new SharpKml.Dom.Scale();

                        sca.X = 2;
                        sca.Y = 2;
                        sca.Z = 2;

                        SharpKml.Dom.Model model = new SharpKml.Dom.Model();
                        model.Location = loc;
                        model.Orientation = ori;
                        model.AltitudeMode = SharpKml.Dom.AltitudeMode.Absolute;
                        model.Scale = sca;

                        SharpKml.Dom.Link link = new SharpKml.Dom.Link();
                        link.Href = new Uri("block_plane_0.dae", UriKind.Relative);

                        model.Link = link;

                        pmplane.Geometry = model;

                        SharpKml.Dom.LookAt la = new SharpKml.Dom.LookAt() 
                        { Altitude = loc.Altitude.Value, Latitude = loc.Latitude.Value, Longitude = loc.Longitude.Value, Tilt = 80,
                            Heading = cs.yaw, AltitudeMode = SharpKml.Dom.AltitudeMode.Absolute, Range = 50};

                        kml.Viewpoint = la;
                        
                        kml.AddFeature(pmplane);

                        SharpKml.Base.Serializer serializer = new SharpKml.Base.Serializer();
                        serializer.Serialize(kml);

                        byte[] buffer = Encoding.ASCII.GetBytes(serializer.Xml);

                        stream.Write(buffer, 0, buffer.Length);

                        stream.Close();
                    }
                    else if (url.Contains("block_plane_0.dae"))
                    {
                        string header = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\n\n";
                        byte[] temp = encoding.GetBytes(header);
                        stream.Write(temp, 0, temp.Length);

                        BinaryReader file = new BinaryReader(File.Open("block_plane_0.dae", FileMode.Open, FileAccess.Read, FileShare.Read));
                        byte[] buffer = new byte[1024];
                        while (file.PeekChar() != -1)
                        {

                            int leng = file.Read(buffer, 0, buffer.Length);

                            stream.Write(buffer, 0, leng);
                        }
                        file.Close();
                        stream.Close();
                    }
                    else if (url.Contains("hud.html"))
                    {
                        string header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\n\n";
                        byte[] temp = encoding.GetBytes(header);
                        stream.Write(temp, 0, temp.Length);

                        BinaryReader file = new BinaryReader(File.Open("hud.html", FileMode.Open, FileAccess.Read, FileShare.Read));
                        byte[] buffer = new byte[1024];
                        while (file.PeekChar() != -1)
                        {

                            int leng = file.Read(buffer, 0, buffer.Length);

                            stream.Write(buffer, 0, leng);
                        }
                        file.Close();
                        stream.Close();
                    }
                    else if (url.ToLower().Contains("hud.jpg") || url.ToLower().Contains("map.jpg") || url.ToLower().Contains("both.jpg"))
                    {
                        string header = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=APMPLANNER\n\n--APMPLANNER\r\n";
                        byte[] temp = encoding.GetBytes(header);
                        stream.Write(temp, 0, temp.Length);

                        while (client.Connected)
                        {
                            System.Threading.Thread.Sleep(200); // 5hz
                            byte[] data = null;

                            if (url.ToLower().Contains("hud"))
                            {
                                GCSViews.FlightData.myhud.streamjpgenable = true;
                                data = GCSViews.FlightData.myhud.streamjpg.ToArray();
                            }
                            else if (url.ToLower().Contains("map"))
                            {
                                GCSViews.FlightData.mymap.streamjpgenable = true;
                                data = GCSViews.FlightData.mymap.streamjpg.ToArray();
                            }
                            else
                            {
                                GCSViews.FlightData.mymap.streamjpgenable = true;
                                GCSViews.FlightData.myhud.streamjpgenable = true;
                                Image img1 = Image.FromStream(GCSViews.FlightData.myhud.streamjpg);
                                Image img2 = Image.FromStream(GCSViews.FlightData.mymap.streamjpg);
                                int bigger = img1.Height > img2.Height ? img1.Height : img2.Height;
                                Image imgout = new Bitmap(img1.Width + img2.Width, bigger);

                                Graphics grap = Graphics.FromImage(imgout);

                                grap.DrawImageUnscaled(img1, 0, 0);
                                grap.DrawImageUnscaled(img2, img1.Width, 0);

                                MemoryStream streamjpg = new MemoryStream();
                                imgout.Save(streamjpg, System.Drawing.Imaging.ImageFormat.Jpeg);
                                data = streamjpg.ToArray();

                            }

                            header = "Content-Type: image/jpeg\r\nContent-Length: " + data.Length + "\r\n\r\n";
                            temp = encoding.GetBytes(header);
                            stream.Write(temp, 0, temp.Length);

                            stream.Write(data, 0, data.Length);

                            header = "\r\n--APMPLANNER\r\n";
                            temp = encoding.GetBytes(header);
                            stream.Write(temp, 0, temp.Length);

                        }
                        GCSViews.FlightData.mymap.streamjpgenable = false;
                        GCSViews.FlightData.myhud.streamjpgenable = false;
                        stream.Close();

                    }
                    stream.Close();
                }
                catch (Exception ee) { Console.WriteLine("Failed mjpg " + ee.Message); }
            }
        }

        private void TOOL_APMFirmware_SelectedIndexChanged(object sender, EventArgs e)
        {
            APMFirmware = (MainV2.Firmwares)Enum.Parse(typeof(MainV2.Firmwares), TOOL_APMFirmware.Text);
            MainV2.cs.firmware = APMFirmware;
        }

        private void MainV2_Resize(object sender, EventArgs e)
        {
            Console.WriteLine("myview width " + MyView.Width + " height " + MyView.Height);
            Console.WriteLine("this   width " + this.Width + " height " + this.Height);
        }

        private void MenuHelp_Click(object sender, EventArgs e)
        {
            MyView.Controls.Clear();

            UserControl temp = new GCSViews.Help();

            fixtheme(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Dock = DockStyle.Fill;

            MyView.Controls.Add(temp);

            temp.ForeColor = Color.White;

            temp.BackColor = Color.FromArgb(0x26, 0x27, 0x28);
        }

        static string baseurl = "http://ardupilot-mega.googlecode.com/git/Tools/ArdupilotMegaPlanner/bin/Release/";

        public static void updatecheck(Label loadinglabel)
        {
            try
            {
                bool update = updatecheck(loadinglabel, baseurl, "");
                System.Diagnostics.Process P = new System.Diagnostics.Process();
                if (MONO)
                {
                    P.StartInfo.FileName = "mono";
                    P.StartInfo.Arguments = " \"" + Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "Updater.exe\"";
                }
                else
                {
                    P.StartInfo.FileName = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "Updater.exe";
                    P.StartInfo.Arguments = "";
                    try
                    {
                        foreach (string newupdater in Directory.GetFiles(Path.GetDirectoryName(Application.ExecutablePath), "Updater.exe*.new"))
                        {
                            File.Copy(newupdater, newupdater.Remove(newupdater.Length - 4), true);
                            File.Delete(newupdater);
                        }
                    }
                    catch (Exception)
                    {
                    }
                }
                if (loadinglabel != null)
                    updatelabel(loadinglabel,"Starting Updater");
                Console.WriteLine("Start " + P.StartInfo.FileName + " with " + P.StartInfo.Arguments);
                P.Start();
                try
                {
                    Application.Exit();
                }
                catch { }
            }
            catch (Exception ex) { MessageBox.Show("Update Failed " + ex.Message); }
        }

        private static void updatelabel(Label loadinglabel, string text)
        {
            MainV2.instance.Invoke((MethodInvoker)delegate
            {
                loadinglabel.Text = text;
            });

            Application.DoEvents();
        }

        private static void checkForUpdate()
        {
              string path = Path.GetFileNameWithoutExtension(Application.ExecutablePath) + ".exe";

                // Create a request using a URL that can receive a post. 
                WebRequest request = WebRequest.Create(baseurl + path);
                request.Timeout = 5000;
                Console.Write(baseurl + path + " ");
                // Set the Method property of the request to POST.
                request.Method = "HEAD";

                ((HttpWebRequest)request).IfModifiedSince = File.GetLastWriteTimeUtc(path);

                // Get the response.
                WebResponse response = request.GetResponse();
                // Display the status.
                Console.WriteLine(((HttpWebResponse)response).StatusDescription);
                // Get the stream containing content returned by the server.
                //dataStream = response.GetResponseStream();
                // Open the stream using a StreamReader for easy access.

                bool getfile = false;

                if (File.Exists(path))
                {
                    FileInfo fi = new FileInfo(path);

                    Console.WriteLine(response.Headers[HttpResponseHeader.ETag]);

                    if (fi.Length != response.ContentLength) // && response.Headers[HttpResponseHeader.ETag] != "0")
                    {
                        StreamWriter sw = new StreamWriter(path + ".etag");
                        sw.WriteLine(response.Headers[HttpResponseHeader.ETag]);
                        sw.Close();
                        getfile = true;
                        Console.WriteLine("NEW FILE " + path);
                    }
                }
                else
                {
                    getfile = true;
                    Console.WriteLine("NEW FILE " + path);
                    // get it
                }

                response.Close();

                if (getfile)
                {
                    DialogResult dr = MessageBox.Show("Update Found\n\nDo you wish to update now?", "Update Now", MessageBoxButtons.YesNo);
                    if (dr == DialogResult.Yes)
                    {
                        doupdate();
                    }
                    else
                    {
                        return;
                    }
                }
        }

        public static void doupdate()
        {
            //System.Threading.Thread t12 = new System.Threading.Thread(delegate()
            { 

            Form loading = new Form();
            loading.Width = 400;
            loading.Height = 150;
            loading.StartPosition = FormStartPosition.CenterScreen;
            loading.TopMost = true;
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainV2));
            loading.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));

            Label loadinglabel = new Label();
            loadinglabel.Location = new System.Drawing.Point(50, 40);
            loadinglabel.Name = "load";
            loadinglabel.AutoSize = true;
            loadinglabel.Text = "Checking...";
            loadinglabel.Size = new System.Drawing.Size(100, 20);

            loading.Controls.Add(loadinglabel);
            loading.Show();

            try { MainV2.updatecheck(loadinglabel); } catch (Exception ex) { Console.WriteLine(ex.ToString()); } 
            
            }
            //); 
            //t12.Name = "Update check thread";
            //t12.Start();
            //MainV2.threads.Add(t12);
        }

        private static bool updatecheck(Label loadinglabel, string baseurl, string subdir)
        {
            bool update = false;
            List<string> files = new List<string>();

            // Create a request using a URL that can receive a post. 
            Console.WriteLine(baseurl);
            WebRequest request = WebRequest.Create(baseurl);
            request.Timeout = 10000;
            // Set the Method property of the request to POST.
            request.Method = "GET";
            // Get the request stream.
            Stream dataStream; //= request.GetRequestStream();
            // Get the response.
            WebResponse response = request.GetResponse();
            // Display the status.
            Console.WriteLine(((HttpWebResponse)response).StatusDescription);
            // Get the stream containing content returned by the server.
            dataStream = response.GetResponseStream();
            // Open the stream using a StreamReader for easy access.
            StreamReader reader = new StreamReader(dataStream);
            // Read the content.
            string responseFromServer = reader.ReadToEnd();
            // Display the content.
            Regex regex = new Regex("href=\"([^\"]+)\"", RegexOptions.IgnoreCase);
            if (regex.IsMatch(responseFromServer))
            {
                MatchCollection matchs = regex.Matches(responseFromServer);
                for (int i = 0; i < matchs.Count; i++)
                {
                    if (matchs[i].Groups[1].Value.ToString().Contains(".."))
                        continue;
                    if (matchs[i].Groups[1].Value.ToString().Contains("http"))
                        continue;
                    files.Add(System.Web.HttpUtility.UrlDecode(matchs[i].Groups[1].Value.ToString()));
                }
            }

            //Console.WriteLine(responseFromServer);
            // Clean up the streams.
            reader.Close();
            dataStream.Close();
            response.Close();

            string dir = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + subdir;
            if (!Directory.Exists(dir))
                Directory.CreateDirectory(dir);
            foreach (string file in files)
            {
                if (file.Equals("/"))
                {
                    continue;
                }
                if (file.EndsWith("/"))
                {
                    update = updatecheck(loadinglabel, baseurl + file, subdir.Replace("/", "\\") + file) && update;
                    continue;
                }
                if (loadinglabel != null)
                    updatelabel(loadinglabel, "Checking " + file);

                string path = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + subdir + file;


                // Create a request using a URL that can receive a post. 
                request = WebRequest.Create(baseurl + file);
                Console.Write(baseurl + file + " ");
                // Set the Method property of the request to POST.
                request.Method = "HEAD";

                ((HttpWebRequest)request).IfModifiedSince = File.GetLastWriteTimeUtc(path);

                // Get the response.
                response = request.GetResponse();
                // Display the status.
                Console.WriteLine(((HttpWebResponse)response).StatusDescription);
                // Get the stream containing content returned by the server.
                //dataStream = response.GetResponseStream();
                // Open the stream using a StreamReader for easy access.

                bool getfile = false;

                if (File.Exists(path))
                {
                    FileInfo fi = new FileInfo(path);

                    Console.WriteLine(response.Headers[HttpResponseHeader.ETag]);

                    if (fi.Length != response.ContentLength) // && response.Headers[HttpResponseHeader.ETag] != "0")
                    {
                        StreamWriter sw = new StreamWriter(path + ".etag");
                        sw.WriteLine(response.Headers[HttpResponseHeader.ETag]);
                        sw.Close();
                        getfile = true;
                        Console.WriteLine("NEW FILE " + file);
                    }
                }
                else
                {
                    getfile = true;
                    Console.WriteLine("NEW FILE " + file);
                    // get it
                }

                reader.Close();
                //dataStream.Close();
                response.Close();

                if (getfile)
                {
                    if (!update)
                    {
                        //DialogResult dr = MessageBox.Show("Update Found\n\nDo you wish to update now?", "Update Now", MessageBoxButtons.YesNo);
                        //if (dr == DialogResult.Yes)
                        {
                            update = true;
                        }
                        //else
                        {
                            //    return;
                        }
                    }
                    if (loadinglabel != null)
                        updatelabel(loadinglabel, "Getting " + file);

                    // from head
                    long bytes = response.ContentLength;

                    // Create a request using a URL that can receive a post. 
                    request = WebRequest.Create(baseurl + file);
                    // Set the Method property of the request to POST.
                    request.Method = "GET";
                    // Get the response.
                    response = request.GetResponse();
                    // Display the status.
                    Console.WriteLine(((HttpWebResponse)response).StatusDescription);
                    // Get the stream containing content returned by the server.
                    dataStream = response.GetResponseStream();
                    
                    long contlen = bytes;

                    byte[] buf1 = new byte[1024];

                    FileStream fs = new FileStream(path + ".new", FileMode.Create); // 

                    DateTime dt = DateTime.Now;

                    dataStream.ReadTimeout = 30000;

                    while (dataStream.CanRead)
                    {
                        Application.DoEvents();
                        try
                        {
                            if (dt.Second != DateTime.Now.Second)
                            {
                                if (loadinglabel != null)
                                    updatelabel(loadinglabel, "Getting " + file + ": " + (((double)(contlen - bytes) / (double)contlen) * 100).ToString("0.0") + "%"); //+ Math.Abs(bytes) + " bytes");
                                dt = DateTime.Now;
                            }
                        }
                        catch { }
                        Console.WriteLine(file + " " + bytes);
                        int len = dataStream.Read(buf1, 0, 1024);
                        if (len == 0)
                            break;
                        bytes -= len;
                        fs.Write(buf1, 0, len);
                    }

                    fs.Close();
                    dataStream.Close();
                    response.Close();
                }


            }

            //P.StartInfo.CreateNoWindow = true;
            //P.StartInfo.RedirectStandardOutput = true;
            return update;


        }

        private void toolStripMenuItem3_Click(object sender, EventArgs e)
        {

        }

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            if (keyData == (Keys.Control | Keys.F))
            {
                Form frm = new temp();
                fixtheme(frm);
                frm.Show();
                return true;
            }
            if (keyData == (Keys.Control | Keys.S))
            {
                ScreenShot();
                return true;
            }
            if (keyData == (Keys.Control | Keys.G)) // test
            {
                Form frm = new SerialOutput();
                fixtheme(frm);
                frm.Show();
                return true;
            }
            if (keyData == (Keys.Control | Keys.T)) // for override connect
            {
                try
                {
                    MainV2.comPort.Open(false);
                }
                catch (Exception ex) { MessageBox.Show(ex.ToString()); }
                return true;
            }
            if (keyData == (Keys.Control | Keys.Y)) // for ryan beall
            {
#if MAVLINK10
                // write
                MainV2.comPort.doCommand(MAVLink.MAV_CMD.PREFLIGHT_STORAGE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                //read
                ///////MainV2.comPort.doCommand(MAVLink.MAV_CMD.PREFLIGHT_STORAGE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
#else
                MainV2.comPort.doAction(MAVLink.MAV_ACTION.MAV_ACTION_STORAGE_WRITE);
#endif
                MessageBox.Show("Done MAV_ACTION_STORAGE_WRITE");
                return true;
            }
            if (keyData == (Keys.Control | Keys.J)) // for jani
            {
                string data = "!!";
                Common.InputBox("inject", "enter data to be written", ref data);
                MainV2.comPort.Write(data + "\r");
                return true;
            }
            return base.ProcessCmdKey(ref msg, keyData);
        }

        public void changelanguage(CultureInfo ci)
        {
            if (ci != null && !Thread.CurrentThread.CurrentUICulture.Equals(ci))
            {
                Thread.CurrentThread.CurrentUICulture = ci;
                config["language"] = ci.Name;
                //System.Threading.Thread.CurrentThread.CurrentCulture = ci;

                HashSet<Control> views = new HashSet<Control> { FlightData, FlightPlanner, Simulation, Firmware };

                foreach (Control view in MyView.Controls)
                    views.Add(view);

                foreach (Control view in views)
                {
                    if (view != null)
                    {
                        ComponentResourceManager rm = new ComponentResourceManager(view.GetType());
                        foreach (Control ctrl in view.Controls)
                            applyresource(rm, ctrl);
                        rm.ApplyResources(view, "$this");
                    }
                }
            }
        }

        private void applyresource(ComponentResourceManager rm, Control ctrl)
        {
            rm.ApplyResources(ctrl, ctrl.Name);
            foreach (Control subctrl in ctrl.Controls)
                applyresource(rm, subctrl);

            if (ctrl.ContextMenu != null)
                applyresource(rm, ctrl.ContextMenu);


            if (ctrl is DataGridView)
            {
                foreach (DataGridViewColumn col in (ctrl as DataGridView).Columns)
                    rm.ApplyResources(col, col.Name);
            }


        }

        private void applyresource(ComponentResourceManager rm, Menu menu)
        {
            rm.ApplyResources(menu, menu.Name);
            foreach (MenuItem submenu in menu.MenuItems)
                applyresource(rm, submenu);
        }

        public static CultureInfo getcultureinfo(string name)
        {
            try
            {
                return new CultureInfo(name);
            }
            catch (Exception)
            {
                return null;
            }
        }

        private void MainV2_FormClosing(object sender, FormClosingEventArgs e)
        {
            config["MainHeight"] = this.Height;
            config["MainWidth"] = this.Width;
            config["MainMaximised"] = this.WindowState.ToString();

            config["MainLocX"] = this.Location.X.ToString();
            config["MainLocY"] = this.Location.Y.ToString();

            try
            {
                comPort.logreadmode = false;
                if (comPort.logfile != null)
                    comPort.logfile.Close();
            }
            catch { }

        }

        public static string getConfig(string paramname)
        {
            if (config[paramname] != null)
                return config[paramname].ToString();
            return "";
        }

        public void changeunits()
        {
            try
            {
                // dist
                if (MainV2.config["distunits"] != null)
                {
                    switch ((Common.distances)Enum.Parse(typeof(Common.distances), MainV2.config["distunits"].ToString()))
                    {
                        case Common.distances.Meters:
                            MainV2.cs.multiplierdist = 1;
                            break;
                        case Common.distances.Feet:
                            MainV2.cs.multiplierdist = 3.2808399f;
                            break;
                    }
                }

                // speed
                if (MainV2.config["speedunits"] != null)
                {
                    switch ((Common.speeds)Enum.Parse(typeof(Common.speeds), MainV2.config["speedunits"].ToString()))
                    {
                        case Common.speeds.ms:
                            MainV2.cs.multiplierspeed = 1;
                            break;
                        case Common.speeds.fps:
                            MainV2.cs.multiplierdist = 3.2808399f;
                            break;
                        case Common.speeds.kph:
                            MainV2.cs.multiplierspeed = 3.6f;
                            break;
                        case Common.speeds.mph:
                            MainV2.cs.multiplierspeed = 2.23693629f;
                            break;
                        case Common.speeds.knots:
                            MainV2.cs.multiplierspeed = 1.94384449f;
                            break;
                    }
                }
            }
            catch { }

        }
        private void CMB_baudrate_TextChanged(object sender, EventArgs e)
        {
            StringBuilder sb = new StringBuilder();
            int baud = 0;
            for (int i = 0; i < CMB_baudrate.Text.Length; i++)
                if (char.IsDigit(CMB_baudrate.Text[i]))
                {
                    sb.Append(CMB_baudrate.Text[i]);
                    baud = baud * 10 + CMB_baudrate.Text[i] - '0';
                }
            if (CMB_baudrate.Text != sb.ToString())
                CMB_baudrate.Text = sb.ToString();
            try
            {
                if (baud > 0 && comPort.BaseStream.BaudRate != baud)
                    comPort.BaseStream.BaudRate = baud;
            }
            catch (Exception) { }
        }
    }
}
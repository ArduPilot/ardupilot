namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigPlanner
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.label33 = new System.Windows.Forms.Label();
            this.CMB_ratesensors = new System.Windows.Forms.ComboBox();
            this.label26 = new System.Windows.Forms.Label();
            this.CMB_videoresolutions = new System.Windows.Forms.ComboBox();
            this.label12 = new System.Windows.Forms.Label();
            this.CHK_GDIPlus = new System.Windows.Forms.CheckBox();
            this.label24 = new System.Windows.Forms.Label();
            this.CHK_loadwponconnect = new System.Windows.Forms.CheckBox();
            this.label23 = new System.Windows.Forms.Label();
            this.NUM_tracklength = new System.Windows.Forms.NumericUpDown();
            this.CHK_speechaltwarning = new System.Windows.Forms.CheckBox();
            this.label108 = new System.Windows.Forms.Label();
            this.CHK_resetapmonconnect = new System.Windows.Forms.CheckBox();
            this.CHK_mavdebug = new System.Windows.Forms.CheckBox();
            this.label107 = new System.Windows.Forms.Label();
            this.CMB_raterc = new System.Windows.Forms.ComboBox();
            this.label104 = new System.Windows.Forms.Label();
            this.label103 = new System.Windows.Forms.Label();
            this.label102 = new System.Windows.Forms.Label();
            this.label101 = new System.Windows.Forms.Label();
            this.CMB_ratestatus = new System.Windows.Forms.ComboBox();
            this.CMB_rateposition = new System.Windows.Forms.ComboBox();
            this.CMB_rateattitude = new System.Windows.Forms.ComboBox();
            this.label99 = new System.Windows.Forms.Label();
            this.label98 = new System.Windows.Forms.Label();
            this.label97 = new System.Windows.Forms.Label();
            this.CMB_speedunits = new System.Windows.Forms.ComboBox();
            this.CMB_distunits = new System.Windows.Forms.ComboBox();
            this.label96 = new System.Windows.Forms.Label();
            this.label95 = new System.Windows.Forms.Label();
            this.CHK_speechbattery = new System.Windows.Forms.CheckBox();
            this.CHK_speechcustom = new System.Windows.Forms.CheckBox();
            this.CHK_speechmode = new System.Windows.Forms.CheckBox();
            this.CHK_speechwaypoint = new System.Windows.Forms.CheckBox();
            this.label94 = new System.Windows.Forms.Label();
            this.CMB_osdcolor = new System.Windows.Forms.ComboBox();
            this.CMB_language = new System.Windows.Forms.ComboBox();
            this.label93 = new System.Windows.Forms.Label();
            this.CHK_enablespeech = new System.Windows.Forms.CheckBox();
            this.CHK_hudshow = new System.Windows.Forms.CheckBox();
            this.label92 = new System.Windows.Forms.Label();
            this.CMB_videosources = new System.Windows.Forms.ComboBox();
            this.BUT_Joystick = new ArdupilotMega.MyButton();
            this.BUT_videostop = new ArdupilotMega.MyButton();
            this.BUT_videostart = new ArdupilotMega.MyButton();
            ((System.ComponentModel.ISupportInitialize)(this.NUM_tracklength)).BeginInit();
            this.SuspendLayout();
            // 
            // label33
            // 
            this.label33.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label33.Location = new System.Drawing.Point(517, 246);
            this.label33.Name = "label33";
            this.label33.Size = new System.Drawing.Size(43, 13);
            this.label33.TabIndex = 87;
            this.label33.Text = "Sensor";
            // 
            // CMB_ratesensors
            // 
            this.CMB_ratesensors.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_ratesensors.FormattingEnabled = true;
            this.CMB_ratesensors.Items.AddRange(new object[] {
            "0",
            "1",
            "3",
            "10",
            "50"});
            this.CMB_ratesensors.Location = new System.Drawing.Point(564, 243);
            this.CMB_ratesensors.Name = "CMB_ratesensors";
            this.CMB_ratesensors.Size = new System.Drawing.Size(40, 21);
            this.CMB_ratesensors.TabIndex = 88;
            // 
            // label26
            // 
            this.label26.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label26.Location = new System.Drawing.Point(15, 52);
            this.label26.Name = "label26";
            this.label26.Size = new System.Drawing.Size(100, 23);
            this.label26.TabIndex = 86;
            this.label26.Text = "Video Format";
            // 
            // CMB_videoresolutions
            // 
            this.CMB_videoresolutions.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_videoresolutions.FormattingEnabled = true;
            this.CMB_videoresolutions.Location = new System.Drawing.Point(124, 49);
            this.CMB_videoresolutions.Name = "CMB_videoresolutions";
            this.CMB_videoresolutions.Size = new System.Drawing.Size(408, 21);
            this.CMB_videoresolutions.TabIndex = 44;
            // 
            // label12
            // 
            this.label12.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label12.Location = new System.Drawing.Point(15, 342);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(61, 13);
            this.label12.TabIndex = 84;
            this.label12.Text = "HUD";
            // 
            // CHK_GDIPlus
            // 
            this.CHK_GDIPlus.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_GDIPlus.Location = new System.Drawing.Point(124, 342);
            this.CHK_GDIPlus.Name = "CHK_GDIPlus";
            this.CHK_GDIPlus.Size = new System.Drawing.Size(177, 17);
            this.CHK_GDIPlus.TabIndex = 85;
            this.CHK_GDIPlus.Text = "GDI+ (old type)";
            this.CHK_GDIPlus.UseVisualStyleBackColor = true;
            this.CHK_GDIPlus.CheckedChanged += new System.EventHandler(this.CHK_GDIPlus_CheckedChanged);
            // 
            // label24
            // 
            this.label24.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label24.Location = new System.Drawing.Point(15, 320);
            this.label24.Name = "label24";
            this.label24.Size = new System.Drawing.Size(61, 13);
            this.label24.TabIndex = 82;
            this.label24.Text = "Waypoints";
            // 
            // CHK_loadwponconnect
            // 
            this.CHK_loadwponconnect.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_loadwponconnect.Location = new System.Drawing.Point(124, 319);
            this.CHK_loadwponconnect.Name = "CHK_loadwponconnect";
            this.CHK_loadwponconnect.Size = new System.Drawing.Size(177, 17);
            this.CHK_loadwponconnect.TabIndex = 83;
            this.CHK_loadwponconnect.Text = "Load Waypoints on connect?";
            this.CHK_loadwponconnect.UseVisualStyleBackColor = true;
            this.CHK_loadwponconnect.CheckedChanged += new System.EventHandler(this.CHK_loadwponconnect_CheckedChanged);
            // 
            // label23
            // 
            this.label23.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label23.Location = new System.Drawing.Point(15, 294);
            this.label23.Name = "label23";
            this.label23.Size = new System.Drawing.Size(103, 18);
            this.label23.TabIndex = 81;
            this.label23.Text = "Track Length";
            // 
            // NUM_tracklength
            // 
            this.NUM_tracklength.Increment = new decimal(new int[] {
            100,
            0,
            0,
            0});
            this.NUM_tracklength.Location = new System.Drawing.Point(124, 293);
            this.NUM_tracklength.Maximum = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            this.NUM_tracklength.Minimum = new decimal(new int[] {
            100,
            0,
            0,
            0});
            this.NUM_tracklength.Name = "NUM_tracklength";
            this.NUM_tracklength.Size = new System.Drawing.Size(67, 20);
            this.NUM_tracklength.TabIndex = 80;
            this.NUM_tracklength.Value = new decimal(new int[] {
            200,
            0,
            0,
            0});
            this.NUM_tracklength.ValueChanged += new System.EventHandler(this.NUM_tracklength_ValueChanged);
            // 
            // CHK_speechaltwarning
            // 
            this.CHK_speechaltwarning.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_speechaltwarning.Location = new System.Drawing.Point(564, 109);
            this.CHK_speechaltwarning.Name = "CHK_speechaltwarning";
            this.CHK_speechaltwarning.Size = new System.Drawing.Size(102, 17);
            this.CHK_speechaltwarning.TabIndex = 79;
            this.CHK_speechaltwarning.Text = "Alt Warning";
            this.CHK_speechaltwarning.UseVisualStyleBackColor = true;
            this.CHK_speechaltwarning.CheckedChanged += new System.EventHandler(this.CHK_speechaltwarning_CheckedChanged);
            // 
            // label108
            // 
            this.label108.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label108.Location = new System.Drawing.Point(15, 271);
            this.label108.Name = "label108";
            this.label108.Size = new System.Drawing.Size(61, 13);
            this.label108.TabIndex = 45;
            this.label108.Text = "APM Reset";
            // 
            // CHK_resetapmonconnect
            // 
            this.CHK_resetapmonconnect.Checked = true;
            this.CHK_resetapmonconnect.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHK_resetapmonconnect.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_resetapmonconnect.Location = new System.Drawing.Point(124, 269);
            this.CHK_resetapmonconnect.Name = "CHK_resetapmonconnect";
            this.CHK_resetapmonconnect.Size = new System.Drawing.Size(163, 17);
            this.CHK_resetapmonconnect.TabIndex = 46;
            this.CHK_resetapmonconnect.Text = "Reset APM on USB Connect";
            this.CHK_resetapmonconnect.UseVisualStyleBackColor = true;
            this.CHK_resetapmonconnect.CheckedChanged += new System.EventHandler(this.CHK_resetapmonconnect_CheckedChanged);
            // 
            // CHK_mavdebug
            // 
            this.CHK_mavdebug.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.CHK_mavdebug.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_mavdebug.Location = new System.Drawing.Point(15, 378);
            this.CHK_mavdebug.Name = "CHK_mavdebug";
            this.CHK_mavdebug.Size = new System.Drawing.Size(144, 17);
            this.CHK_mavdebug.TabIndex = 47;
            this.CHK_mavdebug.Text = "Mavlink Message Debug";
            this.CHK_mavdebug.UseVisualStyleBackColor = true;
            this.CHK_mavdebug.CheckedChanged += new System.EventHandler(this.CHK_mavdebug_CheckedChanged);
            // 
            // label107
            // 
            this.label107.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label107.Location = new System.Drawing.Point(439, 246);
            this.label107.Name = "label107";
            this.label107.Size = new System.Drawing.Size(22, 13);
            this.label107.TabIndex = 48;
            this.label107.Text = "RC";
            // 
            // CMB_raterc
            // 
            this.CMB_raterc.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_raterc.FormattingEnabled = true;
            this.CMB_raterc.Items.AddRange(new object[] {
            "0",
            "1",
            "3",
            "10"});
            this.CMB_raterc.Location = new System.Drawing.Point(470, 242);
            this.CMB_raterc.Name = "CMB_raterc";
            this.CMB_raterc.Size = new System.Drawing.Size(40, 21);
            this.CMB_raterc.TabIndex = 49;
            this.CMB_raterc.SelectedIndexChanged += new System.EventHandler(this.CMB_raterc_SelectedIndexChanged);
            // 
            // label104
            // 
            this.label104.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label104.Location = new System.Drawing.Point(319, 246);
            this.label104.Name = "label104";
            this.label104.Size = new System.Drawing.Size(69, 13);
            this.label104.TabIndex = 50;
            this.label104.Text = "Mode/Status";
            // 
            // label103
            // 
            this.label103.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label103.Location = new System.Drawing.Point(219, 246);
            this.label103.Name = "label103";
            this.label103.Size = new System.Drawing.Size(44, 13);
            this.label103.TabIndex = 51;
            this.label103.Text = "Position";
            // 
            // label102
            // 
            this.label102.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label102.Location = new System.Drawing.Point(121, 246);
            this.label102.Name = "label102";
            this.label102.Size = new System.Drawing.Size(43, 13);
            this.label102.TabIndex = 52;
            this.label102.Text = "Attitude";
            // 
            // label101
            // 
            this.label101.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label101.Location = new System.Drawing.Point(12, 246);
            this.label101.Name = "label101";
            this.label101.Size = new System.Drawing.Size(84, 13);
            this.label101.TabIndex = 53;
            this.label101.Text = "Telemetry Rates";
            // 
            // CMB_ratestatus
            // 
            this.CMB_ratestatus.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_ratestatus.FormattingEnabled = true;
            this.CMB_ratestatus.Items.AddRange(new object[] {
            "0",
            "1",
            "3",
            "10"});
            this.CMB_ratestatus.Location = new System.Drawing.Point(393, 242);
            this.CMB_ratestatus.Name = "CMB_ratestatus";
            this.CMB_ratestatus.Size = new System.Drawing.Size(40, 21);
            this.CMB_ratestatus.TabIndex = 54;
            this.CMB_ratestatus.SelectedIndexChanged += new System.EventHandler(this.CMB_ratestatus_SelectedIndexChanged);
            // 
            // CMB_rateposition
            // 
            this.CMB_rateposition.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_rateposition.FormattingEnabled = true;
            this.CMB_rateposition.Items.AddRange(new object[] {
            "0",
            "1",
            "3",
            "10"});
            this.CMB_rateposition.Location = new System.Drawing.Point(273, 242);
            this.CMB_rateposition.Name = "CMB_rateposition";
            this.CMB_rateposition.Size = new System.Drawing.Size(40, 21);
            this.CMB_rateposition.TabIndex = 55;
            this.CMB_rateposition.SelectedIndexChanged += new System.EventHandler(this.CMB_rateposition_SelectedIndexChanged);
            // 
            // CMB_rateattitude
            // 
            this.CMB_rateattitude.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_rateattitude.FormattingEnabled = true;
            this.CMB_rateattitude.Items.AddRange(new object[] {
            "0",
            "1",
            "3",
            "10"});
            this.CMB_rateattitude.Location = new System.Drawing.Point(173, 242);
            this.CMB_rateattitude.Name = "CMB_rateattitude";
            this.CMB_rateattitude.Size = new System.Drawing.Size(40, 21);
            this.CMB_rateattitude.TabIndex = 56;
            this.CMB_rateattitude.SelectedIndexChanged += new System.EventHandler(this.CMB_rateattitude_SelectedIndexChanged);
            // 
            // label99
            // 
            this.label99.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label99.Location = new System.Drawing.Point(268, 211);
            this.label99.Name = "label99";
            this.label99.Size = new System.Drawing.Size(402, 13);
            this.label99.TabIndex = 57;
            this.label99.Text = "NOTE: The Configuration Tab will NOT display these units, as those are raw values" +
    ".\r\n";
            // 
            // label98
            // 
            this.label98.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label98.Location = new System.Drawing.Point(15, 219);
            this.label98.Name = "label98";
            this.label98.Size = new System.Drawing.Size(65, 13);
            this.label98.TabIndex = 58;
            this.label98.Text = "Speed Units";
            // 
            // label97
            // 
            this.label97.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label97.Location = new System.Drawing.Point(15, 191);
            this.label97.Name = "label97";
            this.label97.Size = new System.Drawing.Size(52, 13);
            this.label97.TabIndex = 59;
            this.label97.Text = "Dist Units";
            // 
            // CMB_speedunits
            // 
            this.CMB_speedunits.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_speedunits.FormattingEnabled = true;
            this.CMB_speedunits.Location = new System.Drawing.Point(124, 216);
            this.CMB_speedunits.Name = "CMB_speedunits";
            this.CMB_speedunits.Size = new System.Drawing.Size(138, 21);
            this.CMB_speedunits.TabIndex = 60;
            this.CMB_speedunits.SelectedIndexChanged += new System.EventHandler(this.CMB_speedunits_SelectedIndexChanged);
            // 
            // CMB_distunits
            // 
            this.CMB_distunits.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_distunits.FormattingEnabled = true;
            this.CMB_distunits.Location = new System.Drawing.Point(124, 189);
            this.CMB_distunits.Name = "CMB_distunits";
            this.CMB_distunits.Size = new System.Drawing.Size(138, 21);
            this.CMB_distunits.TabIndex = 61;
            this.CMB_distunits.SelectedIndexChanged += new System.EventHandler(this.CMB_distunits_SelectedIndexChanged);
            // 
            // label96
            // 
            this.label96.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label96.Location = new System.Drawing.Point(15, 164);
            this.label96.Name = "label96";
            this.label96.Size = new System.Drawing.Size(45, 13);
            this.label96.TabIndex = 62;
            this.label96.Text = "Joystick";
            // 
            // label95
            // 
            this.label95.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label95.Location = new System.Drawing.Point(15, 113);
            this.label95.Name = "label95";
            this.label95.Size = new System.Drawing.Size(44, 13);
            this.label95.TabIndex = 63;
            this.label95.Text = "Speech";
            // 
            // CHK_speechbattery
            // 
            this.CHK_speechbattery.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_speechbattery.Location = new System.Drawing.Point(456, 109);
            this.CHK_speechbattery.Name = "CHK_speechbattery";
            this.CHK_speechbattery.Size = new System.Drawing.Size(102, 17);
            this.CHK_speechbattery.TabIndex = 64;
            this.CHK_speechbattery.Text = "Battery Warning";
            this.CHK_speechbattery.UseVisualStyleBackColor = true;
            this.CHK_speechbattery.CheckedChanged += new System.EventHandler(this.CHK_speechbattery_CheckedChanged);
            // 
            // CHK_speechcustom
            // 
            this.CHK_speechcustom.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_speechcustom.Location = new System.Drawing.Point(363, 109);
            this.CHK_speechcustom.Name = "CHK_speechcustom";
            this.CHK_speechcustom.Size = new System.Drawing.Size(87, 17);
            this.CHK_speechcustom.TabIndex = 65;
            this.CHK_speechcustom.Text = "Time Interval";
            this.CHK_speechcustom.UseVisualStyleBackColor = true;
            this.CHK_speechcustom.CheckedChanged += new System.EventHandler(this.CHK_speechcustom_CheckedChanged);
            // 
            // CHK_speechmode
            // 
            this.CHK_speechmode.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_speechmode.Location = new System.Drawing.Point(307, 109);
            this.CHK_speechmode.Name = "CHK_speechmode";
            this.CHK_speechmode.Size = new System.Drawing.Size(56, 17);
            this.CHK_speechmode.TabIndex = 66;
            this.CHK_speechmode.Text = "Mode ";
            this.CHK_speechmode.UseVisualStyleBackColor = true;
            this.CHK_speechmode.CheckedChanged += new System.EventHandler(this.CHK_speechmode_CheckedChanged);
            // 
            // CHK_speechwaypoint
            // 
            this.CHK_speechwaypoint.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_speechwaypoint.Location = new System.Drawing.Point(230, 109);
            this.CHK_speechwaypoint.Name = "CHK_speechwaypoint";
            this.CHK_speechwaypoint.Size = new System.Drawing.Size(71, 17);
            this.CHK_speechwaypoint.TabIndex = 67;
            this.CHK_speechwaypoint.Text = "Waypoint";
            this.CHK_speechwaypoint.UseVisualStyleBackColor = true;
            this.CHK_speechwaypoint.CheckedChanged += new System.EventHandler(this.CHK_speechwaypoint_CheckedChanged);
            // 
            // label94
            // 
            this.label94.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label94.Location = new System.Drawing.Point(15, 85);
            this.label94.Name = "label94";
            this.label94.Size = new System.Drawing.Size(57, 13);
            this.label94.TabIndex = 68;
            this.label94.Text = "OSD Color";
            // 
            // CMB_osdcolor
            // 
            this.CMB_osdcolor.DrawMode = System.Windows.Forms.DrawMode.OwnerDrawFixed;
            this.CMB_osdcolor.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_osdcolor.FormattingEnabled = true;
            this.CMB_osdcolor.Location = new System.Drawing.Point(124, 82);
            this.CMB_osdcolor.Name = "CMB_osdcolor";
            this.CMB_osdcolor.Size = new System.Drawing.Size(138, 21);
            this.CMB_osdcolor.TabIndex = 69;
            this.CMB_osdcolor.SelectedIndexChanged += new System.EventHandler(this.CMB_osdcolor_SelectedIndexChanged);
            // 
            // CMB_language
            // 
            this.CMB_language.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_language.FormattingEnabled = true;
            this.CMB_language.Location = new System.Drawing.Point(124, 133);
            this.CMB_language.Name = "CMB_language";
            this.CMB_language.Size = new System.Drawing.Size(138, 21);
            this.CMB_language.TabIndex = 70;
            this.CMB_language.SelectedIndexChanged += new System.EventHandler(this.CMB_language_SelectedIndexChanged);
            // 
            // label93
            // 
            this.label93.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label93.Location = new System.Drawing.Point(15, 137);
            this.label93.Name = "label93";
            this.label93.Size = new System.Drawing.Size(69, 13);
            this.label93.TabIndex = 71;
            this.label93.Text = "UI Language";
            // 
            // CHK_enablespeech
            // 
            this.CHK_enablespeech.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_enablespeech.Location = new System.Drawing.Point(124, 109);
            this.CHK_enablespeech.Name = "CHK_enablespeech";
            this.CHK_enablespeech.Size = new System.Drawing.Size(99, 17);
            this.CHK_enablespeech.TabIndex = 72;
            this.CHK_enablespeech.Text = "Enable Speech";
            this.CHK_enablespeech.UseVisualStyleBackColor = true;
            this.CHK_enablespeech.CheckedChanged += new System.EventHandler(this.CHK_enablespeech_CheckedChanged);
            // 
            // CHK_hudshow
            // 
            this.CHK_hudshow.Checked = true;
            this.CHK_hudshow.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHK_hudshow.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_hudshow.Location = new System.Drawing.Point(537, 17);
            this.CHK_hudshow.Name = "CHK_hudshow";
            this.CHK_hudshow.Size = new System.Drawing.Size(125, 17);
            this.CHK_hudshow.TabIndex = 73;
            this.CHK_hudshow.Text = "Enable HUD Overlay";
            this.CHK_hudshow.UseVisualStyleBackColor = true;
            this.CHK_hudshow.Click += new System.EventHandler(this.CHK_hudshow_CheckedChanged);
            // 
            // label92
            // 
            this.label92.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label92.Location = new System.Drawing.Point(15, 18);
            this.label92.Name = "label92";
            this.label92.Size = new System.Drawing.Size(100, 23);
            this.label92.TabIndex = 74;
            this.label92.Text = "Video Device";
            // 
            // CMB_videosources
            // 
            this.CMB_videosources.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_videosources.FormattingEnabled = true;
            this.CMB_videosources.Location = new System.Drawing.Point(124, 15);
            this.CMB_videosources.Name = "CMB_videosources";
            this.CMB_videosources.Size = new System.Drawing.Size(245, 21);
            this.CMB_videosources.TabIndex = 75;
            this.CMB_videosources.SelectedIndexChanged += new System.EventHandler(this.CMB_videosources_SelectedIndexChanged);
            // 
            // BUT_Joystick
            // 
            this.BUT_Joystick.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_Joystick.Location = new System.Drawing.Point(124, 160);
            this.BUT_Joystick.Name = "BUT_Joystick";
            this.BUT_Joystick.Size = new System.Drawing.Size(99, 23);
            this.BUT_Joystick.TabIndex = 76;
            this.BUT_Joystick.Text = "Joystick Setup";
            this.BUT_Joystick.UseVisualStyleBackColor = true;
            this.BUT_Joystick.Click += new System.EventHandler(this.BUT_Joystick_Click);
            // 
            // BUT_videostop
            // 
            this.BUT_videostop.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_videostop.Location = new System.Drawing.Point(456, 13);
            this.BUT_videostop.Name = "BUT_videostop";
            this.BUT_videostop.Size = new System.Drawing.Size(75, 23);
            this.BUT_videostop.TabIndex = 77;
            this.BUT_videostop.Text = "Stop";
            this.BUT_videostop.UseVisualStyleBackColor = true;
            this.BUT_videostop.Click += new System.EventHandler(this.BUT_videostop_Click);
            // 
            // BUT_videostart
            // 
            this.BUT_videostart.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_videostart.Location = new System.Drawing.Point(375, 13);
            this.BUT_videostart.Name = "BUT_videostart";
            this.BUT_videostart.Size = new System.Drawing.Size(75, 23);
            this.BUT_videostart.TabIndex = 78;
            this.BUT_videostart.Text = "Start";
            this.BUT_videostart.UseVisualStyleBackColor = true;
            this.BUT_videostart.Click += new System.EventHandler(this.BUT_videostart_Click);
            // 
            // ConfigPlanner
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.label33);
            this.Controls.Add(this.CMB_ratesensors);
            this.Controls.Add(this.label26);
            this.Controls.Add(this.CMB_videoresolutions);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.CHK_GDIPlus);
            this.Controls.Add(this.label24);
            this.Controls.Add(this.CHK_loadwponconnect);
            this.Controls.Add(this.label23);
            this.Controls.Add(this.NUM_tracklength);
            this.Controls.Add(this.CHK_speechaltwarning);
            this.Controls.Add(this.label108);
            this.Controls.Add(this.CHK_resetapmonconnect);
            this.Controls.Add(this.CHK_mavdebug);
            this.Controls.Add(this.label107);
            this.Controls.Add(this.CMB_raterc);
            this.Controls.Add(this.label104);
            this.Controls.Add(this.label103);
            this.Controls.Add(this.label102);
            this.Controls.Add(this.label101);
            this.Controls.Add(this.CMB_ratestatus);
            this.Controls.Add(this.CMB_rateposition);
            this.Controls.Add(this.CMB_rateattitude);
            this.Controls.Add(this.label99);
            this.Controls.Add(this.label98);
            this.Controls.Add(this.label97);
            this.Controls.Add(this.CMB_speedunits);
            this.Controls.Add(this.CMB_distunits);
            this.Controls.Add(this.label96);
            this.Controls.Add(this.label95);
            this.Controls.Add(this.CHK_speechbattery);
            this.Controls.Add(this.CHK_speechcustom);
            this.Controls.Add(this.CHK_speechmode);
            this.Controls.Add(this.CHK_speechwaypoint);
            this.Controls.Add(this.label94);
            this.Controls.Add(this.CMB_osdcolor);
            this.Controls.Add(this.CMB_language);
            this.Controls.Add(this.label93);
            this.Controls.Add(this.CHK_enablespeech);
            this.Controls.Add(this.CHK_hudshow);
            this.Controls.Add(this.label92);
            this.Controls.Add(this.CMB_videosources);
            this.Controls.Add(this.BUT_Joystick);
            this.Controls.Add(this.BUT_videostop);
            this.Controls.Add(this.BUT_videostart);
            this.Name = "ConfigPlanner";
            this.Size = new System.Drawing.Size(682, 398);
            ((System.ComponentModel.ISupportInitialize)(this.NUM_tracklength)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Label label33;
        private System.Windows.Forms.ComboBox CMB_ratesensors;
        private System.Windows.Forms.Label label26;
        private System.Windows.Forms.ComboBox CMB_videoresolutions;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.CheckBox CHK_GDIPlus;
        private System.Windows.Forms.Label label24;
        private System.Windows.Forms.CheckBox CHK_loadwponconnect;
        private System.Windows.Forms.Label label23;
        private System.Windows.Forms.NumericUpDown NUM_tracklength;
        private System.Windows.Forms.CheckBox CHK_speechaltwarning;
        private System.Windows.Forms.Label label108;
        private System.Windows.Forms.CheckBox CHK_resetapmonconnect;
        private System.Windows.Forms.CheckBox CHK_mavdebug;
        private System.Windows.Forms.Label label107;
        private System.Windows.Forms.ComboBox CMB_raterc;
        private System.Windows.Forms.Label label104;
        private System.Windows.Forms.Label label103;
        private System.Windows.Forms.Label label102;
        private System.Windows.Forms.Label label101;
        private System.Windows.Forms.ComboBox CMB_ratestatus;
        private System.Windows.Forms.ComboBox CMB_rateposition;
        private System.Windows.Forms.ComboBox CMB_rateattitude;
        private System.Windows.Forms.Label label99;
        private System.Windows.Forms.Label label98;
        private System.Windows.Forms.Label label97;
        private System.Windows.Forms.ComboBox CMB_speedunits;
        private System.Windows.Forms.ComboBox CMB_distunits;
        private System.Windows.Forms.Label label96;
        private System.Windows.Forms.Label label95;
        private System.Windows.Forms.CheckBox CHK_speechbattery;
        private System.Windows.Forms.CheckBox CHK_speechcustom;
        private System.Windows.Forms.CheckBox CHK_speechmode;
        private System.Windows.Forms.CheckBox CHK_speechwaypoint;
        private System.Windows.Forms.Label label94;
        private System.Windows.Forms.ComboBox CMB_osdcolor;
        private System.Windows.Forms.ComboBox CMB_language;
        private System.Windows.Forms.Label label93;
        private System.Windows.Forms.CheckBox CHK_enablespeech;
        private System.Windows.Forms.CheckBox CHK_hudshow;
        private System.Windows.Forms.Label label92;
        private System.Windows.Forms.ComboBox CMB_videosources;
        private MyButton BUT_Joystick;
        private MyButton BUT_videostop;
        private MyButton BUT_videostart;
    }
}

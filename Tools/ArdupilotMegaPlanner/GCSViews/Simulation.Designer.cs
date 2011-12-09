namespace ArdupilotMega.GCSViews
{
    partial class Simulation
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

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Simulation));
            this.CHKREV_roll = new System.Windows.Forms.CheckBox();
            this.CHKREV_pitch = new System.Windows.Forms.CheckBox();
            this.CHKREV_rudder = new System.Windows.Forms.CheckBox();
            this.GPSrate = new System.Windows.Forms.ComboBox();
            this.ConnectComPort = new ArdupilotMega.MyButton();
            this.OutputLog = new System.Windows.Forms.RichTextBox();
            this.TXT_roll = new ArdupilotMega.MyLabel();
            this.TXT_pitch = new ArdupilotMega.MyLabel();
            this.TXT_heading = new ArdupilotMega.MyLabel();
            this.TXT_wpdist = new ArdupilotMega.MyLabel();
            this.currentStateBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.TXT_bererror = new ArdupilotMega.MyLabel();
            this.TXT_alterror = new ArdupilotMega.MyLabel();
            this.TXT_lat = new ArdupilotMega.MyLabel();
            this.TXT_long = new ArdupilotMega.MyLabel();
            this.TXT_alt = new ArdupilotMega.MyLabel();
            this.SaveSettings = new ArdupilotMega.MyButton();
            this.RAD_softXplanes = new System.Windows.Forms.RadioButton();
            this.RAD_softFlightGear = new System.Windows.Forms.RadioButton();
            this.TXT_servoroll = new ArdupilotMega.MyLabel();
            this.TXT_servopitch = new ArdupilotMega.MyLabel();
            this.TXT_servorudder = new ArdupilotMega.MyLabel();
            this.TXT_servothrottle = new ArdupilotMega.MyLabel();
            this.panel1 = new System.Windows.Forms.Panel();
            this.label4 = new ArdupilotMega.MyLabel();
            this.label3 = new ArdupilotMega.MyLabel();
            this.label2 = new ArdupilotMega.MyLabel();
            this.label1 = new ArdupilotMega.MyLabel();
            this.panel2 = new System.Windows.Forms.Panel();
            this.label30 = new ArdupilotMega.MyLabel();
            this.TXT_yaw = new ArdupilotMega.MyLabel();
            this.label11 = new ArdupilotMega.MyLabel();
            this.label7 = new ArdupilotMega.MyLabel();
            this.label6 = new ArdupilotMega.MyLabel();
            this.label5 = new ArdupilotMega.MyLabel();
            this.label8 = new ArdupilotMega.MyLabel();
            this.label9 = new ArdupilotMega.MyLabel();
            this.label10 = new ArdupilotMega.MyLabel();
            this.panel3 = new System.Windows.Forms.Panel();
            this.label16 = new ArdupilotMega.MyLabel();
            this.label15 = new ArdupilotMega.MyLabel();
            this.label14 = new ArdupilotMega.MyLabel();
            this.label13 = new ArdupilotMega.MyLabel();
            this.label12 = new ArdupilotMega.MyLabel();
            this.panel4 = new System.Windows.Forms.Panel();
            this.label20 = new ArdupilotMega.MyLabel();
            this.label19 = new ArdupilotMega.MyLabel();
            this.TXT_control_mode = new ArdupilotMega.MyLabel();
            this.TXT_WP = new ArdupilotMega.MyLabel();
            this.label18 = new ArdupilotMega.MyLabel();
            this.label17 = new ArdupilotMega.MyLabel();
            this.panel5 = new System.Windows.Forms.Panel();
            this.zg1 = new ZedGraph.ZedGraphControl();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.panel6 = new System.Windows.Forms.Panel();
            this.label28 = new ArdupilotMega.MyLabel();
            this.label29 = new ArdupilotMega.MyLabel();
            this.label27 = new ArdupilotMega.MyLabel();
            this.label25 = new ArdupilotMega.MyLabel();
            this.TXT_throttlegain = new System.Windows.Forms.TextBox();
            this.label24 = new ArdupilotMega.MyLabel();
            this.label23 = new ArdupilotMega.MyLabel();
            this.label22 = new ArdupilotMega.MyLabel();
            this.label21 = new ArdupilotMega.MyLabel();
            this.TXT_ruddergain = new System.Windows.Forms.TextBox();
            this.TXT_pitchgain = new System.Windows.Forms.TextBox();
            this.TXT_rollgain = new System.Windows.Forms.TextBox();
            this.label26 = new ArdupilotMega.MyLabel();
            this.CHKdisplayall = new System.Windows.Forms.CheckBox();
            this.CHKgraphroll = new System.Windows.Forms.CheckBox();
            this.CHKgraphpitch = new System.Windows.Forms.CheckBox();
            this.CHKgraphrudder = new System.Windows.Forms.CheckBox();
            this.CHKgraphthrottle = new System.Windows.Forms.CheckBox();
            this.but_advsettings = new ArdupilotMega.MyButton();
            this.chkSensor = new System.Windows.Forms.CheckBox();
            this.CHK_quad = new System.Windows.Forms.CheckBox();
            this.BUT_startfgquad = new ArdupilotMega.MyButton();
            this.BUT_startfgplane = new ArdupilotMega.MyButton();
            this.BUT_startxplane = new ArdupilotMega.MyButton();
            this.CHK_heli = new System.Windows.Forms.CheckBox();
            this.RAD_aerosimrc = new System.Windows.Forms.RadioButton();
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
            this.RAD_JSBSim = new System.Windows.Forms.RadioButton();
            ((System.ComponentModel.ISupportInitialize)(this.currentStateBindingSource)).BeginInit();
            this.panel1.SuspendLayout();
            this.panel2.SuspendLayout();
            this.panel3.SuspendLayout();
            this.panel4.SuspendLayout();
            this.panel5.SuspendLayout();
            this.panel6.SuspendLayout();
            this.SuspendLayout();
            // 
            // CHKREV_roll
            // 
            resources.ApplyResources(this.CHKREV_roll, "CHKREV_roll");
            this.CHKREV_roll.Name = "CHKREV_roll";
            this.CHKREV_roll.UseVisualStyleBackColor = true;
            this.CHKREV_roll.CheckedChanged += new System.EventHandler(this.CHKREV_roll_CheckedChanged);
            // 
            // CHKREV_pitch
            // 
            resources.ApplyResources(this.CHKREV_pitch, "CHKREV_pitch");
            this.CHKREV_pitch.Name = "CHKREV_pitch";
            this.CHKREV_pitch.UseVisualStyleBackColor = true;
            this.CHKREV_pitch.CheckedChanged += new System.EventHandler(this.CHKREV_pitch_CheckedChanged);
            // 
            // CHKREV_rudder
            // 
            resources.ApplyResources(this.CHKREV_rudder, "CHKREV_rudder");
            this.CHKREV_rudder.Name = "CHKREV_rudder";
            this.CHKREV_rudder.UseVisualStyleBackColor = true;
            this.CHKREV_rudder.CheckedChanged += new System.EventHandler(this.CHKREV_rudder_CheckedChanged);
            // 
            // GPSrate
            // 
            this.GPSrate.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.GPSrate.FormattingEnabled = true;
            this.GPSrate.Items.AddRange(new object[] {
            resources.GetString("GPSrate.Items"),
            resources.GetString("GPSrate.Items1"),
            resources.GetString("GPSrate.Items2"),
            resources.GetString("GPSrate.Items3"),
            resources.GetString("GPSrate.Items4"),
            resources.GetString("GPSrate.Items5"),
            resources.GetString("GPSrate.Items6"),
            resources.GetString("GPSrate.Items7")});
            resources.ApplyResources(this.GPSrate, "GPSrate");
            this.GPSrate.Name = "GPSrate";
            this.GPSrate.SelectedIndexChanged += new System.EventHandler(this.GPSrate_SelectedIndexChanged);
            this.GPSrate.KeyDown += new System.Windows.Forms.KeyEventHandler(this.GPSrate_KeyDown);
            this.GPSrate.Leave += new System.EventHandler(this.GPSrate_Leave);
            // 
            // ConnectComPort
            // 
            resources.ApplyResources(this.ConnectComPort, "ConnectComPort");
            this.ConnectComPort.Name = "ConnectComPort";
            this.ConnectComPort.UseVisualStyleBackColor = true;
            this.ConnectComPort.Click += new System.EventHandler(this.ConnectComPort_Click);
            // 
            // OutputLog
            // 
            resources.ApplyResources(this.OutputLog, "OutputLog");
            this.OutputLog.Name = "OutputLog";
            this.OutputLog.TextChanged += new System.EventHandler(this.OutputLog_TextChanged);
            // 
            // TXT_roll
            // 
            resources.ApplyResources(this.TXT_roll, "TXT_roll");
            this.TXT_roll.Name = "TXT_roll";
            this.TXT_roll.resize = false;
            // 
            // TXT_pitch
            // 
            resources.ApplyResources(this.TXT_pitch, "TXT_pitch");
            this.TXT_pitch.Name = "TXT_pitch";
            this.TXT_pitch.resize = false;
            // 
            // TXT_heading
            // 
            resources.ApplyResources(this.TXT_heading, "TXT_heading");
            this.TXT_heading.Name = "TXT_heading";
            this.TXT_heading.resize = false;
            // 
            // TXT_wpdist
            // 
            resources.ApplyResources(this.TXT_wpdist, "TXT_wpdist");
            this.TXT_wpdist.Name = "TXT_wpdist";
            this.TXT_wpdist.resize = false;
            // 
            // currentStateBindingSource
            // 
            this.currentStateBindingSource.DataSource = typeof(ArdupilotMega.CurrentState);
            // 
            // TXT_bererror
            // 
            resources.ApplyResources(this.TXT_bererror, "TXT_bererror");
            this.TXT_bererror.Name = "TXT_bererror";
            this.TXT_bererror.resize = false;
            // 
            // TXT_alterror
            // 
            resources.ApplyResources(this.TXT_alterror, "TXT_alterror");
            this.TXT_alterror.Name = "TXT_alterror";
            this.TXT_alterror.resize = false;
            // 
            // TXT_lat
            // 
            resources.ApplyResources(this.TXT_lat, "TXT_lat");
            this.TXT_lat.Name = "TXT_lat";
            this.TXT_lat.resize = false;
            // 
            // TXT_long
            // 
            resources.ApplyResources(this.TXT_long, "TXT_long");
            this.TXT_long.Name = "TXT_long";
            this.TXT_long.resize = false;
            // 
            // TXT_alt
            // 
            resources.ApplyResources(this.TXT_alt, "TXT_alt");
            this.TXT_alt.Name = "TXT_alt";
            this.TXT_alt.resize = false;
            // 
            // SaveSettings
            // 
            resources.ApplyResources(this.SaveSettings, "SaveSettings");
            this.SaveSettings.Name = "SaveSettings";
            this.SaveSettings.UseVisualStyleBackColor = true;
            this.SaveSettings.Click += new System.EventHandler(this.SaveSettings_Click);
            // 
            // RAD_softXplanes
            // 
            resources.ApplyResources(this.RAD_softXplanes, "RAD_softXplanes");
            this.RAD_softXplanes.Checked = true;
            this.RAD_softXplanes.Name = "RAD_softXplanes";
            this.RAD_softXplanes.TabStop = true;
            this.toolTip1.SetToolTip(this.RAD_softXplanes, resources.GetString("RAD_softXplanes.ToolTip"));
            this.RAD_softXplanes.UseVisualStyleBackColor = true;
            this.RAD_softXplanes.CheckedChanged += new System.EventHandler(this.RAD_softXplanes_CheckedChanged);
            // 
            // RAD_softFlightGear
            // 
            resources.ApplyResources(this.RAD_softFlightGear, "RAD_softFlightGear");
            this.RAD_softFlightGear.Name = "RAD_softFlightGear";
            this.toolTip1.SetToolTip(this.RAD_softFlightGear, resources.GetString("RAD_softFlightGear.ToolTip"));
            this.RAD_softFlightGear.UseVisualStyleBackColor = true;
            this.RAD_softFlightGear.CheckedChanged += new System.EventHandler(this.RAD_softFlightGear_CheckedChanged);
            // 
            // TXT_servoroll
            // 
            resources.ApplyResources(this.TXT_servoroll, "TXT_servoroll");
            this.TXT_servoroll.Name = "TXT_servoroll";
            this.TXT_servoroll.resize = false;
            // 
            // TXT_servopitch
            // 
            resources.ApplyResources(this.TXT_servopitch, "TXT_servopitch");
            this.TXT_servopitch.Name = "TXT_servopitch";
            this.TXT_servopitch.resize = false;
            // 
            // TXT_servorudder
            // 
            resources.ApplyResources(this.TXT_servorudder, "TXT_servorudder");
            this.TXT_servorudder.Name = "TXT_servorudder";
            this.TXT_servorudder.resize = false;
            // 
            // TXT_servothrottle
            // 
            resources.ApplyResources(this.TXT_servothrottle, "TXT_servothrottle");
            this.TXT_servothrottle.Name = "TXT_servothrottle";
            this.TXT_servothrottle.resize = false;
            // 
            // panel1
            // 
            this.panel1.Controls.Add(this.label4);
            this.panel1.Controls.Add(this.label3);
            this.panel1.Controls.Add(this.label2);
            this.panel1.Controls.Add(this.label1);
            this.panel1.Controls.Add(this.TXT_lat);
            this.panel1.Controls.Add(this.TXT_long);
            this.panel1.Controls.Add(this.TXT_alt);
            resources.ApplyResources(this.panel1, "panel1");
            this.panel1.Name = "panel1";
            // 
            // label4
            // 
            resources.ApplyResources(this.label4, "label4");
            this.label4.Name = "label4";
            this.label4.resize = false;
            // 
            // label3
            // 
            resources.ApplyResources(this.label3, "label3");
            this.label3.Name = "label3";
            this.label3.resize = false;
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            this.label2.resize = false;
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            this.label1.resize = false;
            // 
            // panel2
            // 
            this.panel2.Controls.Add(this.label30);
            this.panel2.Controls.Add(this.TXT_yaw);
            this.panel2.Controls.Add(this.label11);
            this.panel2.Controls.Add(this.label7);
            this.panel2.Controls.Add(this.label6);
            this.panel2.Controls.Add(this.label5);
            this.panel2.Controls.Add(this.TXT_roll);
            this.panel2.Controls.Add(this.TXT_pitch);
            this.panel2.Controls.Add(this.TXT_heading);
            resources.ApplyResources(this.panel2, "panel2");
            this.panel2.Name = "panel2";
            // 
            // label30
            // 
            resources.ApplyResources(this.label30, "label30");
            this.label30.Name = "label30";
            this.label30.resize = false;
            // 
            // TXT_yaw
            // 
            resources.ApplyResources(this.TXT_yaw, "TXT_yaw");
            this.TXT_yaw.Name = "TXT_yaw";
            this.TXT_yaw.resize = false;
            // 
            // label11
            // 
            resources.ApplyResources(this.label11, "label11");
            this.label11.Name = "label11";
            this.label11.resize = false;
            // 
            // label7
            // 
            resources.ApplyResources(this.label7, "label7");
            this.label7.Name = "label7";
            this.label7.resize = false;
            // 
            // label6
            // 
            resources.ApplyResources(this.label6, "label6");
            this.label6.Name = "label6";
            this.label6.resize = false;
            // 
            // label5
            // 
            resources.ApplyResources(this.label5, "label5");
            this.label5.Name = "label5";
            this.label5.resize = false;
            // 
            // label8
            // 
            resources.ApplyResources(this.label8, "label8");
            this.label8.Name = "label8";
            this.label8.resize = false;
            // 
            // label9
            // 
            resources.ApplyResources(this.label9, "label9");
            this.label9.Name = "label9";
            this.label9.resize = false;
            // 
            // label10
            // 
            resources.ApplyResources(this.label10, "label10");
            this.label10.Name = "label10";
            this.label10.resize = false;
            // 
            // panel3
            // 
            this.panel3.Controls.Add(this.label16);
            this.panel3.Controls.Add(this.label15);
            this.panel3.Controls.Add(this.label14);
            this.panel3.Controls.Add(this.label13);
            this.panel3.Controls.Add(this.label12);
            this.panel3.Controls.Add(this.TXT_servoroll);
            this.panel3.Controls.Add(this.TXT_servopitch);
            this.panel3.Controls.Add(this.TXT_servorudder);
            this.panel3.Controls.Add(this.TXT_servothrottle);
            resources.ApplyResources(this.panel3, "panel3");
            this.panel3.Name = "panel3";
            // 
            // label16
            // 
            resources.ApplyResources(this.label16, "label16");
            this.label16.Name = "label16";
            this.label16.resize = false;
            // 
            // label15
            // 
            resources.ApplyResources(this.label15, "label15");
            this.label15.Name = "label15";
            this.label15.resize = false;
            // 
            // label14
            // 
            resources.ApplyResources(this.label14, "label14");
            this.label14.Name = "label14";
            this.label14.resize = false;
            // 
            // label13
            // 
            resources.ApplyResources(this.label13, "label13");
            this.label13.Name = "label13";
            this.label13.resize = false;
            // 
            // label12
            // 
            resources.ApplyResources(this.label12, "label12");
            this.label12.Name = "label12";
            this.label12.resize = false;
            // 
            // panel4
            // 
            this.panel4.Controls.Add(this.label20);
            this.panel4.Controls.Add(this.label19);
            this.panel4.Controls.Add(this.TXT_control_mode);
            this.panel4.Controls.Add(this.TXT_WP);
            this.panel4.Controls.Add(this.label18);
            this.panel4.Controls.Add(this.label10);
            this.panel4.Controls.Add(this.label9);
            this.panel4.Controls.Add(this.label8);
            this.panel4.Controls.Add(this.TXT_wpdist);
            this.panel4.Controls.Add(this.TXT_bererror);
            this.panel4.Controls.Add(this.TXT_alterror);
            resources.ApplyResources(this.panel4, "panel4");
            this.panel4.Name = "panel4";
            // 
            // label20
            // 
            resources.ApplyResources(this.label20, "label20");
            this.label20.Name = "label20";
            this.label20.resize = false;
            // 
            // label19
            // 
            resources.ApplyResources(this.label19, "label19");
            this.label19.Name = "label19";
            this.label19.resize = false;
            // 
            // TXT_control_mode
            // 
            resources.ApplyResources(this.TXT_control_mode, "TXT_control_mode");
            this.TXT_control_mode.Name = "TXT_control_mode";
            this.TXT_control_mode.resize = false;
            // 
            // TXT_WP
            // 
            resources.ApplyResources(this.TXT_WP, "TXT_WP");
            this.TXT_WP.Name = "TXT_WP";
            this.TXT_WP.resize = false;
            // 
            // label18
            // 
            resources.ApplyResources(this.label18, "label18");
            this.label18.Name = "label18";
            this.label18.resize = false;
            // 
            // label17
            // 
            resources.ApplyResources(this.label17, "label17");
            this.label17.Name = "label17";
            this.label17.resize = false;
            // 
            // panel5
            // 
            this.panel5.Controls.Add(this.ConnectComPort);
            resources.ApplyResources(this.panel5, "panel5");
            this.panel5.Name = "panel5";
            // 
            // zg1
            // 
            resources.ApplyResources(this.zg1, "zg1");
            this.zg1.Name = "zg1";
            this.zg1.ScrollGrace = 0D;
            this.zg1.ScrollMaxX = 0D;
            this.zg1.ScrollMaxY = 0D;
            this.zg1.ScrollMaxY2 = 0D;
            this.zg1.ScrollMinX = 0D;
            this.zg1.ScrollMinY = 0D;
            this.zg1.ScrollMinY2 = 0D;
            // 
            // timer1
            // 
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // panel6
            // 
            this.panel6.Controls.Add(this.label28);
            this.panel6.Controls.Add(this.label29);
            this.panel6.Controls.Add(this.label27);
            this.panel6.Controls.Add(this.label25);
            this.panel6.Controls.Add(this.TXT_throttlegain);
            this.panel6.Controls.Add(this.label24);
            this.panel6.Controls.Add(this.label23);
            this.panel6.Controls.Add(this.label22);
            this.panel6.Controls.Add(this.label21);
            this.panel6.Controls.Add(this.TXT_ruddergain);
            this.panel6.Controls.Add(this.TXT_pitchgain);
            this.panel6.Controls.Add(this.TXT_rollgain);
            resources.ApplyResources(this.panel6, "panel6");
            this.panel6.Name = "panel6";
            // 
            // label28
            // 
            resources.ApplyResources(this.label28, "label28");
            this.label28.Name = "label28";
            this.label28.resize = false;
            // 
            // label29
            // 
            resources.ApplyResources(this.label29, "label29");
            this.label29.Name = "label29";
            this.label29.resize = false;
            // 
            // label27
            // 
            resources.ApplyResources(this.label27, "label27");
            this.label27.Name = "label27";
            this.label27.resize = false;
            // 
            // label25
            // 
            resources.ApplyResources(this.label25, "label25");
            this.label25.Name = "label25";
            this.label25.resize = false;
            // 
            // TXT_throttlegain
            // 
            resources.ApplyResources(this.TXT_throttlegain, "TXT_throttlegain");
            this.TXT_throttlegain.Name = "TXT_throttlegain";
            this.TXT_throttlegain.TextChanged += new System.EventHandler(this.TXT_throttlegain_TextChanged);
            // 
            // label24
            // 
            resources.ApplyResources(this.label24, "label24");
            this.label24.Name = "label24";
            this.label24.resize = false;
            // 
            // label23
            // 
            resources.ApplyResources(this.label23, "label23");
            this.label23.Name = "label23";
            this.label23.resize = false;
            // 
            // label22
            // 
            resources.ApplyResources(this.label22, "label22");
            this.label22.Name = "label22";
            this.label22.resize = false;
            // 
            // label21
            // 
            resources.ApplyResources(this.label21, "label21");
            this.label21.Name = "label21";
            this.label21.resize = false;
            // 
            // TXT_ruddergain
            // 
            resources.ApplyResources(this.TXT_ruddergain, "TXT_ruddergain");
            this.TXT_ruddergain.Name = "TXT_ruddergain";
            this.TXT_ruddergain.TextChanged += new System.EventHandler(this.TXT_ruddergain_TextChanged);
            // 
            // TXT_pitchgain
            // 
            resources.ApplyResources(this.TXT_pitchgain, "TXT_pitchgain");
            this.TXT_pitchgain.Name = "TXT_pitchgain";
            this.TXT_pitchgain.TextChanged += new System.EventHandler(this.TXT_pitchgain_TextChanged);
            // 
            // TXT_rollgain
            // 
            resources.ApplyResources(this.TXT_rollgain, "TXT_rollgain");
            this.TXT_rollgain.Name = "TXT_rollgain";
            this.TXT_rollgain.TextChanged += new System.EventHandler(this.TXT_rollgain_TextChanged);
            // 
            // label26
            // 
            resources.ApplyResources(this.label26, "label26");
            this.label26.Name = "label26";
            this.label26.resize = false;
            // 
            // CHKdisplayall
            // 
            resources.ApplyResources(this.CHKdisplayall, "CHKdisplayall");
            this.CHKdisplayall.Name = "CHKdisplayall";
            this.CHKdisplayall.UseVisualStyleBackColor = true;
            this.CHKdisplayall.CheckedChanged += new System.EventHandler(this.CHKdisplayall_CheckedChanged);
            // 
            // CHKgraphroll
            // 
            resources.ApplyResources(this.CHKgraphroll, "CHKgraphroll");
            this.CHKgraphroll.Checked = true;
            this.CHKgraphroll.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHKgraphroll.Name = "CHKgraphroll";
            this.CHKgraphroll.UseVisualStyleBackColor = true;
            // 
            // CHKgraphpitch
            // 
            resources.ApplyResources(this.CHKgraphpitch, "CHKgraphpitch");
            this.CHKgraphpitch.Checked = true;
            this.CHKgraphpitch.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHKgraphpitch.Name = "CHKgraphpitch";
            this.CHKgraphpitch.UseVisualStyleBackColor = true;
            // 
            // CHKgraphrudder
            // 
            resources.ApplyResources(this.CHKgraphrudder, "CHKgraphrudder");
            this.CHKgraphrudder.Checked = true;
            this.CHKgraphrudder.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHKgraphrudder.Name = "CHKgraphrudder";
            this.CHKgraphrudder.UseVisualStyleBackColor = true;
            // 
            // CHKgraphthrottle
            // 
            resources.ApplyResources(this.CHKgraphthrottle, "CHKgraphthrottle");
            this.CHKgraphthrottle.Checked = true;
            this.CHKgraphthrottle.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHKgraphthrottle.Name = "CHKgraphthrottle";
            this.CHKgraphthrottle.UseVisualStyleBackColor = true;
            // 
            // but_advsettings
            // 
            resources.ApplyResources(this.but_advsettings, "but_advsettings");
            this.but_advsettings.Name = "but_advsettings";
            this.but_advsettings.UseVisualStyleBackColor = true;
            this.but_advsettings.Click += new System.EventHandler(this.but_advsettings_Click);
            // 
            // chkSensor
            // 
            resources.ApplyResources(this.chkSensor, "chkSensor");
            this.chkSensor.Name = "chkSensor";
            this.chkSensor.UseVisualStyleBackColor = true;
            // 
            // CHK_quad
            // 
            resources.ApplyResources(this.CHK_quad, "CHK_quad");
            this.CHK_quad.Name = "CHK_quad";
            this.CHK_quad.UseVisualStyleBackColor = true;
            this.CHK_quad.CheckedChanged += new System.EventHandler(this.CHK_quad_CheckedChanged);
            // 
            // BUT_startfgquad
            // 
            resources.ApplyResources(this.BUT_startfgquad, "BUT_startfgquad");
            this.BUT_startfgquad.Name = "BUT_startfgquad";
            this.BUT_startfgquad.UseVisualStyleBackColor = true;
            this.BUT_startfgquad.Click += new System.EventHandler(this.BUT_startfgquad_Click);
            // 
            // BUT_startfgplane
            // 
            resources.ApplyResources(this.BUT_startfgplane, "BUT_startfgplane");
            this.BUT_startfgplane.Name = "BUT_startfgplane";
            this.BUT_startfgplane.UseVisualStyleBackColor = true;
            this.BUT_startfgplane.Click += new System.EventHandler(this.BUT_startfgplane_Click);
            // 
            // BUT_startxplane
            // 
            resources.ApplyResources(this.BUT_startxplane, "BUT_startxplane");
            this.BUT_startxplane.Name = "BUT_startxplane";
            this.BUT_startxplane.UseVisualStyleBackColor = true;
            this.BUT_startxplane.Click += new System.EventHandler(this.BUT_startxplane_Click);
            // 
            // CHK_heli
            // 
            resources.ApplyResources(this.CHK_heli, "CHK_heli");
            this.CHK_heli.Name = "CHK_heli";
            this.CHK_heli.UseVisualStyleBackColor = true;
            // 
            // RAD_aerosimrc
            // 
            resources.ApplyResources(this.RAD_aerosimrc, "RAD_aerosimrc");
            this.RAD_aerosimrc.Name = "RAD_aerosimrc";
            this.toolTip1.SetToolTip(this.RAD_aerosimrc, resources.GetString("RAD_aerosimrc.ToolTip"));
            this.RAD_aerosimrc.UseVisualStyleBackColor = true;
            this.RAD_aerosimrc.CheckedChanged += new System.EventHandler(this.RAD_aerosimrc_CheckedChanged);
            // 
            // RAD_JSBSim
            // 
            resources.ApplyResources(this.RAD_JSBSim, "RAD_JSBSim");
            this.RAD_JSBSim.Name = "RAD_JSBSim";
            this.toolTip1.SetToolTip(this.RAD_JSBSim, resources.GetString("RAD_JSBSim.ToolTip"));
            this.RAD_JSBSim.UseVisualStyleBackColor = true;
            this.RAD_JSBSim.CheckedChanged += new System.EventHandler(this.RAD_JSBSim_CheckedChanged);
            // 
            // Simulation
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.RAD_JSBSim);
            this.Controls.Add(this.RAD_aerosimrc);
            this.Controls.Add(this.CHK_heli);
            this.Controls.Add(this.BUT_startxplane);
            this.Controls.Add(this.BUT_startfgplane);
            this.Controls.Add(this.BUT_startfgquad);
            this.Controls.Add(this.CHK_quad);
            this.Controls.Add(this.chkSensor);
            this.Controls.Add(this.but_advsettings);
            this.Controls.Add(this.CHKgraphthrottle);
            this.Controls.Add(this.CHKgraphrudder);
            this.Controls.Add(this.CHKgraphpitch);
            this.Controls.Add(this.CHKgraphroll);
            this.Controls.Add(this.CHKdisplayall);
            this.Controls.Add(this.label26);
            this.Controls.Add(this.panel6);
            this.Controls.Add(this.zg1);
            this.Controls.Add(this.panel5);
            this.Controls.Add(this.label17);
            this.Controls.Add(this.panel4);
            this.Controls.Add(this.panel3);
            this.Controls.Add(this.panel2);
            this.Controls.Add(this.panel1);
            this.Controls.Add(this.RAD_softFlightGear);
            this.Controls.Add(this.RAD_softXplanes);
            this.Controls.Add(this.SaveSettings);
            this.Controls.Add(this.OutputLog);
            this.Controls.Add(this.GPSrate);
            this.Controls.Add(this.CHKREV_rudder);
            this.Controls.Add(this.CHKREV_pitch);
            this.Controls.Add(this.CHKREV_roll);
            this.Name = "Simulation";
            this.Load += new System.EventHandler(this.Simulation_Load);
            ((System.ComponentModel.ISupportInitialize)(this.currentStateBindingSource)).EndInit();
            this.panel1.ResumeLayout(false);
            this.panel2.ResumeLayout(false);
            this.panel3.ResumeLayout(false);
            this.panel4.ResumeLayout(false);
            this.panel5.ResumeLayout(false);
            this.panel6.ResumeLayout(false);
            this.panel6.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.CheckBox CHKREV_roll;
        private System.Windows.Forms.CheckBox CHKREV_pitch;
        private System.Windows.Forms.CheckBox CHKREV_rudder;
        private System.Windows.Forms.ComboBox GPSrate;
        private MyButton ConnectComPort;
        private System.Windows.Forms.RichTextBox OutputLog;
        private ArdupilotMega.MyLabel TXT_roll;
        private ArdupilotMega.MyLabel TXT_pitch;
        private ArdupilotMega.MyLabel TXT_heading;
        private ArdupilotMega.MyLabel TXT_wpdist;
        private ArdupilotMega.MyLabel TXT_bererror;
        private ArdupilotMega.MyLabel TXT_alterror;
        private ArdupilotMega.MyLabel TXT_lat;
        private ArdupilotMega.MyLabel TXT_long;
        private ArdupilotMega.MyLabel TXT_alt;
        private MyButton SaveSettings;
        private System.Windows.Forms.RadioButton RAD_softXplanes;
        private System.Windows.Forms.RadioButton RAD_softFlightGear;
        private ArdupilotMega.MyLabel TXT_servoroll;
        private ArdupilotMega.MyLabel TXT_servopitch;
        private ArdupilotMega.MyLabel TXT_servorudder;
        private ArdupilotMega.MyLabel TXT_servothrottle;
        private System.Windows.Forms.Panel panel1;
        private ArdupilotMega.MyLabel label3;
        private ArdupilotMega.MyLabel label2;
        private ArdupilotMega.MyLabel label1;
        private System.Windows.Forms.Panel panel2;
        private ArdupilotMega.MyLabel label4;
        private ArdupilotMega.MyLabel label10;
        private ArdupilotMega.MyLabel label9;
        private ArdupilotMega.MyLabel label8;
        private ArdupilotMega.MyLabel label7;
        private ArdupilotMega.MyLabel label6;
        private ArdupilotMega.MyLabel label5;
        private ArdupilotMega.MyLabel label11;
        private System.Windows.Forms.Panel panel3;
        private ArdupilotMega.MyLabel label16;
        private ArdupilotMega.MyLabel label15;
        private ArdupilotMega.MyLabel label14;
        private ArdupilotMega.MyLabel label13;
        private ArdupilotMega.MyLabel label12;
        private System.Windows.Forms.Panel panel4;
        private ArdupilotMega.MyLabel label17;
        private ArdupilotMega.MyLabel TXT_WP;
        private ArdupilotMega.MyLabel label18;
        private System.Windows.Forms.Panel panel5;
        private ArdupilotMega.MyLabel label20;
        private ArdupilotMega.MyLabel label19;
        private ArdupilotMega.MyLabel TXT_control_mode;
        private ZedGraph.ZedGraphControl zg1;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.Panel panel6;
        private System.Windows.Forms.TextBox TXT_ruddergain;
        private System.Windows.Forms.TextBox TXT_pitchgain;
        private System.Windows.Forms.TextBox TXT_rollgain;
        private ArdupilotMega.MyLabel label24;
        private ArdupilotMega.MyLabel label23;
        private ArdupilotMega.MyLabel label22;
        private ArdupilotMega.MyLabel label21;
        private ArdupilotMega.MyLabel label25;
        private System.Windows.Forms.TextBox TXT_throttlegain;
        private ArdupilotMega.MyLabel label28;
        private ArdupilotMega.MyLabel label29;
        private ArdupilotMega.MyLabel label27;
        private ArdupilotMega.MyLabel label26;
        private System.Windows.Forms.CheckBox CHKdisplayall;
        private System.Windows.Forms.CheckBox CHKgraphroll;
        private System.Windows.Forms.CheckBox CHKgraphpitch;
        private System.Windows.Forms.CheckBox CHKgraphrudder;
        private System.Windows.Forms.CheckBox CHKgraphthrottle;
        private ArdupilotMega.MyLabel label30;
        private ArdupilotMega.MyLabel TXT_yaw;
        private MyButton but_advsettings;
        private System.Windows.Forms.CheckBox chkSensor;
        private System.Windows.Forms.BindingSource currentStateBindingSource;
        private System.Windows.Forms.CheckBox CHK_quad;
        private MyButton BUT_startfgquad;
        private MyButton BUT_startfgplane;
        private MyButton BUT_startxplane;
        private System.Windows.Forms.CheckBox CHK_heli;
        private System.Windows.Forms.RadioButton RAD_aerosimrc;
        private System.Windows.Forms.ToolTip toolTip1;
        private System.Windows.Forms.RadioButton RAD_JSBSim;
    }
}

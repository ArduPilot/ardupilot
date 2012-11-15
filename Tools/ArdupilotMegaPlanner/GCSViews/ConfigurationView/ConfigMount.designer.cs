using ArdupilotMega.Controls;
using ArdupilotMega.Presenter;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigMount
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> pi
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(ConfigMount));
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.LNK_wiki = new System.Windows.Forms.LinkLabel();
            this.label15 = new System.Windows.Forms.Label();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.pictureBox3 = new System.Windows.Forms.PictureBox();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.label16 = new System.Windows.Forms.Label();
            this.label17 = new System.Windows.Forms.Label();
            this.label18 = new System.Windows.Forms.Label();
            this.label19 = new System.Windows.Forms.Label();
            this.label20 = new System.Windows.Forms.Label();
            this.label21 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.mavlinkComboBoxTilt = new System.Windows.Forms.ComboBox();
            this.mavlinkComboBoxRoll = new System.Windows.Forms.ComboBox();
            this.mavlinkComboBoxPan = new System.Windows.Forms.ComboBox();
            this.label22 = new System.Windows.Forms.Label();
            this.label23 = new System.Windows.Forms.Label();
            this.label24 = new System.Windows.Forms.Label();
            this.CMB_inputch_pan = new ArdupilotMega.Controls.MavlinkComboBox();
            this.CMB_inputch_roll = new ArdupilotMega.Controls.MavlinkComboBox();
            this.CMB_inputch_tilt = new ArdupilotMega.Controls.MavlinkComboBox();
            this.mavlinkNumericUpDownTAM = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkNumericUpDownTAMX = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkNumericUpDownTSM = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkNumericUpDownTSMX = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkCheckBoxTR = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.mavlinkNumericUpDownPAM = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkNumericUpDownPAMX = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkNumericUpDownPSM = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkNumericUpDownPSMX = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkCheckBoxPR = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.mavlinkNumericUpDownRAM = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkNumericUpDownRAMX = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkNumericUpDownRSM = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkNumericUpDownRSMX = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkCheckBoxRR = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.LBL_Error = new ArdupilotMega.Controls.LabelWithPseudoOpacity();
            this.PBOX_WarningIcon = new ArdupilotMega.Controls.PictureBoxWithPseudoOpacity();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.label27 = new System.Windows.Forms.Label();
            this.NUD_RETRACT_z = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.label26 = new System.Windows.Forms.Label();
            this.NUD_RETRACT_y = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.label25 = new System.Windows.Forms.Label();
            this.NUD_RETRACT_x = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.label28 = new System.Windows.Forms.Label();
            this.NUD_NEUTRAL_z = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.label29 = new System.Windows.Forms.Label();
            this.NUD_NEUTRAL_y = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.label30 = new System.Windows.Forms.Label();
            this.NUD_NEUTRAL_x = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.label31 = new System.Windows.Forms.Label();
            this.NUD_CONTROL_z = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.label32 = new System.Windows.Forms.Label();
            this.NUD_CONTROL_y = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.label33 = new System.Windows.Forms.Label();
            this.NUD_CONTROL_x = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.CHK_stab_tilt = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.CHK_stab_roll = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.CHK_stab_pan = new ArdupilotMega.Controls.MavlinkCheckBox();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownTAM)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownTAMX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownTSM)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownTSMX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownPAM)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownPAMX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownPSM)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownPSMX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownRAM)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownRAMX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownRSM)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownRSMX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.PBOX_WarningIcon)).BeginInit();
            this.groupBox4.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_RETRACT_z)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_RETRACT_y)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_RETRACT_x)).BeginInit();
            this.groupBox5.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_NEUTRAL_z)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_NEUTRAL_y)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_NEUTRAL_x)).BeginInit();
            this.groupBox6.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_CONTROL_z)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_CONTROL_y)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_CONTROL_x)).BeginInit();
            this.SuspendLayout();
            // 
            // pictureBox1
            // 
            this.pictureBox1.BackColor = System.Drawing.Color.Transparent;
            this.pictureBox1.BackgroundImage = global::ArdupilotMega.Properties.Resources.cameraGimalPitch1;
            resources.ApplyResources(this.pictureBox1, "pictureBox1");
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.TabStop = false;
            // 
            // groupBox1
            // 
            resources.ApplyResources(this.groupBox1, "groupBox1");
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.TabStop = false;
            // 
            // pictureBox2
            // 
            this.pictureBox2.BackgroundImage = global::ArdupilotMega.Properties.Resources.cameraGimalRoll1;
            resources.ApplyResources(this.pictureBox2, "pictureBox2");
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.TabStop = false;
            // 
            // groupBox2
            // 
            resources.ApplyResources(this.groupBox2, "groupBox2");
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.TabStop = false;
            // 
            // label5
            // 
            resources.ApplyResources(this.label5, "label5");
            this.label5.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label5.Name = "label5";
            // 
            // label6
            // 
            resources.ApplyResources(this.label6, "label6");
            this.label6.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label6.Name = "label6";
            // 
            // LNK_wiki
            // 
            resources.ApplyResources(this.LNK_wiki, "LNK_wiki");
            this.LNK_wiki.LinkBehavior = System.Windows.Forms.LinkBehavior.HoverUnderline;
            this.LNK_wiki.LinkColor = System.Drawing.Color.CornflowerBlue;
            this.LNK_wiki.Name = "LNK_wiki";
            this.LNK_wiki.TabStop = true;
            this.LNK_wiki.LinkClicked += new System.Windows.Forms.LinkLabelLinkClickedEventHandler(this.LNK_Wiki_Clicked);
            // 
            // label15
            // 
            resources.ApplyResources(this.label15, "label15");
            this.label15.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label15.Name = "label15";
            // 
            // groupBox3
            // 
            resources.ApplyResources(this.groupBox3, "groupBox3");
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.TabStop = false;
            // 
            // pictureBox3
            // 
            this.pictureBox3.BackgroundImage = global::ArdupilotMega.Properties.Resources.cameraGimalYaw;
            resources.ApplyResources(this.pictureBox3, "pictureBox3");
            this.pictureBox3.Name = "pictureBox3";
            this.pictureBox3.TabStop = false;
            // 
            // label9
            // 
            resources.ApplyResources(this.label9, "label9");
            this.label9.Name = "label9";
            // 
            // label10
            // 
            resources.ApplyResources(this.label10, "label10");
            this.label10.Name = "label10";
            // 
            // label11
            // 
            resources.ApplyResources(this.label11, "label11");
            this.label11.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label11.Name = "label11";
            // 
            // label12
            // 
            resources.ApplyResources(this.label12, "label12");
            this.label12.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label12.Name = "label12";
            // 
            // label13
            // 
            resources.ApplyResources(this.label13, "label13");
            this.label13.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label13.Name = "label13";
            // 
            // label14
            // 
            resources.ApplyResources(this.label14, "label14");
            this.label14.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label14.Name = "label14";
            // 
            // label16
            // 
            resources.ApplyResources(this.label16, "label16");
            this.label16.Name = "label16";
            // 
            // label17
            // 
            resources.ApplyResources(this.label17, "label17");
            this.label17.Name = "label17";
            // 
            // label18
            // 
            resources.ApplyResources(this.label18, "label18");
            this.label18.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label18.Name = "label18";
            // 
            // label19
            // 
            resources.ApplyResources(this.label19, "label19");
            this.label19.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label19.Name = "label19";
            // 
            // label20
            // 
            resources.ApplyResources(this.label20, "label20");
            this.label20.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label20.Name = "label20";
            // 
            // label21
            // 
            resources.ApplyResources(this.label21, "label21");
            this.label21.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label21.Name = "label21";
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // label3
            // 
            resources.ApplyResources(this.label3, "label3");
            this.label3.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label3.Name = "label3";
            // 
            // label4
            // 
            resources.ApplyResources(this.label4, "label4");
            this.label4.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label4.Name = "label4";
            // 
            // label7
            // 
            resources.ApplyResources(this.label7, "label7");
            this.label7.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label7.Name = "label7";
            // 
            // label8
            // 
            resources.ApplyResources(this.label8, "label8");
            this.label8.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label8.Name = "label8";
            // 
            // mavlinkComboBoxTilt
            // 
            this.mavlinkComboBoxTilt.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.mavlinkComboBoxTilt.FormattingEnabled = true;
            resources.ApplyResources(this.mavlinkComboBoxTilt, "mavlinkComboBoxTilt");
            this.mavlinkComboBoxTilt.Name = "mavlinkComboBoxTilt";
            this.mavlinkComboBoxTilt.SelectedIndexChanged += new System.EventHandler(this.mavlinkComboBox_SelectedIndexChanged);
            // 
            // mavlinkComboBoxRoll
            // 
            this.mavlinkComboBoxRoll.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.mavlinkComboBoxRoll.FormattingEnabled = true;
            resources.ApplyResources(this.mavlinkComboBoxRoll, "mavlinkComboBoxRoll");
            this.mavlinkComboBoxRoll.Name = "mavlinkComboBoxRoll";
            this.mavlinkComboBoxRoll.SelectedIndexChanged += new System.EventHandler(this.mavlinkComboBox_SelectedIndexChanged);
            // 
            // mavlinkComboBoxPan
            // 
            this.mavlinkComboBoxPan.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.mavlinkComboBoxPan.FormattingEnabled = true;
            resources.ApplyResources(this.mavlinkComboBoxPan, "mavlinkComboBoxPan");
            this.mavlinkComboBoxPan.Name = "mavlinkComboBoxPan";
            this.mavlinkComboBoxPan.SelectedIndexChanged += new System.EventHandler(this.mavlinkComboBox_SelectedIndexChanged);
            // 
            // label22
            // 
            resources.ApplyResources(this.label22, "label22");
            this.label22.Name = "label22";
            // 
            // label23
            // 
            resources.ApplyResources(this.label23, "label23");
            this.label23.Name = "label23";
            // 
            // label24
            // 
            resources.ApplyResources(this.label24, "label24");
            this.label24.Name = "label24";
            // 
            // CMB_inputch_pan
            // 
            this.CMB_inputch_pan.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            resources.ApplyResources(this.CMB_inputch_pan, "CMB_inputch_pan");
            this.CMB_inputch_pan.FormattingEnabled = true;
            this.CMB_inputch_pan.Name = "CMB_inputch_pan";
            this.CMB_inputch_pan.param = null;
            this.CMB_inputch_pan.ParamName = null;
            // 
            // CMB_inputch_roll
            // 
            this.CMB_inputch_roll.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            resources.ApplyResources(this.CMB_inputch_roll, "CMB_inputch_roll");
            this.CMB_inputch_roll.FormattingEnabled = true;
            this.CMB_inputch_roll.Name = "CMB_inputch_roll";
            this.CMB_inputch_roll.param = null;
            this.CMB_inputch_roll.ParamName = null;
            // 
            // CMB_inputch_tilt
            // 
            this.CMB_inputch_tilt.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            resources.ApplyResources(this.CMB_inputch_tilt, "CMB_inputch_tilt");
            this.CMB_inputch_tilt.FormattingEnabled = true;
            this.CMB_inputch_tilt.Name = "CMB_inputch_tilt";
            this.CMB_inputch_tilt.param = null;
            this.CMB_inputch_tilt.ParamName = null;
            // 
            // mavlinkNumericUpDownTAM
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownTAM, "mavlinkNumericUpDownTAM");
            this.mavlinkNumericUpDownTAM.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownTAM.Max = 1F;
            this.mavlinkNumericUpDownTAM.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownTAM.Min = 0F;
            this.mavlinkNumericUpDownTAM.Name = "mavlinkNumericUpDownTAM";
            this.mavlinkNumericUpDownTAM.param = null;
            this.mavlinkNumericUpDownTAM.ParamName = null;
            this.mavlinkNumericUpDownTAM.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // mavlinkNumericUpDownTAMX
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownTAMX, "mavlinkNumericUpDownTAMX");
            this.mavlinkNumericUpDownTAMX.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownTAMX.Max = 1F;
            this.mavlinkNumericUpDownTAMX.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownTAMX.Min = 0F;
            this.mavlinkNumericUpDownTAMX.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.mavlinkNumericUpDownTAMX.Name = "mavlinkNumericUpDownTAMX";
            this.mavlinkNumericUpDownTAMX.param = null;
            this.mavlinkNumericUpDownTAMX.ParamName = null;
            this.mavlinkNumericUpDownTAMX.Value = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            // 
            // mavlinkNumericUpDownTSM
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownTSM, "mavlinkNumericUpDownTSM");
            this.mavlinkNumericUpDownTSM.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownTSM.Max = 1F;
            this.mavlinkNumericUpDownTSM.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownTSM.Min = 0F;
            this.mavlinkNumericUpDownTSM.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.mavlinkNumericUpDownTSM.Name = "mavlinkNumericUpDownTSM";
            this.mavlinkNumericUpDownTSM.param = null;
            this.mavlinkNumericUpDownTSM.ParamName = null;
            this.mavlinkNumericUpDownTSM.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // mavlinkNumericUpDownTSMX
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownTSMX, "mavlinkNumericUpDownTSMX");
            this.mavlinkNumericUpDownTSMX.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownTSMX.Max = 1F;
            this.mavlinkNumericUpDownTSMX.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownTSMX.Min = 0F;
            this.mavlinkNumericUpDownTSMX.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.mavlinkNumericUpDownTSMX.Name = "mavlinkNumericUpDownTSMX";
            this.mavlinkNumericUpDownTSMX.param = null;
            this.mavlinkNumericUpDownTSMX.ParamName = null;
            this.mavlinkNumericUpDownTSMX.Value = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            // 
            // mavlinkCheckBoxTR
            // 
            resources.ApplyResources(this.mavlinkCheckBoxTR, "mavlinkCheckBoxTR");
            this.mavlinkCheckBoxTR.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(224)))), ((int)(((byte)(224)))), ((int)(((byte)(224)))));
            this.mavlinkCheckBoxTR.Name = "mavlinkCheckBoxTR";
            this.mavlinkCheckBoxTR.OffValue = 0F;
            this.mavlinkCheckBoxTR.OnValue = 1F;
            this.mavlinkCheckBoxTR.param = null;
            this.mavlinkCheckBoxTR.ParamName = null;
            this.mavlinkCheckBoxTR.UseVisualStyleBackColor = true;
            // 
            // mavlinkNumericUpDownPAM
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownPAM, "mavlinkNumericUpDownPAM");
            this.mavlinkNumericUpDownPAM.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownPAM.Max = 1F;
            this.mavlinkNumericUpDownPAM.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownPAM.Min = 0F;
            this.mavlinkNumericUpDownPAM.Name = "mavlinkNumericUpDownPAM";
            this.mavlinkNumericUpDownPAM.param = null;
            this.mavlinkNumericUpDownPAM.ParamName = null;
            this.mavlinkNumericUpDownPAM.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // mavlinkNumericUpDownPAMX
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownPAMX, "mavlinkNumericUpDownPAMX");
            this.mavlinkNumericUpDownPAMX.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownPAMX.Max = 1F;
            this.mavlinkNumericUpDownPAMX.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownPAMX.Min = 0F;
            this.mavlinkNumericUpDownPAMX.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.mavlinkNumericUpDownPAMX.Name = "mavlinkNumericUpDownPAMX";
            this.mavlinkNumericUpDownPAMX.param = null;
            this.mavlinkNumericUpDownPAMX.ParamName = null;
            this.mavlinkNumericUpDownPAMX.Value = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            // 
            // mavlinkNumericUpDownPSM
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownPSM, "mavlinkNumericUpDownPSM");
            this.mavlinkNumericUpDownPSM.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownPSM.Max = 1F;
            this.mavlinkNumericUpDownPSM.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownPSM.Min = 0F;
            this.mavlinkNumericUpDownPSM.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.mavlinkNumericUpDownPSM.Name = "mavlinkNumericUpDownPSM";
            this.mavlinkNumericUpDownPSM.param = null;
            this.mavlinkNumericUpDownPSM.ParamName = null;
            this.mavlinkNumericUpDownPSM.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // mavlinkNumericUpDownPSMX
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownPSMX, "mavlinkNumericUpDownPSMX");
            this.mavlinkNumericUpDownPSMX.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownPSMX.Max = 1F;
            this.mavlinkNumericUpDownPSMX.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownPSMX.Min = 0F;
            this.mavlinkNumericUpDownPSMX.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.mavlinkNumericUpDownPSMX.Name = "mavlinkNumericUpDownPSMX";
            this.mavlinkNumericUpDownPSMX.param = null;
            this.mavlinkNumericUpDownPSMX.ParamName = null;
            this.mavlinkNumericUpDownPSMX.Value = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            // 
            // mavlinkCheckBoxPR
            // 
            resources.ApplyResources(this.mavlinkCheckBoxPR, "mavlinkCheckBoxPR");
            this.mavlinkCheckBoxPR.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(224)))), ((int)(((byte)(224)))), ((int)(((byte)(224)))));
            this.mavlinkCheckBoxPR.Name = "mavlinkCheckBoxPR";
            this.mavlinkCheckBoxPR.OffValue = 0F;
            this.mavlinkCheckBoxPR.OnValue = 1F;
            this.mavlinkCheckBoxPR.param = null;
            this.mavlinkCheckBoxPR.ParamName = null;
            this.mavlinkCheckBoxPR.UseVisualStyleBackColor = true;
            // 
            // mavlinkNumericUpDownRAM
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownRAM, "mavlinkNumericUpDownRAM");
            this.mavlinkNumericUpDownRAM.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownRAM.Max = 1F;
            this.mavlinkNumericUpDownRAM.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownRAM.Min = 0F;
            this.mavlinkNumericUpDownRAM.Name = "mavlinkNumericUpDownRAM";
            this.mavlinkNumericUpDownRAM.param = null;
            this.mavlinkNumericUpDownRAM.ParamName = null;
            this.mavlinkNumericUpDownRAM.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // mavlinkNumericUpDownRAMX
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownRAMX, "mavlinkNumericUpDownRAMX");
            this.mavlinkNumericUpDownRAMX.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownRAMX.Max = 1F;
            this.mavlinkNumericUpDownRAMX.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownRAMX.Min = 0F;
            this.mavlinkNumericUpDownRAMX.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.mavlinkNumericUpDownRAMX.Name = "mavlinkNumericUpDownRAMX";
            this.mavlinkNumericUpDownRAMX.param = null;
            this.mavlinkNumericUpDownRAMX.ParamName = null;
            this.mavlinkNumericUpDownRAMX.Value = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            // 
            // mavlinkNumericUpDownRSM
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownRSM, "mavlinkNumericUpDownRSM");
            this.mavlinkNumericUpDownRSM.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownRSM.Max = 1F;
            this.mavlinkNumericUpDownRSM.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownRSM.Min = 0F;
            this.mavlinkNumericUpDownRSM.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.mavlinkNumericUpDownRSM.Name = "mavlinkNumericUpDownRSM";
            this.mavlinkNumericUpDownRSM.param = null;
            this.mavlinkNumericUpDownRSM.ParamName = null;
            this.mavlinkNumericUpDownRSM.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // mavlinkNumericUpDownRSMX
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownRSMX, "mavlinkNumericUpDownRSMX");
            this.mavlinkNumericUpDownRSMX.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.mavlinkNumericUpDownRSMX.Max = 1F;
            this.mavlinkNumericUpDownRSMX.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.mavlinkNumericUpDownRSMX.Min = 0F;
            this.mavlinkNumericUpDownRSMX.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.mavlinkNumericUpDownRSMX.Name = "mavlinkNumericUpDownRSMX";
            this.mavlinkNumericUpDownRSMX.param = null;
            this.mavlinkNumericUpDownRSMX.ParamName = null;
            this.mavlinkNumericUpDownRSMX.Value = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            // 
            // mavlinkCheckBoxRR
            // 
            resources.ApplyResources(this.mavlinkCheckBoxRR, "mavlinkCheckBoxRR");
            this.mavlinkCheckBoxRR.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(224)))), ((int)(((byte)(224)))), ((int)(((byte)(224)))));
            this.mavlinkCheckBoxRR.Name = "mavlinkCheckBoxRR";
            this.mavlinkCheckBoxRR.OffValue = 0F;
            this.mavlinkCheckBoxRR.OnValue = 1F;
            this.mavlinkCheckBoxRR.param = null;
            this.mavlinkCheckBoxRR.ParamName = null;
            this.mavlinkCheckBoxRR.UseVisualStyleBackColor = true;
            // 
            // LBL_Error
            // 
            resources.ApplyResources(this.LBL_Error, "LBL_Error");
            this.LBL_Error.DoubleBuffered = true;
            this.LBL_Error.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.LBL_Error.Name = "LBL_Error";
            // 
            // PBOX_WarningIcon
            // 
            resources.ApplyResources(this.PBOX_WarningIcon, "PBOX_WarningIcon");
            this.PBOX_WarningIcon.Image = global::ArdupilotMega.Properties.Resources.iconWarning32;
            this.PBOX_WarningIcon.Name = "PBOX_WarningIcon";
            this.PBOX_WarningIcon.Opacity = 0.5F;
            this.PBOX_WarningIcon.TabStop = false;
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.label27);
            this.groupBox4.Controls.Add(this.NUD_RETRACT_z);
            this.groupBox4.Controls.Add(this.label26);
            this.groupBox4.Controls.Add(this.NUD_RETRACT_y);
            this.groupBox4.Controls.Add(this.label25);
            this.groupBox4.Controls.Add(this.NUD_RETRACT_x);
            resources.ApplyResources(this.groupBox4, "groupBox4");
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.TabStop = false;
            // 
            // label27
            // 
            resources.ApplyResources(this.label27, "label27");
            this.label27.Name = "label27";
            // 
            // NUD_RETRACT_z
            // 
            resources.ApplyResources(this.NUD_RETRACT_z, "NUD_RETRACT_z");
            this.NUD_RETRACT_z.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.NUD_RETRACT_z.Max = 1F;
            this.NUD_RETRACT_z.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.NUD_RETRACT_z.Min = 0F;
            this.NUD_RETRACT_z.Name = "NUD_RETRACT_z";
            this.NUD_RETRACT_z.param = null;
            this.NUD_RETRACT_z.ParamName = null;
            this.NUD_RETRACT_z.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // label26
            // 
            resources.ApplyResources(this.label26, "label26");
            this.label26.Name = "label26";
            // 
            // NUD_RETRACT_y
            // 
            resources.ApplyResources(this.NUD_RETRACT_y, "NUD_RETRACT_y");
            this.NUD_RETRACT_y.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.NUD_RETRACT_y.Max = 1F;
            this.NUD_RETRACT_y.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.NUD_RETRACT_y.Min = 0F;
            this.NUD_RETRACT_y.Name = "NUD_RETRACT_y";
            this.NUD_RETRACT_y.param = null;
            this.NUD_RETRACT_y.ParamName = null;
            this.NUD_RETRACT_y.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // label25
            // 
            resources.ApplyResources(this.label25, "label25");
            this.label25.Name = "label25";
            // 
            // NUD_RETRACT_x
            // 
            resources.ApplyResources(this.NUD_RETRACT_x, "NUD_RETRACT_x");
            this.NUD_RETRACT_x.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.NUD_RETRACT_x.Max = 1F;
            this.NUD_RETRACT_x.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.NUD_RETRACT_x.Min = 0F;
            this.NUD_RETRACT_x.Name = "NUD_RETRACT_x";
            this.NUD_RETRACT_x.param = null;
            this.NUD_RETRACT_x.ParamName = null;
            this.NUD_RETRACT_x.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.label28);
            this.groupBox5.Controls.Add(this.NUD_NEUTRAL_z);
            this.groupBox5.Controls.Add(this.label29);
            this.groupBox5.Controls.Add(this.NUD_NEUTRAL_y);
            this.groupBox5.Controls.Add(this.label30);
            this.groupBox5.Controls.Add(this.NUD_NEUTRAL_x);
            resources.ApplyResources(this.groupBox5, "groupBox5");
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.TabStop = false;
            // 
            // label28
            // 
            resources.ApplyResources(this.label28, "label28");
            this.label28.Name = "label28";
            // 
            // NUD_NEUTRAL_z
            // 
            resources.ApplyResources(this.NUD_NEUTRAL_z, "NUD_NEUTRAL_z");
            this.NUD_NEUTRAL_z.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.NUD_NEUTRAL_z.Max = 1F;
            this.NUD_NEUTRAL_z.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.NUD_NEUTRAL_z.Min = 0F;
            this.NUD_NEUTRAL_z.Name = "NUD_NEUTRAL_z";
            this.NUD_NEUTRAL_z.param = null;
            this.NUD_NEUTRAL_z.ParamName = null;
            this.NUD_NEUTRAL_z.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // label29
            // 
            resources.ApplyResources(this.label29, "label29");
            this.label29.Name = "label29";
            // 
            // NUD_NEUTRAL_y
            // 
            resources.ApplyResources(this.NUD_NEUTRAL_y, "NUD_NEUTRAL_y");
            this.NUD_NEUTRAL_y.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.NUD_NEUTRAL_y.Max = 1F;
            this.NUD_NEUTRAL_y.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.NUD_NEUTRAL_y.Min = 0F;
            this.NUD_NEUTRAL_y.Name = "NUD_NEUTRAL_y";
            this.NUD_NEUTRAL_y.param = null;
            this.NUD_NEUTRAL_y.ParamName = null;
            this.NUD_NEUTRAL_y.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // label30
            // 
            resources.ApplyResources(this.label30, "label30");
            this.label30.Name = "label30";
            // 
            // NUD_NEUTRAL_x
            // 
            resources.ApplyResources(this.NUD_NEUTRAL_x, "NUD_NEUTRAL_x");
            this.NUD_NEUTRAL_x.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.NUD_NEUTRAL_x.Max = 1F;
            this.NUD_NEUTRAL_x.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.NUD_NEUTRAL_x.Min = 0F;
            this.NUD_NEUTRAL_x.Name = "NUD_NEUTRAL_x";
            this.NUD_NEUTRAL_x.param = null;
            this.NUD_NEUTRAL_x.ParamName = null;
            this.NUD_NEUTRAL_x.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.label31);
            this.groupBox6.Controls.Add(this.NUD_CONTROL_z);
            this.groupBox6.Controls.Add(this.label32);
            this.groupBox6.Controls.Add(this.NUD_CONTROL_y);
            this.groupBox6.Controls.Add(this.label33);
            this.groupBox6.Controls.Add(this.NUD_CONTROL_x);
            resources.ApplyResources(this.groupBox6, "groupBox6");
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.TabStop = false;
            // 
            // label31
            // 
            resources.ApplyResources(this.label31, "label31");
            this.label31.Name = "label31";
            // 
            // NUD_CONTROL_z
            // 
            resources.ApplyResources(this.NUD_CONTROL_z, "NUD_CONTROL_z");
            this.NUD_CONTROL_z.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.NUD_CONTROL_z.Max = 1F;
            this.NUD_CONTROL_z.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.NUD_CONTROL_z.Min = 0F;
            this.NUD_CONTROL_z.Name = "NUD_CONTROL_z";
            this.NUD_CONTROL_z.param = null;
            this.NUD_CONTROL_z.ParamName = null;
            this.NUD_CONTROL_z.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // label32
            // 
            resources.ApplyResources(this.label32, "label32");
            this.label32.Name = "label32";
            // 
            // NUD_CONTROL_y
            // 
            resources.ApplyResources(this.NUD_CONTROL_y, "NUD_CONTROL_y");
            this.NUD_CONTROL_y.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.NUD_CONTROL_y.Max = 1F;
            this.NUD_CONTROL_y.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.NUD_CONTROL_y.Min = 0F;
            this.NUD_CONTROL_y.Name = "NUD_CONTROL_y";
            this.NUD_CONTROL_y.param = null;
            this.NUD_CONTROL_y.ParamName = null;
            this.NUD_CONTROL_y.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // label33
            // 
            resources.ApplyResources(this.label33, "label33");
            this.label33.Name = "label33";
            // 
            // NUD_CONTROL_x
            // 
            resources.ApplyResources(this.NUD_CONTROL_x, "NUD_CONTROL_x");
            this.NUD_CONTROL_x.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.NUD_CONTROL_x.Max = 1F;
            this.NUD_CONTROL_x.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.NUD_CONTROL_x.Min = 0F;
            this.NUD_CONTROL_x.Name = "NUD_CONTROL_x";
            this.NUD_CONTROL_x.param = null;
            this.NUD_CONTROL_x.ParamName = null;
            this.NUD_CONTROL_x.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // CHK_stab_tilt
            // 
            resources.ApplyResources(this.CHK_stab_tilt, "CHK_stab_tilt");
            this.CHK_stab_tilt.Name = "CHK_stab_tilt";
            this.CHK_stab_tilt.OffValue = 0F;
            this.CHK_stab_tilt.OnValue = 1F;
            this.CHK_stab_tilt.param = null;
            this.CHK_stab_tilt.ParamName = null;
            this.CHK_stab_tilt.UseVisualStyleBackColor = true;
            // 
            // CHK_stab_roll
            // 
            resources.ApplyResources(this.CHK_stab_roll, "CHK_stab_roll");
            this.CHK_stab_roll.Name = "CHK_stab_roll";
            this.CHK_stab_roll.OffValue = 0F;
            this.CHK_stab_roll.OnValue = 1F;
            this.CHK_stab_roll.param = null;
            this.CHK_stab_roll.ParamName = null;
            this.CHK_stab_roll.UseVisualStyleBackColor = true;
            // 
            // CHK_stab_pan
            // 
            resources.ApplyResources(this.CHK_stab_pan, "CHK_stab_pan");
            this.CHK_stab_pan.Name = "CHK_stab_pan";
            this.CHK_stab_pan.OffValue = 0F;
            this.CHK_stab_pan.OnValue = 1F;
            this.CHK_stab_pan.param = null;
            this.CHK_stab_pan.ParamName = null;
            this.CHK_stab_pan.UseVisualStyleBackColor = true;
            // 
            // ConfigMount
            // 
            this.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(43)))), ((int)(((byte)(44)))), ((int)(((byte)(45)))));
            this.Controls.Add(this.CHK_stab_pan);
            this.Controls.Add(this.CHK_stab_roll);
            this.Controls.Add(this.CHK_stab_tilt);
            this.Controls.Add(this.groupBox6);
            this.Controls.Add(this.groupBox5);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.label24);
            this.Controls.Add(this.CMB_inputch_pan);
            this.Controls.Add(this.label23);
            this.Controls.Add(this.CMB_inputch_roll);
            this.Controls.Add(this.label22);
            this.Controls.Add(this.CMB_inputch_tilt);
            this.Controls.Add(this.mavlinkComboBoxPan);
            this.Controls.Add(this.mavlinkComboBoxRoll);
            this.Controls.Add(this.mavlinkComboBoxTilt);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.mavlinkNumericUpDownTAM);
            this.Controls.Add(this.mavlinkNumericUpDownTAMX);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.mavlinkNumericUpDownTSM);
            this.Controls.Add(this.mavlinkNumericUpDownTSMX);
            this.Controls.Add(this.mavlinkCheckBoxTR);
            this.Controls.Add(this.label16);
            this.Controls.Add(this.label17);
            this.Controls.Add(this.label18);
            this.Controls.Add(this.label19);
            this.Controls.Add(this.mavlinkNumericUpDownPAM);
            this.Controls.Add(this.mavlinkNumericUpDownPAMX);
            this.Controls.Add(this.label20);
            this.Controls.Add(this.label21);
            this.Controls.Add(this.mavlinkNumericUpDownPSM);
            this.Controls.Add(this.mavlinkNumericUpDownPSMX);
            this.Controls.Add(this.mavlinkCheckBoxPR);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.mavlinkNumericUpDownRAM);
            this.Controls.Add(this.mavlinkNumericUpDownRAMX);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.mavlinkNumericUpDownRSM);
            this.Controls.Add(this.mavlinkNumericUpDownRSMX);
            this.Controls.Add(this.mavlinkCheckBoxRR);
            this.Controls.Add(this.label15);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.pictureBox3);
            this.Controls.Add(this.LNK_wiki);
            this.Controls.Add(this.LBL_Error);
            this.Controls.Add(this.PBOX_WarningIcon);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.pictureBox2);
            this.Controls.Add(this.pictureBox1);
            this.Name = "ConfigMount";
            resources.ApplyResources(this, "$this");
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownTAM)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownTAMX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownTSM)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownTSMX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownPAM)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownPAMX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownPSM)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownPSMX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownRAM)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownRAMX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownRSM)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownRSMX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.PBOX_WarningIcon)).EndInit();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_RETRACT_z)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_RETRACT_y)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_RETRACT_x)).EndInit();
            this.groupBox5.ResumeLayout(false);
            this.groupBox5.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_NEUTRAL_z)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_NEUTRAL_y)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_NEUTRAL_x)).EndInit();
            this.groupBox6.ResumeLayout(false);
            this.groupBox6.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_CONTROL_z)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_CONTROL_y)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.NUD_CONTROL_x)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.PictureBox pictureBox2;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private PictureBoxWithPseudoOpacity PBOX_WarningIcon;
        private LabelWithPseudoOpacity LBL_Error;
        private System.Windows.Forms.LinkLabel LNK_wiki;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.PictureBox pictureBox3;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private MavlinkNumericUpDown mavlinkNumericUpDownRAM;
        private MavlinkNumericUpDown mavlinkNumericUpDownRAMX;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label14;
        private MavlinkNumericUpDown mavlinkNumericUpDownRSM;
        private MavlinkNumericUpDown mavlinkNumericUpDownRSMX;
        private MavlinkCheckBox mavlinkCheckBoxRR;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.Label label17;
        private System.Windows.Forms.Label label18;
        private System.Windows.Forms.Label label19;
        private MavlinkNumericUpDown mavlinkNumericUpDownPAM;
        private MavlinkNumericUpDown mavlinkNumericUpDownPAMX;
        private System.Windows.Forms.Label label20;
        private System.Windows.Forms.Label label21;
        private MavlinkNumericUpDown mavlinkNumericUpDownPSM;
        private MavlinkNumericUpDown mavlinkNumericUpDownPSMX;
        private MavlinkCheckBox mavlinkCheckBoxPR;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private MavlinkNumericUpDown mavlinkNumericUpDownTAM;
        private MavlinkNumericUpDown mavlinkNumericUpDownTAMX;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private MavlinkNumericUpDown mavlinkNumericUpDownTSM;
        private MavlinkNumericUpDown mavlinkNumericUpDownTSMX;
        private MavlinkCheckBox mavlinkCheckBoxTR;
        private System.Windows.Forms.ComboBox mavlinkComboBoxTilt;
        private System.Windows.Forms.ComboBox mavlinkComboBoxRoll;
        private System.Windows.Forms.ComboBox mavlinkComboBoxPan;
        private MavlinkComboBox CMB_inputch_tilt;
        private System.Windows.Forms.Label label22;
        private System.Windows.Forms.Label label23;
        private MavlinkComboBox CMB_inputch_roll;
        private System.Windows.Forms.Label label24;
        private MavlinkComboBox CMB_inputch_pan;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Label label27;
        private MavlinkNumericUpDown NUD_RETRACT_z;
        private System.Windows.Forms.Label label26;
        private MavlinkNumericUpDown NUD_RETRACT_y;
        private System.Windows.Forms.Label label25;
        private MavlinkNumericUpDown NUD_RETRACT_x;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.Label label28;
        private MavlinkNumericUpDown NUD_NEUTRAL_z;
        private System.Windows.Forms.Label label29;
        private MavlinkNumericUpDown NUD_NEUTRAL_y;
        private System.Windows.Forms.Label label30;
        private MavlinkNumericUpDown NUD_NEUTRAL_x;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.Label label31;
        private MavlinkNumericUpDown NUD_CONTROL_z;
        private System.Windows.Forms.Label label32;
        private MavlinkNumericUpDown NUD_CONTROL_y;
        private System.Windows.Forms.Label label33;
        private MavlinkNumericUpDown NUD_CONTROL_x;
        private MavlinkCheckBox CHK_stab_tilt;
        private MavlinkCheckBox CHK_stab_roll;
        private MavlinkCheckBox CHK_stab_pan;

    }
}

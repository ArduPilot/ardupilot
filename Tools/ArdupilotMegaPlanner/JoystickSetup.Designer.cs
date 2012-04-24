namespace ArdupilotMega
{
    partial class JoystickSetup
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(JoystickSetup));
            this.CMB_joysticks = new System.Windows.Forms.ComboBox();
            this.CMB_CH1 = new System.Windows.Forms.ComboBox();
            this.CMB_CH2 = new System.Windows.Forms.ComboBox();
            this.CMB_CH3 = new System.Windows.Forms.ComboBox();
            this.CMB_CH4 = new System.Windows.Forms.ComboBox();
            this.expo_ch1 = new System.Windows.Forms.TextBox();
            this.expo_ch2 = new System.Windows.Forms.TextBox();
            this.expo_ch3 = new System.Windows.Forms.TextBox();
            this.expo_ch4 = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.revCH1 = new System.Windows.Forms.CheckBox();
            this.revCH2 = new System.Windows.Forms.CheckBox();
            this.revCH3 = new System.Windows.Forms.CheckBox();
            this.revCH4 = new System.Windows.Forms.CheckBox();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.CHK_elevons = new System.Windows.Forms.CheckBox();
            this.revCH5 = new System.Windows.Forms.CheckBox();
            this.label10 = new System.Windows.Forms.Label();
            this.expo_ch5 = new System.Windows.Forms.TextBox();
            this.CMB_CH5 = new System.Windows.Forms.ComboBox();
            this.revCH6 = new System.Windows.Forms.CheckBox();
            this.label11 = new System.Windows.Forms.Label();
            this.expo_ch6 = new System.Windows.Forms.TextBox();
            this.CMB_CH6 = new System.Windows.Forms.ComboBox();
            this.revCH7 = new System.Windows.Forms.CheckBox();
            this.label12 = new System.Windows.Forms.Label();
            this.expo_ch7 = new System.Windows.Forms.TextBox();
            this.CMB_CH7 = new System.Windows.Forms.ComboBox();
            this.revCH8 = new System.Windows.Forms.CheckBox();
            this.label13 = new System.Windows.Forms.Label();
            this.expo_ch8 = new System.Windows.Forms.TextBox();
            this.CMB_CH8 = new System.Windows.Forms.ComboBox();
            this.BUT_detch8 = new ArdupilotMega.Controls.MyButton();
            this.horizontalProgressBar4 = new ArdupilotMega.HorizontalProgressBar();
            this.BUT_detch4 = new ArdupilotMega.Controls.MyButton();
            this.BUT_detch3 = new ArdupilotMega.Controls.MyButton();
            this.BUT_detch2 = new ArdupilotMega.Controls.MyButton();
            this.BUT_detch1 = new ArdupilotMega.Controls.MyButton();
            this.BUT_enable = new ArdupilotMega.Controls.MyButton();
            this.BUT_save = new ArdupilotMega.Controls.MyButton();
            this.progressBar4 = new ArdupilotMega.HorizontalProgressBar();
            this.progressBar3 = new ArdupilotMega.HorizontalProgressBar();
            this.progressBar2 = new ArdupilotMega.HorizontalProgressBar();
            this.progressBar1 = new ArdupilotMega.HorizontalProgressBar();
            this.BUT_detch5 = new ArdupilotMega.Controls.MyButton();
            this.horizontalProgressBar1 = new ArdupilotMega.HorizontalProgressBar();
            this.BUT_detch6 = new ArdupilotMega.Controls.MyButton();
            this.horizontalProgressBar2 = new ArdupilotMega.HorizontalProgressBar();
            this.BUT_detch7 = new ArdupilotMega.Controls.MyButton();
            this.horizontalProgressBar3 = new ArdupilotMega.HorizontalProgressBar();
            this.SuspendLayout();
            // 
            // CMB_joysticks
            // 
            this.CMB_joysticks.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_joysticks, "CMB_joysticks");
            this.CMB_joysticks.Name = "CMB_joysticks";
            this.CMB_joysticks.SelectedIndexChanged += new System.EventHandler(this.CMB_joysticks_SelectedIndexChanged);
            this.CMB_joysticks.Click += new System.EventHandler(this.CMB_joysticks_Click);
            // 
            // CMB_CH1
            // 
            this.CMB_CH1.FormattingEnabled = true;
            this.CMB_CH1.Items.AddRange(new object[] {
            resources.GetString("CMB_CH1.Items"),
            resources.GetString("CMB_CH1.Items1"),
            resources.GetString("CMB_CH1.Items2"),
            resources.GetString("CMB_CH1.Items3")});
            resources.ApplyResources(this.CMB_CH1, "CMB_CH1");
            this.CMB_CH1.Name = "CMB_CH1";
            this.CMB_CH1.SelectedIndexChanged += new System.EventHandler(this.CMB_CH1_SelectedIndexChanged);
            // 
            // CMB_CH2
            // 
            this.CMB_CH2.FormattingEnabled = true;
            this.CMB_CH2.Items.AddRange(new object[] {
            resources.GetString("CMB_CH2.Items"),
            resources.GetString("CMB_CH2.Items1"),
            resources.GetString("CMB_CH2.Items2"),
            resources.GetString("CMB_CH2.Items3")});
            resources.ApplyResources(this.CMB_CH2, "CMB_CH2");
            this.CMB_CH2.Name = "CMB_CH2";
            this.CMB_CH2.SelectedIndexChanged += new System.EventHandler(this.CMB_CH2_SelectedIndexChanged);
            // 
            // CMB_CH3
            // 
            this.CMB_CH3.FormattingEnabled = true;
            this.CMB_CH3.Items.AddRange(new object[] {
            resources.GetString("CMB_CH3.Items"),
            resources.GetString("CMB_CH3.Items1"),
            resources.GetString("CMB_CH3.Items2"),
            resources.GetString("CMB_CH3.Items3")});
            resources.ApplyResources(this.CMB_CH3, "CMB_CH3");
            this.CMB_CH3.Name = "CMB_CH3";
            this.CMB_CH3.SelectedIndexChanged += new System.EventHandler(this.CMB_CH3_SelectedIndexChanged);
            // 
            // CMB_CH4
            // 
            this.CMB_CH4.FormattingEnabled = true;
            this.CMB_CH4.Items.AddRange(new object[] {
            resources.GetString("CMB_CH4.Items"),
            resources.GetString("CMB_CH4.Items1"),
            resources.GetString("CMB_CH4.Items2"),
            resources.GetString("CMB_CH4.Items3")});
            resources.ApplyResources(this.CMB_CH4, "CMB_CH4");
            this.CMB_CH4.Name = "CMB_CH4";
            this.CMB_CH4.SelectedIndexChanged += new System.EventHandler(this.CMB_CH4_SelectedIndexChanged);
            // 
            // expo_ch1
            // 
            this.expo_ch1.BorderStyle = System.Windows.Forms.BorderStyle.None;
            resources.ApplyResources(this.expo_ch1, "expo_ch1");
            this.expo_ch1.Name = "expo_ch1";
            // 
            // expo_ch2
            // 
            this.expo_ch2.BorderStyle = System.Windows.Forms.BorderStyle.None;
            resources.ApplyResources(this.expo_ch2, "expo_ch2");
            this.expo_ch2.Name = "expo_ch2";
            // 
            // expo_ch3
            // 
            this.expo_ch3.BorderStyle = System.Windows.Forms.BorderStyle.None;
            resources.ApplyResources(this.expo_ch3, "expo_ch3");
            this.expo_ch3.Name = "expo_ch3";
            // 
            // expo_ch4
            // 
            this.expo_ch4.BorderStyle = System.Windows.Forms.BorderStyle.None;
            resources.ApplyResources(this.expo_ch4, "expo_ch4");
            this.expo_ch4.Name = "expo_ch4";
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
            this.label3.Name = "label3";
            // 
            // label4
            // 
            resources.ApplyResources(this.label4, "label4");
            this.label4.Name = "label4";
            // 
            // revCH1
            // 
            resources.ApplyResources(this.revCH1, "revCH1");
            this.revCH1.Name = "revCH1";
            this.revCH1.UseVisualStyleBackColor = true;
            this.revCH1.CheckedChanged += new System.EventHandler(this.revCH1_CheckedChanged);
            // 
            // revCH2
            // 
            resources.ApplyResources(this.revCH2, "revCH2");
            this.revCH2.Name = "revCH2";
            this.revCH2.UseVisualStyleBackColor = true;
            this.revCH2.CheckedChanged += new System.EventHandler(this.revCH2_CheckedChanged);
            // 
            // revCH3
            // 
            resources.ApplyResources(this.revCH3, "revCH3");
            this.revCH3.Name = "revCH3";
            this.revCH3.UseVisualStyleBackColor = true;
            this.revCH3.CheckedChanged += new System.EventHandler(this.revCH3_CheckedChanged);
            // 
            // revCH4
            // 
            resources.ApplyResources(this.revCH4, "revCH4");
            this.revCH4.Name = "revCH4";
            this.revCH4.UseVisualStyleBackColor = true;
            this.revCH4.CheckedChanged += new System.EventHandler(this.revCH4_CheckedChanged);
            // 
            // label5
            // 
            resources.ApplyResources(this.label5, "label5");
            this.label5.Name = "label5";
            // 
            // label6
            // 
            resources.ApplyResources(this.label6, "label6");
            this.label6.Name = "label6";
            // 
            // label7
            // 
            resources.ApplyResources(this.label7, "label7");
            this.label7.Name = "label7";
            // 
            // label8
            // 
            resources.ApplyResources(this.label8, "label8");
            this.label8.Name = "label8";
            // 
            // label9
            // 
            resources.ApplyResources(this.label9, "label9");
            this.label9.Name = "label9";
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // CHK_elevons
            // 
            resources.ApplyResources(this.CHK_elevons, "CHK_elevons");
            this.CHK_elevons.Name = "CHK_elevons";
            this.CHK_elevons.UseVisualStyleBackColor = true;
            this.CHK_elevons.CheckedChanged += new System.EventHandler(this.CHK_elevons_CheckedChanged);
            // 
            // revCH5
            // 
            resources.ApplyResources(this.revCH5, "revCH5");
            this.revCH5.Name = "revCH5";
            this.revCH5.UseVisualStyleBackColor = true;
            this.revCH5.CheckedChanged += new System.EventHandler(this.revCH5_CheckedChanged);
            // 
            // label10
            // 
            resources.ApplyResources(this.label10, "label10");
            this.label10.Name = "label10";
            // 
            // expo_ch5
            // 
            this.expo_ch5.BorderStyle = System.Windows.Forms.BorderStyle.None;
            resources.ApplyResources(this.expo_ch5, "expo_ch5");
            this.expo_ch5.Name = "expo_ch5";
            // 
            // CMB_CH5
            // 
            this.CMB_CH5.FormattingEnabled = true;
            this.CMB_CH5.Items.AddRange(new object[] {
            resources.GetString("CMB_CH5.Items"),
            resources.GetString("CMB_CH5.Items1"),
            resources.GetString("CMB_CH5.Items2"),
            resources.GetString("CMB_CH5.Items3")});
            resources.ApplyResources(this.CMB_CH5, "CMB_CH5");
            this.CMB_CH5.Name = "CMB_CH5";
            this.CMB_CH5.SelectedIndexChanged += new System.EventHandler(this.CMB_CH5_SelectedIndexChanged);
            // 
            // revCH6
            // 
            resources.ApplyResources(this.revCH6, "revCH6");
            this.revCH6.Name = "revCH6";
            this.revCH6.UseVisualStyleBackColor = true;
            this.revCH6.CheckedChanged += new System.EventHandler(this.revCH6_CheckedChanged);
            // 
            // label11
            // 
            resources.ApplyResources(this.label11, "label11");
            this.label11.Name = "label11";
            // 
            // expo_ch6
            // 
            this.expo_ch6.BorderStyle = System.Windows.Forms.BorderStyle.None;
            resources.ApplyResources(this.expo_ch6, "expo_ch6");
            this.expo_ch6.Name = "expo_ch6";
            // 
            // CMB_CH6
            // 
            this.CMB_CH6.FormattingEnabled = true;
            this.CMB_CH6.Items.AddRange(new object[] {
            resources.GetString("CMB_CH6.Items"),
            resources.GetString("CMB_CH6.Items1"),
            resources.GetString("CMB_CH6.Items2"),
            resources.GetString("CMB_CH6.Items3")});
            resources.ApplyResources(this.CMB_CH6, "CMB_CH6");
            this.CMB_CH6.Name = "CMB_CH6";
            this.CMB_CH6.SelectedIndexChanged += new System.EventHandler(this.CMB_CH6_SelectedIndexChanged);
            // 
            // revCH7
            // 
            resources.ApplyResources(this.revCH7, "revCH7");
            this.revCH7.Name = "revCH7";
            this.revCH7.UseVisualStyleBackColor = true;
            this.revCH7.CheckedChanged += new System.EventHandler(this.revCH7_CheckedChanged);
            // 
            // label12
            // 
            resources.ApplyResources(this.label12, "label12");
            this.label12.Name = "label12";
            // 
            // expo_ch7
            // 
            this.expo_ch7.BorderStyle = System.Windows.Forms.BorderStyle.None;
            resources.ApplyResources(this.expo_ch7, "expo_ch7");
            this.expo_ch7.Name = "expo_ch7";
            // 
            // CMB_CH7
            // 
            this.CMB_CH7.FormattingEnabled = true;
            this.CMB_CH7.Items.AddRange(new object[] {
            resources.GetString("CMB_CH7.Items"),
            resources.GetString("CMB_CH7.Items1"),
            resources.GetString("CMB_CH7.Items2"),
            resources.GetString("CMB_CH7.Items3")});
            resources.ApplyResources(this.CMB_CH7, "CMB_CH7");
            this.CMB_CH7.Name = "CMB_CH7";
            this.CMB_CH7.SelectedIndexChanged += new System.EventHandler(this.CMB_CH7_SelectedIndexChanged);
            // 
            // revCH8
            // 
            resources.ApplyResources(this.revCH8, "revCH8");
            this.revCH8.Name = "revCH8";
            this.revCH8.UseVisualStyleBackColor = true;
            this.revCH8.CheckedChanged += new System.EventHandler(this.revCH8_CheckedChanged);
            // 
            // label13
            // 
            resources.ApplyResources(this.label13, "label13");
            this.label13.Name = "label13";
            // 
            // expo_ch8
            // 
            this.expo_ch8.BorderStyle = System.Windows.Forms.BorderStyle.None;
            resources.ApplyResources(this.expo_ch8, "expo_ch8");
            this.expo_ch8.Name = "expo_ch8";
            // 
            // CMB_CH8
            // 
            this.CMB_CH8.FormattingEnabled = true;
            this.CMB_CH8.Items.AddRange(new object[] {
            resources.GetString("CMB_CH8.Items"),
            resources.GetString("CMB_CH8.Items1"),
            resources.GetString("CMB_CH8.Items2"),
            resources.GetString("CMB_CH8.Items3")});
            resources.ApplyResources(this.CMB_CH8, "CMB_CH8");
            this.CMB_CH8.Name = "CMB_CH8";
            this.CMB_CH8.SelectedIndexChanged += new System.EventHandler(this.CMB_CH8_SelectedIndexChanged);
            // 
            // BUT_detch8
            // 
            resources.ApplyResources(this.BUT_detch8, "BUT_detch8");
            this.BUT_detch8.Name = "BUT_detch8";
            this.BUT_detch8.UseVisualStyleBackColor = true;
            this.BUT_detch8.Click += new System.EventHandler(this.BUT_detch8_Click);
            // 
            // horizontalProgressBar4
            // 
            resources.ApplyResources(this.horizontalProgressBar4, "horizontalProgressBar4");
            this.horizontalProgressBar4.Label = null;
            this.horizontalProgressBar4.Maximum = 2200;
            this.horizontalProgressBar4.maxline = 0;
            this.horizontalProgressBar4.Minimum = 800;
            this.horizontalProgressBar4.minline = 0;
            this.horizontalProgressBar4.Name = "horizontalProgressBar4";
            this.horizontalProgressBar4.Value = 800;
            // 
            // BUT_detch4
            // 
            resources.ApplyResources(this.BUT_detch4, "BUT_detch4");
            this.BUT_detch4.Name = "BUT_detch4";
            this.BUT_detch4.UseVisualStyleBackColor = true;
            this.BUT_detch4.Click += new System.EventHandler(this.BUT_detch4_Click);
            // 
            // BUT_detch3
            // 
            resources.ApplyResources(this.BUT_detch3, "BUT_detch3");
            this.BUT_detch3.Name = "BUT_detch3";
            this.BUT_detch3.UseVisualStyleBackColor = true;
            this.BUT_detch3.Click += new System.EventHandler(this.BUT_detch3_Click);
            // 
            // BUT_detch2
            // 
            resources.ApplyResources(this.BUT_detch2, "BUT_detch2");
            this.BUT_detch2.Name = "BUT_detch2";
            this.BUT_detch2.UseVisualStyleBackColor = true;
            this.BUT_detch2.Click += new System.EventHandler(this.BUT_detch2_Click);
            // 
            // BUT_detch1
            // 
            resources.ApplyResources(this.BUT_detch1, "BUT_detch1");
            this.BUT_detch1.Name = "BUT_detch1";
            this.BUT_detch1.UseVisualStyleBackColor = true;
            this.BUT_detch1.Click += new System.EventHandler(this.BUT_detch1_Click);
            // 
            // BUT_enable
            // 
            resources.ApplyResources(this.BUT_enable, "BUT_enable");
            this.BUT_enable.Name = "BUT_enable";
            this.BUT_enable.UseVisualStyleBackColor = true;
            this.BUT_enable.Click += new System.EventHandler(this.BUT_enable_Click);
            // 
            // BUT_save
            // 
            resources.ApplyResources(this.BUT_save, "BUT_save");
            this.BUT_save.Name = "BUT_save";
            this.BUT_save.UseVisualStyleBackColor = true;
            this.BUT_save.Click += new System.EventHandler(this.BUT_save_Click);
            // 
            // progressBar4
            // 
            resources.ApplyResources(this.progressBar4, "progressBar4");
            this.progressBar4.Label = null;
            this.progressBar4.Maximum = 2200;
            this.progressBar4.maxline = 0;
            this.progressBar4.Minimum = 800;
            this.progressBar4.minline = 0;
            this.progressBar4.Name = "progressBar4";
            this.progressBar4.Value = 800;
            // 
            // progressBar3
            // 
            resources.ApplyResources(this.progressBar3, "progressBar3");
            this.progressBar3.Label = null;
            this.progressBar3.Maximum = 2200;
            this.progressBar3.maxline = 0;
            this.progressBar3.Minimum = 800;
            this.progressBar3.minline = 0;
            this.progressBar3.Name = "progressBar3";
            this.progressBar3.Value = 800;
            // 
            // progressBar2
            // 
            resources.ApplyResources(this.progressBar2, "progressBar2");
            this.progressBar2.Label = null;
            this.progressBar2.Maximum = 2200;
            this.progressBar2.maxline = 0;
            this.progressBar2.Minimum = 800;
            this.progressBar2.minline = 0;
            this.progressBar2.Name = "progressBar2";
            this.progressBar2.Value = 800;
            // 
            // progressBar1
            // 
            resources.ApplyResources(this.progressBar1, "progressBar1");
            this.progressBar1.Label = null;
            this.progressBar1.Maximum = 2200;
            this.progressBar1.maxline = 0;
            this.progressBar1.Minimum = 800;
            this.progressBar1.minline = 0;
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Value = 800;
            // 
            // BUT_detch5
            // 
            resources.ApplyResources(this.BUT_detch5, "BUT_detch5");
            this.BUT_detch5.Name = "BUT_detch5";
            this.BUT_detch5.UseVisualStyleBackColor = true;
            this.BUT_detch5.Click += new System.EventHandler(this.BUT_detch5_Click);
            // 
            // horizontalProgressBar1
            // 
            resources.ApplyResources(this.horizontalProgressBar1, "horizontalProgressBar1");
            this.horizontalProgressBar1.Label = null;
            this.horizontalProgressBar1.Maximum = 2200;
            this.horizontalProgressBar1.maxline = 0;
            this.horizontalProgressBar1.Minimum = 800;
            this.horizontalProgressBar1.minline = 0;
            this.horizontalProgressBar1.Name = "horizontalProgressBar1";
            this.horizontalProgressBar1.Value = 800;
            // 
            // BUT_detch6
            // 
            resources.ApplyResources(this.BUT_detch6, "BUT_detch6");
            this.BUT_detch6.Name = "BUT_detch6";
            this.BUT_detch6.UseVisualStyleBackColor = true;
            this.BUT_detch6.Click += new System.EventHandler(this.BUT_detch6_Click);
            // 
            // horizontalProgressBar2
            // 
            resources.ApplyResources(this.horizontalProgressBar2, "horizontalProgressBar2");
            this.horizontalProgressBar2.Label = null;
            this.horizontalProgressBar2.Maximum = 2200;
            this.horizontalProgressBar2.maxline = 0;
            this.horizontalProgressBar2.Minimum = 800;
            this.horizontalProgressBar2.minline = 0;
            this.horizontalProgressBar2.Name = "horizontalProgressBar2";
            this.horizontalProgressBar2.Value = 800;
            // 
            // BUT_detch7
            // 
            resources.ApplyResources(this.BUT_detch7, "BUT_detch7");
            this.BUT_detch7.Name = "BUT_detch7";
            this.BUT_detch7.UseVisualStyleBackColor = true;
            this.BUT_detch7.Click += new System.EventHandler(this.BUT_detch7_Click);
            // 
            // horizontalProgressBar3
            // 
            resources.ApplyResources(this.horizontalProgressBar3, "horizontalProgressBar3");
            this.horizontalProgressBar3.Label = null;
            this.horizontalProgressBar3.Maximum = 2200;
            this.horizontalProgressBar3.maxline = 0;
            this.horizontalProgressBar3.Minimum = 800;
            this.horizontalProgressBar3.minline = 0;
            this.horizontalProgressBar3.Name = "horizontalProgressBar3";
            this.horizontalProgressBar3.Value = 800;
            // 
            // JoystickSetup
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.BUT_detch8);
            this.Controls.Add(this.revCH8);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.expo_ch8);
            this.Controls.Add(this.horizontalProgressBar4);
            this.Controls.Add(this.CMB_CH8);
            this.Controls.Add(this.BUT_detch7);
            this.Controls.Add(this.revCH7);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.expo_ch7);
            this.Controls.Add(this.horizontalProgressBar3);
            this.Controls.Add(this.CMB_CH7);
            this.Controls.Add(this.BUT_detch6);
            this.Controls.Add(this.revCH6);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.expo_ch6);
            this.Controls.Add(this.horizontalProgressBar2);
            this.Controls.Add(this.CMB_CH6);
            this.Controls.Add(this.BUT_detch5);
            this.Controls.Add(this.revCH5);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.expo_ch5);
            this.Controls.Add(this.horizontalProgressBar1);
            this.Controls.Add(this.CMB_CH5);
            this.Controls.Add(this.CHK_elevons);
            this.Controls.Add(this.BUT_detch4);
            this.Controls.Add(this.BUT_detch3);
            this.Controls.Add(this.BUT_detch2);
            this.Controls.Add(this.BUT_detch1);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.BUT_enable);
            this.Controls.Add(this.BUT_save);
            this.Controls.Add(this.revCH4);
            this.Controls.Add(this.revCH3);
            this.Controls.Add(this.revCH2);
            this.Controls.Add(this.revCH1);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.expo_ch4);
            this.Controls.Add(this.expo_ch3);
            this.Controls.Add(this.expo_ch2);
            this.Controls.Add(this.expo_ch1);
            this.Controls.Add(this.progressBar4);
            this.Controls.Add(this.progressBar3);
            this.Controls.Add(this.progressBar2);
            this.Controls.Add(this.progressBar1);
            this.Controls.Add(this.CMB_CH4);
            this.Controls.Add(this.CMB_CH3);
            this.Controls.Add(this.CMB_CH2);
            this.Controls.Add(this.CMB_CH1);
            this.Controls.Add(this.CMB_joysticks);
            this.Name = "JoystickSetup";
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.JoystickSetup_FormClosed);
            this.Load += new System.EventHandler(this.Joystick_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox CMB_joysticks;
        private System.Windows.Forms.ComboBox CMB_CH1;
        private System.Windows.Forms.ComboBox CMB_CH2;
        private System.Windows.Forms.ComboBox CMB_CH3;
        private System.Windows.Forms.ComboBox CMB_CH4;
        private HorizontalProgressBar progressBar1;
        private HorizontalProgressBar progressBar2;
        private HorizontalProgressBar progressBar3;
        private HorizontalProgressBar progressBar4;
        private System.Windows.Forms.TextBox expo_ch1;
        private System.Windows.Forms.TextBox expo_ch2;
        private System.Windows.Forms.TextBox expo_ch3;
        private System.Windows.Forms.TextBox expo_ch4;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.CheckBox revCH1;
        private System.Windows.Forms.CheckBox revCH2;
        private System.Windows.Forms.CheckBox revCH3;
        private System.Windows.Forms.CheckBox revCH4;
        private ArdupilotMega.Controls.MyButton BUT_save;
        private ArdupilotMega.Controls.MyButton BUT_enable;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Timer timer1;
        private ArdupilotMega.Controls.MyButton BUT_detch1;
        private ArdupilotMega.Controls.MyButton BUT_detch2;
        private ArdupilotMega.Controls.MyButton BUT_detch3;
        private ArdupilotMega.Controls.MyButton BUT_detch4;
        private System.Windows.Forms.CheckBox CHK_elevons;
        private ArdupilotMega.Controls.MyButton BUT_detch5;
        private System.Windows.Forms.CheckBox revCH5;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox expo_ch5;
        private HorizontalProgressBar horizontalProgressBar1;
        private System.Windows.Forms.ComboBox CMB_CH5;
        private ArdupilotMega.Controls.MyButton BUT_detch6;
        private System.Windows.Forms.CheckBox revCH6;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox expo_ch6;
        private HorizontalProgressBar horizontalProgressBar2;
        private System.Windows.Forms.ComboBox CMB_CH6;
        private ArdupilotMega.Controls.MyButton BUT_detch7;
        private System.Windows.Forms.CheckBox revCH7;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.TextBox expo_ch7;
        private HorizontalProgressBar horizontalProgressBar3;
        private System.Windows.Forms.ComboBox CMB_CH7;
        private ArdupilotMega.Controls.MyButton BUT_detch8;
        private System.Windows.Forms.CheckBox revCH8;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.TextBox expo_ch8;
        private HorizontalProgressBar horizontalProgressBar4;
        private System.Windows.Forms.ComboBox CMB_CH8;
    }
}
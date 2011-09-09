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
            this.BUT_detch4 = new ArdupilotMega.MyButton();
            this.BUT_detch3 = new ArdupilotMega.MyButton();
            this.BUT_detch2 = new ArdupilotMega.MyButton();
            this.BUT_detch1 = new ArdupilotMega.MyButton();
            this.BUT_enable = new ArdupilotMega.MyButton();
            this.BUT_save = new ArdupilotMega.MyButton();
            this.progressBar4 = new ArdupilotMega.HorizontalProgressBar();
            this.progressBar3 = new ArdupilotMega.HorizontalProgressBar();
            this.progressBar2 = new ArdupilotMega.HorizontalProgressBar();
            this.progressBar1 = new ArdupilotMega.HorizontalProgressBar();
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
            // JoystickSetup
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
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
        private MyButton BUT_save;
        private MyButton BUT_enable;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Timer timer1;
        private MyButton BUT_detch1;
        private MyButton BUT_detch2;
        private MyButton BUT_detch3;
        private MyButton BUT_detch4;
    }
}
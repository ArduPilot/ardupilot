namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigFailSafe
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
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(ConfigFailSafe));
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.lbl_currentmode = new System.Windows.Forms.Label();
            this.currentStateBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.LNK_wiki = new System.Windows.Forms.LinkLabel();
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
            this.mavlinkCheckBoxthr_fs = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.mavlinkCheckBoxthr_fs_action = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.mavlinkNumericUpDownthr_fs_value = new ArdupilotMega.Controls.MavlinkNumericUpDown();
            this.mavlinkCheckBoxlong_fs = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.mavlinkCheckBoxgcs_fs = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.mavlinkCheckBoxshort_fs = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.label3 = new System.Windows.Forms.Label();
            this.lbl_armed = new System.Windows.Forms.Label();
            this.lbl_gpslock = new System.Windows.Forms.Label();
            this.horizontalProgressBar9 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar10 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar11 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar12 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar13 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar14 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar15 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar16 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar8 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar7 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar6 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar5 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar4 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar3 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar2 = new ArdupilotMega.HorizontalProgressBar();
            this.horizontalProgressBar1 = new ArdupilotMega.HorizontalProgressBar();
            ((System.ComponentModel.ISupportInitialize)(this.currentStateBindingSource)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownthr_fs_value)).BeginInit();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            // 
            // lbl_currentmode
            // 
            this.lbl_currentmode.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.currentStateBindingSource, "mode", true));
            resources.ApplyResources(this.lbl_currentmode, "lbl_currentmode");
            this.lbl_currentmode.Name = "lbl_currentmode";
            // 
            // currentStateBindingSource
            // 
            this.currentStateBindingSource.DataSource = typeof(ArdupilotMega.CurrentState);
            // 
            // LNK_wiki
            // 
            resources.ApplyResources(this.LNK_wiki, "LNK_wiki");
            this.LNK_wiki.LinkBehavior = System.Windows.Forms.LinkBehavior.HoverUnderline;
            this.LNK_wiki.LinkColor = System.Drawing.Color.CornflowerBlue;
            this.LNK_wiki.Name = "LNK_wiki";
            this.LNK_wiki.TabStop = true;
            this.LNK_wiki.LinkClicked += new System.Windows.Forms.LinkLabelLinkClickedEventHandler(this.LNK_wiki_LinkClicked);
            // 
            // toolTip1
            // 
            this.toolTip1.AutoPopDelay = 20000;
            this.toolTip1.InitialDelay = 500;
            this.toolTip1.ReshowDelay = 100;
            // 
            // mavlinkCheckBoxthr_fs
            // 
            resources.ApplyResources(this.mavlinkCheckBoxthr_fs, "mavlinkCheckBoxthr_fs");
            this.mavlinkCheckBoxthr_fs.Name = "mavlinkCheckBoxthr_fs";
            this.mavlinkCheckBoxthr_fs.OffValue = 0F;
            this.mavlinkCheckBoxthr_fs.OnValue = 1F;
            this.mavlinkCheckBoxthr_fs.param = null;
            this.mavlinkCheckBoxthr_fs.ParamName = null;
            this.toolTip1.SetToolTip(this.mavlinkCheckBoxthr_fs, resources.GetString("mavlinkCheckBoxthr_fs.ToolTip"));
            this.mavlinkCheckBoxthr_fs.UseVisualStyleBackColor = true;
            // 
            // mavlinkCheckBoxthr_fs_action
            // 
            resources.ApplyResources(this.mavlinkCheckBoxthr_fs_action, "mavlinkCheckBoxthr_fs_action");
            this.mavlinkCheckBoxthr_fs_action.Name = "mavlinkCheckBoxthr_fs_action";
            this.mavlinkCheckBoxthr_fs_action.OffValue = 0F;
            this.mavlinkCheckBoxthr_fs_action.OnValue = 1F;
            this.mavlinkCheckBoxthr_fs_action.param = null;
            this.mavlinkCheckBoxthr_fs_action.ParamName = null;
            this.toolTip1.SetToolTip(this.mavlinkCheckBoxthr_fs_action, resources.GetString("mavlinkCheckBoxthr_fs_action.ToolTip"));
            this.mavlinkCheckBoxthr_fs_action.UseVisualStyleBackColor = true;
            // 
            // mavlinkNumericUpDownthr_fs_value
            // 
            resources.ApplyResources(this.mavlinkNumericUpDownthr_fs_value, "mavlinkNumericUpDownthr_fs_value");
            this.mavlinkNumericUpDownthr_fs_value.Max = 1F;
            this.mavlinkNumericUpDownthr_fs_value.Min = 0F;
            this.mavlinkNumericUpDownthr_fs_value.Name = "mavlinkNumericUpDownthr_fs_value";
            this.mavlinkNumericUpDownthr_fs_value.param = null;
            this.mavlinkNumericUpDownthr_fs_value.ParamName = null;
            this.toolTip1.SetToolTip(this.mavlinkNumericUpDownthr_fs_value, resources.GetString("mavlinkNumericUpDownthr_fs_value.ToolTip"));
            // 
            // mavlinkCheckBoxlong_fs
            // 
            resources.ApplyResources(this.mavlinkCheckBoxlong_fs, "mavlinkCheckBoxlong_fs");
            this.mavlinkCheckBoxlong_fs.Name = "mavlinkCheckBoxlong_fs";
            this.mavlinkCheckBoxlong_fs.OffValue = 0F;
            this.mavlinkCheckBoxlong_fs.OnValue = 1F;
            this.mavlinkCheckBoxlong_fs.param = null;
            this.mavlinkCheckBoxlong_fs.ParamName = null;
            this.toolTip1.SetToolTip(this.mavlinkCheckBoxlong_fs, resources.GetString("mavlinkCheckBoxlong_fs.ToolTip"));
            this.mavlinkCheckBoxlong_fs.UseVisualStyleBackColor = true;
            // 
            // mavlinkCheckBoxgcs_fs
            // 
            resources.ApplyResources(this.mavlinkCheckBoxgcs_fs, "mavlinkCheckBoxgcs_fs");
            this.mavlinkCheckBoxgcs_fs.Name = "mavlinkCheckBoxgcs_fs";
            this.mavlinkCheckBoxgcs_fs.OffValue = 0F;
            this.mavlinkCheckBoxgcs_fs.OnValue = 1F;
            this.mavlinkCheckBoxgcs_fs.param = null;
            this.mavlinkCheckBoxgcs_fs.ParamName = null;
            this.toolTip1.SetToolTip(this.mavlinkCheckBoxgcs_fs, resources.GetString("mavlinkCheckBoxgcs_fs.ToolTip"));
            this.mavlinkCheckBoxgcs_fs.UseVisualStyleBackColor = true;
            // 
            // mavlinkCheckBoxshort_fs
            // 
            resources.ApplyResources(this.mavlinkCheckBoxshort_fs, "mavlinkCheckBoxshort_fs");
            this.mavlinkCheckBoxshort_fs.Name = "mavlinkCheckBoxshort_fs";
            this.mavlinkCheckBoxshort_fs.OffValue = 0F;
            this.mavlinkCheckBoxshort_fs.OnValue = 1F;
            this.mavlinkCheckBoxshort_fs.param = null;
            this.mavlinkCheckBoxshort_fs.ParamName = null;
            this.toolTip1.SetToolTip(this.mavlinkCheckBoxshort_fs, resources.GetString("mavlinkCheckBoxshort_fs.ToolTip"));
            this.mavlinkCheckBoxshort_fs.UseVisualStyleBackColor = true;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.mavlinkCheckBoxthr_fs);
            this.groupBox1.Controls.Add(this.mavlinkCheckBoxthr_fs_action);
            this.groupBox1.Controls.Add(this.mavlinkNumericUpDownthr_fs_value);
            this.groupBox1.Controls.Add(this.mavlinkCheckBoxlong_fs);
            this.groupBox1.Controls.Add(this.mavlinkCheckBoxgcs_fs);
            this.groupBox1.Controls.Add(this.mavlinkCheckBoxshort_fs);
            resources.ApplyResources(this.groupBox1, "groupBox1");
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.TabStop = false;
            // 
            // label3
            // 
            resources.ApplyResources(this.label3, "label3");
            this.label3.Name = "label3";
            // 
            // lbl_armed
            // 
            this.lbl_armed.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.currentStateBindingSource, "armed", true));
            resources.ApplyResources(this.lbl_armed, "lbl_armed");
            this.lbl_armed.Name = "lbl_armed";
            this.lbl_armed.Paint += new System.Windows.Forms.PaintEventHandler(this.lbl_armed_Paint);
            // 
            // lbl_gpslock
            // 
            this.lbl_gpslock.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.currentStateBindingSource, "gpsstatus", true));
            resources.ApplyResources(this.lbl_gpslock, "lbl_gpslock");
            this.lbl_gpslock.Name = "lbl_gpslock";
            this.lbl_gpslock.Paint += new System.Windows.Forms.PaintEventHandler(this.lbl_gpslock_Paint);
            // 
            // horizontalProgressBar9
            // 
            this.horizontalProgressBar9.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch8out", true));
            resources.ApplyResources(this.horizontalProgressBar9, "horizontalProgressBar9");
            this.horizontalProgressBar9.Label = "Radio 8";
            this.horizontalProgressBar9.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar9.Maximum = 2000;
            this.horizontalProgressBar9.maxline = 0;
            this.horizontalProgressBar9.Minimum = 1000;
            this.horizontalProgressBar9.minline = 0;
            this.horizontalProgressBar9.Name = "horizontalProgressBar9";
            this.horizontalProgressBar9.Step = 1;
            this.horizontalProgressBar9.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar10
            // 
            this.horizontalProgressBar10.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch7out", true));
            resources.ApplyResources(this.horizontalProgressBar10, "horizontalProgressBar10");
            this.horizontalProgressBar10.Label = "Radio 7";
            this.horizontalProgressBar10.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar10.Maximum = 2000;
            this.horizontalProgressBar10.maxline = 0;
            this.horizontalProgressBar10.Minimum = 1000;
            this.horizontalProgressBar10.minline = 0;
            this.horizontalProgressBar10.Name = "horizontalProgressBar10";
            this.horizontalProgressBar10.Step = 1;
            this.horizontalProgressBar10.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar11
            // 
            this.horizontalProgressBar11.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch6out", true));
            resources.ApplyResources(this.horizontalProgressBar11, "horizontalProgressBar11");
            this.horizontalProgressBar11.Label = "Radio 6";
            this.horizontalProgressBar11.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar11.Maximum = 2000;
            this.horizontalProgressBar11.maxline = 0;
            this.horizontalProgressBar11.Minimum = 1000;
            this.horizontalProgressBar11.minline = 0;
            this.horizontalProgressBar11.Name = "horizontalProgressBar11";
            this.horizontalProgressBar11.Step = 1;
            this.horizontalProgressBar11.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar12
            // 
            this.horizontalProgressBar12.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch5out", true));
            resources.ApplyResources(this.horizontalProgressBar12, "horizontalProgressBar12");
            this.horizontalProgressBar12.Label = "Radio 5";
            this.horizontalProgressBar12.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar12.Maximum = 2000;
            this.horizontalProgressBar12.maxline = 0;
            this.horizontalProgressBar12.Minimum = 1000;
            this.horizontalProgressBar12.minline = 0;
            this.horizontalProgressBar12.Name = "horizontalProgressBar12";
            this.horizontalProgressBar12.Step = 1;
            this.horizontalProgressBar12.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar13
            // 
            this.horizontalProgressBar13.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch4out", true));
            resources.ApplyResources(this.horizontalProgressBar13, "horizontalProgressBar13");
            this.horizontalProgressBar13.Label = "Radio 4";
            this.horizontalProgressBar13.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar13.Maximum = 2000;
            this.horizontalProgressBar13.maxline = 0;
            this.horizontalProgressBar13.Minimum = 1000;
            this.horizontalProgressBar13.minline = 0;
            this.horizontalProgressBar13.Name = "horizontalProgressBar13";
            this.horizontalProgressBar13.Step = 1;
            this.horizontalProgressBar13.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar14
            // 
            this.horizontalProgressBar14.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch3out", true));
            resources.ApplyResources(this.horizontalProgressBar14, "horizontalProgressBar14");
            this.horizontalProgressBar14.Label = "Radio 3";
            this.horizontalProgressBar14.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar14.Maximum = 2000;
            this.horizontalProgressBar14.maxline = 0;
            this.horizontalProgressBar14.Minimum = 1000;
            this.horizontalProgressBar14.minline = 0;
            this.horizontalProgressBar14.Name = "horizontalProgressBar14";
            this.horizontalProgressBar14.Step = 1;
            this.horizontalProgressBar14.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar15
            // 
            this.horizontalProgressBar15.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch1out", true));
            resources.ApplyResources(this.horizontalProgressBar15, "horizontalProgressBar15");
            this.horizontalProgressBar15.Label = "Radio 1";
            this.horizontalProgressBar15.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar15.Maximum = 2000;
            this.horizontalProgressBar15.maxline = 0;
            this.horizontalProgressBar15.Minimum = 1000;
            this.horizontalProgressBar15.minline = 0;
            this.horizontalProgressBar15.Name = "horizontalProgressBar15";
            this.horizontalProgressBar15.Step = 1;
            this.horizontalProgressBar15.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar16
            // 
            this.horizontalProgressBar16.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch2out", true));
            resources.ApplyResources(this.horizontalProgressBar16, "horizontalProgressBar16");
            this.horizontalProgressBar16.Label = "Radio 2";
            this.horizontalProgressBar16.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar16.Maximum = 2000;
            this.horizontalProgressBar16.maxline = 0;
            this.horizontalProgressBar16.Minimum = 1000;
            this.horizontalProgressBar16.minline = 0;
            this.horizontalProgressBar16.Name = "horizontalProgressBar16";
            this.horizontalProgressBar16.Step = 1;
            this.horizontalProgressBar16.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar8
            // 
            this.horizontalProgressBar8.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch8in", true));
            resources.ApplyResources(this.horizontalProgressBar8, "horizontalProgressBar8");
            this.horizontalProgressBar8.Label = "Radio 8";
            this.horizontalProgressBar8.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar8.Maximum = 2000;
            this.horizontalProgressBar8.maxline = 0;
            this.horizontalProgressBar8.Minimum = 1000;
            this.horizontalProgressBar8.minline = 0;
            this.horizontalProgressBar8.Name = "horizontalProgressBar8";
            this.horizontalProgressBar8.Step = 1;
            this.horizontalProgressBar8.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar7
            // 
            this.horizontalProgressBar7.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch7in", true));
            resources.ApplyResources(this.horizontalProgressBar7, "horizontalProgressBar7");
            this.horizontalProgressBar7.Label = "Radio 7";
            this.horizontalProgressBar7.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar7.Maximum = 2000;
            this.horizontalProgressBar7.maxline = 0;
            this.horizontalProgressBar7.Minimum = 1000;
            this.horizontalProgressBar7.minline = 0;
            this.horizontalProgressBar7.Name = "horizontalProgressBar7";
            this.horizontalProgressBar7.Step = 1;
            this.horizontalProgressBar7.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar6
            // 
            this.horizontalProgressBar6.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch6in", true));
            resources.ApplyResources(this.horizontalProgressBar6, "horizontalProgressBar6");
            this.horizontalProgressBar6.Label = "Radio 6";
            this.horizontalProgressBar6.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar6.Maximum = 2000;
            this.horizontalProgressBar6.maxline = 0;
            this.horizontalProgressBar6.Minimum = 1000;
            this.horizontalProgressBar6.minline = 0;
            this.horizontalProgressBar6.Name = "horizontalProgressBar6";
            this.horizontalProgressBar6.Step = 1;
            this.horizontalProgressBar6.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar5
            // 
            this.horizontalProgressBar5.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch5in", true));
            resources.ApplyResources(this.horizontalProgressBar5, "horizontalProgressBar5");
            this.horizontalProgressBar5.Label = "Radio 5";
            this.horizontalProgressBar5.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar5.Maximum = 2000;
            this.horizontalProgressBar5.maxline = 0;
            this.horizontalProgressBar5.Minimum = 1000;
            this.horizontalProgressBar5.minline = 0;
            this.horizontalProgressBar5.Name = "horizontalProgressBar5";
            this.horizontalProgressBar5.Step = 1;
            this.horizontalProgressBar5.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar4
            // 
            this.horizontalProgressBar4.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch4in", true));
            resources.ApplyResources(this.horizontalProgressBar4, "horizontalProgressBar4");
            this.horizontalProgressBar4.Label = "Radio 4";
            this.horizontalProgressBar4.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar4.Maximum = 2000;
            this.horizontalProgressBar4.maxline = 0;
            this.horizontalProgressBar4.Minimum = 1000;
            this.horizontalProgressBar4.minline = 0;
            this.horizontalProgressBar4.Name = "horizontalProgressBar4";
            this.horizontalProgressBar4.Step = 1;
            this.horizontalProgressBar4.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar3
            // 
            this.horizontalProgressBar3.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch3in", true));
            resources.ApplyResources(this.horizontalProgressBar3, "horizontalProgressBar3");
            this.horizontalProgressBar3.Label = "Radio 3";
            this.horizontalProgressBar3.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar3.Maximum = 2000;
            this.horizontalProgressBar3.maxline = 0;
            this.horizontalProgressBar3.Minimum = 1000;
            this.horizontalProgressBar3.minline = 0;
            this.horizontalProgressBar3.Name = "horizontalProgressBar3";
            this.horizontalProgressBar3.Step = 1;
            this.horizontalProgressBar3.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar2
            // 
            this.horizontalProgressBar2.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch1in", true));
            resources.ApplyResources(this.horizontalProgressBar2, "horizontalProgressBar2");
            this.horizontalProgressBar2.Label = "Radio 1";
            this.horizontalProgressBar2.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar2.Maximum = 2000;
            this.horizontalProgressBar2.maxline = 0;
            this.horizontalProgressBar2.Minimum = 1000;
            this.horizontalProgressBar2.minline = 0;
            this.horizontalProgressBar2.Name = "horizontalProgressBar2";
            this.horizontalProgressBar2.Step = 1;
            this.horizontalProgressBar2.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // horizontalProgressBar1
            // 
            this.horizontalProgressBar1.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch2in", true));
            resources.ApplyResources(this.horizontalProgressBar1, "horizontalProgressBar1");
            this.horizontalProgressBar1.Label = "Radio 2";
            this.horizontalProgressBar1.MarqueeAnimationSpeed = 1;
            this.horizontalProgressBar1.Maximum = 2000;
            this.horizontalProgressBar1.maxline = 0;
            this.horizontalProgressBar1.Minimum = 1000;
            this.horizontalProgressBar1.minline = 0;
            this.horizontalProgressBar1.Name = "horizontalProgressBar1";
            this.horizontalProgressBar1.Step = 1;
            this.horizontalProgressBar1.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            // 
            // ConfigFailSafe
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.lbl_gpslock);
            this.Controls.Add(this.lbl_armed);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.LNK_wiki);
            this.Controls.Add(this.lbl_currentmode);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.horizontalProgressBar9);
            this.Controls.Add(this.horizontalProgressBar10);
            this.Controls.Add(this.horizontalProgressBar11);
            this.Controls.Add(this.horizontalProgressBar12);
            this.Controls.Add(this.horizontalProgressBar13);
            this.Controls.Add(this.horizontalProgressBar14);
            this.Controls.Add(this.horizontalProgressBar15);
            this.Controls.Add(this.horizontalProgressBar16);
            this.Controls.Add(this.horizontalProgressBar8);
            this.Controls.Add(this.horizontalProgressBar7);
            this.Controls.Add(this.horizontalProgressBar6);
            this.Controls.Add(this.horizontalProgressBar5);
            this.Controls.Add(this.horizontalProgressBar4);
            this.Controls.Add(this.horizontalProgressBar3);
            this.Controls.Add(this.horizontalProgressBar2);
            this.Controls.Add(this.horizontalProgressBar1);
            this.Name = "ConfigFailSafe";
            ((System.ComponentModel.ISupportInitialize)(this.currentStateBindingSource)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.mavlinkNumericUpDownthr_fs_value)).EndInit();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.BindingSource currentStateBindingSource;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private HorizontalProgressBar horizontalProgressBar9;
        private HorizontalProgressBar horizontalProgressBar10;
        private HorizontalProgressBar horizontalProgressBar11;
        private HorizontalProgressBar horizontalProgressBar12;
        private HorizontalProgressBar horizontalProgressBar13;
        private HorizontalProgressBar horizontalProgressBar14;
        private HorizontalProgressBar horizontalProgressBar15;
        private HorizontalProgressBar horizontalProgressBar16;
        private HorizontalProgressBar horizontalProgressBar8;
        private HorizontalProgressBar horizontalProgressBar7;
        private HorizontalProgressBar horizontalProgressBar6;
        private HorizontalProgressBar horizontalProgressBar5;
        private HorizontalProgressBar horizontalProgressBar4;
        private HorizontalProgressBar horizontalProgressBar3;
        private HorizontalProgressBar horizontalProgressBar2;
        private HorizontalProgressBar horizontalProgressBar1;
        private System.Windows.Forms.Label lbl_currentmode;
        private Controls.MavlinkCheckBox mavlinkCheckBoxthr_fs;
        private Controls.MavlinkNumericUpDown mavlinkNumericUpDownthr_fs_value;
        private System.Windows.Forms.LinkLabel LNK_wiki;
        private Controls.MavlinkCheckBox mavlinkCheckBoxgcs_fs;
        private System.Windows.Forms.ToolTip toolTip1;
        private Controls.MavlinkCheckBox mavlinkCheckBoxshort_fs;
        private Controls.MavlinkCheckBox mavlinkCheckBoxlong_fs;
        private Controls.MavlinkCheckBox mavlinkCheckBoxthr_fs_action;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label lbl_armed;
        private System.Windows.Forms.Label lbl_gpslock;
    }
}

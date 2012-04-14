namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigHardwareOptions
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(ConfigHardwareOptions));
            this.BUT_MagCalibrationLive = new ArdupilotMega.MyButton();
            this.label27 = new System.Windows.Forms.Label();
            this.CMB_sonartype = new System.Windows.Forms.ComboBox();
            this.CHK_enableoptflow = new System.Windows.Forms.CheckBox();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.linkLabelmagdec = new System.Windows.Forms.LinkLabel();
            this.label100 = new System.Windows.Forms.Label();
            this.TXT_declination = new System.Windows.Forms.TextBox();
            this.CHK_enableairspeed = new System.Windows.Forms.CheckBox();
            this.CHK_enablesonar = new System.Windows.Forms.CheckBox();
            this.CHK_enablecompass = new System.Windows.Forms.CheckBox();
            this.pictureBox4 = new System.Windows.Forms.PictureBox();
            this.pictureBox3 = new System.Windows.Forms.PictureBox();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.BUT_MagCalibrationLog = new ArdupilotMega.MyButton();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox4)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            this.SuspendLayout();
            // 
            // BUT_MagCalibrationLive
            // 
            resources.ApplyResources(this.BUT_MagCalibrationLive, "BUT_MagCalibrationLive");
            this.BUT_MagCalibrationLive.Name = "BUT_MagCalibrationLive";
            this.BUT_MagCalibrationLive.UseVisualStyleBackColor = true;
            this.BUT_MagCalibrationLive.Click += new System.EventHandler(this.BUT_MagCalibration_Click);
            // 
            // label27
            // 
            resources.ApplyResources(this.label27, "label27");
            this.label27.Name = "label27";
            // 
            // CMB_sonartype
            // 
            this.CMB_sonartype.FormattingEnabled = true;
            this.CMB_sonartype.Items.AddRange(new object[] {
            resources.GetString("CMB_sonartype.Items"),
            resources.GetString("CMB_sonartype.Items1"),
            resources.GetString("CMB_sonartype.Items2")});
            resources.ApplyResources(this.CMB_sonartype, "CMB_sonartype");
            this.CMB_sonartype.Name = "CMB_sonartype";
            this.CMB_sonartype.SelectedIndexChanged += new System.EventHandler(this.CMB_sonartype_SelectedIndexChanged);
            // 
            // CHK_enableoptflow
            // 
            resources.ApplyResources(this.CHK_enableoptflow, "CHK_enableoptflow");
            this.CHK_enableoptflow.Name = "CHK_enableoptflow";
            this.CHK_enableoptflow.UseVisualStyleBackColor = true;
            this.CHK_enableoptflow.CheckedChanged += new System.EventHandler(this.CHK_enableoptflow_CheckedChanged);
            // 
            // pictureBox2
            // 
            this.pictureBox2.BackColor = System.Drawing.Color.White;
            this.pictureBox2.BackgroundImage = global::ArdupilotMega.Properties.Resources.opticalflow;
            resources.ApplyResources(this.pictureBox2, "pictureBox2");
            this.pictureBox2.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.TabStop = false;
            // 
            // linkLabelmagdec
            // 
            resources.ApplyResources(this.linkLabelmagdec, "linkLabelmagdec");
            this.linkLabelmagdec.Name = "linkLabelmagdec";
            this.linkLabelmagdec.TabStop = true;
            this.linkLabelmagdec.LinkClicked += new System.Windows.Forms.LinkLabelLinkClickedEventHandler(this.linkLabel1_LinkClicked);
            // 
            // label100
            // 
            resources.ApplyResources(this.label100, "label100");
            this.label100.Name = "label100";
            // 
            // TXT_declination
            // 
            resources.ApplyResources(this.TXT_declination, "TXT_declination");
            this.TXT_declination.Name = "TXT_declination";
            this.TXT_declination.Validated += new System.EventHandler(this.TXT_declination_Validated);
            // 
            // CHK_enableairspeed
            // 
            resources.ApplyResources(this.CHK_enableairspeed, "CHK_enableairspeed");
            this.CHK_enableairspeed.Name = "CHK_enableairspeed";
            this.CHK_enableairspeed.UseVisualStyleBackColor = true;
            this.CHK_enableairspeed.CheckedChanged += new System.EventHandler(this.CHK_enableairspeed_CheckedChanged);
            // 
            // CHK_enablesonar
            // 
            resources.ApplyResources(this.CHK_enablesonar, "CHK_enablesonar");
            this.CHK_enablesonar.Name = "CHK_enablesonar";
            this.CHK_enablesonar.UseVisualStyleBackColor = true;
            this.CHK_enablesonar.CheckedChanged += new System.EventHandler(this.CHK_enablesonar_CheckedChanged);
            // 
            // CHK_enablecompass
            // 
            resources.ApplyResources(this.CHK_enablecompass, "CHK_enablecompass");
            this.CHK_enablecompass.Name = "CHK_enablecompass";
            this.CHK_enablecompass.UseVisualStyleBackColor = true;
            this.CHK_enablecompass.CheckedChanged += new System.EventHandler(this.CHK_enablecompass_CheckedChanged);
            // 
            // pictureBox4
            // 
            this.pictureBox4.BackColor = System.Drawing.Color.White;
            this.pictureBox4.BackgroundImage = global::ArdupilotMega.Properties.Resources.airspeed;
            resources.ApplyResources(this.pictureBox4, "pictureBox4");
            this.pictureBox4.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pictureBox4.Name = "pictureBox4";
            this.pictureBox4.TabStop = false;
            // 
            // pictureBox3
            // 
            this.pictureBox3.BackColor = System.Drawing.Color.White;
            this.pictureBox3.BackgroundImage = global::ArdupilotMega.Properties.Resources.sonar;
            resources.ApplyResources(this.pictureBox3, "pictureBox3");
            this.pictureBox3.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pictureBox3.Name = "pictureBox3";
            this.pictureBox3.TabStop = false;
            // 
            // pictureBox1
            // 
            this.pictureBox1.BackgroundImage = global::ArdupilotMega.Properties.Resources.compass;
            resources.ApplyResources(this.pictureBox1, "pictureBox1");
            this.pictureBox1.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.TabStop = false;
            // 
            // BUT_MagCalibrationLog
            // 
            resources.ApplyResources(this.BUT_MagCalibrationLog, "BUT_MagCalibrationLog");
            this.BUT_MagCalibrationLog.Name = "BUT_MagCalibrationLog";
            this.BUT_MagCalibrationLog.UseVisualStyleBackColor = true;
            this.BUT_MagCalibrationLog.Click += new System.EventHandler(this.BUT_MagCalibrationLog_Click);
            // 
            // ConfigHardwareOptions
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.BUT_MagCalibrationLog);
            this.Controls.Add(this.BUT_MagCalibrationLive);
            this.Controls.Add(this.label27);
            this.Controls.Add(this.CMB_sonartype);
            this.Controls.Add(this.CHK_enableoptflow);
            this.Controls.Add(this.pictureBox2);
            this.Controls.Add(this.linkLabelmagdec);
            this.Controls.Add(this.label100);
            this.Controls.Add(this.TXT_declination);
            this.Controls.Add(this.CHK_enableairspeed);
            this.Controls.Add(this.CHK_enablesonar);
            this.Controls.Add(this.CHK_enablecompass);
            this.Controls.Add(this.pictureBox4);
            this.Controls.Add(this.pictureBox3);
            this.Controls.Add(this.pictureBox1);
            this.Name = "ConfigHardwareOptions";
            this.Load += new System.EventHandler(this.ConfigHardwareOptions_Load);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox4)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private MyButton BUT_MagCalibrationLive;
        private System.Windows.Forms.Label label27;
        private System.Windows.Forms.ComboBox CMB_sonartype;
        private System.Windows.Forms.CheckBox CHK_enableoptflow;
        private System.Windows.Forms.PictureBox pictureBox2;
        private System.Windows.Forms.LinkLabel linkLabelmagdec;
        private System.Windows.Forms.Label label100;
        private System.Windows.Forms.TextBox TXT_declination;
        private System.Windows.Forms.CheckBox CHK_enableairspeed;
        private System.Windows.Forms.CheckBox CHK_enablesonar;
        private System.Windows.Forms.CheckBox CHK_enablecompass;
        private System.Windows.Forms.PictureBox pictureBox4;
        private System.Windows.Forms.PictureBox pictureBox3;
        private System.Windows.Forms.PictureBox pictureBox1;
        private MyButton BUT_MagCalibrationLog;
    }
}

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
            this.BUT_MagCalibrationLive = new ArdupilotMega.Controls.MyButton();
            this.label27 = new System.Windows.Forms.Label();
            this.CMB_sonartype = new System.Windows.Forms.ComboBox();
            this.CHK_enableoptflow = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.linkLabelmagdec = new System.Windows.Forms.LinkLabel();
            this.label100 = new System.Windows.Forms.Label();
            this.TXT_declination = new System.Windows.Forms.TextBox();
            this.CHK_enableairspeed = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.CHK_enablesonar = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.CHK_enablecompass = new ArdupilotMega.Controls.MavlinkCheckBox();
            this.pictureBox4 = new System.Windows.Forms.PictureBox();
            this.pictureBox3 = new System.Windows.Forms.PictureBox();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.BUT_MagCalibrationLog = new ArdupilotMega.Controls.MyButton();
            this.CHK_autodec = new System.Windows.Forms.CheckBox();
            this.label5 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.label1 = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.label2 = new System.Windows.Forms.Label();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.label3 = new System.Windows.Forms.Label();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.CHK_airspeeduse = new ArdupilotMega.Controls.MavlinkCheckBox();
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
            resources.GetString("CMB_sonartype.Items2"),
            resources.GetString("CMB_sonartype.Items3")});
            resources.ApplyResources(this.CMB_sonartype, "CMB_sonartype");
            this.CMB_sonartype.Name = "CMB_sonartype";
            this.CMB_sonartype.SelectedIndexChanged += new System.EventHandler(this.CMB_sonartype_SelectedIndexChanged);
            // 
            // CHK_enableoptflow
            // 
            resources.ApplyResources(this.CHK_enableoptflow, "CHK_enableoptflow");
            this.CHK_enableoptflow.Name = "CHK_enableoptflow";
            this.CHK_enableoptflow.OffValue = 0F;
            this.CHK_enableoptflow.OnValue = 1F;
            this.CHK_enableoptflow.param = null;
            this.CHK_enableoptflow.ParamName = null;
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
            this.CHK_enableairspeed.OffValue = 0F;
            this.CHK_enableairspeed.OnValue = 1F;
            this.CHK_enableairspeed.param = null;
            this.CHK_enableairspeed.ParamName = null;
            this.CHK_enableairspeed.UseVisualStyleBackColor = true;
            this.CHK_enableairspeed.CheckedChanged += new System.EventHandler(this.CHK_enableairspeed_CheckedChanged);
            // 
            // CHK_enablesonar
            // 
            resources.ApplyResources(this.CHK_enablesonar, "CHK_enablesonar");
            this.CHK_enablesonar.Name = "CHK_enablesonar";
            this.CHK_enablesonar.OffValue = 0F;
            this.CHK_enablesonar.OnValue = 1F;
            this.CHK_enablesonar.param = null;
            this.CHK_enablesonar.ParamName = null;
            this.CHK_enablesonar.UseVisualStyleBackColor = true;
            this.CHK_enablesonar.CheckedChanged += new System.EventHandler(this.CHK_enablesonar_CheckedChanged);
            // 
            // CHK_enablecompass
            // 
            resources.ApplyResources(this.CHK_enablecompass, "CHK_enablecompass");
            this.CHK_enablecompass.Name = "CHK_enablecompass";
            this.CHK_enablecompass.OffValue = 0F;
            this.CHK_enablecompass.OnValue = 1F;
            this.CHK_enablecompass.param = null;
            this.CHK_enablecompass.ParamName = null;
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
            // CHK_autodec
            // 
            resources.ApplyResources(this.CHK_autodec, "CHK_autodec");
            this.CHK_autodec.Name = "CHK_autodec";
            this.CHK_autodec.UseVisualStyleBackColor = true;
            this.CHK_autodec.CheckedChanged += new System.EventHandler(this.CHK_autodec_CheckedChanged);
            // 
            // label5
            // 
            resources.ApplyResources(this.label5, "label5");
            this.label5.Name = "label5";
            // 
            // groupBox2
            // 
            resources.ApplyResources(this.groupBox2, "groupBox2");
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.TabStop = false;
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            // 
            // groupBox1
            // 
            resources.ApplyResources(this.groupBox1, "groupBox1");
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.TabStop = false;
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // groupBox3
            // 
            resources.ApplyResources(this.groupBox3, "groupBox3");
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.TabStop = false;
            // 
            // label3
            // 
            resources.ApplyResources(this.label3, "label3");
            this.label3.Name = "label3";
            // 
            // groupBox4
            // 
            resources.ApplyResources(this.groupBox4, "groupBox4");
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.TabStop = false;
            // 
            // CHK_airspeeduse
            // 
            resources.ApplyResources(this.CHK_airspeeduse, "CHK_airspeeduse");
            this.CHK_airspeeduse.Name = "CHK_airspeeduse";
            this.CHK_airspeeduse.OffValue = 0F;
            this.CHK_airspeeduse.OnValue = 1F;
            this.CHK_airspeeduse.param = null;
            this.CHK_airspeeduse.ParamName = null;
            this.CHK_airspeeduse.UseVisualStyleBackColor = true;
            // 
            // ConfigHardwareOptions
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.CHK_airspeeduse);
            this.Controls.Add(this.CHK_enableoptflow);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.CHK_enableairspeed);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.CHK_enablesonar);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.CHK_enablecompass);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.CHK_autodec);
            this.Controls.Add(this.BUT_MagCalibrationLog);
            this.Controls.Add(this.BUT_MagCalibrationLive);
            this.Controls.Add(this.label27);
            this.Controls.Add(this.CMB_sonartype);
            this.Controls.Add(this.pictureBox2);
            this.Controls.Add(this.linkLabelmagdec);
            this.Controls.Add(this.label100);
            this.Controls.Add(this.TXT_declination);
            this.Controls.Add(this.pictureBox4);
            this.Controls.Add(this.pictureBox3);
            this.Controls.Add(this.pictureBox1);
            this.Name = "ConfigHardwareOptions";
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox4)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private ArdupilotMega.Controls.MyButton BUT_MagCalibrationLive;
        private System.Windows.Forms.Label label27;
        private System.Windows.Forms.ComboBox CMB_sonartype;
        private ArdupilotMega.Controls.MavlinkCheckBox CHK_enableoptflow;
        private System.Windows.Forms.PictureBox pictureBox2;
        private System.Windows.Forms.LinkLabel linkLabelmagdec;
        private System.Windows.Forms.Label label100;
        private System.Windows.Forms.TextBox TXT_declination;
        private ArdupilotMega.Controls.MavlinkCheckBox CHK_enableairspeed;
        private ArdupilotMega.Controls.MavlinkCheckBox CHK_enablesonar;
        private ArdupilotMega.Controls.MavlinkCheckBox CHK_enablecompass;
        private System.Windows.Forms.PictureBox pictureBox4;
        private System.Windows.Forms.PictureBox pictureBox3;
        private System.Windows.Forms.PictureBox pictureBox1;
        private ArdupilotMega.Controls.MyButton BUT_MagCalibrationLog;
        private System.Windows.Forms.CheckBox CHK_autodec;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.GroupBox groupBox4;
        private Controls.MavlinkCheckBox CHK_airspeeduse;
    }
}

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
            this.BUT_MagCalibration = new ArdupilotMega.MyButton();
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
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox4)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            this.SuspendLayout();
            // 
            // BUT_MagCalibration
            // 
            this.BUT_MagCalibration.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_MagCalibration.Location = new System.Drawing.Point(340, 13);
            this.BUT_MagCalibration.Name = "BUT_MagCalibration";
            this.BUT_MagCalibration.Size = new System.Drawing.Size(75, 23);
            this.BUT_MagCalibration.TabIndex = 47;
            this.BUT_MagCalibration.Text = "Calibration";
            this.BUT_MagCalibration.UseVisualStyleBackColor = true;
            // 
            // label27
            // 
            this.label27.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label27.Location = new System.Drawing.Point(445, 45);
            this.label27.Name = "label27";
            this.label27.Size = new System.Drawing.Size(150, 20);
            this.label27.TabIndex = 46;
            this.label27.Text = "in Degrees eg 2° 3\' W is -2.3";
            // 
            // CMB_sonartype
            // 
            this.CMB_sonartype.FormattingEnabled = true;
            this.CMB_sonartype.Items.AddRange(new object[] {
            "XL-EZ0",
            "LV-EZ0",
            "XL-EZL0"});
            this.CMB_sonartype.Location = new System.Drawing.Point(243, 122);
            this.CMB_sonartype.Name = "CMB_sonartype";
            this.CMB_sonartype.Size = new System.Drawing.Size(121, 21);
            this.CMB_sonartype.TabIndex = 45;
            // 
            // CHK_enableoptflow
            // 
            this.CHK_enableoptflow.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_enableoptflow.Location = new System.Drawing.Point(97, 285);
            this.CHK_enableoptflow.Name = "CHK_enableoptflow";
            this.CHK_enableoptflow.Size = new System.Drawing.Size(134, 19);
            this.CHK_enableoptflow.TabIndex = 44;
            this.CHK_enableoptflow.Text = "Enable Optical Flow";
            this.CHK_enableoptflow.UseVisualStyleBackColor = true;
            // 
            // pictureBox2
            // 
            this.pictureBox2.BackColor = System.Drawing.Color.White;
            this.pictureBox2.BackgroundImage = global::ArdupilotMega.Properties.Resources.opticalflow;
            this.pictureBox2.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pictureBox2.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pictureBox2.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.pictureBox2.Location = new System.Drawing.Point(13, 259);
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.Size = new System.Drawing.Size(75, 75);
            this.pictureBox2.TabIndex = 43;
            this.pictureBox2.TabStop = false;
            // 
            // linkLabelmagdec
            // 
            this.linkLabelmagdec.AutoSize = true;
            this.linkLabelmagdec.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.linkLabelmagdec.Location = new System.Drawing.Point(325, 68);
            this.linkLabelmagdec.Name = "linkLabelmagdec";
            this.linkLabelmagdec.Size = new System.Drawing.Size(104, 13);
            this.linkLabelmagdec.TabIndex = 42;
            this.linkLabelmagdec.TabStop = true;
            this.linkLabelmagdec.Text = "Declination WebSite";
            // 
            // label100
            // 
            this.label100.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label100.Location = new System.Drawing.Point(240, 45);
            this.label100.Name = "label100";
            this.label100.Size = new System.Drawing.Size(72, 16);
            this.label100.TabIndex = 38;
            this.label100.Text = "Declination";
            // 
            // TXT_declination
            // 
            this.TXT_declination.Location = new System.Drawing.Point(318, 45);
            this.TXT_declination.Name = "TXT_declination";
            this.TXT_declination.Size = new System.Drawing.Size(121, 20);
            this.TXT_declination.TabIndex = 37;
            // 
            // CHK_enableairspeed
            // 
            this.CHK_enableairspeed.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_enableairspeed.Location = new System.Drawing.Point(97, 202);
            this.CHK_enableairspeed.Name = "CHK_enableairspeed";
            this.CHK_enableairspeed.Size = new System.Drawing.Size(103, 17);
            this.CHK_enableairspeed.TabIndex = 39;
            this.CHK_enableairspeed.Text = "Enable Airspeed";
            this.CHK_enableairspeed.UseVisualStyleBackColor = true;
            // 
            // CHK_enablesonar
            // 
            this.CHK_enablesonar.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_enablesonar.Location = new System.Drawing.Point(94, 124);
            this.CHK_enablesonar.Name = "CHK_enablesonar";
            this.CHK_enablesonar.Size = new System.Drawing.Size(90, 17);
            this.CHK_enablesonar.TabIndex = 40;
            this.CHK_enablesonar.Text = "Enable Sonar";
            this.CHK_enablesonar.UseVisualStyleBackColor = true;
            // 
            // CHK_enablecompass
            // 
            this.CHK_enablecompass.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.CHK_enablecompass.Location = new System.Drawing.Point(97, 44);
            this.CHK_enablecompass.Name = "CHK_enablecompass";
            this.CHK_enablecompass.Size = new System.Drawing.Size(105, 17);
            this.CHK_enablecompass.TabIndex = 41;
            this.CHK_enablecompass.Text = "Enable Compass";
            this.CHK_enablecompass.UseVisualStyleBackColor = true;
            // 
            // pictureBox4
            // 
            this.pictureBox4.BackColor = System.Drawing.Color.White;
            this.pictureBox4.BackgroundImage = global::ArdupilotMega.Properties.Resources.airspeed;
            this.pictureBox4.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pictureBox4.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pictureBox4.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.pictureBox4.Location = new System.Drawing.Point(13, 176);
            this.pictureBox4.Name = "pictureBox4";
            this.pictureBox4.Size = new System.Drawing.Size(75, 75);
            this.pictureBox4.TabIndex = 36;
            this.pictureBox4.TabStop = false;
            // 
            // pictureBox3
            // 
            this.pictureBox3.BackColor = System.Drawing.Color.White;
            this.pictureBox3.BackgroundImage = global::ArdupilotMega.Properties.Resources.sonar;
            this.pictureBox3.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pictureBox3.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pictureBox3.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.pictureBox3.Location = new System.Drawing.Point(13, 94);
            this.pictureBox3.Name = "pictureBox3";
            this.pictureBox3.Size = new System.Drawing.Size(75, 75);
            this.pictureBox3.TabIndex = 35;
            this.pictureBox3.TabStop = false;
            // 
            // pictureBox1
            // 
            this.pictureBox1.BackgroundImage = global::ArdupilotMega.Properties.Resources.compass;
            this.pictureBox1.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pictureBox1.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pictureBox1.ErrorImage = null;
            this.pictureBox1.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.pictureBox1.InitialImage = null;
            this.pictureBox1.Location = new System.Drawing.Point(13, 13);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(75, 75);
            this.pictureBox1.TabIndex = 34;
            this.pictureBox1.TabStop = false;
            // 
            // ConfigHardwareOptions
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.BUT_MagCalibration);
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
            this.Size = new System.Drawing.Size(602, 351);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox4)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private MyButton BUT_MagCalibration;
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
    }
}

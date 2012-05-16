namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigRadioInput
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(ConfigRadioInput));
            this.groupBoxElevons = new System.Windows.Forms.GroupBox();
            this.CHK_mixmode = new System.Windows.Forms.CheckBox();
            this.CHK_elevonch2rev = new System.Windows.Forms.CheckBox();
            this.CHK_elevonrev = new System.Windows.Forms.CheckBox();
            this.CHK_elevonch1rev = new System.Windows.Forms.CheckBox();
            this.CHK_revch3 = new System.Windows.Forms.CheckBox();
            this.CHK_revch4 = new System.Windows.Forms.CheckBox();
            this.CHK_revch2 = new System.Windows.Forms.CheckBox();
            this.CHK_revch1 = new System.Windows.Forms.CheckBox();
            this.BUT_Calibrateradio = new ArdupilotMega.Controls.MyButton();
            this.BAR8 = new ArdupilotMega.HorizontalProgressBar2();
            this.currentStateBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.BAR7 = new ArdupilotMega.HorizontalProgressBar2();
            this.BAR6 = new ArdupilotMega.HorizontalProgressBar2();
            this.BAR5 = new ArdupilotMega.HorizontalProgressBar2();
            this.BARpitch = new ArdupilotMega.VerticalProgressBar2();
            this.BARthrottle = new ArdupilotMega.VerticalProgressBar2();
            this.BARyaw = new ArdupilotMega.HorizontalProgressBar2();
            this.BARroll = new ArdupilotMega.HorizontalProgressBar2();
            this.groupBoxElevons.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.currentStateBindingSource)).BeginInit();
            this.SuspendLayout();
            // 
            // groupBoxElevons
            // 
            this.groupBoxElevons.Controls.Add(this.CHK_mixmode);
            this.groupBoxElevons.Controls.Add(this.CHK_elevonch2rev);
            this.groupBoxElevons.Controls.Add(this.CHK_elevonrev);
            this.groupBoxElevons.Controls.Add(this.CHK_elevonch1rev);
            resources.ApplyResources(this.groupBoxElevons, "groupBoxElevons");
            this.groupBoxElevons.Name = "groupBoxElevons";
            this.groupBoxElevons.TabStop = false;
            // 
            // CHK_mixmode
            // 
            resources.ApplyResources(this.CHK_mixmode, "CHK_mixmode");
            this.CHK_mixmode.Name = "CHK_mixmode";
            this.CHK_mixmode.UseVisualStyleBackColor = true;
            this.CHK_mixmode.CheckedChanged += new System.EventHandler(this.CHK_mixmode_CheckedChanged);
            // 
            // CHK_elevonch2rev
            // 
            resources.ApplyResources(this.CHK_elevonch2rev, "CHK_elevonch2rev");
            this.CHK_elevonch2rev.Name = "CHK_elevonch2rev";
            this.CHK_elevonch2rev.UseVisualStyleBackColor = true;
            this.CHK_elevonch2rev.CheckedChanged += new System.EventHandler(this.CHK_elevonch2rev_CheckedChanged);
            // 
            // CHK_elevonrev
            // 
            resources.ApplyResources(this.CHK_elevonrev, "CHK_elevonrev");
            this.CHK_elevonrev.Name = "CHK_elevonrev";
            this.CHK_elevonrev.UseVisualStyleBackColor = true;
            this.CHK_elevonrev.CheckedChanged += new System.EventHandler(this.CHK_elevonrev_CheckedChanged);
            // 
            // CHK_elevonch1rev
            // 
            resources.ApplyResources(this.CHK_elevonch1rev, "CHK_elevonch1rev");
            this.CHK_elevonch1rev.Name = "CHK_elevonch1rev";
            this.CHK_elevonch1rev.UseVisualStyleBackColor = true;
            this.CHK_elevonch1rev.CheckedChanged += new System.EventHandler(this.CHK_elevonch1rev_CheckedChanged);
            // 
            // CHK_revch3
            // 
            resources.ApplyResources(this.CHK_revch3, "CHK_revch3");
            this.CHK_revch3.Name = "CHK_revch3";
            this.CHK_revch3.UseVisualStyleBackColor = true;
            this.CHK_revch3.CheckedChanged += new System.EventHandler(this.CHK_revch3_CheckedChanged);
            // 
            // CHK_revch4
            // 
            resources.ApplyResources(this.CHK_revch4, "CHK_revch4");
            this.CHK_revch4.Name = "CHK_revch4";
            this.CHK_revch4.UseVisualStyleBackColor = true;
            this.CHK_revch4.CheckedChanged += new System.EventHandler(this.CHK_revch4_CheckedChanged);
            // 
            // CHK_revch2
            // 
            resources.ApplyResources(this.CHK_revch2, "CHK_revch2");
            this.CHK_revch2.Name = "CHK_revch2";
            this.CHK_revch2.UseVisualStyleBackColor = true;
            this.CHK_revch2.CheckedChanged += new System.EventHandler(this.CHK_revch2_CheckedChanged);
            // 
            // CHK_revch1
            // 
            resources.ApplyResources(this.CHK_revch1, "CHK_revch1");
            this.CHK_revch1.Name = "CHK_revch1";
            this.CHK_revch1.UseVisualStyleBackColor = true;
            this.CHK_revch1.CheckedChanged += new System.EventHandler(this.CHK_revch1_CheckedChanged);
            // 
            // BUT_Calibrateradio
            // 
            resources.ApplyResources(this.BUT_Calibrateradio, "BUT_Calibrateradio");
            this.BUT_Calibrateradio.Name = "BUT_Calibrateradio";
            this.BUT_Calibrateradio.UseVisualStyleBackColor = true;
            this.BUT_Calibrateradio.Click += new System.EventHandler(this.BUT_Calibrateradio_Click);
            // 
            // BAR8
            // 
            this.BAR8.BackgroundColor = System.Drawing.Color.FromArgb(((int)(((byte)(20)))), ((int)(((byte)(20)))), ((int)(((byte)(255)))));
            this.BAR8.BorderColor = System.Drawing.SystemColors.ActiveBorder;
            this.BAR8.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch8in", true));
            this.BAR8.Label = "Radio 8";
            resources.ApplyResources(this.BAR8, "BAR8");
            this.BAR8.Maximum = 2200;
            this.BAR8.maxline = 0;
            this.BAR8.Minimum = 800;
            this.BAR8.minline = 0;
            this.BAR8.Name = "BAR8";
            this.BAR8.Value = 1500;
            this.BAR8.ValueColor = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(0)))), ((int)(((byte)(255)))));
            // 
            // currentStateBindingSource
            // 
            this.currentStateBindingSource.DataSource = typeof(ArdupilotMega.CurrentState);
            // 
            // BAR7
            // 
            this.BAR7.BackgroundColor = System.Drawing.Color.FromArgb(((int)(((byte)(20)))), ((int)(((byte)(20)))), ((int)(((byte)(255)))));
            this.BAR7.BorderColor = System.Drawing.SystemColors.ActiveBorder;
            this.BAR7.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch7in", true));
            this.BAR7.Label = "Radio 7";
            resources.ApplyResources(this.BAR7, "BAR7");
            this.BAR7.Maximum = 2200;
            this.BAR7.maxline = 0;
            this.BAR7.Minimum = 800;
            this.BAR7.minline = 0;
            this.BAR7.Name = "BAR7";
            this.BAR7.Value = 1500;
            this.BAR7.ValueColor = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(0)))), ((int)(((byte)(255)))));
            // 
            // BAR6
            // 
            this.BAR6.BackgroundColor = System.Drawing.Color.FromArgb(((int)(((byte)(20)))), ((int)(((byte)(20)))), ((int)(((byte)(255)))));
            this.BAR6.BorderColor = System.Drawing.SystemColors.ActiveBorder;
            this.BAR6.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch6in", true));
            this.BAR6.Label = "Radio 6";
            resources.ApplyResources(this.BAR6, "BAR6");
            this.BAR6.Maximum = 2200;
            this.BAR6.maxline = 0;
            this.BAR6.Minimum = 800;
            this.BAR6.minline = 0;
            this.BAR6.Name = "BAR6";
            this.BAR6.Value = 1500;
            this.BAR6.ValueColor = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(0)))), ((int)(((byte)(255)))));
            // 
            // BAR5
            // 
            this.BAR5.BackgroundColor = System.Drawing.Color.FromArgb(((int)(((byte)(20)))), ((int)(((byte)(20)))), ((int)(((byte)(255)))));
            this.BAR5.BorderColor = System.Drawing.SystemColors.ActiveBorder;
            this.BAR5.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch5in", true));
            this.BAR5.Label = "Radio 5";
            resources.ApplyResources(this.BAR5, "BAR5");
            this.BAR5.Maximum = 2200;
            this.BAR5.maxline = 0;
            this.BAR5.Minimum = 800;
            this.BAR5.minline = 0;
            this.BAR5.Name = "BAR5";
            this.BAR5.Value = 1500;
            this.BAR5.ValueColor = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(0)))), ((int)(((byte)(255)))));
            // 
            // BARpitch
            // 
            this.BARpitch.BackgroundColor = System.Drawing.Color.FromArgb(((int)(((byte)(20)))), ((int)(((byte)(20)))), ((int)(((byte)(255)))));
            this.BARpitch.BorderColor = System.Drawing.SystemColors.ActiveBorder;
            this.BARpitch.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch2in", true));
            this.BARpitch.Label = "Pitch";
            resources.ApplyResources(this.BARpitch, "BARpitch");
            this.BARpitch.Maximum = 2200;
            this.BARpitch.maxline = 0;
            this.BARpitch.Minimum = 800;
            this.BARpitch.minline = 0;
            this.BARpitch.Name = "BARpitch";
            this.BARpitch.Value = 1500;
            this.BARpitch.ValueColor = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(0)))), ((int)(((byte)(255)))));
            // 
            // BARthrottle
            // 
            this.BARthrottle.BackgroundColor = System.Drawing.Color.FromArgb(((int)(((byte)(67)))), ((int)(((byte)(68)))), ((int)(((byte)(69)))));
            this.BARthrottle.BorderColor = System.Drawing.SystemColors.ActiveBorder;
            this.BARthrottle.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch3in", true));
            this.BARthrottle.Label = "Throttle";
            resources.ApplyResources(this.BARthrottle, "BARthrottle");
            this.BARthrottle.Maximum = 2200;
            this.BARthrottle.maxline = 0;
            this.BARthrottle.Minimum = 800;
            this.BARthrottle.minline = 0;
            this.BARthrottle.Name = "BARthrottle";
            this.BARthrottle.Value = 1000;
            this.BARthrottle.ValueColor = System.Drawing.Color.Magenta;
            // 
            // BARyaw
            // 
            this.BARyaw.BackgroundColor = System.Drawing.Color.FromArgb(((int)(((byte)(20)))), ((int)(((byte)(20)))), ((int)(((byte)(255)))));
            this.BARyaw.BorderColor = System.Drawing.SystemColors.ActiveBorder;
            this.BARyaw.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch4in", true));
            this.BARyaw.Label = "Yaw";
            resources.ApplyResources(this.BARyaw, "BARyaw");
            this.BARyaw.Maximum = 2200;
            this.BARyaw.maxline = 0;
            this.BARyaw.Minimum = 800;
            this.BARyaw.minline = 0;
            this.BARyaw.Name = "BARyaw";
            this.BARyaw.Value = 1500;
            this.BARyaw.ValueColor = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(0)))), ((int)(((byte)(255)))));
            // 
            // BARroll
            // 
            this.BARroll.BackgroundColor = System.Drawing.Color.FromArgb(((int)(((byte)(20)))), ((int)(((byte)(20)))), ((int)(((byte)(255)))));
            this.BARroll.BorderColor = System.Drawing.SystemColors.ActiveBorder;
            this.BARroll.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.currentStateBindingSource, "ch1in", true));
            this.BARroll.Label = "Roll";
            resources.ApplyResources(this.BARroll, "BARroll");
            this.BARroll.Maximum = 2200;
            this.BARroll.maxline = 0;
            this.BARroll.Minimum = 800;
            this.BARroll.minline = 0;
            this.BARroll.Name = "BARroll";
            this.BARroll.Value = 1500;
            this.BARroll.ValueColor = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(0)))), ((int)(((byte)(255)))));
            // 
            // ConfigRadioInput
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.groupBoxElevons);
            this.Controls.Add(this.CHK_revch3);
            this.Controls.Add(this.CHK_revch4);
            this.Controls.Add(this.CHK_revch2);
            this.Controls.Add(this.CHK_revch1);
            this.Controls.Add(this.BUT_Calibrateradio);
            this.Controls.Add(this.BAR8);
            this.Controls.Add(this.BAR7);
            this.Controls.Add(this.BAR6);
            this.Controls.Add(this.BAR5);
            this.Controls.Add(this.BARpitch);
            this.Controls.Add(this.BARthrottle);
            this.Controls.Add(this.BARyaw);
            this.Controls.Add(this.BARroll);
            this.Name = "ConfigRadioInput";
            this.Load += new System.EventHandler(this.ConfigRadioInput_Load);
            this.groupBoxElevons.ResumeLayout(false);
            this.groupBoxElevons.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.currentStateBindingSource)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.GroupBox groupBoxElevons;
        private System.Windows.Forms.CheckBox CHK_mixmode;
        private System.Windows.Forms.CheckBox CHK_elevonch2rev;
        private System.Windows.Forms.CheckBox CHK_elevonrev;
        private System.Windows.Forms.CheckBox CHK_elevonch1rev;
        private System.Windows.Forms.CheckBox CHK_revch3;
        private System.Windows.Forms.CheckBox CHK_revch4;
        private System.Windows.Forms.CheckBox CHK_revch2;
        private System.Windows.Forms.CheckBox CHK_revch1;
        private ArdupilotMega.Controls.MyButton BUT_Calibrateradio;
        private HorizontalProgressBar2 BAR8;
        private HorizontalProgressBar2 BAR7;
        private HorizontalProgressBar2 BAR6;
        private HorizontalProgressBar2 BAR5;
        private VerticalProgressBar2 BARpitch;
        private VerticalProgressBar2 BARthrottle;
        private HorizontalProgressBar2 BARyaw;
        private HorizontalProgressBar2 BARroll;
        private System.Windows.Forms.BindingSource currentStateBindingSource;
    }
}

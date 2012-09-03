namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigAP_Limits
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(ConfigAP_Limits));
            this.LNK_wiki = new System.Windows.Forms.LinkLabel();
            this.LIM_ENABLED = new Controls.MavlinkCheckBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.LIM_GPSLCK_REQ = new System.Windows.Forms.CheckBox();
            this.LIM_FNC_REQ = new System.Windows.Forms.CheckBox();
            this.LIM_ALT_REQ = new System.Windows.Forms.CheckBox();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.LIM_REQUIRED = new System.Windows.Forms.CheckBox();
            this.myLabel4 = new ArdupilotMega.Controls.MyLabel();
            this.LIM_CHANNEL = new System.Windows.Forms.NumericUpDown();
            this.myLabel3 = new ArdupilotMega.Controls.MyLabel();
            this.LIM_FNC_RAD = new System.Windows.Forms.NumericUpDown();
            this.LIM_FNC_SMPL = new Controls.MavlinkCheckBox();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.LIM_GPSLCK_ON = new System.Windows.Forms.CheckBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.LIM_ALT_ON = new System.Windows.Forms.CheckBox();
            this.myLabel1 = new ArdupilotMega.Controls.MyLabel();
            this.LIM_ALT_MAX = new System.Windows.Forms.NumericUpDown();
            this.myLabel2 = new ArdupilotMega.Controls.MyLabel();
            this.LIM_ALT_MIN = new System.Windows.Forms.NumericUpDown();
            this.groupBox1.SuspendLayout();
            this.groupBox5.SuspendLayout();
            this.groupBox4.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_CHANNEL)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_FNC_RAD)).BeginInit();
            this.groupBox3.SuspendLayout();
            this.groupBox2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_ALT_MAX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_ALT_MIN)).BeginInit();
            this.SuspendLayout();
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
            // LIM_ENABLED
            // 
            resources.ApplyResources(this.LIM_ENABLED, "LIM_ENABLED");
            this.LIM_ENABLED.Name = "LIM_ENABLED";
            this.LIM_ENABLED.UseVisualStyleBackColor = true;
            this.LIM_ENABLED.CheckedChanged += new System.EventHandler(this.LIM_ENABLED_CheckedChanged);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.textBox1);
            this.groupBox1.Controls.Add(this.groupBox5);
            this.groupBox1.Controls.Add(this.groupBox4);
            this.groupBox1.Controls.Add(this.groupBox3);
            this.groupBox1.Controls.Add(this.groupBox2);
            resources.ApplyResources(this.groupBox1, "groupBox1");
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.TabStop = false;
            // 
            // textBox1
            // 
            resources.ApplyResources(this.textBox1, "textBox1");
            this.textBox1.Name = "textBox1";
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.LIM_GPSLCK_REQ);
            this.groupBox5.Controls.Add(this.LIM_FNC_REQ);
            this.groupBox5.Controls.Add(this.LIM_ALT_REQ);
            resources.ApplyResources(this.groupBox5, "groupBox5");
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.TabStop = false;
            // 
            // LIM_GPSLCK_REQ
            // 
            resources.ApplyResources(this.LIM_GPSLCK_REQ, "LIM_GPSLCK_REQ");
            this.LIM_GPSLCK_REQ.Name = "LIM_GPSLCK_REQ";
            this.LIM_GPSLCK_REQ.UseVisualStyleBackColor = true;
            this.LIM_GPSLCK_REQ.CheckedChanged += new System.EventHandler(this.ProcessChange);
            // 
            // LIM_FNC_REQ
            // 
            resources.ApplyResources(this.LIM_FNC_REQ, "LIM_FNC_REQ");
            this.LIM_FNC_REQ.Name = "LIM_FNC_REQ";
            this.LIM_FNC_REQ.UseVisualStyleBackColor = true;
            this.LIM_FNC_REQ.CheckedChanged += new System.EventHandler(this.ProcessChange);
            // 
            // LIM_ALT_REQ
            // 
            resources.ApplyResources(this.LIM_ALT_REQ, "LIM_ALT_REQ");
            this.LIM_ALT_REQ.Name = "LIM_ALT_REQ";
            this.LIM_ALT_REQ.UseVisualStyleBackColor = true;
            this.LIM_ALT_REQ.CheckedChanged += new System.EventHandler(this.ProcessChange);
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.LIM_REQUIRED);
            this.groupBox4.Controls.Add(this.myLabel4);
            this.groupBox4.Controls.Add(this.LIM_CHANNEL);
            this.groupBox4.Controls.Add(this.myLabel3);
            this.groupBox4.Controls.Add(this.LIM_FNC_RAD);
            this.groupBox4.Controls.Add(this.LIM_FNC_SMPL);
            resources.ApplyResources(this.groupBox4, "groupBox4");
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.TabStop = false;
            // 
            // LIM_REQUIRED
            // 
            resources.ApplyResources(this.LIM_REQUIRED, "LIM_REQUIRED");
            this.LIM_REQUIRED.Name = "LIM_REQUIRED";
            this.LIM_REQUIRED.UseVisualStyleBackColor = true;
            this.LIM_REQUIRED.CheckedChanged += new System.EventHandler(this.LIM_REQUIRED_CheckedChanged);
            // 
            // myLabel4
            // 
            resources.ApplyResources(this.myLabel4, "myLabel4");
            this.myLabel4.Name = "myLabel4";
            this.myLabel4.resize = false;
            // 
            // LIM_CHANNEL
            // 
            resources.ApplyResources(this.LIM_CHANNEL, "LIM_CHANNEL");
            this.LIM_CHANNEL.Maximum = new decimal(new int[] {
            8,
            0,
            0,
            0});
            this.LIM_CHANNEL.Name = "LIM_CHANNEL";
            this.LIM_CHANNEL.Value = new decimal(new int[] {
            7,
            0,
            0,
            0});
            this.LIM_CHANNEL.ValueChanged += new System.EventHandler(this.ProcessChange);
            // 
            // myLabel3
            // 
            resources.ApplyResources(this.myLabel3, "myLabel3");
            this.myLabel3.Name = "myLabel3";
            this.myLabel3.resize = false;
            // 
            // LIM_FNC_RAD
            // 
            resources.ApplyResources(this.LIM_FNC_RAD, "LIM_FNC_RAD");
            this.LIM_FNC_RAD.Name = "LIM_FNC_RAD";
            this.LIM_FNC_RAD.Value = new decimal(new int[] {
            30,
            0,
            0,
            0});
            this.LIM_FNC_RAD.ValueChanged += new System.EventHandler(this.ProcessChange);
            // 
            // LIM_FNC_SMPL
            // 
            resources.ApplyResources(this.LIM_FNC_SMPL, "LIM_FNC_SMPL");
            this.LIM_FNC_SMPL.Name = "LIM_FNC_SMPL";
            this.LIM_FNC_SMPL.UseVisualStyleBackColor = true;
            this.LIM_FNC_SMPL.CheckedChanged += new System.EventHandler(this.ProcessChange);
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.LIM_GPSLCK_ON);
            resources.ApplyResources(this.groupBox3, "groupBox3");
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.TabStop = false;
            // 
            // LIM_GPSLCK_ON
            // 
            resources.ApplyResources(this.LIM_GPSLCK_ON, "LIM_GPSLCK_ON");
            this.LIM_GPSLCK_ON.Name = "LIM_GPSLCK_ON";
            this.LIM_GPSLCK_ON.UseVisualStyleBackColor = true;
            this.LIM_GPSLCK_ON.CheckedChanged += new System.EventHandler(this.ProcessChange);
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.LIM_ALT_ON);
            this.groupBox2.Controls.Add(this.myLabel1);
            this.groupBox2.Controls.Add(this.LIM_ALT_MAX);
            this.groupBox2.Controls.Add(this.myLabel2);
            this.groupBox2.Controls.Add(this.LIM_ALT_MIN);
            resources.ApplyResources(this.groupBox2, "groupBox2");
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.TabStop = false;
            // 
            // LIM_ALT_ON
            // 
            resources.ApplyResources(this.LIM_ALT_ON, "LIM_ALT_ON");
            this.LIM_ALT_ON.Name = "LIM_ALT_ON";
            this.LIM_ALT_ON.UseVisualStyleBackColor = true;
            this.LIM_ALT_ON.CheckedChanged += new System.EventHandler(this.ProcessChange);
            // 
            // myLabel1
            // 
            resources.ApplyResources(this.myLabel1, "myLabel1");
            this.myLabel1.Name = "myLabel1";
            this.myLabel1.resize = false;
            // 
            // LIM_ALT_MAX
            // 
            resources.ApplyResources(this.LIM_ALT_MAX, "LIM_ALT_MAX");
            this.LIM_ALT_MAX.Name = "LIM_ALT_MAX";
            this.LIM_ALT_MAX.Value = new decimal(new int[] {
            30,
            0,
            0,
            0});
            this.LIM_ALT_MAX.ValueChanged += new System.EventHandler(this.ProcessChange);
            // 
            // myLabel2
            // 
            resources.ApplyResources(this.myLabel2, "myLabel2");
            this.myLabel2.Name = "myLabel2";
            this.myLabel2.resize = false;
            // 
            // LIM_ALT_MIN
            // 
            resources.ApplyResources(this.LIM_ALT_MIN, "LIM_ALT_MIN");
            this.LIM_ALT_MIN.Name = "LIM_ALT_MIN";
            this.LIM_ALT_MIN.ValueChanged += new System.EventHandler(this.ProcessChange);
            // 
            // ConfigAP_Limits
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.LIM_ENABLED);
            this.Controls.Add(this.LNK_wiki);
            this.Name = "ConfigAP_Limits";
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox5.ResumeLayout(false);
            this.groupBox5.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_CHANNEL)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_FNC_RAD)).EndInit();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_ALT_MAX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.LIM_ALT_MIN)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.LinkLabel LNK_wiki;
        private Controls.MavlinkCheckBox LIM_ENABLED;
        private System.Windows.Forms.GroupBox groupBox1;
        private Controls.MyLabel myLabel2;
        private Controls.MyLabel myLabel1;
        private System.Windows.Forms.NumericUpDown LIM_ALT_MAX;
        private System.Windows.Forms.NumericUpDown LIM_ALT_MIN;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.CheckBox LIM_ALT_ON;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.CheckBox LIM_GPSLCK_ON;
        private System.Windows.Forms.GroupBox groupBox4;
        private Controls.MyLabel myLabel3;
        private System.Windows.Forms.NumericUpDown LIM_FNC_RAD;
        private Controls.MavlinkCheckBox LIM_FNC_SMPL;
        private Controls.MyLabel myLabel4;
        private System.Windows.Forms.NumericUpDown LIM_CHANNEL;
        private System.Windows.Forms.CheckBox LIM_REQUIRED;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.CheckBox LIM_FNC_REQ;
        private System.Windows.Forms.CheckBox LIM_ALT_REQ;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.CheckBox LIM_GPSLCK_REQ;

    }
}

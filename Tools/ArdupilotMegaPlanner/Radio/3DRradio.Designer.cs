namespace ArdupilotMega
{
    partial class _3DRradio
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
            this.Progressbar = new System.Windows.Forms.ProgressBar();
            this.S1 = new System.Windows.Forms.ComboBox();
            this.label1 = new System.Windows.Forms.Label();
            this.S0 = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.S2 = new System.Windows.Forms.ComboBox();
            this.label4 = new System.Windows.Forms.Label();
            this.S3 = new System.Windows.Forms.ComboBox();
            this.label5 = new System.Windows.Forms.Label();
            this.S4 = new System.Windows.Forms.ComboBox();
            this.label6 = new System.Windows.Forms.Label();
            this.S5 = new System.Windows.Forms.CheckBox();
            this.label7 = new System.Windows.Forms.Label();
            this.S6 = new System.Windows.Forms.CheckBox();
            this.label8 = new System.Windows.Forms.Label();
            this.S7 = new System.Windows.Forms.CheckBox();
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
            this.RS7 = new System.Windows.Forms.CheckBox();
            this.RS6 = new System.Windows.Forms.CheckBox();
            this.RS5 = new System.Windows.Forms.CheckBox();
            this.RS4 = new System.Windows.Forms.ComboBox();
            this.RS3 = new System.Windows.Forms.ComboBox();
            this.RS2 = new System.Windows.Forms.ComboBox();
            this.RS1 = new System.Windows.Forms.ComboBox();
            this.RS0 = new System.Windows.Forms.TextBox();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.RTI = new System.Windows.Forms.TextBox();
            this.ATI = new System.Windows.Forms.TextBox();
            this.RSSI = new System.Windows.Forms.TextBox();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.BUT_savesettings = new ArdupilotMega.MyButton();
            this.BUT_getcurrent = new ArdupilotMega.MyButton();
            this.lbl_status = new System.Windows.Forms.Label();
            this.BUT_upload = new ArdupilotMega.MyButton();
            this.BUT_syncS2 = new ArdupilotMega.MyButton();
            this.BUT_syncS3 = new ArdupilotMega.MyButton();
            this.BUT_syncS5 = new ArdupilotMega.MyButton();
            this.SuspendLayout();
            // 
            // Progressbar
            // 
            this.Progressbar.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.Progressbar.Location = new System.Drawing.Point(12, 402);
            this.Progressbar.Name = "Progressbar";
            this.Progressbar.Size = new System.Drawing.Size(294, 36);
            this.Progressbar.TabIndex = 2;
            // 
            // S1
            // 
            this.S1.FormattingEnabled = true;
            this.S1.Items.AddRange(new object[] {
            "115",
            "111",
            "57",
            "38",
            "19",
            "9",
            "4",
            "2",
            "1"});
            this.S1.Location = new System.Drawing.Point(87, 141);
            this.S1.Name = "S1";
            this.S1.Size = new System.Drawing.Size(80, 21);
            this.S1.TabIndex = 4;
            this.toolTip1.SetToolTip(this.S1, "Serial Baud Rate 57 = 57600");
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(15, 149);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(32, 13);
            this.label1.TabIndex = 5;
            this.label1.Text = "Baud";
            // 
            // S0
            // 
            this.S0.Location = new System.Drawing.Point(87, 115);
            this.S0.Name = "S0";
            this.S0.ReadOnly = true;
            this.S0.Size = new System.Drawing.Size(80, 20);
            this.S0.TabIndex = 7;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(15, 122);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(39, 13);
            this.label2.TabIndex = 8;
            this.label2.Text = "Format";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(15, 176);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(53, 13);
            this.label3.TabIndex = 10;
            this.label3.Text = "Air Speed";
            // 
            // S2
            // 
            this.S2.FormattingEnabled = true;
            this.S2.Items.AddRange(new object[] {
            "192",
            "160",
            "128",
            "96",
            "64",
            "32",
            "16"});
            this.S2.Location = new System.Drawing.Point(87, 168);
            this.S2.Name = "S2";
            this.S2.Size = new System.Drawing.Size(80, 21);
            this.S2.TabIndex = 9;
            this.toolTip1.SetToolTip(this.S2, "the inter-radio data rate in rounded kbps. So 128 means");
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(15, 203);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(38, 13);
            this.label4.TabIndex = 12;
            this.label4.Text = "Net ID";
            // 
            // S3
            // 
            this.S3.FormattingEnabled = true;
            this.S3.Items.AddRange(new object[] {
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "8",
            "9",
            "10",
            "11",
            "12",
            "13",
            "14",
            "15",
            "16",
            "17",
            "18",
            "19",
            "20",
            "21",
            "22",
            "23",
            "24",
            "25",
            "26",
            "27",
            "28",
            "29",
            "30"});
            this.S3.Location = new System.Drawing.Point(87, 195);
            this.S3.Name = "S3";
            this.S3.Size = new System.Drawing.Size(80, 21);
            this.S3.TabIndex = 11;
            this.toolTip1.SetToolTip(this.S3, "a 16 bit \'network ID\'. This is used to seed the frequency");
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(15, 230);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(52, 13);
            this.label5.TabIndex = 14;
            this.label5.Text = "Tx Power";
            // 
            // S4
            // 
            this.S4.FormattingEnabled = true;
            this.S4.Items.AddRange(new object[] {
            "0",
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "8",
            "9",
            "10",
            "11",
            "12",
            "13",
            "14",
            "15",
            "16",
            "17",
            "18",
            "19",
            "20"});
            this.S4.Location = new System.Drawing.Point(87, 222);
            this.S4.Name = "S4";
            this.S4.Size = new System.Drawing.Size(80, 21);
            this.S4.TabIndex = 13;
            this.toolTip1.SetToolTip(this.S4, "the transmit power in dBm. 20dBm is 100mW. It is useful to");
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(15, 257);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(28, 13);
            this.label6.TabIndex = 16;
            this.label6.Text = "ECC";
            // 
            // S5
            // 
            this.S5.Location = new System.Drawing.Point(87, 249);
            this.S5.Name = "S5";
            this.S5.Size = new System.Drawing.Size(80, 21);
            this.S5.TabIndex = 15;
            this.toolTip1.SetToolTip(this.S5, "to enable/disable the golay error correcting code. It defaults");
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(15, 284);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(44, 13);
            this.label7.TabIndex = 18;
            this.label7.Text = "Mavlink";
            // 
            // S6
            // 
            this.S6.Location = new System.Drawing.Point(87, 276);
            this.S6.Name = "S6";
            this.S6.Size = new System.Drawing.Size(80, 21);
            this.S6.TabIndex = 17;
            this.toolTip1.SetToolTip(this.S6, "enables/disables MAVLink packet framing. This tries to align");
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(15, 311);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(68, 13);
            this.label8.TabIndex = 20;
            this.label8.Text = "Op Pre Send";
            // 
            // S7
            // 
            this.S7.Location = new System.Drawing.Point(87, 303);
            this.S7.Name = "S7";
            this.S7.Size = new System.Drawing.Size(80, 21);
            this.S7.TabIndex = 19;
            this.toolTip1.SetToolTip(this.S7, "enables/disables \"opportunistic resend\". When enabled the");
            // 
            // RS7
            // 
            this.RS7.Location = new System.Drawing.Point(201, 303);
            this.RS7.Name = "RS7";
            this.RS7.Size = new System.Drawing.Size(80, 21);
            this.RS7.TabIndex = 29;
            this.toolTip1.SetToolTip(this.RS7, "enables/disables \"opportunistic resend\". When enabled the");
            // 
            // RS6
            // 
            this.RS6.Location = new System.Drawing.Point(201, 276);
            this.RS6.Name = "RS6";
            this.RS6.Size = new System.Drawing.Size(80, 21);
            this.RS6.TabIndex = 28;
            this.toolTip1.SetToolTip(this.RS6, "enables/disables MAVLink packet framing. This tries to align");
            // 
            // RS5
            // 
            this.RS5.Location = new System.Drawing.Point(201, 249);
            this.RS5.Name = "RS5";
            this.RS5.Size = new System.Drawing.Size(80, 21);
            this.RS5.TabIndex = 27;
            this.toolTip1.SetToolTip(this.RS5, "to enable/disable the golay error correcting code. It defaults");
            // 
            // RS4
            // 
            this.RS4.FormattingEnabled = true;
            this.RS4.Items.AddRange(new object[] {
            "0",
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "8",
            "9",
            "10",
            "11",
            "12",
            "13",
            "14",
            "15",
            "16",
            "17",
            "18",
            "19",
            "20"});
            this.RS4.Location = new System.Drawing.Point(201, 222);
            this.RS4.Name = "RS4";
            this.RS4.Size = new System.Drawing.Size(80, 21);
            this.RS4.TabIndex = 26;
            this.toolTip1.SetToolTip(this.RS4, "the transmit power in dBm. 20dBm is 100mW. It is useful to");
            // 
            // RS3
            // 
            this.RS3.FormattingEnabled = true;
            this.RS3.Items.AddRange(new object[] {
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "8",
            "9",
            "10",
            "11",
            "12",
            "13",
            "14",
            "15",
            "16",
            "17",
            "18",
            "19",
            "20",
            "21",
            "22",
            "23",
            "24",
            "25",
            "26",
            "27",
            "28",
            "29",
            "30"});
            this.RS3.Location = new System.Drawing.Point(201, 195);
            this.RS3.Name = "RS3";
            this.RS3.Size = new System.Drawing.Size(80, 21);
            this.RS3.TabIndex = 25;
            this.toolTip1.SetToolTip(this.RS3, "a 16 bit \'network ID\'. This is used to seed the frequency");
            // 
            // RS2
            // 
            this.RS2.FormattingEnabled = true;
            this.RS2.Items.AddRange(new object[] {
            "192",
            "160",
            "128",
            "96",
            "64",
            "32",
            "16"});
            this.RS2.Location = new System.Drawing.Point(201, 168);
            this.RS2.Name = "RS2";
            this.RS2.Size = new System.Drawing.Size(80, 21);
            this.RS2.TabIndex = 24;
            this.toolTip1.SetToolTip(this.RS2, "the inter-radio data rate in rounded kbps. So 128 means");
            // 
            // RS1
            // 
            this.RS1.FormattingEnabled = true;
            this.RS1.Items.AddRange(new object[] {
            "115",
            "111",
            "57",
            "38",
            "19",
            "9",
            "4",
            "2",
            "1"});
            this.RS1.Location = new System.Drawing.Point(201, 141);
            this.RS1.Name = "RS1";
            this.RS1.Size = new System.Drawing.Size(80, 21);
            this.RS1.TabIndex = 22;
            this.toolTip1.SetToolTip(this.RS1, "Serial Baud Rate 57 = 57600");
            // 
            // RS0
            // 
            this.RS0.Location = new System.Drawing.Point(201, 115);
            this.RS0.Name = "RS0";
            this.RS0.ReadOnly = true;
            this.RS0.Size = new System.Drawing.Size(80, 20);
            this.RS0.TabIndex = 23;
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(108, 9);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(33, 13);
            this.label9.TabIndex = 30;
            this.label9.Text = "Local";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(225, 9);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(44, 13);
            this.label10.TabIndex = 31;
            this.label10.Text = "Remote";
            // 
            // RTI
            // 
            this.RTI.Location = new System.Drawing.Point(201, 25);
            this.RTI.Name = "RTI";
            this.RTI.ReadOnly = true;
            this.RTI.Size = new System.Drawing.Size(80, 20);
            this.RTI.TabIndex = 33;
            // 
            // ATI
            // 
            this.ATI.Location = new System.Drawing.Point(87, 25);
            this.ATI.Name = "ATI";
            this.ATI.ReadOnly = true;
            this.ATI.Size = new System.Drawing.Size(80, 20);
            this.ATI.TabIndex = 32;
            // 
            // RSSI
            // 
            this.RSSI.Location = new System.Drawing.Point(87, 51);
            this.RSSI.Multiline = true;
            this.RSSI.Name = "RSSI";
            this.RSSI.ReadOnly = true;
            this.RSSI.Size = new System.Drawing.Size(194, 58);
            this.RSSI.TabIndex = 34;
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(15, 32);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(42, 13);
            this.label11.TabIndex = 36;
            this.label11.Text = "Version";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(15, 58);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(32, 13);
            this.label12.TabIndex = 37;
            this.label12.Text = "RSSI";
            // 
            // BUT_savesettings
            // 
            this.BUT_savesettings.Location = new System.Drawing.Point(99, 330);
            this.BUT_savesettings.Name = "BUT_savesettings";
            this.BUT_savesettings.Size = new System.Drawing.Size(75, 39);
            this.BUT_savesettings.TabIndex = 21;
            this.BUT_savesettings.Text = "Save Settings";
            this.BUT_savesettings.UseVisualStyleBackColor = true;
            this.BUT_savesettings.Click += new System.EventHandler(this.BUT_savesettings_Click);
            // 
            // BUT_getcurrent
            // 
            this.BUT_getcurrent.Location = new System.Drawing.Point(18, 330);
            this.BUT_getcurrent.Name = "BUT_getcurrent";
            this.BUT_getcurrent.Size = new System.Drawing.Size(75, 39);
            this.BUT_getcurrent.TabIndex = 6;
            this.BUT_getcurrent.Text = "Load Settings";
            this.BUT_getcurrent.UseVisualStyleBackColor = true;
            this.BUT_getcurrent.Click += new System.EventHandler(this.BUT_getcurrent_Click);
            // 
            // lbl_status
            // 
            this.lbl_status.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.lbl_status.BackColor = System.Drawing.Color.Transparent;
            this.lbl_status.Location = new System.Drawing.Point(12, 374);
            this.lbl_status.Name = "lbl_status";
            this.lbl_status.Size = new System.Drawing.Size(294, 22);
            this.lbl_status.TabIndex = 3;
            // 
            // BUT_upload
            // 
            this.BUT_upload.Location = new System.Drawing.Point(180, 330);
            this.BUT_upload.Name = "BUT_upload";
            this.BUT_upload.Size = new System.Drawing.Size(127, 39);
            this.BUT_upload.TabIndex = 0;
            this.BUT_upload.Text = "Upload Firmware (Local)";
            this.BUT_upload.UseVisualStyleBackColor = true;
            this.BUT_upload.Click += new System.EventHandler(this.BUT_upload_Click);
            // 
            // BUT_syncS2
            // 
            this.BUT_syncS2.Location = new System.Drawing.Point(173, 168);
            this.BUT_syncS2.Name = "BUT_syncS2";
            this.BUT_syncS2.Size = new System.Drawing.Size(22, 23);
            this.BUT_syncS2.TabIndex = 38;
            this.BUT_syncS2.Text = ">";
            this.BUT_syncS2.UseVisualStyleBackColor = true;
            this.BUT_syncS2.Click += new System.EventHandler(this.BUT_syncS2_Click);
            // 
            // BUT_syncS3
            // 
            this.BUT_syncS3.Location = new System.Drawing.Point(173, 195);
            this.BUT_syncS3.Name = "BUT_syncS3";
            this.BUT_syncS3.Size = new System.Drawing.Size(22, 23);
            this.BUT_syncS3.TabIndex = 39;
            this.BUT_syncS3.Text = ">";
            this.BUT_syncS3.UseVisualStyleBackColor = true;
            this.BUT_syncS3.Click += new System.EventHandler(this.BUT_syncS3_Click);
            // 
            // BUT_syncS5
            // 
            this.BUT_syncS5.Location = new System.Drawing.Point(173, 247);
            this.BUT_syncS5.Name = "BUT_syncS5";
            this.BUT_syncS5.Size = new System.Drawing.Size(22, 23);
            this.BUT_syncS5.TabIndex = 40;
            this.BUT_syncS5.Text = ">";
            this.BUT_syncS5.UseVisualStyleBackColor = true;
            this.BUT_syncS5.Click += new System.EventHandler(this.BUT_syncS5_Click);
            // 
            // _3DRradio
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(318, 444);
            this.Controls.Add(this.BUT_syncS5);
            this.Controls.Add(this.BUT_syncS3);
            this.Controls.Add(this.BUT_syncS2);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.RSSI);
            this.Controls.Add(this.RTI);
            this.Controls.Add(this.ATI);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.RS7);
            this.Controls.Add(this.RS6);
            this.Controls.Add(this.RS5);
            this.Controls.Add(this.RS4);
            this.Controls.Add(this.RS3);
            this.Controls.Add(this.RS2);
            this.Controls.Add(this.RS0);
            this.Controls.Add(this.RS1);
            this.Controls.Add(this.BUT_savesettings);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.S7);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.S6);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.S5);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.S4);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.S3);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.S2);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.S0);
            this.Controls.Add(this.BUT_getcurrent);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.S1);
            this.Controls.Add(this.lbl_status);
            this.Controls.Add(this.Progressbar);
            this.Controls.Add(this.BUT_upload);
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.MinimumSize = new System.Drawing.Size(334, 482);
            this.Name = "_3DRradio";
            this.Text = "3DRradio";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private MyButton BUT_upload;
        private System.Windows.Forms.ProgressBar Progressbar;
        private System.Windows.Forms.Label lbl_status;
        private System.Windows.Forms.ComboBox S1;
        private System.Windows.Forms.Label label1;
        private MyButton BUT_getcurrent;
        private System.Windows.Forms.TextBox S0;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.ComboBox S2;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.ComboBox S3;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.ComboBox S4;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.CheckBox S5;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.CheckBox S6;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.CheckBox S7;
        private MyButton BUT_savesettings;
        private System.Windows.Forms.ToolTip toolTip1;
        private System.Windows.Forms.CheckBox RS7;
        private System.Windows.Forms.CheckBox RS6;
        private System.Windows.Forms.CheckBox RS5;
        private System.Windows.Forms.ComboBox RS4;
        private System.Windows.Forms.ComboBox RS3;
        private System.Windows.Forms.ComboBox RS2;
        private System.Windows.Forms.TextBox RS0;
        private System.Windows.Forms.ComboBox RS1;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox RTI;
        private System.Windows.Forms.TextBox ATI;
        private System.Windows.Forms.TextBox RSSI;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private MyButton BUT_syncS2;
        private MyButton BUT_syncS3;
        private MyButton BUT_syncS5;
    }
}
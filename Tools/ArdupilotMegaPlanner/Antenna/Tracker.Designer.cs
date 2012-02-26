namespace ArdupilotMega.Antenna
{
    partial class Tracker
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Tracker));
            this.CMB_interface = new System.Windows.Forms.ComboBox();
            this.label1 = new System.Windows.Forms.Label();
            this.CMB_baudrate = new System.Windows.Forms.ComboBox();
            this.CMB_serialport = new System.Windows.Forms.ComboBox();
            this.BUT_connect = new ArdupilotMega.MyButton();
            this.TRK_pantrim = new System.Windows.Forms.TrackBar();
            this.TXT_panstart = new System.Windows.Forms.TextBox();
            this.TXT_panstop = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.TXT_tiltstop = new System.Windows.Forms.TextBox();
            this.TXT_tiltstart = new System.Windows.Forms.TextBox();
            this.TRK_tilttrim = new System.Windows.Forms.TrackBar();
            ((System.ComponentModel.ISupportInitialize)(this.TRK_pantrim)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.TRK_tilttrim)).BeginInit();
            this.SuspendLayout();
            // 
            // CMB_interface
            // 
            this.CMB_interface.FormattingEnabled = true;
            this.CMB_interface.Items.AddRange(new object[] {
            "Maestro"});
            this.CMB_interface.Location = new System.Drawing.Point(83, 10);
            this.CMB_interface.Name = "CMB_interface";
            this.CMB_interface.Size = new System.Drawing.Size(121, 21);
            this.CMB_interface.TabIndex = 0;
            this.CMB_interface.Text = "Maestro";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(13, 13);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(49, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "Interface";
            // 
            // CMB_baudrate
            // 
            this.CMB_baudrate.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_baudrate.FormattingEnabled = true;
            this.CMB_baudrate.Items.AddRange(new object[] {
            "4800",
            "9600",
            "14400",
            "19200",
            "28800",
            "38400",
            "57600",
            "115200"});
            this.CMB_baudrate.Location = new System.Drawing.Point(337, 9);
            this.CMB_baudrate.Name = "CMB_baudrate";
            this.CMB_baudrate.Size = new System.Drawing.Size(121, 21);
            this.CMB_baudrate.TabIndex = 5;
            // 
            // CMB_serialport
            // 
            this.CMB_serialport.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_serialport.FormattingEnabled = true;
            this.CMB_serialport.Location = new System.Drawing.Point(210, 10);
            this.CMB_serialport.Name = "CMB_serialport";
            this.CMB_serialport.Size = new System.Drawing.Size(121, 21);
            this.CMB_serialport.TabIndex = 3;
            // 
            // BUT_connect
            // 
            this.BUT_connect.Location = new System.Drawing.Point(476, 9);
            this.BUT_connect.Name = "BUT_connect";
            this.BUT_connect.Size = new System.Drawing.Size(75, 23);
            this.BUT_connect.TabIndex = 4;
            this.BUT_connect.Text = "Connect";
            this.BUT_connect.UseVisualStyleBackColor = true;
            this.BUT_connect.Click += new System.EventHandler(this.BUT_connect_Click);
            // 
            // TRK_pantrim
            // 
            this.TRK_pantrim.Location = new System.Drawing.Point(83, 65);
            this.TRK_pantrim.Maximum = 180;
            this.TRK_pantrim.Minimum = -180;
            this.TRK_pantrim.Name = "TRK_pantrim";
            this.TRK_pantrim.Size = new System.Drawing.Size(375, 45);
            this.TRK_pantrim.TabIndex = 6;
            this.TRK_pantrim.Scroll += new System.EventHandler(this.TRK_pantrim_Scroll);
            // 
            // TXT_panstart
            // 
            this.TXT_panstart.Location = new System.Drawing.Point(13, 65);
            this.TXT_panstart.Name = "TXT_panstart";
            this.TXT_panstart.Size = new System.Drawing.Size(64, 20);
            this.TXT_panstart.TabIndex = 7;
            this.TXT_panstart.Text = "-90";
            // 
            // TXT_panstop
            // 
            this.TXT_panstop.Location = new System.Drawing.Point(464, 65);
            this.TXT_panstop.Name = "TXT_panstop";
            this.TXT_panstop.Size = new System.Drawing.Size(64, 20);
            this.TXT_panstop.TabIndex = 8;
            this.TXT_panstop.Text = "90";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(461, 49);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(32, 13);
            this.label2.TabIndex = 9;
            this.label2.Text = "Right";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(261, 49);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(38, 13);
            this.label3.TabIndex = 10;
            this.label3.Text = "Center";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(13, 49);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(25, 13);
            this.label4.TabIndex = 11;
            this.label4.Text = "Left";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(13, 125);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(35, 13);
            this.label5.TabIndex = 17;
            this.label5.Text = "Down";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(261, 125);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(38, 13);
            this.label6.TabIndex = 16;
            this.label6.Text = "Center";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(461, 125);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(21, 13);
            this.label7.TabIndex = 15;
            this.label7.Text = "Up";
            // 
            // TXT_tiltstop
            // 
            this.TXT_tiltstop.Location = new System.Drawing.Point(464, 141);
            this.TXT_tiltstop.Name = "TXT_tiltstop";
            this.TXT_tiltstop.Size = new System.Drawing.Size(64, 20);
            this.TXT_tiltstop.TabIndex = 14;
            this.TXT_tiltstop.Text = "90";
            // 
            // TXT_tiltstart
            // 
            this.TXT_tiltstart.Location = new System.Drawing.Point(13, 141);
            this.TXT_tiltstart.Name = "TXT_tiltstart";
            this.TXT_tiltstart.Size = new System.Drawing.Size(64, 20);
            this.TXT_tiltstart.TabIndex = 13;
            this.TXT_tiltstart.Text = "0";
            // 
            // TRK_tilttrim
            // 
            this.TRK_tilttrim.Location = new System.Drawing.Point(83, 141);
            this.TRK_tilttrim.Maximum = 90;
            this.TRK_tilttrim.Minimum = -90;
            this.TRK_tilttrim.Name = "TRK_tilttrim";
            this.TRK_tilttrim.Size = new System.Drawing.Size(375, 45);
            this.TRK_tilttrim.TabIndex = 12;
            this.TRK_tilttrim.Value = 45;
            this.TRK_tilttrim.Scroll += new System.EventHandler(this.TRK_tilttrim_Scroll);
            // 
            // Tracker
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(569, 195);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.TXT_tiltstop);
            this.Controls.Add(this.TXT_tiltstart);
            this.Controls.Add(this.TRK_tilttrim);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.TXT_panstop);
            this.Controls.Add(this.TXT_panstart);
            this.Controls.Add(this.TRK_pantrim);
            this.Controls.Add(this.CMB_baudrate);
            this.Controls.Add(this.BUT_connect);
            this.Controls.Add(this.CMB_serialport);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.CMB_interface);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "Tracker";
            this.Text = "Tracker";
            ((System.ComponentModel.ISupportInitialize)(this.TRK_pantrim)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.TRK_tilttrim)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox CMB_interface;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ComboBox CMB_baudrate;
        private MyButton BUT_connect;
        private System.Windows.Forms.ComboBox CMB_serialport;
        private System.Windows.Forms.TrackBar TRK_pantrim;
        private System.Windows.Forms.TextBox TXT_panstart;
        private System.Windows.Forms.TextBox TXT_panstop;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox TXT_tiltstop;
        private System.Windows.Forms.TextBox TXT_tiltstart;
        private System.Windows.Forms.TrackBar TRK_tilttrim;
    }
}
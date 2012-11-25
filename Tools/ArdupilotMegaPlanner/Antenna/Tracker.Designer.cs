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
            this.TRK_pantrim = new System.Windows.Forms.TrackBar();
            this.TXT_panrange = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.TXT_tiltrange = new System.Windows.Forms.TextBox();
            this.TRK_tilttrim = new System.Windows.Forms.TrackBar();
            this.label2 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.CHK_revpan = new System.Windows.Forms.CheckBox();
            this.CHK_revtilt = new System.Windows.Forms.CheckBox();
            this.TXT_pwmrangepan = new System.Windows.Forms.TextBox();
            this.TXT_pwmrangetilt = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.BUT_connect = new ArdupilotMega.Controls.MyButton();
            this.LBL_pantrim = new System.Windows.Forms.Label();
            this.LBL_tilttrim = new System.Windows.Forms.Label();
            this.BUT_find = new ArdupilotMega.Controls.MyButton();
            this.TXT_centerpan = new System.Windows.Forms.TextBox();
            this.TXT_centertilt = new System.Windows.Forms.TextBox();
            this.label13 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.TRK_pantrim)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.TRK_tilttrim)).BeginInit();
            this.SuspendLayout();
            // 
            // CMB_interface
            // 
            this.CMB_interface.FormattingEnabled = true;
            this.CMB_interface.Items.AddRange(new object[] {
            resources.GetString("CMB_interface.Items"),
            resources.GetString("CMB_interface.Items1")});
            resources.ApplyResources(this.CMB_interface, "CMB_interface");
            this.CMB_interface.Name = "CMB_interface";
            this.CMB_interface.SelectedIndexChanged += new System.EventHandler(this.CMB_interface_SelectedIndexChanged);
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            // 
            // CMB_baudrate
            // 
            this.CMB_baudrate.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_baudrate.FormattingEnabled = true;
            this.CMB_baudrate.Items.AddRange(new object[] {
            resources.GetString("CMB_baudrate.Items"),
            resources.GetString("CMB_baudrate.Items1"),
            resources.GetString("CMB_baudrate.Items2"),
            resources.GetString("CMB_baudrate.Items3"),
            resources.GetString("CMB_baudrate.Items4"),
            resources.GetString("CMB_baudrate.Items5"),
            resources.GetString("CMB_baudrate.Items6"),
            resources.GetString("CMB_baudrate.Items7")});
            resources.ApplyResources(this.CMB_baudrate, "CMB_baudrate");
            this.CMB_baudrate.Name = "CMB_baudrate";
            // 
            // CMB_serialport
            // 
            this.CMB_serialport.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_serialport.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_serialport, "CMB_serialport");
            this.CMB_serialport.Name = "CMB_serialport";
            // 
            // TRK_pantrim
            // 
            resources.ApplyResources(this.TRK_pantrim, "TRK_pantrim");
            this.TRK_pantrim.Maximum = 360;
            this.TRK_pantrim.Minimum = -360;
            this.TRK_pantrim.Name = "TRK_pantrim";
            this.TRK_pantrim.TickFrequency = 5;
            this.TRK_pantrim.Scroll += new System.EventHandler(this.TRK_pantrim_Scroll);
            // 
            // TXT_panrange
            // 
            resources.ApplyResources(this.TXT_panrange, "TXT_panrange");
            this.TXT_panrange.Name = "TXT_panrange";
            this.TXT_panrange.TextChanged += new System.EventHandler(this.TXT_panrange_TextChanged);
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
            // TXT_tiltrange
            // 
            resources.ApplyResources(this.TXT_tiltrange, "TXT_tiltrange");
            this.TXT_tiltrange.Name = "TXT_tiltrange";
            this.TXT_tiltrange.TextChanged += new System.EventHandler(this.TXT_tiltrange_TextChanged);
            // 
            // TRK_tilttrim
            // 
            resources.ApplyResources(this.TRK_tilttrim, "TRK_tilttrim");
            this.TRK_tilttrim.Maximum = 180;
            this.TRK_tilttrim.Minimum = -180;
            this.TRK_tilttrim.Name = "TRK_tilttrim";
            this.TRK_tilttrim.TickFrequency = 5;
            this.TRK_tilttrim.Scroll += new System.EventHandler(this.TRK_tilttrim_Scroll);
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // label7
            // 
            resources.ApplyResources(this.label7, "label7");
            this.label7.Name = "label7";
            // 
            // CHK_revpan
            // 
            resources.ApplyResources(this.CHK_revpan, "CHK_revpan");
            this.CHK_revpan.Name = "CHK_revpan";
            this.CHK_revpan.UseVisualStyleBackColor = true;
            this.CHK_revpan.CheckedChanged += new System.EventHandler(this.CHK_revpan_CheckedChanged);
            // 
            // CHK_revtilt
            // 
            resources.ApplyResources(this.CHK_revtilt, "CHK_revtilt");
            this.CHK_revtilt.Name = "CHK_revtilt";
            this.CHK_revtilt.UseVisualStyleBackColor = true;
            this.CHK_revtilt.CheckedChanged += new System.EventHandler(this.CHK_revtilt_CheckedChanged);
            // 
            // TXT_pwmrangepan
            // 
            resources.ApplyResources(this.TXT_pwmrangepan, "TXT_pwmrangepan");
            this.TXT_pwmrangepan.Name = "TXT_pwmrangepan";
            // 
            // TXT_pwmrangetilt
            // 
            resources.ApplyResources(this.TXT_pwmrangetilt, "TXT_pwmrangetilt");
            this.TXT_pwmrangetilt.Name = "TXT_pwmrangetilt";
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
            // label10
            // 
            resources.ApplyResources(this.label10, "label10");
            this.label10.Name = "label10";
            // 
            // label11
            // 
            resources.ApplyResources(this.label11, "label11");
            this.label11.Name = "label11";
            // 
            // label12
            // 
            resources.ApplyResources(this.label12, "label12");
            this.label12.Name = "label12";
            // 
            // BUT_connect
            // 
            resources.ApplyResources(this.BUT_connect, "BUT_connect");
            this.BUT_connect.Name = "BUT_connect";
            this.BUT_connect.UseVisualStyleBackColor = true;
            this.BUT_connect.Click += new System.EventHandler(this.BUT_connect_Click);
            // 
            // LBL_pantrim
            // 
            resources.ApplyResources(this.LBL_pantrim, "LBL_pantrim");
            this.LBL_pantrim.Name = "LBL_pantrim";
            // 
            // LBL_tilttrim
            // 
            resources.ApplyResources(this.LBL_tilttrim, "LBL_tilttrim");
            this.LBL_tilttrim.Name = "LBL_tilttrim";
            // 
            // BUT_find
            // 
            resources.ApplyResources(this.BUT_find, "BUT_find");
            this.BUT_find.Name = "BUT_find";
            this.BUT_find.UseVisualStyleBackColor = true;
            this.BUT_find.Click += new System.EventHandler(this.BUT_find_Click);
            // 
            // TXT_centerpan
            // 
            resources.ApplyResources(this.TXT_centerpan, "TXT_centerpan");
            this.TXT_centerpan.Name = "TXT_centerpan";
            this.TXT_centerpan.TextChanged += new System.EventHandler(this.TXT_centerpan_TextChanged);
            // 
            // TXT_centertilt
            // 
            resources.ApplyResources(this.TXT_centertilt, "TXT_centertilt");
            this.TXT_centertilt.Name = "TXT_centertilt";
            this.TXT_centertilt.TextChanged += new System.EventHandler(this.TXT_centertilt_TextChanged);
            // 
            // label13
            // 
            resources.ApplyResources(this.label13, "label13");
            this.label13.Name = "label13";
            // 
            // label14
            // 
            resources.ApplyResources(this.label14, "label14");
            this.label14.Name = "label14";
            // 
            // Tracker
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.label14);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.TXT_centertilt);
            this.Controls.Add(this.TXT_centerpan);
            this.Controls.Add(this.BUT_find);
            this.Controls.Add(this.LBL_tilttrim);
            this.Controls.Add(this.LBL_pantrim);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.TXT_pwmrangetilt);
            this.Controls.Add(this.TXT_pwmrangepan);
            this.Controls.Add(this.CHK_revtilt);
            this.Controls.Add(this.CHK_revpan);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.TXT_tiltrange);
            this.Controls.Add(this.TRK_tilttrim);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.TXT_panrange);
            this.Controls.Add(this.TRK_pantrim);
            this.Controls.Add(this.CMB_baudrate);
            this.Controls.Add(this.BUT_connect);
            this.Controls.Add(this.CMB_serialport);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.CMB_interface);
            this.Name = "Tracker";
            ((System.ComponentModel.ISupportInitialize)(this.TRK_pantrim)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.TRK_tilttrim)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox CMB_interface;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ComboBox CMB_baudrate;
        private ArdupilotMega.Controls.MyButton BUT_connect;
        private System.Windows.Forms.ComboBox CMB_serialport;
        private System.Windows.Forms.TrackBar TRK_pantrim;
        private System.Windows.Forms.TextBox TXT_panrange;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox TXT_tiltrange;
        private System.Windows.Forms.TrackBar TRK_tilttrim;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.CheckBox CHK_revpan;
        private System.Windows.Forms.CheckBox CHK_revtilt;
        private System.Windows.Forms.TextBox TXT_pwmrangepan;
        private System.Windows.Forms.TextBox TXT_pwmrangetilt;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label LBL_pantrim;
        private System.Windows.Forms.Label LBL_tilttrim;
        private Controls.MyButton BUT_find;
        private System.Windows.Forms.TextBox TXT_centerpan;
        private System.Windows.Forms.TextBox TXT_centertilt;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label14;
    }
}
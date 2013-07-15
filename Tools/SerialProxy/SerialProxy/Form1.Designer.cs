namespace SerialProxy
{
    partial class Form1
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
            this.CHK_reset = new System.Windows.Forms.ToolStripMenuItem();
            this.baudrate = new System.Windows.Forms.ToolStripComboBox();
            this.Comports = new System.Windows.Forms.ToolStripComboBox();
            this.ConnectComPort = new System.Windows.Forms.Button();
            this.tcpport = new System.Windows.Forms.ToolStripTextBox();
            this.outputlog = new System.Windows.Forms.RichTextBox();
            this.statusStrip1 = new System.Windows.Forms.StatusStrip();
            this.StatusCom = new System.Windows.Forms.ToolStripStatusLabel();
            this.StatusTCP = new System.Windows.Forms.ToolStripStatusLabel();
            this.menuStrip1 = new System.Windows.Forms.MenuStrip();
            this.optionsToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.toolStripMenuItem1 = new System.Windows.Forms.ToolStripMenuItem();
            this.toolStripMenuItem4 = new System.Windows.Forms.ToolStripMenuItem();
            this.toolStripMenuItem2 = new System.Windows.Forms.ToolStripMenuItem();
            this.toolStripMenuItem5 = new System.Windows.Forms.ToolStripMenuItem();
            this.convertCRLF = new System.Windows.Forms.ToolStripMenuItem();
            this.statusStrip1.SuspendLayout();
            this.menuStrip1.SuspendLayout();
            this.SuspendLayout();
            // 
            // CHK_reset
            // 
            this.CHK_reset.Checked = true;
            this.CHK_reset.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHK_reset.Name = "CHK_reset";
            this.CHK_reset.Size = new System.Drawing.Size(276, 22);
            this.CHK_reset.Text = "Reset Arduino on first new connection";
            this.CHK_reset.Click += new System.EventHandler(this.CHK_reset_Click);
            // 
            // baudrate
            // 
            this.baudrate.Items.AddRange(new object[] {
            "2400",
            "4800",
            "9600",
            "19200",
            "38400",
            "57600",
            "115200 "});
            this.baudrate.Name = "baudrate";
            this.baudrate.Size = new System.Drawing.Size(121, 23);
            this.baudrate.Text = "38400";
            // 
            // Comports
            // 
            this.Comports.Name = "Comports";
            this.Comports.Size = new System.Drawing.Size(121, 23);
            this.Comports.Text = "Com Port";
            this.Comports.SelectedIndexChanged += new System.EventHandler(this.Comports_SelectedIndexChanged);
            this.Comports.Click += new System.EventHandler(this.Comports_Click);
            // 
            // ConnectComPort
            // 
            this.ConnectComPort.Location = new System.Drawing.Point(12, 27);
            this.ConnectComPort.Name = "ConnectComPort";
            this.ConnectComPort.Size = new System.Drawing.Size(128, 23);
            this.ConnectComPort.TabIndex = 5;
            this.ConnectComPort.Text = "Connect";
            this.ConnectComPort.UseVisualStyleBackColor = true;
            this.ConnectComPort.Click += new System.EventHandler(this.ConnectComPort_Click);
            // 
            // tcpport
            // 
            this.tcpport.Name = "tcpport";
            this.tcpport.Size = new System.Drawing.Size(100, 23);
            this.tcpport.Text = "5001";
            // 
            // outputlog
            // 
            this.outputlog.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.outputlog.Location = new System.Drawing.Point(13, 56);
            this.outputlog.Name = "outputlog";
            this.outputlog.ReadOnly = true;
            this.outputlog.Size = new System.Drawing.Size(379, 171);
            this.outputlog.TabIndex = 31;
            this.outputlog.Text = "";
            this.outputlog.TextChanged += new System.EventHandler(this.outputlog_TextChanged);
            // 
            // statusStrip1
            // 
            this.statusStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.StatusCom,
            this.StatusTCP});
            this.statusStrip1.Location = new System.Drawing.Point(0, 230);
            this.statusStrip1.Name = "statusStrip1";
            this.statusStrip1.Size = new System.Drawing.Size(405, 22);
            this.statusStrip1.TabIndex = 32;
            this.statusStrip1.Text = "statusStrip1";
            // 
            // StatusCom
            // 
            this.StatusCom.Name = "StatusCom";
            this.StatusCom.Size = new System.Drawing.Size(108, 17);
            this.StatusCom.Text = "Com Disconnected";
            // 
            // StatusTCP
            // 
            this.StatusTCP.Name = "StatusTCP";
            this.StatusTCP.Size = new System.Drawing.Size(104, 17);
            this.StatusTCP.Text = "TCP Disconnected";
            // 
            // menuStrip1
            // 
            this.menuStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.optionsToolStripMenuItem});
            this.menuStrip1.Location = new System.Drawing.Point(0, 0);
            this.menuStrip1.Name = "menuStrip1";
            this.menuStrip1.Size = new System.Drawing.Size(405, 24);
            this.menuStrip1.TabIndex = 33;
            this.menuStrip1.Text = "menuStrip1";
            // 
            // optionsToolStripMenuItem
            // 
            this.optionsToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolStripMenuItem1,
            this.toolStripMenuItem2});
            this.optionsToolStripMenuItem.Name = "optionsToolStripMenuItem";
            this.optionsToolStripMenuItem.Size = new System.Drawing.Size(61, 20);
            this.optionsToolStripMenuItem.Text = "Options";
            // 
            // toolStripMenuItem1
            // 
            this.toolStripMenuItem1.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.Comports,
            this.toolStripMenuItem4,
            this.baudrate,
            this.CHK_reset,
            this.convertCRLF});
            this.toolStripMenuItem1.Name = "toolStripMenuItem1";
            this.toolStripMenuItem1.Size = new System.Drawing.Size(152, 22);
            this.toolStripMenuItem1.Text = "Serial Options";
            // 
            // toolStripMenuItem4
            // 
            this.toolStripMenuItem4.Name = "toolStripMenuItem4";
            this.toolStripMenuItem4.Size = new System.Drawing.Size(253, 22);
            this.toolStripMenuItem4.Text = "Baud Rate";
            // 
            // toolStripMenuItem2
            // 
            this.toolStripMenuItem2.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolStripMenuItem5,
            this.tcpport});
            this.toolStripMenuItem2.Name = "toolStripMenuItem2";
            this.toolStripMenuItem2.Size = new System.Drawing.Size(152, 22);
            this.toolStripMenuItem2.Text = "TCP Options";
            // 
            // toolStripMenuItem5
            // 
            this.toolStripMenuItem5.Name = "toolStripMenuItem5";
            this.toolStripMenuItem5.Size = new System.Drawing.Size(160, 22);
            this.toolStripMenuItem5.Text = "TCP Port";
            // 
            // convertCRLF
            // 
            this.convertCRLF.Name = "convertCRLF";
            this.convertCRLF.Size = new System.Drawing.Size(253, 22);
            this.convertCRLF.Text = "Convert CR LF to nil";
            this.convertCRLF.Click += new System.EventHandler(this.toolStripMenuItem3_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(405, 252);
            this.Controls.Add(this.statusStrip1);
            this.Controls.Add(this.menuStrip1);
            this.Controls.Add(this.ConnectComPort);
            this.Controls.Add(this.outputlog);
            this.MainMenuStrip = this.menuStrip1;
            this.Name = "Form1";
            this.Text = "SerialProxy by Michael Oborne";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            this.statusStrip1.ResumeLayout(false);
            this.statusStrip1.PerformLayout();
            this.menuStrip1.ResumeLayout(false);
            this.menuStrip1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button ConnectComPort;
        private System.Windows.Forms.ToolStripTextBox tcpport;
        private System.Windows.Forms.RichTextBox outputlog;
        private System.Windows.Forms.StatusStrip statusStrip1;
        private System.Windows.Forms.ToolStripStatusLabel StatusCom;
        private System.Windows.Forms.ToolStripStatusLabel StatusTCP;
        private System.Windows.Forms.MenuStrip menuStrip1;
        private System.Windows.Forms.ToolStripMenuItem optionsToolStripMenuItem;
        private System.Windows.Forms.ToolStripComboBox Comports;
        private System.Windows.Forms.ToolStripComboBox baudrate;
        private System.Windows.Forms.ToolStripMenuItem CHK_reset;
        private System.Windows.Forms.ToolStripMenuItem toolStripMenuItem1;
        private System.Windows.Forms.ToolStripMenuItem toolStripMenuItem2;
        private System.Windows.Forms.ToolStripMenuItem toolStripMenuItem4;
        private System.Windows.Forms.ToolStripMenuItem toolStripMenuItem5;
        private System.Windows.Forms.ToolStripMenuItem convertCRLF;
    }
}


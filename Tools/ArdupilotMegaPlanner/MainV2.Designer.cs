namespace ArdupilotMega
{
    partial class MainV2
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainV2));
            this.MenuFlightData = new System.Windows.Forms.ToolStripButton();
            this.MenuFlightPlanner = new System.Windows.Forms.ToolStripButton();
            this.MenuConfiguration = new System.Windows.Forms.ToolStripButton();
            this.MenuSimulation = new System.Windows.Forms.ToolStripButton();
            this.MenuFirmware = new System.Windows.Forms.ToolStripButton();
            this.MenuConnect = new System.Windows.Forms.ToolStripButton();
            this.CMB_serialport = new System.Windows.Forms.ToolStripComboBox();
            this.MainMenu = new System.Windows.Forms.MenuStrip();
            this.MenuTerminal = new System.Windows.Forms.ToolStripButton();
            this.CMB_baudrate = new System.Windows.Forms.ToolStripComboBox();
            this.TOOL_APMFirmware = new System.Windows.Forms.ToolStripComboBox();
            this.MenuHelp = new System.Windows.Forms.ToolStripButton();
            this.toolStripMenuItem1 = new System.Windows.Forms.ToolStripMenuItem();
            this.MyView = new System.Windows.Forms.Panel();
            this.MainMenu.SuspendLayout();
            this.SuspendLayout();
            // 
            // MenuFlightData
            // 
            this.MenuFlightData.AutoSize = false;
            this.MenuFlightData.BackgroundImage = global::ArdupilotMega.Properties.Resources.data;
            this.MenuFlightData.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Image;
            this.MenuFlightData.ImageTransparentColor = System.Drawing.Color.Magenta;
            this.MenuFlightData.Margin = new System.Windows.Forms.Padding(0);
            this.MenuFlightData.Name = "MenuFlightData";
            this.MenuFlightData.Padding = new System.Windows.Forms.Padding(0, 0, 72, 72);
            this.MenuFlightData.Size = new System.Drawing.Size(76, 76);
            this.MenuFlightData.Click += new System.EventHandler(this.MenuFlightData_Click);
            // 
            // MenuFlightPlanner
            // 
            this.MenuFlightPlanner.AutoSize = false;
            this.MenuFlightPlanner.BackgroundImage = global::ArdupilotMega.Properties.Resources.planner;
            this.MenuFlightPlanner.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Image;
            this.MenuFlightPlanner.ImageTransparentColor = System.Drawing.Color.Magenta;
            this.MenuFlightPlanner.Margin = new System.Windows.Forms.Padding(0);
            this.MenuFlightPlanner.Name = "MenuFlightPlanner";
            this.MenuFlightPlanner.Padding = new System.Windows.Forms.Padding(0, 0, 72, 72);
            this.MenuFlightPlanner.Size = new System.Drawing.Size(76, 76);
            this.MenuFlightPlanner.TextImageRelation = System.Windows.Forms.TextImageRelation.ImageAboveText;
            this.MenuFlightPlanner.ToolTipText = "Flight Planner";
            this.MenuFlightPlanner.Click += new System.EventHandler(this.MenuFlightPlanner_Click);
            // 
            // MenuConfiguration
            // 
            this.MenuConfiguration.AutoSize = false;
            this.MenuConfiguration.BackgroundImage = global::ArdupilotMega.Properties.Resources.configuration;
            this.MenuConfiguration.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Image;
            this.MenuConfiguration.ImageTransparentColor = System.Drawing.Color.Magenta;
            this.MenuConfiguration.Margin = new System.Windows.Forms.Padding(0);
            this.MenuConfiguration.Name = "MenuConfiguration";
            this.MenuConfiguration.Padding = new System.Windows.Forms.Padding(0, 0, 72, 72);
            this.MenuConfiguration.Size = new System.Drawing.Size(76, 76);
            this.MenuConfiguration.TextImageRelation = System.Windows.Forms.TextImageRelation.ImageAboveText;
            this.MenuConfiguration.ToolTipText = "Configuration";
            this.MenuConfiguration.Click += new System.EventHandler(this.MenuConfiguration_Click);
            // 
            // MenuSimulation
            // 
            this.MenuSimulation.AutoSize = false;
            this.MenuSimulation.BackgroundImage = global::ArdupilotMega.Properties.Resources.simulation;
            this.MenuSimulation.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Image;
            this.MenuSimulation.ImageTransparentColor = System.Drawing.Color.Magenta;
            this.MenuSimulation.Margin = new System.Windows.Forms.Padding(0);
            this.MenuSimulation.Name = "MenuSimulation";
            this.MenuSimulation.Padding = new System.Windows.Forms.Padding(0, 0, 72, 72);
            this.MenuSimulation.Size = new System.Drawing.Size(76, 76);
            this.MenuSimulation.TextImageRelation = System.Windows.Forms.TextImageRelation.ImageAboveText;
            this.MenuSimulation.ToolTipText = "Simulation";
            this.MenuSimulation.Click += new System.EventHandler(this.MenuSimulation_Click);
            // 
            // MenuFirmware
            // 
            this.MenuFirmware.AutoSize = false;
            this.MenuFirmware.BackgroundImage = global::ArdupilotMega.Properties.Resources.firmware;
            this.MenuFirmware.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Image;
            this.MenuFirmware.ImageTransparentColor = System.Drawing.Color.Magenta;
            this.MenuFirmware.Margin = new System.Windows.Forms.Padding(0);
            this.MenuFirmware.Name = "MenuFirmware";
            this.MenuFirmware.Padding = new System.Windows.Forms.Padding(0, 0, 72, 72);
            this.MenuFirmware.Size = new System.Drawing.Size(76, 76);
            this.MenuFirmware.TextImageRelation = System.Windows.Forms.TextImageRelation.ImageAboveText;
            this.MenuFirmware.ToolTipText = "Firmware";
            this.MenuFirmware.Click += new System.EventHandler(this.MenuFirmware_Click);
            // 
            // MenuConnect
            // 
            this.MenuConnect.Alignment = System.Windows.Forms.ToolStripItemAlignment.Right;
            this.MenuConnect.AutoSize = false;
            this.MenuConnect.BackgroundImage = global::ArdupilotMega.Properties.Resources.connect;
            this.MenuConnect.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Image;
            this.MenuConnect.ImageTransparentColor = System.Drawing.Color.Magenta;
            this.MenuConnect.Margin = new System.Windows.Forms.Padding(0);
            this.MenuConnect.Name = "MenuConnect";
            this.MenuConnect.Padding = new System.Windows.Forms.Padding(0, 0, 72, 72);
            this.MenuConnect.Size = new System.Drawing.Size(76, 76);
            this.MenuConnect.Click += new System.EventHandler(this.MenuConnect_Click);
            // 
            // CMB_serialport
            // 
            this.CMB_serialport.Alignment = System.Windows.Forms.ToolStripItemAlignment.Right;
            this.CMB_serialport.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_serialport.Name = "CMB_serialport";
            this.CMB_serialport.Size = new System.Drawing.Size(150, 76);
            this.CMB_serialport.SelectedIndexChanged += new System.EventHandler(this.CMB_serialport_SelectedIndexChanged);
            this.CMB_serialport.Enter += new System.EventHandler(this.CMB_serialport_Enter);
            this.CMB_serialport.Click += new System.EventHandler(this.CMB_serialport_Click);
            // 
            // MainMenu
            // 
            this.MainMenu.AutoSize = false;
            this.MainMenu.BackColor = System.Drawing.SystemColors.Control;
            this.MainMenu.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("MainMenu.BackgroundImage")));
            this.MainMenu.GripMargin = new System.Windows.Forms.Padding(0);
            this.MainMenu.ImageScalingSize = new System.Drawing.Size(76, 76);
            this.MainMenu.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.MenuFlightData,
            this.MenuFlightPlanner,
            this.MenuConfiguration,
            this.MenuSimulation,
            this.MenuFirmware,
            this.MenuTerminal,
            this.MenuConnect,
            this.CMB_baudrate,
            this.CMB_serialport,
            this.TOOL_APMFirmware,
            this.MenuHelp});
            this.MainMenu.LayoutStyle = System.Windows.Forms.ToolStripLayoutStyle.HorizontalStackWithOverflow;
            this.MainMenu.Location = new System.Drawing.Point(0, 0);
            this.MainMenu.Name = "MainMenu";
            this.MainMenu.Padding = new System.Windows.Forms.Padding(0);
            this.MainMenu.RenderMode = System.Windows.Forms.ToolStripRenderMode.Professional;
            this.MainMenu.Size = new System.Drawing.Size(1008, 76);
            this.MainMenu.TabIndex = 1;
            this.MainMenu.Text = "menuStrip1";
            // 
            // MenuTerminal
            // 
            this.MenuTerminal.AutoSize = false;
            this.MenuTerminal.BackgroundImage = global::ArdupilotMega.Properties.Resources.terminal;
            this.MenuTerminal.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Image;
            this.MenuTerminal.ImageTransparentColor = System.Drawing.Color.Magenta;
            this.MenuTerminal.Margin = new System.Windows.Forms.Padding(0);
            this.MenuTerminal.Name = "MenuTerminal";
            this.MenuTerminal.Padding = new System.Windows.Forms.Padding(0, 0, 72, 72);
            this.MenuTerminal.Size = new System.Drawing.Size(76, 76);
            this.MenuTerminal.TextImageRelation = System.Windows.Forms.TextImageRelation.ImageAboveText;
            this.MenuTerminal.ToolTipText = "Terminal";
            this.MenuTerminal.Click += new System.EventHandler(this.MenuTerminal_Click);
            // 
            // CMB_baudrate
            // 
            this.CMB_baudrate.Alignment = System.Windows.Forms.ToolStripItemAlignment.Right;
            this.CMB_baudrate.Items.AddRange(new object[] {
            "4800",
            "9600",
            "14400",
            "19200",
            "28800",
            "38400",
            "57600",
            "115200"});
            this.CMB_baudrate.Name = "CMB_baudrate";
            this.CMB_baudrate.Size = new System.Drawing.Size(76, 76);
            this.CMB_baudrate.SelectedIndexChanged += new System.EventHandler(this.CMB_baudrate_SelectedIndexChanged);
            this.CMB_baudrate.TextChanged += new System.EventHandler(this.CMB_baudrate_TextChanged);
            // 
            // TOOL_APMFirmware
            // 
            this.TOOL_APMFirmware.Alignment = System.Windows.Forms.ToolStripItemAlignment.Right;
            this.TOOL_APMFirmware.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.TOOL_APMFirmware.MaxDropDownItems = 2;
            this.TOOL_APMFirmware.Name = "TOOL_APMFirmware";
            this.TOOL_APMFirmware.Size = new System.Drawing.Size(121, 76);
            this.TOOL_APMFirmware.SelectedIndexChanged += new System.EventHandler(this.TOOL_APMFirmware_SelectedIndexChanged);
            // 
            // MenuHelp
            // 
            this.MenuHelp.AutoSize = false;
            this.MenuHelp.BackgroundImage = global::ArdupilotMega.Properties.Resources.help;
            this.MenuHelp.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Image;
            this.MenuHelp.ImageTransparentColor = System.Drawing.Color.Magenta;
            this.MenuHelp.Margin = new System.Windows.Forms.Padding(0);
            this.MenuHelp.Name = "MenuHelp";
            this.MenuHelp.Padding = new System.Windows.Forms.Padding(0, 0, 72, 72);
            this.MenuHelp.Size = new System.Drawing.Size(76, 76);
            this.MenuHelp.TextImageRelation = System.Windows.Forms.TextImageRelation.ImageAboveText;
            this.MenuHelp.ToolTipText = "Terminal";
            this.MenuHelp.Click += new System.EventHandler(this.MenuHelp_Click);
            // 
            // toolStripMenuItem1
            // 
            this.toolStripMenuItem1.Name = "toolStripMenuItem1";
            this.toolStripMenuItem1.Size = new System.Drawing.Size(141, 20);
            this.toolStripMenuItem1.Text = "toolStripMenuItem1";
            // 
            // MyView
            // 
            this.MyView.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(38)))), ((int)(((byte)(39)))), ((int)(((byte)(40)))));
            this.MyView.Dock = System.Windows.Forms.DockStyle.Fill;
            this.MyView.ForeColor = System.Drawing.Color.White;
            this.MyView.Location = new System.Drawing.Point(0, 76);
            this.MyView.Margin = new System.Windows.Forms.Padding(0);
            this.MyView.Name = "MyView";
            this.MyView.Size = new System.Drawing.Size(1008, 461);
            this.MyView.TabIndex = 3;
            // 
            // MainV2
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1008, 537);
            this.Controls.Add(this.MyView);
            this.Controls.Add(this.MainMenu);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.KeyPreview = true;
            this.MainMenuStrip = this.MainMenu;
            this.MinimumSize = new System.Drawing.Size(1024, 575);
            this.Name = "MainV2";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "APM Planner - By Michael Oborne";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.MainV2_FormClosing);
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.MainV2_FormClosed);
            this.Load += new System.EventHandler(this.MainV2_Load);
            this.Resize += new System.EventHandler(this.MainV2_Resize);
            this.MainMenu.ResumeLayout(false);
            this.MainMenu.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.ToolStripButton MenuFlightData;
        private System.Windows.Forms.ToolStripButton MenuFlightPlanner;
        private System.Windows.Forms.ToolStripButton MenuConfiguration;
        private System.Windows.Forms.ToolStripButton MenuSimulation;
        private System.Windows.Forms.ToolStripButton MenuFirmware;
        private System.Windows.Forms.ToolStripComboBox CMB_serialport;
        private System.Windows.Forms.ToolStripButton MenuConnect;
        private System.Windows.Forms.MenuStrip MainMenu;
        private System.Windows.Forms.ToolStripMenuItem toolStripMenuItem1;
        private System.Windows.Forms.ToolStripComboBox CMB_baudrate;
        private System.Windows.Forms.Panel MyView;
        private System.Windows.Forms.ToolStripButton MenuTerminal;
        private System.Windows.Forms.ToolStripComboBox TOOL_APMFirmware;
        private System.Windows.Forms.ToolStripButton MenuHelp;
        //public static WebCam_Capture.WebCamCapture webCamCapture1;

    }
}
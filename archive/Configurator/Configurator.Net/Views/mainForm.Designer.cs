using System;
namespace ArducopterConfigurator
{
    partial class mainForm
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(mainForm));
            this.tabCtrlMonitorVms = new System.Windows.Forms.TabControl();
            this.mainVmBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.availablePortsBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.toolTip = new System.Windows.Forms.ToolTip(this.components);
            this.button3 = new System.Windows.Forms.Button();
            this.button4 = new System.Windows.Forms.Button();
            this.btnConnect = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.comboBox1 = new System.Windows.Forms.ComboBox();
            this.linkLabel1 = new System.Windows.Forms.LinkLabel();
            this.label1 = new System.Windows.Forms.Label();
            this.cmboComPorts = new System.Windows.Forms.ComboBox();
            this.lblConnectionStatus = new System.Windows.Forms.Label();
            this.availableBaudRatesBindingSource = new System.Windows.Forms.BindingSource(this.components);
            ((System.ComponentModel.ISupportInitialize)(this.mainVmBindingSource)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.availablePortsBindingSource)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.availableBaudRatesBindingSource)).BeginInit();
            this.SuspendLayout();
            // 
            // tabCtrlMonitorVms
            // 
            this.tabCtrlMonitorVms.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.tabCtrlMonitorVms.DataBindings.Add(new System.Windows.Forms.Binding("Enabled", this.mainVmBindingSource, "IsConnected", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.tabCtrlMonitorVms.HotTrack = true;
            this.tabCtrlMonitorVms.Location = new System.Drawing.Point(12, 10);
            this.tabCtrlMonitorVms.Name = "tabCtrlMonitorVms";
            this.tabCtrlMonitorVms.SelectedIndex = 0;
            this.tabCtrlMonitorVms.Size = new System.Drawing.Size(530, 383);
            this.tabCtrlMonitorVms.TabIndex = 3;
            this.tabCtrlMonitorVms.Selected += new System.Windows.Forms.TabControlEventHandler(this.tabCtrlConfigs_Selected);
            // 
            // mainVmBindingSource
            // 
            this.mainVmBindingSource.DataSource = typeof(ArducopterConfigurator.PresentationModels.MainVm);
            // 
            // button3
            // 
            this.button3.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.button3.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.mainVmBindingSource, "RestoreDefaultsCommand", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.button3.Image = ((System.Drawing.Image)(resources.GetObject("button3.Image")));
            this.button3.Location = new System.Drawing.Point(502, 399);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(40, 34);
            this.button3.TabIndex = 10;
            this.toolTip.SetToolTip(this.button3, "Restore Defaults");
            this.button3.UseVisualStyleBackColor = true;
            // 
            // button4
            // 
            this.button4.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.button4.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.mainVmBindingSource, "WriteToEepromCommand", true));
            this.button4.Image = ((System.Drawing.Image)(resources.GetObject("button4.Image")));
            this.button4.Location = new System.Drawing.Point(456, 399);
            this.button4.Name = "button4";
            this.button4.Size = new System.Drawing.Size(40, 34);
            this.button4.TabIndex = 11;
            this.toolTip.SetToolTip(this.button4, "Save to Eeprom");
            this.button4.UseVisualStyleBackColor = true;
            // 
            // btnConnect
            // 
            this.btnConnect.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.btnConnect.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.btnConnect.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.mainVmBindingSource, "ConnectCommand", true));
            this.btnConnect.Image = ((System.Drawing.Image)(resources.GetObject("btnConnect.Image")));
            this.btnConnect.Location = new System.Drawing.Point(165, 408);
            this.btnConnect.Name = "btnConnect";
            this.btnConnect.Size = new System.Drawing.Size(26, 26);
            this.btnConnect.TabIndex = 6;
            this.toolTip.SetToolTip(this.btnConnect, "Connect");
            this.btnConnect.UseVisualStyleBackColor = true;
            // 
            // button1
            // 
            this.button1.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.button1.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.button1.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.mainVmBindingSource, "DisconnectCommand", true));
            this.button1.Image = ((System.Drawing.Image)(resources.GetObject("button1.Image")));
            this.button1.Location = new System.Drawing.Point(196, 408);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(26, 26);
            this.button1.TabIndex = 7;
            this.toolTip.SetToolTip(this.button1, "Disconnect");
            this.button1.UseVisualStyleBackColor = true;
            // 
            // comboBox1
            // 
            this.comboBox1.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.comboBox1.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.mainVmBindingSource, "SelectedBaudRate", true));
            this.comboBox1.DataSource = this.availableBaudRatesBindingSource;
            this.comboBox1.FormattingEnabled = true;
            this.comboBox1.Location = new System.Drawing.Point(80, 411);
            this.comboBox1.Name = "comboBox1";
            this.comboBox1.Size = new System.Drawing.Size(79, 21);
            this.comboBox1.TabIndex = 15;
            this.toolTip.SetToolTip(this.comboBox1, "Baud Rate");
            // 
            // linkLabel1
            // 
            this.linkLabel1.AutoSize = true;
            this.linkLabel1.Location = new System.Drawing.Point(355, 423);
            this.linkLabel1.Name = "linkLabel1";
            this.linkLabel1.Size = new System.Drawing.Size(84, 13);
            this.linkLabel1.TabIndex = 13;
            this.linkLabel1.TabStop = true;
            this.linkLabel1.Text = "Andrew Radford";
            this.linkLabel1.LinkClicked += new System.Windows.Forms.LinkLabelLinkClickedEventHandler(this.linkLabel1_LinkClicked);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(355, 407);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(89, 13);
            this.label1.TabIndex = 14;
            this.label1.Text = "Use with caution!";
            // 
            // cmboComPorts
            // 
            this.cmboComPorts.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.cmboComPorts.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.mainVmBindingSource, "SelectedPort", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.cmboComPorts.DataSource = this.availablePortsBindingSource;
            this.cmboComPorts.FormattingEnabled = true;
            this.cmboComPorts.Location = new System.Drawing.Point(11, 410);
            this.cmboComPorts.Name = "cmboComPorts";
            this.cmboComPorts.Size = new System.Drawing.Size(63, 21);
            this.cmboComPorts.TabIndex = 5;
            this.cmboComPorts.DropDown += new System.EventHandler(this.cmboComPorts_DropDown);
            // 
            // lblConnectionStatus
            // 
            this.lblConnectionStatus.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.lblConnectionStatus.AutoSize = true;
            this.lblConnectionStatus.Location = new System.Drawing.Point(228, 419);
            this.lblConnectionStatus.Name = "lblConnectionStatus";
            this.lblConnectionStatus.Size = new System.Drawing.Size(112, 13);
            this.lblConnectionStatus.TabIndex = 8;
            this.lblConnectionStatus.Text = "connection string here";
            // 
            // availableBaudRatesBindingSource
            // 
            this.availableBaudRatesBindingSource.DataMember = "AvailableBaudRates";
            this.availableBaudRatesBindingSource.DataSource = this.mainVmBindingSource;
            // 
            // mainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(554, 445);
            this.Controls.Add(this.comboBox1);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.lblConnectionStatus);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.linkLabel1);
            this.Controls.Add(this.cmboComPorts);
            this.Controls.Add(this.button4);
            this.Controls.Add(this.btnConnect);
            this.Controls.Add(this.button3);
            this.Controls.Add(this.tabCtrlMonitorVms);
            this.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.mainVmBindingSource, "Name", true));
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.MinimumSize = new System.Drawing.Size(550, 400);
            this.Name = "mainForm";
            this.Load += new System.EventHandler(this.MainFormLoaded);
            this.SizeChanged += new System.EventHandler(this.mainForm_SizeChanged);
            ((System.ComponentModel.ISupportInitialize)(this.mainVmBindingSource)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.availablePortsBindingSource)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.availableBaudRatesBindingSource)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TabControl tabCtrlMonitorVms;
        private System.Windows.Forms.BindingSource mainVmBindingSource;
        private System.Windows.Forms.BindingSource availablePortsBindingSource;
        private System.Windows.Forms.ToolTip toolTip;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.Button button4;
        private System.Windows.Forms.LinkLabel linkLabel1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button btnConnect;
        private System.Windows.Forms.ComboBox cmboComPorts;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Label lblConnectionStatus;
        private System.Windows.Forms.ComboBox comboBox1;
        private System.Windows.Forms.BindingSource availableBaudRatesBindingSource;
    }
}


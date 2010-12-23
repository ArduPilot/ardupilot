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
            this.cmboComPorts = new System.Windows.Forms.ComboBox();
            this.availablePortsBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.btnConnect = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.lblConnectionStatus = new System.Windows.Forms.Label();
            this.button2 = new System.Windows.Forms.Button();
            this.toolTip = new System.Windows.Forms.ToolTip(this.components);
            ((System.ComponentModel.ISupportInitialize)(this.mainVmBindingSource)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.availablePortsBindingSource)).BeginInit();
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
            this.tabCtrlMonitorVms.Size = new System.Drawing.Size(530, 396);
            this.tabCtrlMonitorVms.TabIndex = 3;
            this.tabCtrlMonitorVms.Selected += new System.Windows.Forms.TabControlEventHandler(this.tabCtrlConfigs_Selected);
            // 
            // mainVmBindingSource
            // 
            this.mainVmBindingSource.DataSource = typeof(ArducopterConfigurator.PresentationModels.MainVm);
            // 
            // cmboComPorts
            // 
            this.cmboComPorts.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.cmboComPorts.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.mainVmBindingSource, "SelectedPort", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.cmboComPorts.DataSource = this.availablePortsBindingSource;
            this.cmboComPorts.FormattingEnabled = true;
            this.cmboComPorts.Location = new System.Drawing.Point(12, 412);
            this.cmboComPorts.Name = "cmboComPorts";
            this.cmboComPorts.Size = new System.Drawing.Size(79, 21);
            this.cmboComPorts.TabIndex = 5;
            // 
            // btnConnect
            // 
            this.btnConnect.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.btnConnect.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.btnConnect.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.mainVmBindingSource, "ConnectCommand", true));
            this.btnConnect.Image = ((System.Drawing.Image)(resources.GetObject("btnConnect.Image")));
            this.btnConnect.Location = new System.Drawing.Point(126, 411);
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
            this.button1.Location = new System.Drawing.Point(157, 411);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(26, 26);
            this.button1.TabIndex = 7;
            this.toolTip.SetToolTip(this.button1, "Disconnect");
            this.button1.UseVisualStyleBackColor = true;
            // 
            // lblConnectionStatus
            // 
            this.lblConnectionStatus.AutoSize = true;
            this.lblConnectionStatus.Location = new System.Drawing.Point(307, 417);
            this.lblConnectionStatus.Name = "lblConnectionStatus";
            this.lblConnectionStatus.Size = new System.Drawing.Size(112, 13);
            this.lblConnectionStatus.TabIndex = 8;
            this.lblConnectionStatus.Text = "connection string here";
            // 
            // button2
            // 
            this.button2.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.button2.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.button2.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.mainVmBindingSource, "RefreshPortListCommand", true));
            this.button2.Image = ((System.Drawing.Image)(resources.GetObject("button2.Image")));
            this.button2.Location = new System.Drawing.Point(97, 410);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(26, 26);
            this.button2.TabIndex = 9;
            this.toolTip.SetToolTip(this.button2, "Refresh port list");
            this.button2.UseVisualStyleBackColor = true;
            // 
            // mainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(554, 445);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.lblConnectionStatus);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.btnConnect);
            this.Controls.Add(this.cmboComPorts);
            this.Controls.Add(this.tabCtrlMonitorVms);
            this.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.mainVmBindingSource, "Name", true));
            this.Name = "mainForm";
            this.Load += new System.EventHandler(this.MainFormLoaded);
            this.SizeChanged += new System.EventHandler(this.mainForm_SizeChanged);
            ((System.ComponentModel.ISupportInitialize)(this.mainVmBindingSource)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.availablePortsBindingSource)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TabControl tabCtrlMonitorVms;
        private System.Windows.Forms.BindingSource mainVmBindingSource;
        private System.Windows.Forms.ComboBox cmboComPorts;
        private System.Windows.Forms.Button btnConnect;
        private System.Windows.Forms.BindingSource availablePortsBindingSource;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Label lblConnectionStatus;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.ToolTip toolTip;
    }
}


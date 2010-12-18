using ArducopterConfigurator.PresentationModels;

namespace ArducopterConfigurator
{
    partial class SerialMonitorView
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
            this.txtSend = new System.Windows.Forms.TextBox();
            this.txtBoxRx = new System.Windows.Forms.TextBox();
            this.serialMonitorVmBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.txtBoxTx = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.serialMonitorVmBindingSource)).BeginInit();
            this.SuspendLayout();
            // 
            // txtSend
            // 
            this.txtSend.Location = new System.Drawing.Point(-167, 167);
            this.txtSend.Name = "txtSend";
            this.txtSend.Size = new System.Drawing.Size(10, 20);
            this.txtSend.TabIndex = 4;
            // 
            // txtBoxRx
            // 
            this.txtBoxRx.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.txtBoxRx.CausesValidation = false;
            this.txtBoxRx.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.serialMonitorVmBindingSource, "ReceviedText", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.txtBoxRx.Location = new System.Drawing.Point(3, 3);
            this.txtBoxRx.Multiline = true;
            this.txtBoxRx.Name = "txtBoxRx";
            this.txtBoxRx.ReadOnly = true;
            this.txtBoxRx.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.txtBoxRx.Size = new System.Drawing.Size(469, 356);
            this.txtBoxRx.TabIndex = 3;
            // 
            // serialMonitorVmBindingSource
            // 
            this.serialMonitorVmBindingSource.DataSource = typeof(ArducopterConfigurator.PresentationModels.SerialMonitorVm);
            // 
            // txtBoxTx
            // 
            this.txtBoxTx.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.txtBoxTx.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.serialMonitorVmBindingSource, "SendText", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.txtBoxTx.Location = new System.Drawing.Point(4, 365);
            this.txtBoxTx.Name = "txtBoxTx";
            this.txtBoxTx.Size = new System.Drawing.Size(357, 20);
            this.txtBoxTx.TabIndex = 5;
            // 
            // button1
            // 
            this.button1.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.button1.Location = new System.Drawing.Point(367, 365);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(104, 23);
            this.button1.TabIndex = 6;
            this.button1.Text = "Send Command";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // SerialMonitorView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.Controls.Add(this.button1);
            this.Controls.Add(this.txtBoxTx);
            this.Controls.Add(this.txtSend);
            this.Controls.Add(this.txtBoxRx);
            this.Name = "SerialMonitorView";
            this.Size = new System.Drawing.Size(475, 388);
            ((System.ComponentModel.ISupportInitialize)(this.serialMonitorVmBindingSource)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TextBox txtSend;
        private System.Windows.Forms.TextBox txtBoxRx;
        private System.Windows.Forms.TextBox txtBoxTx;
        private System.Windows.Forms.BindingSource serialMonitorVmBindingSource;
        private System.Windows.Forms.Button button1;
    }
}

using ArducopterConfigurator.PresentationModels;

namespace ArducopterConfigurator
{
    partial class FlightControlPidsView
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
            this.button1 = new System.Windows.Forms.Button();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.stableConfigView1 = new ArducopterConfigurator.Views.StableConfigView();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.acroConfigView1 = new ArducopterConfigurator.Views.AcroConfigView();
            this.FlightControlPidsBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.FlightControlPidsBindingSource)).BeginInit();
            this.SuspendLayout();
            // 
            // txtSend
            // 
            this.txtSend.Location = new System.Drawing.Point(-167, 167);
            this.txtSend.Name = "txtSend";
            this.txtSend.Size = new System.Drawing.Size(10, 20);
            this.txtSend.TabIndex = 4;
            // 
            // button1
            // 
            this.button1.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.button1.Location = new System.Drawing.Point(980, 929);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(104, 23);
            this.button1.TabIndex = 6;
            this.button1.Text = "Send Command";
            this.button1.UseVisualStyleBackColor = true;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.stableConfigView1);
            this.groupBox1.Location = new System.Drawing.Point(7, 9);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(283, 176);
            this.groupBox1.TabIndex = 9;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Stable Mode";
            // 
            // stableConfigView1
            // 
            this.stableConfigView1.Location = new System.Drawing.Point(7, 20);
            this.stableConfigView1.Name = "stableConfigView1";
            this.stableConfigView1.Size = new System.Drawing.Size(271, 150);
            this.stableConfigView1.TabIndex = 0;
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.acroConfigView1);
            this.groupBox2.Location = new System.Drawing.Point(7, 191);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(283, 164);
            this.groupBox2.TabIndex = 10;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Acro Mode";
            // 
            // acroConfigView1
            // 
            this.acroConfigView1.Location = new System.Drawing.Point(7, 20);
            this.acroConfigView1.Name = "acroConfigView1";
            this.acroConfigView1.Size = new System.Drawing.Size(271, 136);
            this.acroConfigView1.TabIndex = 0;
            // 
            // FlightControlPidsBindingSource
            // 
            this.FlightControlPidsBindingSource.DataSource = typeof(ArducopterConfigurator.PresentationModels.FlightControlPidsVm);
            // 
            // FlightControlPidsView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.txtSend);
            this.Name = "FlightControlPidsView";
            this.Size = new System.Drawing.Size(542, 476);
            this.groupBox1.ResumeLayout(false);
            this.groupBox2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.FlightControlPidsBindingSource)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TextBox txtSend;
        private System.Windows.Forms.BindingSource FlightControlPidsBindingSource;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private ArducopterConfigurator.Views.StableConfigView stableConfigView1;
        private ArducopterConfigurator.Views.AcroConfigView acroConfigView1;
    }
}

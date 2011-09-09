namespace ArducopterConfigurator.Views
{
    partial class MotorCommandsView
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
            this.trckBarLeft = new System.Windows.Forms.TrackBar();
            this.btnStop = new System.Windows.Forms.Button();
            this.btnSend = new System.Windows.Forms.Button();
            this.trackBar1 = new System.Windows.Forms.TrackBar();
            this.trackBar2 = new System.Windows.Forms.TrackBar();
            this.trackBar3 = new System.Windows.Forms.TrackBar();
            ((System.ComponentModel.ISupportInitialize)(this.trckBarLeft)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar3)).BeginInit();
            this.SuspendLayout();
            // 
            // trckBarLeft
            // 
            this.trckBarLeft.LargeChange = 50;
            this.trckBarLeft.Location = new System.Drawing.Point(3, 52);
            this.trckBarLeft.Maximum = 1500;
            this.trckBarLeft.Minimum = 1000;
            this.trckBarLeft.Name = "trckBarLeft";
            this.trckBarLeft.Orientation = System.Windows.Forms.Orientation.Vertical;
            this.trckBarLeft.Size = new System.Drawing.Size(45, 104);
            this.trckBarLeft.SmallChange = 10;
            this.trckBarLeft.TabIndex = 0;
            this.trckBarLeft.TickFrequency = 50;
            this.trckBarLeft.Value = 1000;
            // 
            // btnStop
            // 
            this.btnStop.Location = new System.Drawing.Point(54, 52);
            this.btnStop.Name = "btnStop";
            this.btnStop.Size = new System.Drawing.Size(107, 92);
            this.btnStop.TabIndex = 4;
            this.btnStop.Text = "Stop";
            this.btnStop.UseVisualStyleBackColor = true;
            // 
            // btnSend
            // 
            this.btnSend.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.btnSend.Enabled = false;
            this.btnSend.Location = new System.Drawing.Point(202, 203);
            this.btnSend.Name = "btnSend";
            this.btnSend.Size = new System.Drawing.Size(57, 27);
            this.btnSend.TabIndex = 5;
            this.btnSend.Text = "Send";
            this.btnSend.UseVisualStyleBackColor = true;
            // 
            // trackBar1
            // 
            this.trackBar1.LargeChange = 50;
            this.trackBar1.Location = new System.Drawing.Point(180, 52);
            this.trackBar1.Maximum = 1500;
            this.trackBar1.Minimum = 1000;
            this.trackBar1.Name = "trackBar1";
            this.trackBar1.Orientation = System.Windows.Forms.Orientation.Vertical;
            this.trackBar1.Size = new System.Drawing.Size(45, 104);
            this.trackBar1.SmallChange = 10;
            this.trackBar1.TabIndex = 6;
            this.trackBar1.TickFrequency = 50;
            this.trackBar1.Value = 1000;
            // 
            // trackBar2
            // 
            this.trackBar2.LargeChange = 50;
            this.trackBar2.Location = new System.Drawing.Point(57, 150);
            this.trackBar2.Maximum = 1500;
            this.trackBar2.Minimum = 1000;
            this.trackBar2.Name = "trackBar2";
            this.trackBar2.Size = new System.Drawing.Size(104, 45);
            this.trackBar2.SmallChange = 10;
            this.trackBar2.TabIndex = 7;
            this.trackBar2.TickFrequency = 50;
            this.trackBar2.Value = 1000;
            // 
            // trackBar3
            // 
            this.trackBar3.LargeChange = 50;
            this.trackBar3.Location = new System.Drawing.Point(54, 3);
            this.trackBar3.Maximum = 1500;
            this.trackBar3.Minimum = 1000;
            this.trackBar3.Name = "trackBar3";
            this.trackBar3.Size = new System.Drawing.Size(104, 45);
            this.trackBar3.SmallChange = 10;
            this.trackBar3.TabIndex = 8;
            this.trackBar3.TickFrequency = 50;
            this.trackBar3.Value = 1000;
            // 
            // MotorCommandsView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.trackBar3);
            this.Controls.Add(this.trackBar2);
            this.Controls.Add(this.trackBar1);
            this.Controls.Add(this.btnSend);
            this.Controls.Add(this.btnStop);
            this.Controls.Add(this.trckBarLeft);
            this.Name = "MotorCommandsView";
            this.Size = new System.Drawing.Size(262, 233);
            ((System.ComponentModel.ISupportInitialize)(this.trckBarLeft)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar3)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TrackBar trckBarLeft;
        private System.Windows.Forms.Button btnStop;
        private System.Windows.Forms.Button btnSend;
        private System.Windows.Forms.TrackBar trackBar1;
        private System.Windows.Forms.TrackBar trackBar2;
        private System.Windows.Forms.TrackBar trackBar3;
    }
}

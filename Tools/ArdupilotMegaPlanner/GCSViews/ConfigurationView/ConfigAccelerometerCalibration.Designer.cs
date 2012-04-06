namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigAccelerometerCalibration
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
            this.label28 = new System.Windows.Forms.Label();
            this.label16 = new System.Windows.Forms.Label();
            this.label15 = new System.Windows.Forms.Label();
            this.pictureBoxQuadX = new System.Windows.Forms.PictureBox();
            this.pictureBoxQuad = new System.Windows.Forms.PictureBox();
            this.BUT_levelac2 = new ArdupilotMega.MyButton();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxQuadX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxQuad)).BeginInit();
            this.SuspendLayout();
            // 
            // label28
            // 
            this.label28.AutoSize = true;
            this.label28.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label28.Location = new System.Drawing.Point(124, 13);
            this.label28.Name = "label28";
            this.label28.Size = new System.Drawing.Size(210, 13);
            this.label28.TabIndex = 15;
            this.label28.Text = "Level your quad to set default accel offsets";
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label16.Location = new System.Drawing.Point(124, 308);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(192, 26);
            this.label16.TabIndex = 13;
            this.label16.Text = "NOTE: images are for presentation only\r\nwill work with hexa\'s etc";
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label15.Location = new System.Drawing.Point(167, 99);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(102, 13);
            this.label15.TabIndex = 12;
            this.label15.Text = "Frame Setup (+ or x)";
            // 
            // pictureBoxQuadX
            // 
            this.pictureBoxQuadX.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pictureBoxQuadX.Image = global::ArdupilotMega.Properties.Resources.quadx;
            this.pictureBoxQuadX.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.pictureBoxQuadX.Location = new System.Drawing.Point(226, 115);
            this.pictureBoxQuadX.Name = "pictureBoxQuadX";
            this.pictureBoxQuadX.Size = new System.Drawing.Size(190, 190);
            this.pictureBoxQuadX.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pictureBoxQuadX.TabIndex = 11;
            this.pictureBoxQuadX.TabStop = false;
            this.pictureBoxQuadX.Click += new System.EventHandler(this.pictureBoxQuadX_Click);
            // 
            // pictureBoxQuad
            // 
            this.pictureBoxQuad.Cursor = System.Windows.Forms.Cursors.Hand;
            this.pictureBoxQuad.Image = global::ArdupilotMega.Properties.Resources.quad;
            this.pictureBoxQuad.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.pictureBoxQuad.Location = new System.Drawing.Point(19, 115);
            this.pictureBoxQuad.Name = "pictureBoxQuad";
            this.pictureBoxQuad.Size = new System.Drawing.Size(190, 190);
            this.pictureBoxQuad.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pictureBoxQuad.TabIndex = 10;
            this.pictureBoxQuad.TabStop = false;
            // 
            // BUT_levelac2
            // 
            this.BUT_levelac2.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_levelac2.Location = new System.Drawing.Point(181, 42);
            this.BUT_levelac2.Name = "BUT_levelac2";
            this.BUT_levelac2.Size = new System.Drawing.Size(75, 23);
            this.BUT_levelac2.TabIndex = 14;
            this.BUT_levelac2.Text = "Level";
            this.BUT_levelac2.UseVisualStyleBackColor = true;
            // 
            // ConfigAccelerometerCalibration
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.label28);
            this.Controls.Add(this.label16);
            this.Controls.Add(this.label15);
            this.Controls.Add(this.pictureBoxQuadX);
            this.Controls.Add(this.pictureBoxQuad);
            this.Controls.Add(this.BUT_levelac2);
            this.Name = "ConfigAccelerometerCalibration";
            this.Size = new System.Drawing.Size(439, 356);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxQuadX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxQuad)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label label28;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.PictureBox pictureBoxQuadX;
        private System.Windows.Forms.PictureBox pictureBoxQuad;
        private MyButton BUT_levelac2;
    }
}

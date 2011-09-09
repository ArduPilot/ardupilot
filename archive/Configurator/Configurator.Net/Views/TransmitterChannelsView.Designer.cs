namespace ArducopterConfigurator.Views
{
    partial class TransmitterChannelsView
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(TransmitterChannelsView));
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.TransmitterChannelsBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.label9 = new System.Windows.Forms.Label();
            this.textBox6 = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.textBox3 = new System.Windows.Forms.TextBox();
            this.textBox4 = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.textBox5 = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.linearIndicatorControl3 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl1 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl2 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl4 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl5 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl6 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.label12 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.button3 = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
            this.button4 = new System.Windows.Forms.Button();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.TransmitterChannelsBindingSource)).BeginInit();
            this.SuspendLayout();
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(35, 11);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(43, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "Throttle";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(227, 11);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(31, 13);
            this.label2.TabIndex = 3;
            this.label2.Text = "Pitch";
            // 
            // TransmitterChannelsBindingSource
            // 
            this.TransmitterChannelsBindingSource.DataSource = typeof(ArducopterConfigurator.PresentationModels.TransmitterChannelsVm);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(12, 177);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(28, 13);
            this.label9.TabIndex = 30;
            this.label9.Text = "Yaw";
            // 
            // textBox6
            // 
            this.textBox6.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox6.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Yaw", true));
            this.textBox6.Enabled = false;
            this.textBox6.Location = new System.Drawing.Point(115, 157);
            this.textBox6.Name = "textBox6";
            this.textBox6.ReadOnly = true;
            this.textBox6.Size = new System.Drawing.Size(35, 13);
            this.textBox6.TabIndex = 29;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(172, 177);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(25, 13);
            this.label3.TabIndex = 33;
            this.label3.Text = "Roll";
            // 
            // textBox1
            // 
            this.textBox1.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox1.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Roll", true));
            this.textBox1.Enabled = false;
            this.textBox1.Location = new System.Drawing.Point(275, 157);
            this.textBox1.Name = "textBox1";
            this.textBox1.ReadOnly = true;
            this.textBox1.Size = new System.Drawing.Size(35, 13);
            this.textBox1.TabIndex = 32;
            // 
            // textBox2
            // 
            this.textBox2.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox2.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Throttle", true));
            this.textBox2.Enabled = false;
            this.textBox2.Location = new System.Drawing.Point(43, 141);
            this.textBox2.Name = "textBox2";
            this.textBox2.ReadOnly = true;
            this.textBox2.Size = new System.Drawing.Size(35, 13);
            this.textBox2.TabIndex = 35;
            // 
            // textBox3
            // 
            this.textBox3.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox3.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Pitch", true));
            this.textBox3.Enabled = false;
            this.textBox3.Location = new System.Drawing.Point(223, 141);
            this.textBox3.Name = "textBox3";
            this.textBox3.ReadOnly = true;
            this.textBox3.Size = new System.Drawing.Size(35, 13);
            this.textBox3.TabIndex = 36;
            // 
            // textBox4
            // 
            this.textBox4.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox4.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Mode", true));
            this.textBox4.Enabled = false;
            this.textBox4.Location = new System.Drawing.Point(99, 87);
            this.textBox4.Name = "textBox4";
            this.textBox4.ReadOnly = true;
            this.textBox4.Size = new System.Drawing.Size(35, 13);
            this.textBox4.TabIndex = 39;
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(104, 11);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(34, 13);
            this.label4.TabIndex = 38;
            this.label4.Text = "Mode";
            // 
            // textBox5
            // 
            this.textBox5.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox5.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Aux", true));
            this.textBox5.Enabled = false;
            this.textBox5.Location = new System.Drawing.Point(154, 87);
            this.textBox5.Name = "textBox5";
            this.textBox5.ReadOnly = true;
            this.textBox5.Size = new System.Drawing.Size(35, 13);
            this.textBox5.TabIndex = 42;
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(154, 11);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(25, 13);
            this.label5.TabIndex = 41;
            this.label5.Text = "Aux";
            // 
            // linearIndicatorControl3
            // 
            this.linearIndicatorControl3.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl3.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl3.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl3.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl3.BarDividersCount = 20;
            this.linearIndicatorControl3.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl3.DataBindings.Add(new System.Windows.Forms.Binding("MaxWaterMark", this.TransmitterChannelsBindingSource, "ThrottleMax", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl3.DataBindings.Add(new System.Windows.Forms.Binding("MinWatermark", this.TransmitterChannelsBindingSource, "ThrottleMin", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl3.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Throttle", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl3.IsVertical = true;
            this.linearIndicatorControl3.Location = new System.Drawing.Point(54, 30);
            this.linearIndicatorControl3.Max = 2000;
            this.linearIndicatorControl3.MaxWaterMark = 0;
            this.linearIndicatorControl3.Min = 1000;
            this.linearIndicatorControl3.MinWatermark = 0;
            this.linearIndicatorControl3.Name = "linearIndicatorControl3";
            this.linearIndicatorControl3.Offset = 0;
            this.linearIndicatorControl3.Size = new System.Drawing.Size(14, 111);
            this.linearIndicatorControl3.TabIndex = 43;
            this.linearIndicatorControl3.Value = 1600;
            this.linearIndicatorControl3.WatermarkLineColor = System.Drawing.Color.Red;
            // 
            // linearIndicatorControl1
            // 
            this.linearIndicatorControl1.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl1.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl1.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl1.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl1.BarDividersCount = 20;
            this.linearIndicatorControl1.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl1.DataBindings.Add(new System.Windows.Forms.Binding("MaxWaterMark", this.TransmitterChannelsBindingSource, "PitchMax", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl1.DataBindings.Add(new System.Windows.Forms.Binding("MinWatermark", this.TransmitterChannelsBindingSource, "PitchMin", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl1.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Pitch", true));
            this.linearIndicatorControl1.IsVertical = true;
            this.linearIndicatorControl1.Location = new System.Drawing.Point(233, 30);
            this.linearIndicatorControl1.Max = 2000;
            this.linearIndicatorControl1.MaxWaterMark = 0;
            this.linearIndicatorControl1.Min = 1000;
            this.linearIndicatorControl1.MinWatermark = 0;
            this.linearIndicatorControl1.Name = "linearIndicatorControl1";
            this.linearIndicatorControl1.Offset = 0;
            this.linearIndicatorControl1.Size = new System.Drawing.Size(14, 111);
            this.linearIndicatorControl1.TabIndex = 44;
            this.linearIndicatorControl1.Value = 1100;
            this.linearIndicatorControl1.WatermarkLineColor = System.Drawing.Color.Red;
            // 
            // linearIndicatorControl2
            // 
            this.linearIndicatorControl2.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl2.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl2.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl2.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl2.BarDividersCount = 20;
            this.linearIndicatorControl2.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl2.DataBindings.Add(new System.Windows.Forms.Binding("MaxWaterMark", this.TransmitterChannelsBindingSource, "ModeMax", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl2.DataBindings.Add(new System.Windows.Forms.Binding("MinWatermark", this.TransmitterChannelsBindingSource, "ModeMin", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl2.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Mode", true));
            this.linearIndicatorControl2.IsVertical = true;
            this.linearIndicatorControl2.Location = new System.Drawing.Point(109, 30);
            this.linearIndicatorControl2.Max = 2000;
            this.linearIndicatorControl2.MaxWaterMark = 0;
            this.linearIndicatorControl2.Min = 1000;
            this.linearIndicatorControl2.MinWatermark = 0;
            this.linearIndicatorControl2.Name = "linearIndicatorControl2";
            this.linearIndicatorControl2.Offset = 0;
            this.linearIndicatorControl2.Size = new System.Drawing.Size(14, 57);
            this.linearIndicatorControl2.TabIndex = 45;
            this.linearIndicatorControl2.Value = 1050;
            this.linearIndicatorControl2.WatermarkLineColor = System.Drawing.Color.Red;
            // 
            // linearIndicatorControl4
            // 
            this.linearIndicatorControl4.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl4.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl4.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl4.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl4.BarDividersCount = 20;
            this.linearIndicatorControl4.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl4.DataBindings.Add(new System.Windows.Forms.Binding("MaxWaterMark", this.TransmitterChannelsBindingSource, "AuxMax", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl4.DataBindings.Add(new System.Windows.Forms.Binding("MinWatermark", this.TransmitterChannelsBindingSource, "AuxMin", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl4.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Aux", true));
            this.linearIndicatorControl4.IsVertical = true;
            this.linearIndicatorControl4.Location = new System.Drawing.Point(164, 30);
            this.linearIndicatorControl4.Max = 2000;
            this.linearIndicatorControl4.MaxWaterMark = 0;
            this.linearIndicatorControl4.Min = 1000;
            this.linearIndicatorControl4.MinWatermark = 0;
            this.linearIndicatorControl4.Name = "linearIndicatorControl4";
            this.linearIndicatorControl4.Offset = 0;
            this.linearIndicatorControl4.Size = new System.Drawing.Size(14, 57);
            this.linearIndicatorControl4.TabIndex = 46;
            this.linearIndicatorControl4.Value = 1900;
            this.linearIndicatorControl4.WatermarkLineColor = System.Drawing.Color.Red;
            // 
            // linearIndicatorControl5
            // 
            this.linearIndicatorControl5.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl5.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl5.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl5.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl5.BarDividersCount = 20;
            this.linearIndicatorControl5.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl5.DataBindings.Add(new System.Windows.Forms.Binding("MaxWaterMark", this.TransmitterChannelsBindingSource, "RollMax", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl5.DataBindings.Add(new System.Windows.Forms.Binding("MinWatermark", this.TransmitterChannelsBindingSource, "RollMin", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl5.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Roll", true));
            this.linearIndicatorControl5.IsVertical = false;
            this.linearIndicatorControl5.Location = new System.Drawing.Point(175, 160);
            this.linearIndicatorControl5.Max = 2000;
            this.linearIndicatorControl5.MaxWaterMark = 0;
            this.linearIndicatorControl5.Min = 1000;
            this.linearIndicatorControl5.MinWatermark = 0;
            this.linearIndicatorControl5.Name = "linearIndicatorControl5";
            this.linearIndicatorControl5.Offset = 0;
            this.linearIndicatorControl5.Size = new System.Drawing.Size(100, 14);
            this.linearIndicatorControl5.TabIndex = 47;
            this.linearIndicatorControl5.Value = 1300;
            this.linearIndicatorControl5.WatermarkLineColor = System.Drawing.Color.Red;
            // 
            // linearIndicatorControl6
            // 
            this.linearIndicatorControl6.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl6.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl6.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl6.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl6.BarDividersCount = 20;
            this.linearIndicatorControl6.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl6.DataBindings.Add(new System.Windows.Forms.Binding("MaxWaterMark", this.TransmitterChannelsBindingSource, "YawMax", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl6.DataBindings.Add(new System.Windows.Forms.Binding("MinWatermark", this.TransmitterChannelsBindingSource, "YawMin", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.linearIndicatorControl6.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Yaw", true));
            this.linearIndicatorControl6.IsVertical = false;
            this.linearIndicatorControl6.Location = new System.Drawing.Point(15, 160);
            this.linearIndicatorControl6.Max = 2000;
            this.linearIndicatorControl6.MaxWaterMark = 0;
            this.linearIndicatorControl6.Min = 1000;
            this.linearIndicatorControl6.MinWatermark = 0;
            this.linearIndicatorControl6.Name = "linearIndicatorControl6";
            this.linearIndicatorControl6.Offset = 0;
            this.linearIndicatorControl6.Size = new System.Drawing.Size(100, 14);
            this.linearIndicatorControl6.TabIndex = 48;
            this.linearIndicatorControl6.Value = 1200;
            this.linearIndicatorControl6.WatermarkLineColor = System.Drawing.Color.Red;
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(317, 64);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(84, 13);
            this.label12.TabIndex = 53;
            this.label12.Text = "Save Calibration";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(317, 35);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(48, 13);
            this.label11.TabIndex = 52;
            this.label11.Text = "Calibrate";
            // 
            // button3
            // 
            this.button3.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.TransmitterChannelsBindingSource, "StartCalibrationCommand", true));
            this.button3.Image = ((System.Drawing.Image)(resources.GetObject("button3.Image")));
            this.button3.Location = new System.Drawing.Point(285, 28);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(26, 26);
            this.button3.TabIndex = 51;
            this.button3.UseVisualStyleBackColor = true;
            // 
            // button2
            // 
            this.button2.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.TransmitterChannelsBindingSource, "SaveCalibrationCommand", true));
            this.button2.Image = ((System.Drawing.Image)(resources.GetObject("button2.Image")));
            this.button2.Location = new System.Drawing.Point(285, 57);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(26, 26);
            this.button2.TabIndex = 50;
            this.button2.UseVisualStyleBackColor = true;
            // 
            // button4
            // 
            this.button4.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.TransmitterChannelsBindingSource, "CancelCalibrationCommand", true));
            this.button4.Location = new System.Drawing.Point(285, 87);
            this.button4.Name = "button4";
            this.button4.Size = new System.Drawing.Size(59, 26);
            this.button4.TabIndex = 54;
            this.button4.Text = "Cancel";
            this.button4.UseVisualStyleBackColor = true;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(96, 103);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(50, 13);
            this.label6.TabIndex = 55;
            this.label6.Text = "(Alt Hold)";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(152, 103);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(56, 13);
            this.label7.TabIndex = 56;
            this.label7.Text = "(Pos Hold)";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(96, 128);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(108, 13);
            this.label8.TabIndex = 57;
            this.label8.Text = "(Low value activates)";
            // 
            // TransmitterChannelsView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.button4);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.button3);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.linearIndicatorControl6);
            this.Controls.Add(this.linearIndicatorControl5);
            this.Controls.Add(this.linearIndicatorControl4);
            this.Controls.Add(this.linearIndicatorControl2);
            this.Controls.Add(this.linearIndicatorControl1);
            this.Controls.Add(this.linearIndicatorControl3);
            this.Controls.Add(this.textBox5);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.textBox4);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.textBox3);
            this.Controls.Add(this.textBox2);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.textBox6);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Name = "TransmitterChannelsView";
            this.Size = new System.Drawing.Size(440, 300);
            ((System.ComponentModel.ISupportInitialize)(this.TransmitterChannelsBindingSource)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.BindingSource TransmitterChannelsBindingSource;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox textBox6;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.TextBox textBox2;
        private System.Windows.Forms.TextBox textBox3;
        private System.Windows.Forms.TextBox textBox4;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox textBox5;
        private System.Windows.Forms.Label label5;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl3;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl1;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl2;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl4;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl5;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl6;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Button button4;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
    }
}

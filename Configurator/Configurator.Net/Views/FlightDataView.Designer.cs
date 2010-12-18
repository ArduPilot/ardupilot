namespace ArducopterConfigurator.views
{
    partial class FlightDataView
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
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.label5 = new System.Windows.Forms.Label();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.textBox3 = new System.Windows.Forms.TextBox();
            this.progressBar2 = new System.Windows.Forms.ProgressBar();
            this.label6 = new System.Windows.Forms.Label();
            this.textBox4 = new System.Windows.Forms.TextBox();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.textBox5 = new System.Windows.Forms.TextBox();
            this.label9 = new System.Windows.Forms.Label();
            this.textBox6 = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.textBox7 = new System.Windows.Forms.TextBox();
            this.progressBar3 = new System.Windows.Forms.ProgressBar();
            this.FlightDataVmBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.verticalProgressBar8 = new ArducopterConfigurator.VerticalProgressBar();
            this.verticalProgressBar6 = new ArducopterConfigurator.VerticalProgressBar();
            this.verticalProgressBar5 = new ArducopterConfigurator.VerticalProgressBar();
            this.verticalProgressBar4 = new ArducopterConfigurator.VerticalProgressBar();
            this.verticalProgressBar3 = new ArducopterConfigurator.VerticalProgressBar();
            this.verticalProgressBar2 = new ArducopterConfigurator.VerticalProgressBar();
            this.verticalProgressBar1 = new ArducopterConfigurator.VerticalProgressBar();
            this.textBox8 = new System.Windows.Forms.TextBox();
            this.textBox9 = new System.Windows.Forms.TextBox();
            this.textBox10 = new System.Windows.Forms.TextBox();
            ((System.ComponentModel.ISupportInitialize)(this.FlightDataVmBindingSource)).BeginInit();
            this.SuspendLayout();
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(12, 138);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(25, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "Left";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(68, 138);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(31, 13);
            this.label2.TabIndex = 3;
            this.label2.Text = "Front";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(124, 138);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(30, 13);
            this.label3.TabIndex = 5;
            this.label3.Text = "Rear";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(176, 138);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(32, 13);
            this.label4.TabIndex = 7;
            this.label4.Text = "Right";
            // 
            // textBox1
            // 
            this.textBox1.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "MotorLeft", true, System.Windows.Forms.DataSourceUpdateMode.OnValidation, null, "N0"));
            this.textBox1.Location = new System.Drawing.Point(2, 247);
            this.textBox1.Name = "textBox1";
            this.textBox1.ReadOnly = true;
            this.textBox1.Size = new System.Drawing.Size(47, 20);
            this.textBox1.TabIndex = 8;
            // 
            // progressBar1
            // 
            this.progressBar1.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "GyroRoll", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.progressBar1.Location = new System.Drawing.Point(12, 25);
            this.progressBar1.Maximum = 1000;
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Size = new System.Drawing.Size(100, 23);
            this.progressBar1.TabIndex = 9;
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(9, 9);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(50, 13);
            this.label5.TabIndex = 10;
            this.label5.Text = "Roll Gyro";
            // 
            // textBox2
            // 
            this.textBox2.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "GyroRoll", true));
            this.textBox2.Location = new System.Drawing.Point(124, 28);
            this.textBox2.Name = "textBox2";
            this.textBox2.ReadOnly = true;
            this.textBox2.Size = new System.Drawing.Size(47, 20);
            this.textBox2.TabIndex = 12;
            // 
            // textBox3
            // 
            this.textBox3.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "AccelRoll", true, System.Windows.Forms.DataSourceUpdateMode.Never, null, "N0"));
            this.textBox3.Location = new System.Drawing.Point(124, 68);
            this.textBox3.Name = "textBox3";
            this.textBox3.ReadOnly = true;
            this.textBox3.Size = new System.Drawing.Size(47, 20);
            this.textBox3.TabIndex = 13;
            // 
            // progressBar2
            // 
            this.progressBar2.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "AccelRoll", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.progressBar2.Location = new System.Drawing.Point(12, 68);
            this.progressBar2.Maximum = 1000;
            this.progressBar2.Name = "progressBar2";
            this.progressBar2.Size = new System.Drawing.Size(100, 23);
            this.progressBar2.TabIndex = 14;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(9, 52);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(57, 13);
            this.label6.TabIndex = 15;
            this.label6.Text = "Roll Accell";
            // 
            // textBox4
            // 
            this.textBox4.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "GyroPitch", true, System.Windows.Forms.DataSourceUpdateMode.Never, null, "N0"));
            this.textBox4.Location = new System.Drawing.Point(187, 109);
            this.textBox4.Name = "textBox4";
            this.textBox4.ReadOnly = true;
            this.textBox4.Size = new System.Drawing.Size(47, 20);
            this.textBox4.TabIndex = 17;
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(184, 5);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(56, 13);
            this.label7.TabIndex = 18;
            this.label7.Text = "Pitch Gyro";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(244, 5);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(61, 13);
            this.label8.TabIndex = 21;
            this.label8.Text = "Pitch Accel";
            // 
            // textBox5
            // 
            this.textBox5.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "AccelPitch", true, System.Windows.Forms.DataSourceUpdateMode.Never, null, "N0"));
            this.textBox5.Location = new System.Drawing.Point(247, 109);
            this.textBox5.Name = "textBox5";
            this.textBox5.ReadOnly = true;
            this.textBox5.Size = new System.Drawing.Size(47, 20);
            this.textBox5.TabIndex = 20;
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(231, 141);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(53, 13);
            this.label9.TabIndex = 27;
            this.label9.Text = "Yaw Gyro";
            // 
            // textBox6
            // 
            this.textBox6.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "GyroYaw", true, System.Windows.Forms.DataSourceUpdateMode.Never, null, "N0"));
            this.textBox6.Location = new System.Drawing.Point(340, 157);
            this.textBox6.Name = "textBox6";
            this.textBox6.ReadOnly = true;
            this.textBox6.Size = new System.Drawing.Size(47, 20);
            this.textBox6.TabIndex = 26;
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(340, 5);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(44, 13);
            this.label10.TabIndex = 24;
            this.label10.Text = "Accel Z";
            // 
            // textBox7
            // 
            this.textBox7.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "AccelZ", true, System.Windows.Forms.DataSourceUpdateMode.Never, null, "N0"));
            this.textBox7.Location = new System.Drawing.Point(343, 109);
            this.textBox7.Name = "textBox7";
            this.textBox7.ReadOnly = true;
            this.textBox7.Size = new System.Drawing.Size(47, 20);
            this.textBox7.TabIndex = 23;
            // 
            // progressBar3
            // 
            this.progressBar3.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "GyroYaw", true));
            this.progressBar3.Location = new System.Drawing.Point(234, 157);
            this.progressBar3.Maximum = 1000;
            this.progressBar3.Name = "progressBar3";
            this.progressBar3.Size = new System.Drawing.Size(100, 23);
            this.progressBar3.TabIndex = 28;
            // 
            // FlightDataVmBindingSource
            // 
            this.FlightDataVmBindingSource.DataSource = typeof(ArducopterConfigurator.PresentationModels.FlightDataVm);
            // 
            // verticalProgressBar8
            // 
            this.verticalProgressBar8.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "AccelZ", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.verticalProgressBar8.Location = new System.Drawing.Point(354, 21);
            this.verticalProgressBar8.Maximum = 1000;
            this.verticalProgressBar8.Name = "verticalProgressBar8";
            this.verticalProgressBar8.Size = new System.Drawing.Size(24, 82);
            this.verticalProgressBar8.Step = 1;
            this.verticalProgressBar8.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            this.verticalProgressBar8.TabIndex = 22;
            this.verticalProgressBar8.Value = 500;
            // 
            // verticalProgressBar6
            // 
            this.verticalProgressBar6.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "AccelPitch", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.verticalProgressBar6.Location = new System.Drawing.Point(258, 21);
            this.verticalProgressBar6.Maximum = 1000;
            this.verticalProgressBar6.Name = "verticalProgressBar6";
            this.verticalProgressBar6.Size = new System.Drawing.Size(24, 82);
            this.verticalProgressBar6.Step = 1;
            this.verticalProgressBar6.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            this.verticalProgressBar6.TabIndex = 19;
            this.verticalProgressBar6.Value = 500;
            // 
            // verticalProgressBar5
            // 
            this.verticalProgressBar5.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "GyroPitch", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.verticalProgressBar5.Location = new System.Drawing.Point(198, 21);
            this.verticalProgressBar5.Maximum = 1000;
            this.verticalProgressBar5.Name = "verticalProgressBar5";
            this.verticalProgressBar5.Size = new System.Drawing.Size(24, 82);
            this.verticalProgressBar5.Step = 1;
            this.verticalProgressBar5.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            this.verticalProgressBar5.TabIndex = 16;
            this.verticalProgressBar5.Value = 500;
            // 
            // verticalProgressBar4
            // 
            this.verticalProgressBar4.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorRight", true));
            this.verticalProgressBar4.Location = new System.Drawing.Point(176, 157);
            this.verticalProgressBar4.Maximum = 2000;
            this.verticalProgressBar4.Minimum = 1000;
            this.verticalProgressBar4.Name = "verticalProgressBar4";
            this.verticalProgressBar4.Size = new System.Drawing.Size(24, 84);
            this.verticalProgressBar4.Step = 1;
            this.verticalProgressBar4.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            this.verticalProgressBar4.TabIndex = 6;
            this.verticalProgressBar4.Value = 1000;
            // 
            // verticalProgressBar3
            // 
            this.verticalProgressBar3.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorRear", true));
            this.verticalProgressBar3.Location = new System.Drawing.Point(124, 157);
            this.verticalProgressBar3.Maximum = 2000;
            this.verticalProgressBar3.Minimum = 1000;
            this.verticalProgressBar3.Name = "verticalProgressBar3";
            this.verticalProgressBar3.Size = new System.Drawing.Size(24, 84);
            this.verticalProgressBar3.Step = 1;
            this.verticalProgressBar3.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            this.verticalProgressBar3.TabIndex = 4;
            this.verticalProgressBar3.Value = 1000;
            // 
            // verticalProgressBar2
            // 
            this.verticalProgressBar2.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorFront", true));
            this.verticalProgressBar2.Location = new System.Drawing.Point(68, 157);
            this.verticalProgressBar2.Maximum = 2000;
            this.verticalProgressBar2.Minimum = 1000;
            this.verticalProgressBar2.Name = "verticalProgressBar2";
            this.verticalProgressBar2.Size = new System.Drawing.Size(24, 84);
            this.verticalProgressBar2.Step = 1;
            this.verticalProgressBar2.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            this.verticalProgressBar2.TabIndex = 2;
            this.verticalProgressBar2.Value = 1000;
            // 
            // verticalProgressBar1
            // 
            this.verticalProgressBar1.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorLeft", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.verticalProgressBar1.Location = new System.Drawing.Point(12, 157);
            this.verticalProgressBar1.Maximum = 2000;
            this.verticalProgressBar1.Minimum = 1000;
            this.verticalProgressBar1.Name = "verticalProgressBar1";
            this.verticalProgressBar1.Size = new System.Drawing.Size(24, 84);
            this.verticalProgressBar1.Step = 1;
            this.verticalProgressBar1.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            this.verticalProgressBar1.TabIndex = 0;
            this.verticalProgressBar1.Value = 1000;
            // 
            // textBox8
            // 
            this.textBox8.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "MotorFront", true));
            this.textBox8.Location = new System.Drawing.Point(55, 247);
            this.textBox8.Name = "textBox8";
            this.textBox8.ReadOnly = true;
            this.textBox8.Size = new System.Drawing.Size(47, 20);
            this.textBox8.TabIndex = 29;
            // 
            // textBox9
            // 
            this.textBox9.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "MotorRear", true));
            this.textBox9.Location = new System.Drawing.Point(108, 247);
            this.textBox9.Name = "textBox9";
            this.textBox9.ReadOnly = true;
            this.textBox9.Size = new System.Drawing.Size(47, 20);
            this.textBox9.TabIndex = 30;
            // 
            // textBox10
            // 
            this.textBox10.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "MotorRight", true));
            this.textBox10.Location = new System.Drawing.Point(161, 247);
            this.textBox10.Name = "textBox10";
            this.textBox10.ReadOnly = true;
            this.textBox10.Size = new System.Drawing.Size(47, 20);
            this.textBox10.TabIndex = 31;
            // 
            // FlightDataView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.textBox10);
            this.Controls.Add(this.textBox9);
            this.Controls.Add(this.textBox8);
            this.Controls.Add(this.progressBar3);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.textBox6);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.textBox7);
            this.Controls.Add(this.verticalProgressBar8);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.textBox5);
            this.Controls.Add(this.verticalProgressBar6);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.textBox4);
            this.Controls.Add(this.verticalProgressBar5);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.progressBar2);
            this.Controls.Add(this.textBox3);
            this.Controls.Add(this.textBox2);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.progressBar1);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.verticalProgressBar4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.verticalProgressBar3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.verticalProgressBar2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.verticalProgressBar1);
            this.DoubleBuffered = true;
            this.Name = "FlightDataView";
            this.Size = new System.Drawing.Size(402, 275);
            ((System.ComponentModel.ISupportInitialize)(this.FlightDataVmBindingSource)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.BindingSource FlightDataVmBindingSource;
        private VerticalProgressBar verticalProgressBar1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private VerticalProgressBar verticalProgressBar2;
        private System.Windows.Forms.Label label3;
        private VerticalProgressBar verticalProgressBar3;
        private System.Windows.Forms.Label label4;
        private VerticalProgressBar verticalProgressBar4;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.ProgressBar progressBar1;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox textBox2;
        private System.Windows.Forms.TextBox textBox3;
        private System.Windows.Forms.ProgressBar progressBar2;
        private System.Windows.Forms.Label label6;
        private VerticalProgressBar verticalProgressBar5;
        private System.Windows.Forms.TextBox textBox4;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox textBox5;
        private VerticalProgressBar verticalProgressBar6;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox textBox6;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox textBox7;
        private VerticalProgressBar verticalProgressBar8;
        private System.Windows.Forms.ProgressBar progressBar3;
        private System.Windows.Forms.TextBox textBox8;
        private System.Windows.Forms.TextBox textBox9;
        private System.Windows.Forms.TextBox textBox10;
    }
}

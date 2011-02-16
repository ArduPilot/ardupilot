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
            this.FlightDataVmBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.label5 = new System.Windows.Forms.Label();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.textBox3 = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.textBox4 = new System.Windows.Forms.TextBox();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.textBox5 = new System.Windows.Forms.TextBox();
            this.label9 = new System.Windows.Forms.Label();
            this.textBox6 = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.textBox7 = new System.Windows.Forms.TextBox();
            this.textBox8 = new System.Windows.Forms.TextBox();
            this.textBox9 = new System.Windows.Forms.TextBox();
            this.textBox10 = new System.Windows.Forms.TextBox();
            this.indicatorRollGyro = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.indicatorRollAccel = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl1 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl2 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl3 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl4 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl5 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl6 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl7 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl8 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
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
            this.textBox1.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "MotorLeft", true));
            this.textBox1.Location = new System.Drawing.Point(2, 247);
            this.textBox1.Name = "textBox1";
            this.textBox1.ReadOnly = true;
            this.textBox1.Size = new System.Drawing.Size(47, 20);
            this.textBox1.TabIndex = 8;
            // 
            // FlightDataVmBindingSource
            // 
            this.FlightDataVmBindingSource.DataSource = typeof(ArducopterConfigurator.PresentationModels.SensorsVm);
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
            this.textBox3.Location = new System.Drawing.Point(124, 78);
            this.textBox3.Name = "textBox3";
            this.textBox3.ReadOnly = true;
            this.textBox3.Size = new System.Drawing.Size(47, 20);
            this.textBox3.TabIndex = 13;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(12, 62);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(55, 13);
            this.label6.TabIndex = 15;
            this.label6.Text = "Roll Accel";
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
            // indicatorRollGyro
            // 
            this.indicatorRollGyro.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.indicatorRollGyro.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.indicatorRollGyro.BarBorderColor = System.Drawing.Color.Maroon;
            this.indicatorRollGyro.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.indicatorRollGyro.BarDividersCount = 20;
            this.indicatorRollGyro.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.indicatorRollGyro.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "GyroRoll", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.indicatorRollGyro.IsVertical = false;
            this.indicatorRollGyro.Location = new System.Drawing.Point(12, 28);
            this.indicatorRollGyro.Max = 500;
            this.indicatorRollGyro.MaxWaterMark = 0;
            this.indicatorRollGyro.Min = -500;
            this.indicatorRollGyro.MinWatermark = 0;
            this.indicatorRollGyro.Name = "indicatorRollGyro";
            this.indicatorRollGyro.Offset = 0;
            this.indicatorRollGyro.Size = new System.Drawing.Size(109, 20);
            this.indicatorRollGyro.TabIndex = 32;
            this.indicatorRollGyro.Value = 100;
            this.indicatorRollGyro.WatermarkLineColor = System.Drawing.Color.DarkGray;
            // 
            // indicatorRollAccel
            // 
            this.indicatorRollAccel.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.indicatorRollAccel.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.indicatorRollAccel.BarBorderColor = System.Drawing.Color.Maroon;
            this.indicatorRollAccel.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.indicatorRollAccel.BarDividersCount = 20;
            this.indicatorRollAccel.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.indicatorRollAccel.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "AccelRoll", true));
            this.indicatorRollAccel.IsVertical = false;
            this.indicatorRollAccel.Location = new System.Drawing.Point(12, 78);
            this.indicatorRollAccel.Max = 500;
            this.indicatorRollAccel.MaxWaterMark = 0;
            this.indicatorRollAccel.Min = -500;
            this.indicatorRollAccel.MinWatermark = 0;
            this.indicatorRollAccel.Name = "indicatorRollAccel";
            this.indicatorRollAccel.Offset = 0;
            this.indicatorRollAccel.Size = new System.Drawing.Size(109, 20);
            this.indicatorRollAccel.TabIndex = 33;
            this.indicatorRollAccel.Value = 100;
            this.indicatorRollAccel.WatermarkLineColor = System.Drawing.Color.DarkGray;
            // 
            // linearIndicatorControl1
            // 
            this.linearIndicatorControl1.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl1.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl1.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl1.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl1.BarDividersCount = 20;
            this.linearIndicatorControl1.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl1.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "GyroPitch", true));
            this.linearIndicatorControl1.IsVertical = true;
            this.linearIndicatorControl1.Location = new System.Drawing.Point(199, 21);
            this.linearIndicatorControl1.Max = 500;
            this.linearIndicatorControl1.MaxWaterMark = 0;
            this.linearIndicatorControl1.Min = -500;
            this.linearIndicatorControl1.MinWatermark = 0;
            this.linearIndicatorControl1.Name = "linearIndicatorControl1";
            this.linearIndicatorControl1.Offset = 0;
            this.linearIndicatorControl1.Size = new System.Drawing.Size(20, 82);
            this.linearIndicatorControl1.TabIndex = 34;
            this.linearIndicatorControl1.Value = 100;
            this.linearIndicatorControl1.WatermarkLineColor = System.Drawing.Color.DarkGray;
            // 
            // linearIndicatorControl2
            // 
            this.linearIndicatorControl2.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl2.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl2.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl2.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl2.BarDividersCount = 20;
            this.linearIndicatorControl2.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl2.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "AccelPitch", true));
            this.linearIndicatorControl2.IsVertical = true;
            this.linearIndicatorControl2.Location = new System.Drawing.Point(264, 21);
            this.linearIndicatorControl2.Max = 500;
            this.linearIndicatorControl2.MaxWaterMark = 0;
            this.linearIndicatorControl2.Min = -500;
            this.linearIndicatorControl2.MinWatermark = 0;
            this.linearIndicatorControl2.Name = "linearIndicatorControl2";
            this.linearIndicatorControl2.Offset = 0;
            this.linearIndicatorControl2.Size = new System.Drawing.Size(20, 82);
            this.linearIndicatorControl2.TabIndex = 35;
            this.linearIndicatorControl2.Value = 100;
            this.linearIndicatorControl2.WatermarkLineColor = System.Drawing.Color.Red;
            // 
            // linearIndicatorControl3
            // 
            this.linearIndicatorControl3.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl3.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl3.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl3.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl3.BarDividersCount = 20;
            this.linearIndicatorControl3.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl3.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorLeft", true));
            this.linearIndicatorControl3.IsVertical = true;
            this.linearIndicatorControl3.Location = new System.Drawing.Point(15, 157);
            this.linearIndicatorControl3.Max = 2000;
            this.linearIndicatorControl3.MaxWaterMark = 0;
            this.linearIndicatorControl3.Min = 1000;
            this.linearIndicatorControl3.MinWatermark = 0;
            this.linearIndicatorControl3.Name = "linearIndicatorControl3";
            this.linearIndicatorControl3.Offset = 0;
            this.linearIndicatorControl3.Size = new System.Drawing.Size(20, 84);
            this.linearIndicatorControl3.TabIndex = 36;
            this.linearIndicatorControl3.Value = 100;
            this.linearIndicatorControl3.WatermarkLineColor = System.Drawing.Color.DarkGray;
            // 
            // linearIndicatorControl4
            // 
            this.linearIndicatorControl4.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl4.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl4.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl4.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl4.BarDividersCount = 20;
            this.linearIndicatorControl4.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl4.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorFront", true));
            this.linearIndicatorControl4.IsVertical = true;
            this.linearIndicatorControl4.Location = new System.Drawing.Point(71, 157);
            this.linearIndicatorControl4.Max = 2000;
            this.linearIndicatorControl4.MaxWaterMark = 0;
            this.linearIndicatorControl4.Min = 1000;
            this.linearIndicatorControl4.MinWatermark = 0;
            this.linearIndicatorControl4.Name = "linearIndicatorControl4";
            this.linearIndicatorControl4.Offset = 0;
            this.linearIndicatorControl4.Size = new System.Drawing.Size(20, 84);
            this.linearIndicatorControl4.TabIndex = 37;
            this.linearIndicatorControl4.Value = 100;
            this.linearIndicatorControl4.WatermarkLineColor = System.Drawing.Color.DarkGray;
            // 
            // linearIndicatorControl5
            // 
            this.linearIndicatorControl5.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl5.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl5.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl5.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl5.BarDividersCount = 20;
            this.linearIndicatorControl5.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl5.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorRear", true));
            this.linearIndicatorControl5.IsVertical = true;
            this.linearIndicatorControl5.Location = new System.Drawing.Point(124, 157);
            this.linearIndicatorControl5.Max = 2000;
            this.linearIndicatorControl5.MaxWaterMark = 0;
            this.linearIndicatorControl5.Min = 1000;
            this.linearIndicatorControl5.MinWatermark = 0;
            this.linearIndicatorControl5.Name = "linearIndicatorControl5";
            this.linearIndicatorControl5.Offset = 0;
            this.linearIndicatorControl5.Size = new System.Drawing.Size(20, 84);
            this.linearIndicatorControl5.TabIndex = 38;
            this.linearIndicatorControl5.Value = 100;
            this.linearIndicatorControl5.WatermarkLineColor = System.Drawing.Color.DarkGray;
            // 
            // linearIndicatorControl6
            // 
            this.linearIndicatorControl6.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl6.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl6.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl6.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl6.BarDividersCount = 20;
            this.linearIndicatorControl6.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl6.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorRear", true));
            this.linearIndicatorControl6.IsVertical = true;
            this.linearIndicatorControl6.Location = new System.Drawing.Point(179, 157);
            this.linearIndicatorControl6.Max = 2000;
            this.linearIndicatorControl6.MaxWaterMark = 0;
            this.linearIndicatorControl6.Min = 1000;
            this.linearIndicatorControl6.MinWatermark = 0;
            this.linearIndicatorControl6.Name = "linearIndicatorControl6";
            this.linearIndicatorControl6.Offset = 0;
            this.linearIndicatorControl6.Size = new System.Drawing.Size(20, 84);
            this.linearIndicatorControl6.TabIndex = 39;
            this.linearIndicatorControl6.Value = 100;
            this.linearIndicatorControl6.WatermarkLineColor = System.Drawing.Color.DarkGray;
            // 
            // linearIndicatorControl7
            // 
            this.linearIndicatorControl7.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl7.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl7.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl7.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl7.BarDividersCount = 20;
            this.linearIndicatorControl7.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl7.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "GyroYaw", true));
            this.linearIndicatorControl7.IsVertical = false;
            this.linearIndicatorControl7.Location = new System.Drawing.Point(225, 157);
            this.linearIndicatorControl7.Max = 500;
            this.linearIndicatorControl7.MaxWaterMark = 0;
            this.linearIndicatorControl7.Min = -500;
            this.linearIndicatorControl7.MinWatermark = 0;
            this.linearIndicatorControl7.Name = "linearIndicatorControl7";
            this.linearIndicatorControl7.Offset = 0;
            this.linearIndicatorControl7.Size = new System.Drawing.Size(109, 20);
            this.linearIndicatorControl7.TabIndex = 40;
            this.linearIndicatorControl7.Value = 100;
            this.linearIndicatorControl7.WatermarkLineColor = System.Drawing.Color.DarkGray;
            // 
            // linearIndicatorControl8
            // 
            this.linearIndicatorControl8.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl8.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl8.BarBorderColor = System.Drawing.Color.Maroon;
            this.linearIndicatorControl8.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl8.BarDividersCount = 20;
            this.linearIndicatorControl8.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl8.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "AccelZ", true));
            this.linearIndicatorControl8.IsVertical = true;
            this.linearIndicatorControl8.Location = new System.Drawing.Point(353, 21);
            this.linearIndicatorControl8.Max = 500;
            this.linearIndicatorControl8.MaxWaterMark = 0;
            this.linearIndicatorControl8.Min = -500;
            this.linearIndicatorControl8.MinWatermark = 0;
            this.linearIndicatorControl8.Name = "linearIndicatorControl8";
            this.linearIndicatorControl8.Offset = 0;
            this.linearIndicatorControl8.Size = new System.Drawing.Size(20, 82);
            this.linearIndicatorControl8.TabIndex = 41;
            this.linearIndicatorControl8.Value = 100;
            this.linearIndicatorControl8.WatermarkLineColor = System.Drawing.Color.DarkGray;
            // 
            // FlightDataView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.linearIndicatorControl8);
            this.Controls.Add(this.linearIndicatorControl7);
            this.Controls.Add(this.linearIndicatorControl6);
            this.Controls.Add(this.linearIndicatorControl5);
            this.Controls.Add(this.linearIndicatorControl4);
            this.Controls.Add(this.linearIndicatorControl3);
            this.Controls.Add(this.linearIndicatorControl2);
            this.Controls.Add(this.linearIndicatorControl1);
            this.Controls.Add(this.indicatorRollAccel);
            this.Controls.Add(this.indicatorRollGyro);
            this.Controls.Add(this.textBox10);
            this.Controls.Add(this.textBox9);
            this.Controls.Add(this.textBox8);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.textBox6);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.textBox7);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.textBox5);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.textBox4);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.textBox3);
            this.Controls.Add(this.textBox2);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.DoubleBuffered = true;
            this.Name = "FlightDataView";
            this.Size = new System.Drawing.Size(402, 275);
            ((System.ComponentModel.ISupportInitialize)(this.FlightDataVmBindingSource)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.BindingSource FlightDataVmBindingSource;



        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox textBox2;
        private System.Windows.Forms.TextBox textBox3;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox textBox4;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox textBox5;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox textBox6;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox textBox7;
        private System.Windows.Forms.TextBox textBox8;
        private System.Windows.Forms.TextBox textBox9;
        private System.Windows.Forms.TextBox textBox10;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl indicatorRollGyro;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl indicatorRollAccel;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl1;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl2;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl3;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl4;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl5;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl6;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl7;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl8;
    }
}

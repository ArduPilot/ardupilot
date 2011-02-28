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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(FlightDataView));
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
            this.linearIndicatorControl7 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.linearIndicatorControl8 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.textBox11 = new System.Windows.Forms.TextBox();
            this.textBox12 = new System.Windows.Forms.TextBox();
            this.textBox13 = new System.Windows.Forms.TextBox();
            this.button2 = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.cirularIndicatorControl1 = new ArducopterConfigurator.Views.controls.CirularIndicatorControl();
            this.cirularIndicatorControl2 = new ArducopterConfigurator.Views.controls.CirularIndicatorControl();
            this.cirularIndicatorControl3 = new ArducopterConfigurator.Views.controls.CirularIndicatorControl();
            this.cirularIndicatorControl4 = new ArducopterConfigurator.Views.controls.CirularIndicatorControl();
            this.cirularIndicatorControl5 = new ArducopterConfigurator.Views.controls.CirularIndicatorControl();
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
            this.label1 = new System.Windows.Forms.Label();
            this.textBox14 = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.compassControl1 = new ArducopterConfigurator.Views.controls.CompassControl();
            ((System.ComponentModel.ISupportInitialize)(this.FlightDataVmBindingSource)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            this.SuspendLayout();
            // 
            // textBox1
            // 
            this.textBox1.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox1.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "MotorLeft", true));
            this.textBox1.DataBindings.Add(new System.Windows.Forms.Binding("Visible", this.FlightDataVmBindingSource, "IsArmed", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.textBox1.Enabled = false;
            this.textBox1.Location = new System.Drawing.Point(99, 162);
            this.textBox1.Name = "textBox1";
            this.textBox1.ReadOnly = true;
            this.textBox1.Size = new System.Drawing.Size(38, 13);
            this.textBox1.TabIndex = 8;
            // 
            // FlightDataVmBindingSource
            // 
            this.FlightDataVmBindingSource.DataSource = typeof(ArducopterConfigurator.PresentationModels.SensorsVm);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(82, 48);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(15, 13);
            this.label5.TabIndex = 10;
            this.label5.Text = "G";
            // 
            // textBox2
            // 
            this.textBox2.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox2.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "GyroRoll", true));
            this.textBox2.Enabled = false;
            this.textBox2.Location = new System.Drawing.Point(217, 48);
            this.textBox2.Name = "textBox2";
            this.textBox2.ReadOnly = true;
            this.textBox2.Size = new System.Drawing.Size(35, 13);
            this.textBox2.TabIndex = 12;
            // 
            // textBox3
            // 
            this.textBox3.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox3.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "AccelRoll", true, System.Windows.Forms.DataSourceUpdateMode.Never, null, "N0"));
            this.textBox3.Enabled = false;
            this.textBox3.Location = new System.Drawing.Point(217, 29);
            this.textBox3.Name = "textBox3";
            this.textBox3.ReadOnly = true;
            this.textBox3.Size = new System.Drawing.Size(35, 13);
            this.textBox3.TabIndex = 13;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(82, 28);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(14, 13);
            this.label6.TabIndex = 15;
            this.label6.Text = "A";
            // 
            // textBox4
            // 
            this.textBox4.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox4.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "GyroPitch", true, System.Windows.Forms.DataSourceUpdateMode.Never, null, "N0"));
            this.textBox4.Enabled = false;
            this.textBox4.Location = new System.Drawing.Point(65, 171);
            this.textBox4.Name = "textBox4";
            this.textBox4.ReadOnly = true;
            this.textBox4.Size = new System.Drawing.Size(35, 13);
            this.textBox4.TabIndex = 17;
            this.textBox4.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(48, 67);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(14, 13);
            this.label7.TabIndex = 18;
            this.label7.Text = "A";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(68, 67);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(15, 13);
            this.label8.TabIndex = 21;
            this.label8.Text = "G";
            // 
            // textBox5
            // 
            this.textBox5.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox5.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "AccelPitch", true, System.Windows.Forms.DataSourceUpdateMode.Never, null, "N0"));
            this.textBox5.Enabled = false;
            this.textBox5.Location = new System.Drawing.Point(27, 171);
            this.textBox5.Name = "textBox5";
            this.textBox5.ReadOnly = true;
            this.textBox5.Size = new System.Drawing.Size(35, 13);
            this.textBox5.TabIndex = 20;
            this.textBox5.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(43, 221);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(53, 13);
            this.label9.TabIndex = 27;
            this.label9.Text = "Yaw Gyro";
            // 
            // textBox6
            // 
            this.textBox6.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox6.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "GyroYaw", true, System.Windows.Forms.DataSourceUpdateMode.Never, null, "N0"));
            this.textBox6.Enabled = false;
            this.textBox6.Location = new System.Drawing.Point(216, 222);
            this.textBox6.Name = "textBox6";
            this.textBox6.ReadOnly = true;
            this.textBox6.Size = new System.Drawing.Size(35, 13);
            this.textBox6.TabIndex = 26;
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(221, 67);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(44, 13);
            this.label10.TabIndex = 24;
            this.label10.Text = "Accel Z";
            // 
            // textBox7
            // 
            this.textBox7.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox7.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "AccelZ", true, System.Windows.Forms.DataSourceUpdateMode.Never, null, "N0"));
            this.textBox7.Enabled = false;
            this.textBox7.Location = new System.Drawing.Point(230, 171);
            this.textBox7.Name = "textBox7";
            this.textBox7.ReadOnly = true;
            this.textBox7.Size = new System.Drawing.Size(35, 13);
            this.textBox7.TabIndex = 23;
            this.textBox7.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // textBox8
            // 
            this.textBox8.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox8.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "MotorFront", true));
            this.textBox8.DataBindings.Add(new System.Windows.Forms.Binding("Visible", this.FlightDataVmBindingSource, "IsArmed", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.textBox8.Enabled = false;
            this.textBox8.Location = new System.Drawing.Point(182, 83);
            this.textBox8.Name = "textBox8";
            this.textBox8.ReadOnly = true;
            this.textBox8.Size = new System.Drawing.Size(38, 13);
            this.textBox8.TabIndex = 29;
            // 
            // textBox9
            // 
            this.textBox9.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox9.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "MotorRear", true));
            this.textBox9.DataBindings.Add(new System.Windows.Forms.Binding("Visible", this.FlightDataVmBindingSource, "IsArmed", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.textBox9.Enabled = false;
            this.textBox9.Location = new System.Drawing.Point(107, 192);
            this.textBox9.Name = "textBox9";
            this.textBox9.ReadOnly = true;
            this.textBox9.Size = new System.Drawing.Size(32, 13);
            this.textBox9.TabIndex = 30;
            // 
            // textBox10
            // 
            this.textBox10.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox10.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "MotorRight", true));
            this.textBox10.DataBindings.Add(new System.Windows.Forms.Binding("Visible", this.FlightDataVmBindingSource, "IsArmed", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.textBox10.Enabled = false;
            this.textBox10.Location = new System.Drawing.Point(186, 163);
            this.textBox10.Name = "textBox10";
            this.textBox10.ReadOnly = true;
            this.textBox10.Size = new System.Drawing.Size(38, 13);
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
            this.indicatorRollGyro.Location = new System.Drawing.Point(102, 48);
            this.indicatorRollGyro.Max = 500;
            this.indicatorRollGyro.MaxWaterMark = 0;
            this.indicatorRollGyro.Min = -500;
            this.indicatorRollGyro.MinWatermark = 0;
            this.indicatorRollGyro.Name = "indicatorRollGyro";
            this.indicatorRollGyro.Offset = 0;
            this.indicatorRollGyro.Size = new System.Drawing.Size(109, 14);
            this.indicatorRollGyro.TabIndex = 32;
            this.toolTip1.SetToolTip(this.indicatorRollGyro, "Roll Gyro");
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
            this.indicatorRollAccel.Location = new System.Drawing.Point(102, 28);
            this.indicatorRollAccel.Max = 500;
            this.indicatorRollAccel.MaxWaterMark = 0;
            this.indicatorRollAccel.Min = -500;
            this.indicatorRollAccel.MinWatermark = 0;
            this.indicatorRollAccel.Name = "indicatorRollAccel";
            this.indicatorRollAccel.Offset = 0;
            this.indicatorRollAccel.Size = new System.Drawing.Size(109, 14);
            this.indicatorRollAccel.TabIndex = 33;
            this.toolTip1.SetToolTip(this.indicatorRollAccel, "Roll Accelerometer");
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
            this.linearIndicatorControl1.Location = new System.Drawing.Point(66, 83);
            this.linearIndicatorControl1.Max = 500;
            this.linearIndicatorControl1.MaxWaterMark = 0;
            this.linearIndicatorControl1.Min = -500;
            this.linearIndicatorControl1.MinWatermark = 0;
            this.linearIndicatorControl1.Name = "linearIndicatorControl1";
            this.linearIndicatorControl1.Offset = 0;
            this.linearIndicatorControl1.Size = new System.Drawing.Size(14, 82);
            this.linearIndicatorControl1.TabIndex = 34;
            this.toolTip1.SetToolTip(this.linearIndicatorControl1, "Pitch Gyro");
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
            this.linearIndicatorControl2.Location = new System.Drawing.Point(48, 83);
            this.linearIndicatorControl2.Max = 500;
            this.linearIndicatorControl2.MaxWaterMark = 0;
            this.linearIndicatorControl2.Min = -500;
            this.linearIndicatorControl2.MinWatermark = 0;
            this.linearIndicatorControl2.Name = "linearIndicatorControl2";
            this.linearIndicatorControl2.Offset = 0;
            this.linearIndicatorControl2.Size = new System.Drawing.Size(14, 82);
            this.linearIndicatorControl2.TabIndex = 35;
            this.toolTip1.SetToolTip(this.linearIndicatorControl2, "Pitch Accelerometer");
            this.linearIndicatorControl2.Value = 100;
            this.linearIndicatorControl2.WatermarkLineColor = System.Drawing.Color.Red;
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
            this.linearIndicatorControl7.Location = new System.Drawing.Point(102, 221);
            this.linearIndicatorControl7.Max = 500;
            this.linearIndicatorControl7.MaxWaterMark = 0;
            this.linearIndicatorControl7.Min = -500;
            this.linearIndicatorControl7.MinWatermark = 0;
            this.linearIndicatorControl7.Name = "linearIndicatorControl7";
            this.linearIndicatorControl7.Offset = 0;
            this.linearIndicatorControl7.Size = new System.Drawing.Size(109, 14);
            this.linearIndicatorControl7.TabIndex = 40;
            this.toolTip1.SetToolTip(this.linearIndicatorControl7, "Yaw Gyro");
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
            this.linearIndicatorControl8.Location = new System.Drawing.Point(241, 83);
            this.linearIndicatorControl8.Max = 500;
            this.linearIndicatorControl8.MaxWaterMark = 0;
            this.linearIndicatorControl8.Min = -500;
            this.linearIndicatorControl8.MinWatermark = 0;
            this.linearIndicatorControl8.Name = "linearIndicatorControl8";
            this.linearIndicatorControl8.Offset = 0;
            this.linearIndicatorControl8.Size = new System.Drawing.Size(14, 82);
            this.linearIndicatorControl8.TabIndex = 41;
            this.toolTip1.SetToolTip(this.linearIndicatorControl8, "Z Accelerometer");
            this.linearIndicatorControl8.Value = 100;
            this.linearIndicatorControl8.WatermarkLineColor = System.Drawing.Color.DarkGray;
            // 
            // textBox11
            // 
            this.textBox11.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.textBox11.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "AccelRollOffset", true));
            this.textBox11.Location = new System.Drawing.Point(139, 7);
            this.textBox11.Name = "textBox11";
            this.textBox11.ReadOnly = true;
            this.textBox11.Size = new System.Drawing.Size(35, 20);
            this.textBox11.TabIndex = 42;
            this.textBox11.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.toolTip1.SetToolTip(this.textBox11, "Roll Accelerometer Calibration Offset");
            // 
            // textBox12
            // 
            this.textBox12.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.textBox12.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "AccelZOffset", true));
            this.textBox12.Location = new System.Drawing.Point(260, 108);
            this.textBox12.Name = "textBox12";
            this.textBox12.ReadOnly = true;
            this.textBox12.Size = new System.Drawing.Size(34, 20);
            this.textBox12.TabIndex = 43;
            this.textBox12.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.toolTip1.SetToolTip(this.textBox12, "Z Accelerometer Calibration Offset");
            // 
            // textBox13
            // 
            this.textBox13.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.textBox13.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "AccelPitchOffset", true));
            this.textBox13.Location = new System.Drawing.Point(8, 108);
            this.textBox13.Name = "textBox13";
            this.textBox13.ReadOnly = true;
            this.textBox13.Size = new System.Drawing.Size(34, 20);
            this.textBox13.TabIndex = 44;
            this.textBox13.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.toolTip1.SetToolTip(this.textBox13, "Pictch Accelerometer Calibration Offset");
            // 
            // button2
            // 
            this.button2.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.FlightDataVmBindingSource, "RefreshCalibrationOffsetsCommand", true));
            this.button2.Image = ((System.Drawing.Image)(resources.GetObject("button2.Image")));
            this.button2.Location = new System.Drawing.Point(304, 139);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(26, 26);
            this.button2.TabIndex = 46;
            this.button2.UseVisualStyleBackColor = true;
            // 
            // button1
            // 
            this.button1.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.FlightDataVmBindingSource, "UpdateCalibrationOffsetsCommand", true));
            this.button1.Image = ((System.Drawing.Image)(resources.GetObject("button1.Image")));
            this.button1.Location = new System.Drawing.Point(304, 110);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(26, 26);
            this.button1.TabIndex = 45;
            this.button1.UseVisualStyleBackColor = true;
            // 
            // button3
            // 
            this.button3.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.FlightDataVmBindingSource, "CalculateCalibrationOffsetsCommand", true));
            this.button3.Image = ((System.Drawing.Image)(resources.GetObject("button3.Image")));
            this.button3.Location = new System.Drawing.Point(304, 81);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(26, 26);
            this.button3.TabIndex = 47;
            this.toolTip1.SetToolTip(this.button3, "Calibrate Accelerometers");
            this.button3.UseVisualStyleBackColor = true;
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(336, 88);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(78, 13);
            this.label11.TabIndex = 48;
            this.label11.Text = "Begin Calibrate";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(336, 117);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(84, 13);
            this.label12.TabIndex = 49;
            this.label12.Text = "Save Calibration";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(336, 148);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(96, 13);
            this.label13.TabIndex = 50;
            this.label13.Text = "Refresh Calibration";
            // 
            // pictureBox1
            // 
            this.pictureBox1.BackColor = System.Drawing.Color.DimGray;
            this.pictureBox1.Location = new System.Drawing.Point(117, 138);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(89, 6);
            this.pictureBox1.TabIndex = 51;
            this.pictureBox1.TabStop = false;
            // 
            // pictureBox2
            // 
            this.pictureBox2.BackColor = System.Drawing.Color.DimGray;
            this.pictureBox2.Location = new System.Drawing.Point(155, 102);
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.Size = new System.Drawing.Size(6, 67);
            this.pictureBox2.TabIndex = 52;
            this.pictureBox2.TabStop = false;
            // 
            // cirularIndicatorControl1
            // 
            this.cirularIndicatorControl1.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.cirularIndicatorControl1.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.cirularIndicatorControl1.BarBorderColor = System.Drawing.Color.DarkGray;
            this.cirularIndicatorControl1.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.cirularIndicatorControl1.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.cirularIndicatorControl1.IsReveresed = false;
            this.cirularIndicatorControl1.IsVertical = false;
            this.cirularIndicatorControl1.Location = new System.Drawing.Point(120, 115);
            this.cirularIndicatorControl1.Max = 100;
            this.cirularIndicatorControl1.Min = 0;
            this.cirularIndicatorControl1.Name = "cirularIndicatorControl1";
            this.cirularIndicatorControl1.Offset = 0;
            this.cirularIndicatorControl1.Size = new System.Drawing.Size(40, 40);
            this.cirularIndicatorControl1.TabIndex = 53;
            this.cirularIndicatorControl1.Value = 50;
            // 
            // cirularIndicatorControl2
            // 
            this.cirularIndicatorControl2.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.cirularIndicatorControl2.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.cirularIndicatorControl2.BarBorderColor = System.Drawing.Color.DimGray;
            this.cirularIndicatorControl2.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.cirularIndicatorControl2.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(128)))), ((int)(((byte)(128)))));
            this.cirularIndicatorControl2.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorLeft", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.cirularIndicatorControl2.IsReveresed = false;
            this.cirularIndicatorControl2.IsVertical = true;
            this.cirularIndicatorControl2.Location = new System.Drawing.Point(99, 121);
            this.cirularIndicatorControl2.Max = 2000;
            this.cirularIndicatorControl2.Min = 1000;
            this.cirularIndicatorControl2.Name = "cirularIndicatorControl2";
            this.cirularIndicatorControl2.Offset = 0;
            this.cirularIndicatorControl2.Size = new System.Drawing.Size(40, 40);
            this.cirularIndicatorControl2.TabIndex = 53;
            this.cirularIndicatorControl2.Value = 1500;
            // 
            // cirularIndicatorControl3
            // 
            this.cirularIndicatorControl3.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.cirularIndicatorControl3.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.cirularIndicatorControl3.BarBorderColor = System.Drawing.Color.DimGray;
            this.cirularIndicatorControl3.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.cirularIndicatorControl3.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(128)))), ((int)(((byte)(128)))));
            this.cirularIndicatorControl3.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorRight", true));
            this.cirularIndicatorControl3.IsReveresed = false;
            this.cirularIndicatorControl3.IsVertical = true;
            this.cirularIndicatorControl3.Location = new System.Drawing.Point(182, 121);
            this.cirularIndicatorControl3.Max = 2000;
            this.cirularIndicatorControl3.Min = 1000;
            this.cirularIndicatorControl3.Name = "cirularIndicatorControl3";
            this.cirularIndicatorControl3.Offset = 0;
            this.cirularIndicatorControl3.Size = new System.Drawing.Size(40, 40);
            this.cirularIndicatorControl3.TabIndex = 54;
            this.cirularIndicatorControl3.Value = 1500;
            // 
            // cirularIndicatorControl4
            // 
            this.cirularIndicatorControl4.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.cirularIndicatorControl4.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.cirularIndicatorControl4.BarBorderColor = System.Drawing.Color.DimGray;
            this.cirularIndicatorControl4.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.cirularIndicatorControl4.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(128)))), ((int)(((byte)(128)))));
            this.cirularIndicatorControl4.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorRear", true));
            this.cirularIndicatorControl4.IsReveresed = false;
            this.cirularIndicatorControl4.IsVertical = true;
            this.cirularIndicatorControl4.Location = new System.Drawing.Point(138, 157);
            this.cirularIndicatorControl4.Max = 2000;
            this.cirularIndicatorControl4.Min = 1000;
            this.cirularIndicatorControl4.Name = "cirularIndicatorControl4";
            this.cirularIndicatorControl4.Offset = 0;
            this.cirularIndicatorControl4.Size = new System.Drawing.Size(40, 40);
            this.cirularIndicatorControl4.TabIndex = 55;
            this.cirularIndicatorControl4.Value = 1500;
            // 
            // cirularIndicatorControl5
            // 
            this.cirularIndicatorControl5.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.cirularIndicatorControl5.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.cirularIndicatorControl5.BarBorderColor = System.Drawing.Color.DimGray;
            this.cirularIndicatorControl5.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.cirularIndicatorControl5.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(128)))), ((int)(((byte)(128)))));
            this.cirularIndicatorControl5.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "MotorFront", true));
            this.cirularIndicatorControl5.IsReveresed = false;
            this.cirularIndicatorControl5.IsVertical = true;
            this.cirularIndicatorControl5.Location = new System.Drawing.Point(138, 83);
            this.cirularIndicatorControl5.Max = 2000;
            this.cirularIndicatorControl5.Min = 1000;
            this.cirularIndicatorControl5.Name = "cirularIndicatorControl5";
            this.cirularIndicatorControl5.Offset = 0;
            this.cirularIndicatorControl5.Size = new System.Drawing.Size(40, 40);
            this.cirularIndicatorControl5.TabIndex = 56;
            this.cirularIndicatorControl5.Value = 1500;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.label1.DataBindings.Add(new System.Windows.Forms.Binding("Visible", this.FlightDataVmBindingSource, "IsArmed", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.Color.Red;
            this.label1.Location = new System.Drawing.Point(308, 29);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(59, 19);
            this.label1.TabIndex = 57;
            this.label1.Text = "ARMED";
            // 
            // textBox14
            // 
            this.textBox14.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox14.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.FlightDataVmBindingSource, "CompassHeadingDegrees", true, System.Windows.Forms.DataSourceUpdateMode.OnValidation, null, "N0"));
            this.textBox14.Enabled = false;
            this.textBox14.Location = new System.Drawing.Point(264, 222);
            this.textBox14.Name = "textBox14";
            this.textBox14.ReadOnly = true;
            this.textBox14.Size = new System.Drawing.Size(35, 13);
            this.textBox14.TabIndex = 58;
            this.textBox14.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(255, 205);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(47, 13);
            this.label2.TabIndex = 59;
            this.label2.Text = "Heading";
            // 
            // compassControl1
            // 
            this.compassControl1.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.compassControl1.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.compassControl1.BarBorderColor = System.Drawing.Color.Black;
            this.compassControl1.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.compassControl1.BarLight = System.Drawing.Color.LightBlue;
            this.compassControl1.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.FlightDataVmBindingSource, "CompassHeadingDegrees", true));
            this.compassControl1.Location = new System.Drawing.Point(308, 192);
            this.compassControl1.Name = "compassControl1";
            this.compassControl1.Size = new System.Drawing.Size(50, 50);
            this.compassControl1.TabIndex = 0;
            this.compassControl1.Value = 90;
            // 
            // FlightDataView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.compassControl1);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.textBox14);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.cirularIndicatorControl5);
            this.Controls.Add(this.cirularIndicatorControl4);
            this.Controls.Add(this.cirularIndicatorControl3);
            this.Controls.Add(this.cirularIndicatorControl2);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.button3);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.textBox13);
            this.Controls.Add(this.textBox12);
            this.Controls.Add(this.textBox11);
            this.Controls.Add(this.linearIndicatorControl8);
            this.Controls.Add(this.linearIndicatorControl7);
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
            this.Controls.Add(this.pictureBox1);
            this.Controls.Add(this.pictureBox2);
            this.DoubleBuffered = true;
            this.Name = "FlightDataView";
            this.Size = new System.Drawing.Size(453, 273);
            ((System.ComponentModel.ISupportInitialize)(this.FlightDataVmBindingSource)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.BindingSource FlightDataVmBindingSource;
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
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl7;
        private ArducopterConfigurator.Views.controls.LinearIndicatorControl linearIndicatorControl8;
        private System.Windows.Forms.TextBox textBox11;
        private System.Windows.Forms.TextBox textBox12;
        private System.Windows.Forms.TextBox textBox13;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.PictureBox pictureBox2;
        private ArducopterConfigurator.Views.controls.CirularIndicatorControl cirularIndicatorControl1;
        private ArducopterConfigurator.Views.controls.CirularIndicatorControl cirularIndicatorControl2;
        private ArducopterConfigurator.Views.controls.CirularIndicatorControl cirularIndicatorControl3;
        private ArducopterConfigurator.Views.controls.CirularIndicatorControl cirularIndicatorControl4;
        private ArducopterConfigurator.Views.controls.CirularIndicatorControl cirularIndicatorControl5;
        private System.Windows.Forms.ToolTip toolTip1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox textBox14;
        private System.Windows.Forms.Label label2;
        private ArducopterConfigurator.Views.controls.CompassControl compassControl1;
    }
}

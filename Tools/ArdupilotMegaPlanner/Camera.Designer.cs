namespace ArdupilotMega
{
    partial class Camera
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
            this.num_agl = new System.Windows.Forms.NumericUpDown();
            this.label2 = new System.Windows.Forms.Label();
            this.num_focallength = new System.Windows.Forms.NumericUpDown();
            this.TXT_fovH = new System.Windows.Forms.TextBox();
            this.TXT_fovV = new System.Windows.Forms.TextBox();
            this.TXT_fovAV = new System.Windows.Forms.TextBox();
            this.TXT_fovAH = new System.Windows.Forms.TextBox();
            this.TXT_cmpixel = new System.Windows.Forms.TextBox();
            this.TXT_imgwidth = new System.Windows.Forms.TextBox();
            this.TXT_imgheight = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.TXT_sensheight = new System.Windows.Forms.TextBox();
            this.TXT_senswidth = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.num_overlap = new System.Windows.Forms.NumericUpDown();
            this.label15 = new System.Windows.Forms.Label();
            this.num_sidelap = new System.Windows.Forms.NumericUpDown();
            this.CHK_camdirection = new System.Windows.Forms.CheckBox();
            this.label9 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.TXT_distacflphotos = new System.Windows.Forms.TextBox();
            this.TXT_distflphotos = new System.Windows.Forms.TextBox();
            this.CMB_camera = new System.Windows.Forms.ComboBox();
            this.BUT_save = new ArdupilotMega.MyButton();
            ((System.ComponentModel.ISupportInitialize)(this.num_agl)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_focallength)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_overlap)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_sidelap)).BeginInit();
            this.SuspendLayout();
            // 
            // num_agl
            // 
            this.num_agl.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.num_agl.Location = new System.Drawing.Point(12, 38);
            this.num_agl.Maximum = new decimal(new int[] {
            10000,
            0,
            0,
            0});
            this.num_agl.Name = "num_agl";
            this.num_agl.Size = new System.Drawing.Size(64, 20);
            this.num_agl.TabIndex = 1;
            this.num_agl.Value = new decimal(new int[] {
            200,
            0,
            0,
            0});
            this.num_agl.ValueChanged += new System.EventHandler(this.num_agl_ValueChanged);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(82, 40);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(72, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "Height m (agl)";
            // 
            // num_focallength
            // 
            this.num_focallength.DecimalPlaces = 1;
            this.num_focallength.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.num_focallength.Location = new System.Drawing.Point(12, 64);
            this.num_focallength.Maximum = new decimal(new int[] {
            180,
            0,
            0,
            0});
            this.num_focallength.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.num_focallength.Name = "num_focallength";
            this.num_focallength.Size = new System.Drawing.Size(64, 20);
            this.num_focallength.TabIndex = 6;
            this.num_focallength.Value = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.num_focallength.ValueChanged += new System.EventHandler(this.num_focallength_ValueChanged);
            // 
            // TXT_fovH
            // 
            this.TXT_fovH.Location = new System.Drawing.Point(361, 12);
            this.TXT_fovH.Name = "TXT_fovH";
            this.TXT_fovH.Size = new System.Drawing.Size(100, 20);
            this.TXT_fovH.TabIndex = 10;
            // 
            // TXT_fovV
            // 
            this.TXT_fovV.Location = new System.Drawing.Point(361, 39);
            this.TXT_fovV.Name = "TXT_fovV";
            this.TXT_fovV.Size = new System.Drawing.Size(100, 20);
            this.TXT_fovV.TabIndex = 11;
            // 
            // TXT_fovAV
            // 
            this.TXT_fovAV.Location = new System.Drawing.Point(361, 92);
            this.TXT_fovAV.Name = "TXT_fovAV";
            this.TXT_fovAV.Size = new System.Drawing.Size(100, 20);
            this.TXT_fovAV.TabIndex = 14;
            // 
            // TXT_fovAH
            // 
            this.TXT_fovAH.Location = new System.Drawing.Point(361, 65);
            this.TXT_fovAH.Name = "TXT_fovAH";
            this.TXT_fovAH.Size = new System.Drawing.Size(100, 20);
            this.TXT_fovAH.TabIndex = 13;
            // 
            // TXT_cmpixel
            // 
            this.TXT_cmpixel.Location = new System.Drawing.Point(361, 118);
            this.TXT_cmpixel.Name = "TXT_cmpixel";
            this.TXT_cmpixel.Size = new System.Drawing.Size(100, 20);
            this.TXT_cmpixel.TabIndex = 16;
            // 
            // TXT_imgwidth
            // 
            this.TXT_imgwidth.Location = new System.Drawing.Point(12, 90);
            this.TXT_imgwidth.Name = "TXT_imgwidth";
            this.TXT_imgwidth.Size = new System.Drawing.Size(64, 20);
            this.TXT_imgwidth.TabIndex = 17;
            this.TXT_imgwidth.Text = "4608";
            this.TXT_imgwidth.TextChanged += new System.EventHandler(this.TXT_imgwidth_TextChanged);
            // 
            // TXT_imgheight
            // 
            this.TXT_imgheight.Location = new System.Drawing.Point(12, 116);
            this.TXT_imgheight.Name = "TXT_imgheight";
            this.TXT_imgheight.Size = new System.Drawing.Size(64, 20);
            this.TXT_imgheight.TabIndex = 18;
            this.TXT_imgheight.Text = "3456";
            this.TXT_imgheight.TextChanged += new System.EventHandler(this.TXT_imgheight_TextChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(82, 71);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(69, 13);
            this.label1.TabIndex = 19;
            this.label1.Text = "Focal Length";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(298, 19);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(56, 13);
            this.label6.TabIndex = 21;
            this.label6.Text = "FOV H (m)";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(299, 72);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(45, 13);
            this.label7.TabIndex = 22;
            this.label7.Text = "Angle H";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(300, 99);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(44, 13);
            this.label8.TabIndex = 23;
            this.label8.Text = "Angle V";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(299, 46);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(55, 13);
            this.label10.TabIndex = 25;
            this.label10.Text = "FOV V (m)";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(299, 125);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(50, 13);
            this.label12.TabIndex = 27;
            this.label12.Text = "CM/Pixel";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(82, 93);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(60, 13);
            this.label13.TabIndex = 28;
            this.label13.Text = "Pixel Width";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(82, 119);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(63, 13);
            this.label14.TabIndex = 29;
            this.label14.Text = "Pixel Height";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(82, 171);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(74, 13);
            this.label3.TabIndex = 33;
            this.label3.Text = "Sensor Height";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(82, 145);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(71, 13);
            this.label4.TabIndex = 32;
            this.label4.Text = "Sensor Width";
            // 
            // TXT_sensheight
            // 
            this.TXT_sensheight.Location = new System.Drawing.Point(12, 168);
            this.TXT_sensheight.Name = "TXT_sensheight";
            this.TXT_sensheight.Size = new System.Drawing.Size(64, 20);
            this.TXT_sensheight.TabIndex = 31;
            this.TXT_sensheight.Text = "4.62";
            this.TXT_sensheight.TextChanged += new System.EventHandler(this.TXT_sensheight_TextChanged);
            // 
            // TXT_senswidth
            // 
            this.TXT_senswidth.Location = new System.Drawing.Point(12, 142);
            this.TXT_senswidth.Name = "TXT_senswidth";
            this.TXT_senswidth.Size = new System.Drawing.Size(64, 20);
            this.TXT_senswidth.TabIndex = 30;
            this.TXT_senswidth.Text = "6.16";
            this.TXT_senswidth.TextChanged += new System.EventHandler(this.TXT_senswidth_TextChanged);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(82, 201);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(44, 13);
            this.label5.TabIndex = 35;
            this.label5.Text = "Overlap";
            // 
            // num_overlap
            // 
            this.num_overlap.DecimalPlaces = 1;
            this.num_overlap.Location = new System.Drawing.Point(12, 194);
            this.num_overlap.Name = "num_overlap";
            this.num_overlap.Size = new System.Drawing.Size(64, 20);
            this.num_overlap.TabIndex = 34;
            this.num_overlap.Value = new decimal(new int[] {
            60,
            0,
            0,
            0});
            this.num_overlap.ValueChanged += new System.EventHandler(this.num_overlap_ValueChanged);
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.Location = new System.Drawing.Point(82, 227);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(42, 13);
            this.label15.TabIndex = 37;
            this.label15.Text = "Sidelap";
            // 
            // num_sidelap
            // 
            this.num_sidelap.DecimalPlaces = 1;
            this.num_sidelap.Location = new System.Drawing.Point(12, 220);
            this.num_sidelap.Name = "num_sidelap";
            this.num_sidelap.Size = new System.Drawing.Size(64, 20);
            this.num_sidelap.TabIndex = 36;
            this.num_sidelap.Value = new decimal(new int[] {
            30,
            0,
            0,
            0});
            this.num_sidelap.ValueChanged += new System.EventHandler(this.num_sidelap_ValueChanged);
            // 
            // CHK_camdirection
            // 
            this.CHK_camdirection.AutoSize = true;
            this.CHK_camdirection.Checked = true;
            this.CHK_camdirection.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHK_camdirection.Location = new System.Drawing.Point(13, 247);
            this.CHK_camdirection.Name = "CHK_camdirection";
            this.CHK_camdirection.Size = new System.Drawing.Size(150, 17);
            this.CHK_camdirection.TabIndex = 38;
            this.CHK_camdirection.Text = "Camera top facing forward";
            this.CHK_camdirection.UseVisualStyleBackColor = true;
            this.CHK_camdirection.CheckedChanged += new System.EventHandler(this.CHK_camdirection_CheckedChanged);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(261, 198);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(86, 13);
            this.label9.TabIndex = 42;
            this.label9.Text = "Across Flight line";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(261, 171);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(94, 13);
            this.label11.TabIndex = 41;
            this.label11.Text = "Flight line distance";
            // 
            // TXT_distacflphotos
            // 
            this.TXT_distacflphotos.Location = new System.Drawing.Point(361, 195);
            this.TXT_distacflphotos.Name = "TXT_distacflphotos";
            this.TXT_distacflphotos.Size = new System.Drawing.Size(100, 20);
            this.TXT_distacflphotos.TabIndex = 40;
            // 
            // TXT_distflphotos
            // 
            this.TXT_distflphotos.Location = new System.Drawing.Point(361, 168);
            this.TXT_distflphotos.Name = "TXT_distflphotos";
            this.TXT_distflphotos.Size = new System.Drawing.Size(100, 20);
            this.TXT_distflphotos.TabIndex = 39;
            // 
            // CMB_camera
            // 
            this.CMB_camera.FormattingEnabled = true;
            this.CMB_camera.Location = new System.Drawing.Point(13, 13);
            this.CMB_camera.Name = "CMB_camera";
            this.CMB_camera.Size = new System.Drawing.Size(143, 21);
            this.CMB_camera.TabIndex = 43;
            this.CMB_camera.SelectedIndexChanged += new System.EventHandler(this.CMB_camera_SelectedIndexChanged);
            // 
            // BUT_save
            // 
            this.BUT_save.Location = new System.Drawing.Point(163, 10);
            this.BUT_save.Name = "BUT_save";
            this.BUT_save.Size = new System.Drawing.Size(75, 23);
            this.BUT_save.TabIndex = 44;
            this.BUT_save.Text = "Save";
            this.BUT_save.UseVisualStyleBackColor = true;
            this.BUT_save.Click += new System.EventHandler(this.BUT_save_Click);
            // 
            // Camera
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(473, 275);
            this.Controls.Add(this.BUT_save);
            this.Controls.Add(this.CMB_camera);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.TXT_distacflphotos);
            this.Controls.Add(this.TXT_distflphotos);
            this.Controls.Add(this.CHK_camdirection);
            this.Controls.Add(this.label15);
            this.Controls.Add(this.num_sidelap);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.num_overlap);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.TXT_sensheight);
            this.Controls.Add(this.TXT_senswidth);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.TXT_imgheight);
            this.Controls.Add(this.TXT_imgwidth);
            this.Controls.Add(this.TXT_cmpixel);
            this.Controls.Add(this.TXT_fovAV);
            this.Controls.Add(this.TXT_fovAH);
            this.Controls.Add(this.TXT_fovV);
            this.Controls.Add(this.TXT_fovH);
            this.Controls.Add(this.num_focallength);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.num_agl);
            this.Name = "Camera";
            this.Text = "Camera";
            this.Load += new System.EventHandler(this.Camera_Load);
            ((System.ComponentModel.ISupportInitialize)(this.num_agl)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_focallength)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_overlap)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_sidelap)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.NumericUpDown num_agl;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.NumericUpDown num_focallength;
        private System.Windows.Forms.TextBox TXT_fovH;
        private System.Windows.Forms.TextBox TXT_fovV;
        private System.Windows.Forms.TextBox TXT_fovAV;
        private System.Windows.Forms.TextBox TXT_fovAH;
        private System.Windows.Forms.TextBox TXT_cmpixel;
        private System.Windows.Forms.TextBox TXT_imgwidth;
        private System.Windows.Forms.TextBox TXT_imgheight;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox TXT_sensheight;
        private System.Windows.Forms.TextBox TXT_senswidth;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.NumericUpDown num_overlap;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.NumericUpDown num_sidelap;
        private System.Windows.Forms.CheckBox CHK_camdirection;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox TXT_distacflphotos;
        private System.Windows.Forms.TextBox TXT_distflphotos;
        private System.Windows.Forms.ComboBox CMB_camera;
        private MyButton BUT_save;
    }
}
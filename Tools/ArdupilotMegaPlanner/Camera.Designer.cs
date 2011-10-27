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
            this.num_megapixel = new System.Windows.Forms.NumericUpDown();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.num_focallength = new System.Windows.Forms.NumericUpDown();
            this.label4 = new System.Windows.Forms.Label();
            this.num_focalmultip = new System.Windows.Forms.NumericUpDown();
            this.TXT_fovH = new System.Windows.Forms.TextBox();
            this.TXT_fovV = new System.Windows.Forms.TextBox();
            this.TXT_fovD = new System.Windows.Forms.TextBox();
            this.TXT_fovAD = new System.Windows.Forms.TextBox();
            this.TXT_fovAV = new System.Windows.Forms.TextBox();
            this.TXT_fovAH = new System.Windows.Forms.TextBox();
            this.TXT_cmpixel = new System.Windows.Forms.TextBox();
            this.TXT_imgwidth = new System.Windows.Forms.TextBox();
            this.TXT_imgheight = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.num_agl)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_megapixel)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_focallength)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_focalmultip)).BeginInit();
            this.SuspendLayout();
            // 
            // num_agl
            // 
            this.num_agl.Location = new System.Drawing.Point(12, 12);
            this.num_agl.Maximum = new decimal(new int[] {
            10000,
            0,
            0,
            0});
            this.num_agl.Name = "num_agl";
            this.num_agl.Size = new System.Drawing.Size(120, 20);
            this.num_agl.TabIndex = 1;
            this.num_agl.Value = new decimal(new int[] {
            200,
            0,
            0,
            0});
            this.num_agl.ValueChanged += new System.EventHandler(this.num_agl_ValueChanged);
            // 
            // num_megapixel
            // 
            this.num_megapixel.DecimalPlaces = 1;
            this.num_megapixel.Increment = new decimal(new int[] {
            5,
            0,
            0,
            65536});
            this.num_megapixel.Location = new System.Drawing.Point(13, 38);
            this.num_megapixel.Name = "num_megapixel";
            this.num_megapixel.Size = new System.Drawing.Size(120, 20);
            this.num_megapixel.TabIndex = 2;
            this.num_megapixel.Value = new decimal(new int[] {
            8,
            0,
            0,
            0});
            this.num_megapixel.ValueChanged += new System.EventHandler(this.num_megapixel_ValueChanged);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(140, 19);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(72, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "Height m (agl)";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(140, 45);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(95, 13);
            this.label3.TabIndex = 5;
            this.label3.Text = "MegaPixel Camera";
            // 
            // num_focallength
            // 
            this.num_focallength.DecimalPlaces = 1;
            this.num_focallength.Increment = new decimal(new int[] {
            5,
            0,
            0,
            65536});
            this.num_focallength.Location = new System.Drawing.Point(245, 12);
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
            this.num_focallength.Size = new System.Drawing.Size(120, 20);
            this.num_focallength.TabIndex = 6;
            this.num_focallength.Value = new decimal(new int[] {
            35,
            0,
            0,
            0});
            this.num_focallength.ValueChanged += new System.EventHandler(this.num_focallength_ValueChanged);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(242, 72);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(99, 13);
            this.label4.TabIndex = 8;
            this.label4.Text = "2 x tan-1( l / (2 x f) )";
            // 
            // num_focalmultip
            // 
            this.num_focalmultip.DecimalPlaces = 1;
            this.num_focalmultip.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.num_focalmultip.Location = new System.Drawing.Point(245, 38);
            this.num_focalmultip.Maximum = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.num_focalmultip.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.num_focalmultip.Name = "num_focalmultip";
            this.num_focalmultip.Size = new System.Drawing.Size(120, 20);
            this.num_focalmultip.TabIndex = 9;
            this.num_focalmultip.Value = new decimal(new int[] {
            15,
            0,
            0,
            65536});
            this.num_focalmultip.ValueChanged += new System.EventHandler(this.num_focalmultip_ValueChanged);
            // 
            // TXT_fovH
            // 
            this.TXT_fovH.Location = new System.Drawing.Point(72, 138);
            this.TXT_fovH.Name = "TXT_fovH";
            this.TXT_fovH.Size = new System.Drawing.Size(100, 20);
            this.TXT_fovH.TabIndex = 10;
            // 
            // TXT_fovV
            // 
            this.TXT_fovV.Location = new System.Drawing.Point(72, 165);
            this.TXT_fovV.Name = "TXT_fovV";
            this.TXT_fovV.Size = new System.Drawing.Size(100, 20);
            this.TXT_fovV.TabIndex = 11;
            // 
            // TXT_fovD
            // 
            this.TXT_fovD.Location = new System.Drawing.Point(72, 192);
            this.TXT_fovD.Name = "TXT_fovD";
            this.TXT_fovD.Size = new System.Drawing.Size(100, 20);
            this.TXT_fovD.TabIndex = 12;
            // 
            // TXT_fovAD
            // 
            this.TXT_fovAD.Location = new System.Drawing.Point(286, 192);
            this.TXT_fovAD.Name = "TXT_fovAD";
            this.TXT_fovAD.Size = new System.Drawing.Size(100, 20);
            this.TXT_fovAD.TabIndex = 15;
            // 
            // TXT_fovAV
            // 
            this.TXT_fovAV.Location = new System.Drawing.Point(286, 165);
            this.TXT_fovAV.Name = "TXT_fovAV";
            this.TXT_fovAV.Size = new System.Drawing.Size(100, 20);
            this.TXT_fovAV.TabIndex = 14;
            // 
            // TXT_fovAH
            // 
            this.TXT_fovAH.Location = new System.Drawing.Point(286, 138);
            this.TXT_fovAH.Name = "TXT_fovAH";
            this.TXT_fovAH.Size = new System.Drawing.Size(100, 20);
            this.TXT_fovAH.TabIndex = 13;
            // 
            // TXT_cmpixel
            // 
            this.TXT_cmpixel.Location = new System.Drawing.Point(176, 218);
            this.TXT_cmpixel.Name = "TXT_cmpixel";
            this.TXT_cmpixel.Size = new System.Drawing.Size(100, 20);
            this.TXT_cmpixel.TabIndex = 16;
            // 
            // TXT_imgwidth
            // 
            this.TXT_imgwidth.Location = new System.Drawing.Point(93, 244);
            this.TXT_imgwidth.Name = "TXT_imgwidth";
            this.TXT_imgwidth.Size = new System.Drawing.Size(100, 20);
            this.TXT_imgwidth.TabIndex = 17;
            // 
            // TXT_imgheight
            // 
            this.TXT_imgheight.Location = new System.Drawing.Point(255, 244);
            this.TXT_imgheight.Name = "TXT_imgheight";
            this.TXT_imgheight.Size = new System.Drawing.Size(100, 20);
            this.TXT_imgheight.TabIndex = 18;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(371, 19);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(69, 13);
            this.label1.TabIndex = 19;
            this.label1.Text = "Focal Length";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(371, 45);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(88, 13);
            this.label5.TabIndex = 20;
            this.label5.Text = "Frame Muli factor";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(9, 145);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(36, 13);
            this.label6.TabIndex = 21;
            this.label6.Text = "Dist H";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(235, 145);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(45, 13);
            this.label7.TabIndex = 22;
            this.label7.Text = "Angle H";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(236, 172);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(44, 13);
            this.label8.TabIndex = 23;
            this.label8.Text = "Angle V";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(235, 199);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(45, 13);
            this.label9.TabIndex = 24;
            this.label9.Text = "Angle D";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(10, 172);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(35, 13);
            this.label10.TabIndex = 25;
            this.label10.Text = "Dist V";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(10, 199);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(36, 13);
            this.label11.TabIndex = 26;
            this.label11.Text = "Dist D";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(125, 225);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(50, 13);
            this.label12.TabIndex = 27;
            this.label12.Text = "CM/Pixel";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(37, 251);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(39, 13);
            this.label13.TabIndex = 28;
            this.label13.Text = "Pixel X";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(199, 253);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(39, 13);
            this.label14.TabIndex = 29;
            this.label14.Text = "Pixel Y";
            // 
            // Camera
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(473, 275);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.TXT_imgheight);
            this.Controls.Add(this.TXT_imgwidth);
            this.Controls.Add(this.TXT_cmpixel);
            this.Controls.Add(this.TXT_fovAD);
            this.Controls.Add(this.TXT_fovAV);
            this.Controls.Add(this.TXT_fovAH);
            this.Controls.Add(this.TXT_fovD);
            this.Controls.Add(this.TXT_fovV);
            this.Controls.Add(this.TXT_fovH);
            this.Controls.Add(this.num_focalmultip);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.num_focallength);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.num_megapixel);
            this.Controls.Add(this.num_agl);
            this.Name = "Camera";
            this.Text = "Camera";
            ((System.ComponentModel.ISupportInitialize)(this.num_agl)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_megapixel)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_focallength)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.num_focalmultip)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.NumericUpDown num_agl;
        private System.Windows.Forms.NumericUpDown num_megapixel;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.NumericUpDown num_focallength;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.NumericUpDown num_focalmultip;
        private System.Windows.Forms.TextBox TXT_fovH;
        private System.Windows.Forms.TextBox TXT_fovV;
        private System.Windows.Forms.TextBox TXT_fovD;
        private System.Windows.Forms.TextBox TXT_fovAD;
        private System.Windows.Forms.TextBox TXT_fovAV;
        private System.Windows.Forms.TextBox TXT_fovAH;
        private System.Windows.Forms.TextBox TXT_cmpixel;
        private System.Windows.Forms.TextBox TXT_imgwidth;
        private System.Windows.Forms.TextBox TXT_imgheight;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label14;
    }
}
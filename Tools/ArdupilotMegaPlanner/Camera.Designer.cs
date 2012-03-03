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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Camera));
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
            resources.ApplyResources(this.num_agl, "num_agl");
            this.num_agl.Maximum = new decimal(new int[] {
            10000,
            0,
            0,
            0});
            this.num_agl.Name = "num_agl";
            this.num_agl.Value = new decimal(new int[] {
            200,
            0,
            0,
            0});
            this.num_agl.ValueChanged += new System.EventHandler(this.num_agl_ValueChanged);
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // num_focallength
            // 
            this.num_focallength.DecimalPlaces = 1;
            this.num_focallength.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            resources.ApplyResources(this.num_focallength, "num_focallength");
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
            this.num_focallength.Value = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.num_focallength.ValueChanged += new System.EventHandler(this.num_focallength_ValueChanged);
            // 
            // TXT_fovH
            // 
            resources.ApplyResources(this.TXT_fovH, "TXT_fovH");
            this.TXT_fovH.Name = "TXT_fovH";
            // 
            // TXT_fovV
            // 
            resources.ApplyResources(this.TXT_fovV, "TXT_fovV");
            this.TXT_fovV.Name = "TXT_fovV";
            // 
            // TXT_fovAV
            // 
            resources.ApplyResources(this.TXT_fovAV, "TXT_fovAV");
            this.TXT_fovAV.Name = "TXT_fovAV";
            // 
            // TXT_fovAH
            // 
            resources.ApplyResources(this.TXT_fovAH, "TXT_fovAH");
            this.TXT_fovAH.Name = "TXT_fovAH";
            // 
            // TXT_cmpixel
            // 
            resources.ApplyResources(this.TXT_cmpixel, "TXT_cmpixel");
            this.TXT_cmpixel.Name = "TXT_cmpixel";
            // 
            // TXT_imgwidth
            // 
            resources.ApplyResources(this.TXT_imgwidth, "TXT_imgwidth");
            this.TXT_imgwidth.Name = "TXT_imgwidth";
            this.TXT_imgwidth.TextChanged += new System.EventHandler(this.TXT_imgwidth_TextChanged);
            // 
            // TXT_imgheight
            // 
            resources.ApplyResources(this.TXT_imgheight, "TXT_imgheight");
            this.TXT_imgheight.Name = "TXT_imgheight";
            this.TXT_imgheight.TextChanged += new System.EventHandler(this.TXT_imgheight_TextChanged);
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            // 
            // label6
            // 
            resources.ApplyResources(this.label6, "label6");
            this.label6.Name = "label6";
            // 
            // label7
            // 
            resources.ApplyResources(this.label7, "label7");
            this.label7.Name = "label7";
            // 
            // label8
            // 
            resources.ApplyResources(this.label8, "label8");
            this.label8.Name = "label8";
            // 
            // label10
            // 
            resources.ApplyResources(this.label10, "label10");
            this.label10.Name = "label10";
            // 
            // label12
            // 
            resources.ApplyResources(this.label12, "label12");
            this.label12.Name = "label12";
            // 
            // label13
            // 
            resources.ApplyResources(this.label13, "label13");
            this.label13.Name = "label13";
            // 
            // label14
            // 
            resources.ApplyResources(this.label14, "label14");
            this.label14.Name = "label14";
            // 
            // label3
            // 
            resources.ApplyResources(this.label3, "label3");
            this.label3.Name = "label3";
            // 
            // label4
            // 
            resources.ApplyResources(this.label4, "label4");
            this.label4.Name = "label4";
            // 
            // TXT_sensheight
            // 
            resources.ApplyResources(this.TXT_sensheight, "TXT_sensheight");
            this.TXT_sensheight.Name = "TXT_sensheight";
            this.TXT_sensheight.TextChanged += new System.EventHandler(this.TXT_sensheight_TextChanged);
            // 
            // TXT_senswidth
            // 
            resources.ApplyResources(this.TXT_senswidth, "TXT_senswidth");
            this.TXT_senswidth.Name = "TXT_senswidth";
            this.TXT_senswidth.TextChanged += new System.EventHandler(this.TXT_senswidth_TextChanged);
            // 
            // label5
            // 
            resources.ApplyResources(this.label5, "label5");
            this.label5.Name = "label5";
            // 
            // num_overlap
            // 
            this.num_overlap.DecimalPlaces = 1;
            resources.ApplyResources(this.num_overlap, "num_overlap");
            this.num_overlap.Name = "num_overlap";
            this.num_overlap.Value = new decimal(new int[] {
            60,
            0,
            0,
            0});
            this.num_overlap.ValueChanged += new System.EventHandler(this.num_overlap_ValueChanged);
            // 
            // label15
            // 
            resources.ApplyResources(this.label15, "label15");
            this.label15.Name = "label15";
            // 
            // num_sidelap
            // 
            this.num_sidelap.DecimalPlaces = 1;
            resources.ApplyResources(this.num_sidelap, "num_sidelap");
            this.num_sidelap.Name = "num_sidelap";
            this.num_sidelap.Value = new decimal(new int[] {
            30,
            0,
            0,
            0});
            this.num_sidelap.ValueChanged += new System.EventHandler(this.num_sidelap_ValueChanged);
            // 
            // CHK_camdirection
            // 
            resources.ApplyResources(this.CHK_camdirection, "CHK_camdirection");
            this.CHK_camdirection.Checked = true;
            this.CHK_camdirection.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHK_camdirection.Name = "CHK_camdirection";
            this.CHK_camdirection.UseVisualStyleBackColor = true;
            this.CHK_camdirection.CheckedChanged += new System.EventHandler(this.CHK_camdirection_CheckedChanged);
            // 
            // label9
            // 
            resources.ApplyResources(this.label9, "label9");
            this.label9.Name = "label9";
            // 
            // label11
            // 
            resources.ApplyResources(this.label11, "label11");
            this.label11.Name = "label11";
            // 
            // TXT_distacflphotos
            // 
            resources.ApplyResources(this.TXT_distacflphotos, "TXT_distacflphotos");
            this.TXT_distacflphotos.Name = "TXT_distacflphotos";
            // 
            // TXT_distflphotos
            // 
            resources.ApplyResources(this.TXT_distflphotos, "TXT_distflphotos");
            this.TXT_distflphotos.Name = "TXT_distflphotos";
            // 
            // CMB_camera
            // 
            this.CMB_camera.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_camera, "CMB_camera");
            this.CMB_camera.Name = "CMB_camera";
            this.CMB_camera.SelectedIndexChanged += new System.EventHandler(this.CMB_camera_SelectedIndexChanged);
            // 
            // BUT_save
            // 
            resources.ApplyResources(this.BUT_save, "BUT_save");
            this.BUT_save.Name = "BUT_save";
            this.BUT_save.UseVisualStyleBackColor = true;
            this.BUT_save.Click += new System.EventHandler(this.BUT_save_Click);
            // 
            // Camera
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
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
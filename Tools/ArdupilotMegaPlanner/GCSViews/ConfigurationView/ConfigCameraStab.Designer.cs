using ArdupilotMega.Controls;
using ArdupilotMega.Presenter;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigCameraStab
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> pi
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
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.numericUpDown1 = new System.Windows.Forms.NumericUpDown();
            this.presenterBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.checkBox1 = new System.Windows.Forms.CheckBox();
            this.numericUpDown2 = new System.Windows.Forms.NumericUpDown();
            this.numericUpDown3 = new System.Windows.Forms.NumericUpDown();
            this.numericUpDown4 = new System.Windows.Forms.NumericUpDown();
            this.BUT_WriteValues = new ArdupilotMega.Controls.MyButton();
            this.BUT_SetDefaults = new ArdupilotMega.Controls.MyButton();
            this.BUT_Refresh = new ArdupilotMega.Controls.MyButton();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.numericUpDown5 = new System.Windows.Forms.NumericUpDown();
            this.numericUpDown6 = new System.Windows.Forms.NumericUpDown();
            this.numericUpDown7 = new System.Windows.Forms.NumericUpDown();
            this.checkBox2 = new System.Windows.Forms.CheckBox();
            this.numericUpDown8 = new System.Windows.Forms.NumericUpDown();
            this.PBOX_WarningIcon = new ArdupilotMega.Controls.PictureBoxWithPseudoOpacity();
            this.LBL_Error = new ArdupilotMega.Controls.LabelWithPseudoOpacity();
            this.LNK_wiki = new System.Windows.Forms.LinkLabel();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.presenterBindingSource)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown3)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown4)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown5)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown6)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown7)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown8)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.PBOX_WarningIcon)).BeginInit();
            this.SuspendLayout();
            // 
            // pictureBox1
            // 
            this.pictureBox1.BackColor = System.Drawing.Color.Transparent;
            this.pictureBox1.BackgroundImage = global::ArdupilotMega.Properties.Resources.cameraGimalPitch1;
            this.pictureBox1.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pictureBox1.Location = new System.Drawing.Point(33, 52);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(203, 112);
            this.pictureBox1.TabIndex = 0;
            this.pictureBox1.TabStop = false;
            // 
            // numericUpDown1
            // 
            this.numericUpDown1.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.presenterBindingSource, "CameraPitchGain", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.numericUpDown1.DecimalPlaces = 2;
            this.numericUpDown1.Increment = new decimal(new int[] {
            5,
            0,
            0,
            131072});
            this.numericUpDown1.Location = new System.Drawing.Point(376, 77);
            this.numericUpDown1.Maximum = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.numericUpDown1.Name = "numericUpDown1";
            this.numericUpDown1.Size = new System.Drawing.Size(55, 20);
            this.numericUpDown1.TabIndex = 4;
            this.numericUpDown1.Value = new decimal(new int[] {
            10,
            0,
            0,
            65536});
            // 
            // presenterBindingSource
            // 
            this.presenterBindingSource.DataSource = typeof(ArdupilotMega.Presenter.ConfigCameraStabPresenter);
            // 
            // checkBox1
            // 
            this.checkBox1.AutoSize = true;
            this.checkBox1.DataBindings.Add(new System.Windows.Forms.Binding("Checked", this.presenterBindingSource, "CameraPitchReverse", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.checkBox1.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(224)))), ((int)(((byte)(224)))), ((int)(((byte)(224)))));
            this.checkBox1.Location = new System.Drawing.Point(365, 129);
            this.checkBox1.Name = "checkBox1";
            this.checkBox1.Size = new System.Drawing.Size(66, 17);
            this.checkBox1.TabIndex = 5;
            this.checkBox1.Text = "Reverse";
            this.checkBox1.UseVisualStyleBackColor = true;
            // 
            // numericUpDown2
            // 
            this.numericUpDown2.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.presenterBindingSource, "CameraPitchMax", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.numericUpDown2.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.numericUpDown2.Location = new System.Drawing.Point(273, 129);
            this.numericUpDown2.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.numericUpDown2.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.numericUpDown2.Name = "numericUpDown2";
            this.numericUpDown2.Size = new System.Drawing.Size(59, 20);
            this.numericUpDown2.TabIndex = 6;
            this.numericUpDown2.Value = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            // 
            // numericUpDown3
            // 
            this.numericUpDown3.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.presenterBindingSource, "CameraPitchTrim", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.numericUpDown3.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.numericUpDown3.Location = new System.Drawing.Point(273, 103);
            this.numericUpDown3.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.numericUpDown3.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.numericUpDown3.Name = "numericUpDown3";
            this.numericUpDown3.Size = new System.Drawing.Size(59, 20);
            this.numericUpDown3.TabIndex = 7;
            this.numericUpDown3.Value = new decimal(new int[] {
            1500,
            0,
            0,
            0});
            // 
            // numericUpDown4
            // 
            this.numericUpDown4.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.presenterBindingSource, "CameraPitchMin", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.numericUpDown4.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.numericUpDown4.Location = new System.Drawing.Point(273, 77);
            this.numericUpDown4.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.numericUpDown4.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.numericUpDown4.Name = "numericUpDown4";
            this.numericUpDown4.Size = new System.Drawing.Size(59, 20);
            this.numericUpDown4.TabIndex = 8;
            this.numericUpDown4.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // BUT_WriteValues
            // 
            this.BUT_WriteValues.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.BUT_WriteValues.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.presenterBindingSource, "WriteValuesCommand", true));
            this.BUT_WriteValues.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_WriteValues.Location = new System.Drawing.Point(568, 383);
            this.BUT_WriteValues.Name = "BUT_WriteValues";
            this.BUT_WriteValues.Size = new System.Drawing.Size(94, 37);
            this.BUT_WriteValues.TabIndex = 49;
            this.BUT_WriteValues.Text = "Write";
            this.BUT_WriteValues.UseVisualStyleBackColor = true;
            // 
            // BUT_SetDefaults
            // 
            this.BUT_SetDefaults.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.BUT_SetDefaults.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.presenterBindingSource, "SetDefaultsCommand", true, System.Windows.Forms.DataSourceUpdateMode.Never));
            this.BUT_SetDefaults.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_SetDefaults.Location = new System.Drawing.Point(30, 397);
            this.BUT_SetDefaults.Name = "BUT_SetDefaults";
            this.BUT_SetDefaults.Size = new System.Drawing.Size(78, 23);
            this.BUT_SetDefaults.TabIndex = 50;
            this.BUT_SetDefaults.Text = "Set Defaults";
            this.BUT_SetDefaults.UseVisualStyleBackColor = true;
            // 
            // BUT_Refresh
            // 
            this.BUT_Refresh.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.BUT_Refresh.DataBindings.Add(new System.Windows.Forms.Binding("Tag", this.presenterBindingSource, "RefreshValuesCommand", true));
            this.BUT_Refresh.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.BUT_Refresh.Location = new System.Drawing.Point(114, 397);
            this.BUT_Refresh.Name = "BUT_Refresh";
            this.BUT_Refresh.Size = new System.Drawing.Size(78, 23);
            this.BUT_Refresh.TabIndex = 51;
            this.BUT_Refresh.Text = "Refresh";
            this.BUT_Refresh.UseVisualStyleBackColor = true;
            // 
            // groupBox1
            // 
            this.groupBox1.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.groupBox1.Location = new System.Drawing.Point(17, 216);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(635, 5);
            this.groupBox1.TabIndex = 52;
            this.groupBox1.TabStop = false;
            // 
            // pictureBox2
            // 
            this.pictureBox2.BackgroundImage = global::ArdupilotMega.Properties.Resources.cameraGimalRoll1;
            this.pictureBox2.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.pictureBox2.Location = new System.Drawing.Point(33, 207);
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.Size = new System.Drawing.Size(203, 112);
            this.pictureBox2.TabIndex = 53;
            this.pictureBox2.TabStop = false;
            // 
            // groupBox2
            // 
            this.groupBox2.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.groupBox2.Location = new System.Drawing.Point(17, 59);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(635, 5);
            this.groupBox2.TabIndex = 59;
            this.groupBox2.TabStop = false;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label1.Location = new System.Drawing.Point(243, 79);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(24, 13);
            this.label1.TabIndex = 60;
            this.label1.Text = "Min";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label2.Location = new System.Drawing.Point(243, 105);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(27, 13);
            this.label2.TabIndex = 61;
            this.label2.Text = "Trim";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label3.Location = new System.Drawing.Point(243, 133);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(27, 13);
            this.label3.TabIndex = 62;
            this.label3.Text = "Max";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label4.Location = new System.Drawing.Point(346, 79);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(29, 13);
            this.label4.TabIndex = 63;
            this.label4.Text = "Gain";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label5.Location = new System.Drawing.Point(21, 40);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(44, 20);
            this.label5.TabIndex = 64;
            this.label5.Text = "Pitch";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label6.Location = new System.Drawing.Point(20, 198);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(36, 20);
            this.label6.TabIndex = 65;
            this.label6.Text = "Roll";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label7.Location = new System.Drawing.Point(346, 237);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(29, 13);
            this.label7.TabIndex = 74;
            this.label7.Text = "Gain";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label8.Location = new System.Drawing.Point(243, 291);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(27, 13);
            this.label8.TabIndex = 73;
            this.label8.Text = "Max";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label9.Location = new System.Drawing.Point(243, 263);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(27, 13);
            this.label9.TabIndex = 72;
            this.label9.Text = "Trim";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.label10.Location = new System.Drawing.Point(243, 237);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(24, 13);
            this.label10.TabIndex = 71;
            this.label10.Text = "Min";
            // 
            // numericUpDown5
            // 
            this.numericUpDown5.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.presenterBindingSource, "CameraRollMin", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.numericUpDown5.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.numericUpDown5.Location = new System.Drawing.Point(273, 235);
            this.numericUpDown5.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.numericUpDown5.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.numericUpDown5.Name = "numericUpDown5";
            this.numericUpDown5.Size = new System.Drawing.Size(59, 20);
            this.numericUpDown5.TabIndex = 70;
            this.numericUpDown5.Value = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            // 
            // numericUpDown6
            // 
            this.numericUpDown6.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.presenterBindingSource, "CameraRollTrim", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.numericUpDown6.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.numericUpDown6.Location = new System.Drawing.Point(273, 261);
            this.numericUpDown6.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.numericUpDown6.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.numericUpDown6.Name = "numericUpDown6";
            this.numericUpDown6.Size = new System.Drawing.Size(59, 20);
            this.numericUpDown6.TabIndex = 69;
            this.numericUpDown6.Value = new decimal(new int[] {
            1500,
            0,
            0,
            0});
            // 
            // numericUpDown7
            // 
            this.numericUpDown7.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.presenterBindingSource, "CameraRollMax", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.numericUpDown7.Increment = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.numericUpDown7.Location = new System.Drawing.Point(273, 287);
            this.numericUpDown7.Maximum = new decimal(new int[] {
            2200,
            0,
            0,
            0});
            this.numericUpDown7.Minimum = new decimal(new int[] {
            800,
            0,
            0,
            0});
            this.numericUpDown7.Name = "numericUpDown7";
            this.numericUpDown7.Size = new System.Drawing.Size(59, 20);
            this.numericUpDown7.TabIndex = 68;
            this.numericUpDown7.Value = new decimal(new int[] {
            2000,
            0,
            0,
            0});
            // 
            // checkBox2
            // 
            this.checkBox2.AutoSize = true;
            this.checkBox2.DataBindings.Add(new System.Windows.Forms.Binding("Checked", this.presenterBindingSource, "CameraRollReverse", true));
            this.checkBox2.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(224)))), ((int)(((byte)(224)))), ((int)(((byte)(224)))));
            this.checkBox2.Location = new System.Drawing.Point(365, 287);
            this.checkBox2.Name = "checkBox2";
            this.checkBox2.Size = new System.Drawing.Size(66, 17);
            this.checkBox2.TabIndex = 67;
            this.checkBox2.Text = "Reverse";
            this.checkBox2.UseVisualStyleBackColor = true;
            // 
            // numericUpDown8
            // 
            this.numericUpDown8.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.presenterBindingSource, "CameraRollGain", true, System.Windows.Forms.DataSourceUpdateMode.OnPropertyChanged));
            this.numericUpDown8.DecimalPlaces = 2;
            this.numericUpDown8.Increment = new decimal(new int[] {
            5,
            0,
            0,
            131072});
            this.numericUpDown8.Location = new System.Drawing.Point(376, 235);
            this.numericUpDown8.Maximum = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.numericUpDown8.Name = "numericUpDown8";
            this.numericUpDown8.Size = new System.Drawing.Size(55, 20);
            this.numericUpDown8.TabIndex = 66;
            this.numericUpDown8.Value = new decimal(new int[] {
            10,
            0,
            0,
            65536});
            // 
            // PBOX_WarningIcon
            // 
            this.PBOX_WarningIcon.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.PBOX_WarningIcon.Image = global::ArdupilotMega.Properties.Resources.iconWarning32;
            this.PBOX_WarningIcon.Location = new System.Drawing.Point(264, 389);
            this.PBOX_WarningIcon.Name = "PBOX_WarningIcon";
            this.PBOX_WarningIcon.Opacity = 0.5F;
            this.PBOX_WarningIcon.Size = new System.Drawing.Size(32, 32);
            this.PBOX_WarningIcon.TabIndex = 75;
            this.PBOX_WarningIcon.TabStop = false;
            // 
            // LBL_Error
            // 
            this.LBL_Error.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.LBL_Error.AutoSize = true;
            this.LBL_Error.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.presenterBindingSource, "Error", true));
            this.LBL_Error.ForeColor = System.Drawing.Color.WhiteSmoke;
            this.LBL_Error.Location = new System.Drawing.Point(303, 402);
            this.LBL_Error.Name = "LBL_Error";
            this.LBL_Error.Size = new System.Drawing.Size(138, 13);
            this.LBL_Error.TabIndex = 76;
            this.LBL_Error.Text = "Error Message of some kind";
            // 
            // LNK_wiki
            // 
            this.LNK_wiki.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.LNK_wiki.AutoSize = true;
            this.LNK_wiki.LinkBehavior = System.Windows.Forms.LinkBehavior.HoverUnderline;
            this.LNK_wiki.LinkColor = System.Drawing.Color.CornflowerBlue;
            this.LNK_wiki.Location = new System.Drawing.Point(624, 9);
            this.LNK_wiki.Name = "LNK_wiki";
            this.LNK_wiki.Size = new System.Drawing.Size(28, 13);
            this.LNK_wiki.TabIndex = 77;
            this.LNK_wiki.TabStop = true;
            this.LNK_wiki.Text = "Wiki";
            this.LNK_wiki.LinkClicked += new System.Windows.Forms.LinkLabelLinkClickedEventHandler(this.LNK_Wiki_Clicked);
            // 
            // ConfigCameraStab
            // 
            this.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(38)))), ((int)(((byte)(39)))), ((int)(((byte)(40)))));
            this.Controls.Add(this.LNK_wiki);
            this.Controls.Add(this.LBL_Error);
            this.Controls.Add(this.PBOX_WarningIcon);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.numericUpDown5);
            this.Controls.Add(this.numericUpDown6);
            this.Controls.Add(this.numericUpDown7);
            this.Controls.Add(this.checkBox2);
            this.Controls.Add(this.numericUpDown8);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.BUT_Refresh);
            this.Controls.Add(this.BUT_SetDefaults);
            this.Controls.Add(this.BUT_WriteValues);
            this.Controls.Add(this.numericUpDown4);
            this.Controls.Add(this.numericUpDown3);
            this.Controls.Add(this.numericUpDown2);
            this.Controls.Add(this.checkBox1);
            this.Controls.Add(this.numericUpDown1);
            this.Controls.Add(this.pictureBox2);
            this.Controls.Add(this.pictureBox1);
            this.Name = "ConfigCameraStab";
            this.Size = new System.Drawing.Size(674, 432);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.presenterBindingSource)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown3)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown4)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown5)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown6)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown7)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown8)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.PBOX_WarningIcon)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.NumericUpDown numericUpDown1;
        private System.Windows.Forms.CheckBox checkBox1;
        private System.Windows.Forms.NumericUpDown numericUpDown2;
        private System.Windows.Forms.NumericUpDown numericUpDown3;
        private System.Windows.Forms.NumericUpDown numericUpDown4;
        private Controls.MyButton BUT_WriteValues;
        private Controls.MyButton BUT_SetDefaults;
        private Controls.MyButton BUT_Refresh;
        private System.Windows.Forms.PictureBox pictureBox2;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.NumericUpDown numericUpDown5;
        private System.Windows.Forms.NumericUpDown numericUpDown6;
        private System.Windows.Forms.NumericUpDown numericUpDown7;
        private System.Windows.Forms.CheckBox checkBox2;
        private System.Windows.Forms.NumericUpDown numericUpDown8;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.BindingSource presenterBindingSource;
        private PictureBoxWithPseudoOpacity PBOX_WarningIcon;
        private LabelWithPseudoOpacity LBL_Error;
        private System.Windows.Forms.LinkLabel LNK_wiki;

    }
}

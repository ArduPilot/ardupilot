namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigSignalization
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
            this.L_beeper_pin = new System.Windows.Forms.Label();
            this.L_beeper_functions = new System.Windows.Forms.Label();
            this.CB_beeper_0 = new System.Windows.Forms.CheckBox();
            this.CB_beeper_1 = new System.Windows.Forms.CheckBox();
            this.CB_beeper_2 = new System.Windows.Forms.CheckBox();
            this.CB_beeper_3 = new System.Windows.Forms.CheckBox();
            this.CB_beeper_4 = new System.Windows.Forms.CheckBox();
            this.CB_beeper_5 = new System.Windows.Forms.CheckBox();
            this.CB_beeper_6 = new System.Windows.Forms.CheckBox();
            this.CB_beeper_7 = new System.Windows.Forms.CheckBox();
            this.CMB_beeper_pin = new System.Windows.Forms.ComboBox();
            this.GB_leds = new System.Windows.Forms.GroupBox();
            this.P_leds_mode_legacy = new System.Windows.Forms.Panel();
            this.CB_leds_legacy_1 = new System.Windows.Forms.CheckBox();
            this.CB_leds_legacy_2 = new System.Windows.Forms.CheckBox();
            this.CB_leds_legacy_3 = new System.Windows.Forms.CheckBox();
            this.CB_leds_legacy_4 = new System.Windows.Forms.CheckBox();
            this.CB_leds_legacy_5 = new System.Windows.Forms.CheckBox();
            this.CB_leds_legacy_6 = new System.Windows.Forms.CheckBox();
            this.CB_leds_legacy_7 = new System.Windows.Forms.CheckBox();
            this.CB_leds_legacy_8 = new System.Windows.Forms.CheckBox();
            this.L_leds_type = new System.Windows.Forms.Label();
            this.CMB_leds_style = new System.Windows.Forms.ComboBox();
            this.P_leds_mode_combo = new System.Windows.Forms.Panel();
            this.L_led_1 = new System.Windows.Forms.Label();
            this.CMB_led_1 = new System.Windows.Forms.ComboBox();
            this.L_led_2 = new System.Windows.Forms.Label();
            this.CMB_led_2 = new System.Windows.Forms.ComboBox();
            this.L_led_3 = new System.Windows.Forms.Label();
            this.CMB_led_3 = new System.Windows.Forms.ComboBox();
            this.L_led_4 = new System.Windows.Forms.Label();
            this.CMB_led_4 = new System.Windows.Forms.ComboBox();
            this.L_led_5 = new System.Windows.Forms.Label();
            this.CMB_led_5 = new System.Windows.Forms.ComboBox();
            this.L_led_6 = new System.Windows.Forms.Label();
            this.CMB_led_6 = new System.Windows.Forms.ComboBox();
            this.L_led_7 = new System.Windows.Forms.Label();
            this.CMB_led_7 = new System.Windows.Forms.ComboBox();
            this.L_led_8 = new System.Windows.Forms.Label();
            this.CMB_led_8 = new System.Windows.Forms.ComboBox();
            this.GB_beeper = new System.Windows.Forms.GroupBox();
            this.B_beeper_write = new ArdupilotMega.Controls.MyButton();
            this.B_leds_write = new ArdupilotMega.Controls.MyButton();
            this.GB_leds.SuspendLayout();
            this.P_leds_mode_legacy.SuspendLayout();
            this.P_leds_mode_combo.SuspendLayout();
            this.GB_beeper.SuspendLayout();
            this.SuspendLayout();
            // 
            // L_beeper_pin
            // 
            this.L_beeper_pin.AutoSize = true;
            this.L_beeper_pin.Location = new System.Drawing.Point(6, 31);
            this.L_beeper_pin.Name = "L_beeper_pin";
            this.L_beeper_pin.Size = new System.Drawing.Size(186, 13);
            this.L_beeper_pin.TabIndex = 4;
            this.L_beeper_pin.Text = "Select pin where beeper is connected\r\n";
            // 
            // L_beeper_functions
            // 
            this.L_beeper_functions.AutoSize = true;
            this.L_beeper_functions.Location = new System.Drawing.Point(6, 75);
            this.L_beeper_functions.Name = "L_beeper_functions";
            this.L_beeper_functions.Size = new System.Drawing.Size(105, 13);
            this.L_beeper_functions.TabIndex = 5;
            this.L_beeper_functions.Text = "Select when to beep";
            // 
            // CB_beeper_0
            // 
            this.CB_beeper_0.AutoSize = true;
            this.CB_beeper_0.Location = new System.Drawing.Point(9, 91);
            this.CB_beeper_0.Name = "CB_beeper_0";
            this.CB_beeper_0.Size = new System.Drawing.Size(75, 17);
            this.CB_beeper_0.TabIndex = 2;
            this.CB_beeper_0.Text = "On startup";
            this.CB_beeper_0.UseVisualStyleBackColor = true;
            // 
            // CB_beeper_1
            // 
            this.CB_beeper_1.AutoSize = true;
            this.CB_beeper_1.Location = new System.Drawing.Point(9, 114);
            this.CB_beeper_1.Name = "CB_beeper_1";
            this.CB_beeper_1.Size = new System.Drawing.Size(74, 17);
            this.CB_beeper_1.TabIndex = 3;
            this.CB_beeper_1.Text = "On arming";
            this.CB_beeper_1.UseVisualStyleBackColor = true;
            // 
            // CB_beeper_2
            // 
            this.CB_beeper_2.AutoSize = true;
            this.CB_beeper_2.Location = new System.Drawing.Point(9, 137);
            this.CB_beeper_2.Name = "CB_beeper_2";
            this.CB_beeper_2.Size = new System.Drawing.Size(87, 17);
            this.CB_beeper_2.TabIndex = 4;
            this.CB_beeper_2.Text = "On disarming";
            this.CB_beeper_2.UseVisualStyleBackColor = true;
            // 
            // CB_beeper_3
            // 
            this.CB_beeper_3.AutoSize = true;
            this.CB_beeper_3.Location = new System.Drawing.Point(9, 160);
            this.CB_beeper_3.Name = "CB_beeper_3";
            this.CB_beeper_3.Size = new System.Drawing.Size(159, 17);
            this.CB_beeper_3.TabIndex = 5;
            this.CB_beeper_3.Text = "When armed and on ground";
            this.CB_beeper_3.UseVisualStyleBackColor = true;
            // 
            // CB_beeper_4
            // 
            this.CB_beeper_4.AutoSize = true;
            this.CB_beeper_4.Location = new System.Drawing.Point(9, 181);
            this.CB_beeper_4.Name = "CB_beeper_4";
            this.CB_beeper_4.Size = new System.Drawing.Size(111, 17);
            this.CB_beeper_4.TabIndex = 6;
            this.CB_beeper_4.Text = "On GPS home set";
            this.CB_beeper_4.UseVisualStyleBackColor = true;
            // 
            // CB_beeper_5
            // 
            this.CB_beeper_5.AutoSize = true;
            this.CB_beeper_5.Location = new System.Drawing.Point(9, 204);
            this.CB_beeper_5.Name = "CB_beeper_5";
            this.CB_beeper_5.Size = new System.Drawing.Size(97, 17);
            this.CB_beeper_5.TabIndex = 7;
            this.CB_beeper_5.Text = "On low voltage";
            this.CB_beeper_5.UseVisualStyleBackColor = true;
            // 
            // CB_beeper_6
            // 
            this.CB_beeper_6.AutoSize = true;
            this.CB_beeper_6.Location = new System.Drawing.Point(9, 227);
            this.CB_beeper_6.Name = "CB_beeper_6";
            this.CB_beeper_6.Size = new System.Drawing.Size(112, 17);
            this.CB_beeper_6.TabIndex = 8;
            this.CB_beeper_6.Text = "When CH7 is high";
            this.CB_beeper_6.UseVisualStyleBackColor = true;
            // 
            // CB_beeper_7
            // 
            this.CB_beeper_7.AutoSize = true;
            this.CB_beeper_7.Location = new System.Drawing.Point(9, 250);
            this.CB_beeper_7.Name = "CB_beeper_7";
            this.CB_beeper_7.Size = new System.Drawing.Size(142, 17);
            this.CB_beeper_7.TabIndex = 9;
            this.CB_beeper_7.Text = "When in auto land mode";
            this.CB_beeper_7.UseVisualStyleBackColor = true;
            // 
            // CMB_beeper_pin
            // 
            this.CMB_beeper_pin.AutoCompleteMode = System.Windows.Forms.AutoCompleteMode.SuggestAppend;
            this.CMB_beeper_pin.AutoCompleteSource = System.Windows.Forms.AutoCompleteSource.ListItems;
            this.CMB_beeper_pin.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_beeper_pin.Enabled = false;
            this.CMB_beeper_pin.FormattingEnabled = true;
            this.CMB_beeper_pin.Items.AddRange(new object[] {
            "AN0",
            "AN1",
            "AN2",
            "AN3",
            "AN4",
            "AN5",
            "AN6",
            "AN7",
            "AN8",
            "AN9",
            "AN10",
            "AN11",
            "AN12",
            "AN13",
            "AN14",
            "AN15"});
            this.CMB_beeper_pin.Location = new System.Drawing.Point(6, 47);
            this.CMB_beeper_pin.Name = "CMB_beeper_pin";
            this.CMB_beeper_pin.Size = new System.Drawing.Size(121, 21);
            this.CMB_beeper_pin.TabIndex = 1;
            // 
            // GB_leds
            // 
            this.GB_leds.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)));
            this.GB_leds.Controls.Add(this.P_leds_mode_legacy);
            this.GB_leds.Controls.Add(this.B_leds_write);
            this.GB_leds.Controls.Add(this.L_leds_type);
            this.GB_leds.Controls.Add(this.CMB_leds_style);
            this.GB_leds.Controls.Add(this.P_leds_mode_combo);
            this.GB_leds.Location = new System.Drawing.Point(253, 0);
            this.GB_leds.Name = "GB_leds";
            this.GB_leds.Size = new System.Drawing.Size(252, 347);
            this.GB_leds.TabIndex = 10;
            this.GB_leds.TabStop = false;
            this.GB_leds.Text = "LEDs settings";
            // 
            // P_leds_mode_legacy
            // 
            this.P_leds_mode_legacy.Controls.Add(this.CB_leds_legacy_1);
            this.P_leds_mode_legacy.Controls.Add(this.CB_leds_legacy_2);
            this.P_leds_mode_legacy.Controls.Add(this.CB_leds_legacy_3);
            this.P_leds_mode_legacy.Controls.Add(this.CB_leds_legacy_4);
            this.P_leds_mode_legacy.Controls.Add(this.CB_leds_legacy_5);
            this.P_leds_mode_legacy.Controls.Add(this.CB_leds_legacy_6);
            this.P_leds_mode_legacy.Controls.Add(this.CB_leds_legacy_7);
            this.P_leds_mode_legacy.Controls.Add(this.CB_leds_legacy_8);
            this.P_leds_mode_legacy.Location = new System.Drawing.Point(2, 73);
            this.P_leds_mode_legacy.Name = "P_leds_mode_legacy";
            this.P_leds_mode_legacy.Size = new System.Drawing.Size(247, 214);
            this.P_leds_mode_legacy.TabIndex = 23;
            // 
            // CB_leds_legacy_1
            // 
            this.CB_leds_legacy_1.AutoSize = true;
            this.CB_leds_legacy_1.Location = new System.Drawing.Point(7, 2);
            this.CB_leds_legacy_1.Name = "CB_leds_legacy_1";
            this.CB_leds_legacy_1.Size = new System.Drawing.Size(53, 17);
            this.CB_leds_legacy_1.TabIndex = 0;
            this.CB_leds_legacy_1.Text = "Motor";
            this.CB_leds_legacy_1.UseVisualStyleBackColor = true;
            // 
            // CB_leds_legacy_2
            // 
            this.CB_leds_legacy_2.AutoSize = true;
            this.CB_leds_legacy_2.Location = new System.Drawing.Point(7, 23);
            this.CB_leds_legacy_2.Name = "CB_leds_legacy_2";
            this.CB_leds_legacy_2.Size = new System.Drawing.Size(48, 17);
            this.CB_leds_legacy_2.TabIndex = 1;
            this.CB_leds_legacy_2.Text = "GPS";
            this.CB_leds_legacy_2.UseVisualStyleBackColor = true;
            // 
            // CB_leds_legacy_3
            // 
            this.CB_leds_legacy_3.AutoSize = true;
            this.CB_leds_legacy_3.Location = new System.Drawing.Point(7, 46);
            this.CB_leds_legacy_3.Name = "CB_leds_legacy_3";
            this.CB_leds_legacy_3.Size = new System.Drawing.Size(44, 17);
            this.CB_leds_legacy_3.TabIndex = 2;
            this.CB_leds_legacy_3.Text = "Aux";
            this.CB_leds_legacy_3.UseVisualStyleBackColor = true;
            // 
            // CB_leds_legacy_4
            // 
            this.CB_leds_legacy_4.AutoSize = true;
            this.CB_leds_legacy_4.Location = new System.Drawing.Point(7, 69);
            this.CB_leds_legacy_4.Name = "CB_leds_legacy_4";
            this.CB_leds_legacy_4.Size = new System.Drawing.Size(60, 17);
            this.CB_leds_legacy_4.TabIndex = 3;
            this.CB_leds_legacy_4.Text = "Beeper";
            this.CB_leds_legacy_4.UseVisualStyleBackColor = true;
            // 
            // CB_leds_legacy_5
            // 
            this.CB_leds_legacy_5.AutoSize = true;
            this.CB_leds_legacy_5.Location = new System.Drawing.Point(7, 92);
            this.CB_leds_legacy_5.Name = "CB_leds_legacy_5";
            this.CB_leds_legacy_5.Size = new System.Drawing.Size(219, 17);
            this.CB_leds_legacy_5.TabIndex = 4;
            this.CB_leds_legacy_5.Text = "Fast flash (0) / oscillate (1) on low battery";
            this.CB_leds_legacy_5.UseVisualStyleBackColor = true;
            // 
            // CB_leds_legacy_6
            // 
            this.CB_leds_legacy_6.AutoSize = true;
            this.CB_leds_legacy_6.Location = new System.Drawing.Point(7, 115);
            this.CB_leds_legacy_6.Name = "CB_leds_legacy_6";
            this.CB_leds_legacy_6.Size = new System.Drawing.Size(99, 17);
            this.CB_leds_legacy_6.TabIndex = 5;
            this.CB_leds_legacy_6.Text = "Motor nav blink";
            this.CB_leds_legacy_6.UseVisualStyleBackColor = true;
            // 
            // CB_leds_legacy_7
            // 
            this.CB_leds_legacy_7.AutoSize = true;
            this.CB_leds_legacy_7.Location = new System.Drawing.Point(7, 138);
            this.CB_leds_legacy_7.Name = "CB_leds_legacy_7";
            this.CB_leds_legacy_7.Size = new System.Drawing.Size(94, 17);
            this.CB_leds_legacy_7.TabIndex = 6;
            this.CB_leds_legacy_7.Text = "GPS nav blink";
            this.CB_leds_legacy_7.UseVisualStyleBackColor = true;
            // 
            // CB_leds_legacy_8
            // 
            this.CB_leds_legacy_8.AutoSize = true;
            this.CB_leds_legacy_8.Enabled = false;
            this.CB_leds_legacy_8.Location = new System.Drawing.Point(7, 161);
            this.CB_leds_legacy_8.Name = "CB_leds_legacy_8";
            this.CB_leds_legacy_8.Size = new System.Drawing.Size(63, 17);
            this.CB_leds_legacy_8.TabIndex = 7;
            this.CB_leds_legacy_8.Text = "Unused";
            this.CB_leds_legacy_8.UseVisualStyleBackColor = true;
            // 
            // L_leds_type
            // 
            this.L_leds_type.AutoSize = true;
            this.L_leds_type.Location = new System.Drawing.Point(6, 31);
            this.L_leds_type.Name = "L_leds_type";
            this.L_leds_type.Size = new System.Drawing.Size(139, 13);
            this.L_leds_type.TabIndex = 10;
            this.L_leds_type.Text = "Select type of LEDs handler";
            // 
            // CMB_leds_style
            // 
            this.CMB_leds_style.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_leds_style.Enabled = false;
            this.CMB_leds_style.FormattingEnabled = true;
            this.CMB_leds_style.Location = new System.Drawing.Point(7, 46);
            this.CMB_leds_style.Name = "CMB_leds_style";
            this.CMB_leds_style.Size = new System.Drawing.Size(121, 21);
            this.CMB_leds_style.TabIndex = 21;
            this.CMB_leds_style.SelectedIndexChanged += new System.EventHandler(this.CMB_leds_style_SelectedIndexChanged);
            // 
            // P_leds_mode_combo
            // 
            this.P_leds_mode_combo.Controls.Add(this.L_led_1);
            this.P_leds_mode_combo.Controls.Add(this.CMB_led_1);
            this.P_leds_mode_combo.Controls.Add(this.L_led_2);
            this.P_leds_mode_combo.Controls.Add(this.CMB_led_2);
            this.P_leds_mode_combo.Controls.Add(this.L_led_3);
            this.P_leds_mode_combo.Controls.Add(this.CMB_led_3);
            this.P_leds_mode_combo.Controls.Add(this.L_led_4);
            this.P_leds_mode_combo.Controls.Add(this.CMB_led_4);
            this.P_leds_mode_combo.Controls.Add(this.L_led_5);
            this.P_leds_mode_combo.Controls.Add(this.CMB_led_5);
            this.P_leds_mode_combo.Controls.Add(this.L_led_6);
            this.P_leds_mode_combo.Controls.Add(this.CMB_led_6);
            this.P_leds_mode_combo.Controls.Add(this.L_led_7);
            this.P_leds_mode_combo.Controls.Add(this.CMB_led_7);
            this.P_leds_mode_combo.Controls.Add(this.L_led_8);
            this.P_leds_mode_combo.Controls.Add(this.CMB_led_8);
            this.P_leds_mode_combo.Location = new System.Drawing.Point(2, 73);
            this.P_leds_mode_combo.Name = "P_leds_mode_combo";
            this.P_leds_mode_combo.Size = new System.Drawing.Size(247, 214);
            this.P_leds_mode_combo.TabIndex = 23;
            // 
            // L_led_1
            // 
            this.L_led_1.AutoSize = true;
            this.L_led_1.Location = new System.Drawing.Point(6, 7);
            this.L_led_1.Name = "L_led_1";
            this.L_led_1.Size = new System.Drawing.Size(34, 13);
            this.L_led_1.TabIndex = 23;
            this.L_led_1.Text = "LED1";
            // 
            // CMB_led_1
            // 
            this.CMB_led_1.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_led_1.FormattingEnabled = true;
            this.CMB_led_1.Location = new System.Drawing.Point(46, 5);
            this.CMB_led_1.Name = "CMB_led_1";
            this.CMB_led_1.Size = new System.Drawing.Size(166, 21);
            this.CMB_led_1.TabIndex = 24;
            this.CMB_led_1.Format += new System.Windows.Forms.ListControlConvertEventHandler(this.CMB_Format);
            // 
            // L_led_2
            // 
            this.L_led_2.AutoSize = true;
            this.L_led_2.Location = new System.Drawing.Point(6, 34);
            this.L_led_2.Name = "L_led_2";
            this.L_led_2.Size = new System.Drawing.Size(34, 13);
            this.L_led_2.TabIndex = 25;
            this.L_led_2.Text = "LED2";
            // 
            // CMB_led_2
            // 
            this.CMB_led_2.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_led_2.FormattingEnabled = true;
            this.CMB_led_2.Location = new System.Drawing.Point(46, 31);
            this.CMB_led_2.Name = "CMB_led_2";
            this.CMB_led_2.Size = new System.Drawing.Size(166, 21);
            this.CMB_led_2.TabIndex = 26;
            this.CMB_led_2.Format += new System.Windows.Forms.ListControlConvertEventHandler(this.CMB_Format);
            // 
            // L_led_3
            // 
            this.L_led_3.AutoSize = true;
            this.L_led_3.Location = new System.Drawing.Point(6, 61);
            this.L_led_3.Name = "L_led_3";
            this.L_led_3.Size = new System.Drawing.Size(34, 13);
            this.L_led_3.TabIndex = 27;
            this.L_led_3.Text = "LED3";
            // 
            // CMB_led_3
            // 
            this.CMB_led_3.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_led_3.FormattingEnabled = true;
            this.CMB_led_3.Location = new System.Drawing.Point(46, 57);
            this.CMB_led_3.Name = "CMB_led_3";
            this.CMB_led_3.Size = new System.Drawing.Size(166, 21);
            this.CMB_led_3.TabIndex = 28;
            this.CMB_led_3.Format += new System.Windows.Forms.ListControlConvertEventHandler(this.CMB_Format);
            // 
            // L_led_4
            // 
            this.L_led_4.AutoSize = true;
            this.L_led_4.Location = new System.Drawing.Point(6, 88);
            this.L_led_4.Name = "L_led_4";
            this.L_led_4.Size = new System.Drawing.Size(34, 13);
            this.L_led_4.TabIndex = 29;
            this.L_led_4.Text = "LED4";
            // 
            // CMB_led_4
            // 
            this.CMB_led_4.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_led_4.FormattingEnabled = true;
            this.CMB_led_4.Location = new System.Drawing.Point(46, 83);
            this.CMB_led_4.Name = "CMB_led_4";
            this.CMB_led_4.Size = new System.Drawing.Size(166, 21);
            this.CMB_led_4.TabIndex = 30;
            this.CMB_led_4.Format += new System.Windows.Forms.ListControlConvertEventHandler(this.CMB_Format);
            // 
            // L_led_5
            // 
            this.L_led_5.AutoSize = true;
            this.L_led_5.Location = new System.Drawing.Point(6, 115);
            this.L_led_5.Name = "L_led_5";
            this.L_led_5.Size = new System.Drawing.Size(34, 13);
            this.L_led_5.TabIndex = 31;
            this.L_led_5.Text = "LED5";
            // 
            // CMB_led_5
            // 
            this.CMB_led_5.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_led_5.FormattingEnabled = true;
            this.CMB_led_5.Location = new System.Drawing.Point(46, 109);
            this.CMB_led_5.Name = "CMB_led_5";
            this.CMB_led_5.Size = new System.Drawing.Size(166, 21);
            this.CMB_led_5.TabIndex = 32;
            this.CMB_led_5.Format += new System.Windows.Forms.ListControlConvertEventHandler(this.CMB_Format);
            // 
            // L_led_6
            // 
            this.L_led_6.AutoSize = true;
            this.L_led_6.Location = new System.Drawing.Point(6, 142);
            this.L_led_6.Name = "L_led_6";
            this.L_led_6.Size = new System.Drawing.Size(34, 13);
            this.L_led_6.TabIndex = 33;
            this.L_led_6.Text = "LED6";
            // 
            // CMB_led_6
            // 
            this.CMB_led_6.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_led_6.FormattingEnabled = true;
            this.CMB_led_6.Location = new System.Drawing.Point(46, 135);
            this.CMB_led_6.Name = "CMB_led_6";
            this.CMB_led_6.Size = new System.Drawing.Size(166, 21);
            this.CMB_led_6.TabIndex = 34;
            this.CMB_led_6.Format += new System.Windows.Forms.ListControlConvertEventHandler(this.CMB_Format);
            // 
            // L_led_7
            // 
            this.L_led_7.AutoSize = true;
            this.L_led_7.Location = new System.Drawing.Point(6, 169);
            this.L_led_7.Name = "L_led_7";
            this.L_led_7.Size = new System.Drawing.Size(34, 13);
            this.L_led_7.TabIndex = 35;
            this.L_led_7.Text = "LED7";
            // 
            // CMB_led_7
            // 
            this.CMB_led_7.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_led_7.FormattingEnabled = true;
            this.CMB_led_7.Location = new System.Drawing.Point(46, 161);
            this.CMB_led_7.Name = "CMB_led_7";
            this.CMB_led_7.Size = new System.Drawing.Size(166, 21);
            this.CMB_led_7.TabIndex = 36;
            this.CMB_led_7.Format += new System.Windows.Forms.ListControlConvertEventHandler(this.CMB_Format);
            // 
            // L_led_8
            // 
            this.L_led_8.AutoSize = true;
            this.L_led_8.Location = new System.Drawing.Point(6, 196);
            this.L_led_8.Name = "L_led_8";
            this.L_led_8.Size = new System.Drawing.Size(34, 13);
            this.L_led_8.TabIndex = 37;
            this.L_led_8.Text = "LED8";
            // 
            // CMB_led_8
            // 
            this.CMB_led_8.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_led_8.FormattingEnabled = true;
            this.CMB_led_8.Location = new System.Drawing.Point(46, 187);
            this.CMB_led_8.Name = "CMB_led_8";
            this.CMB_led_8.Size = new System.Drawing.Size(166, 21);
            this.CMB_led_8.TabIndex = 38;
            this.CMB_led_8.Format += new System.Windows.Forms.ListControlConvertEventHandler(this.CMB_Format);
            // 
            // GB_beeper
            // 
            this.GB_beeper.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)));
            this.GB_beeper.Controls.Add(this.B_beeper_write);
            this.GB_beeper.Controls.Add(this.L_beeper_pin);
            this.GB_beeper.Controls.Add(this.CB_beeper_0);
            this.GB_beeper.Controls.Add(this.CB_beeper_4);
            this.GB_beeper.Controls.Add(this.CMB_beeper_pin);
            this.GB_beeper.Controls.Add(this.CB_beeper_5);
            this.GB_beeper.Controls.Add(this.L_beeper_functions);
            this.GB_beeper.Controls.Add(this.CB_beeper_6);
            this.GB_beeper.Controls.Add(this.CB_beeper_1);
            this.GB_beeper.Controls.Add(this.CB_beeper_7);
            this.GB_beeper.Controls.Add(this.CB_beeper_3);
            this.GB_beeper.Controls.Add(this.CB_beeper_2);
            this.GB_beeper.Location = new System.Drawing.Point(0, 0);
            this.GB_beeper.Name = "GB_beeper";
            this.GB_beeper.Size = new System.Drawing.Size(252, 347);
            this.GB_beeper.TabIndex = 11;
            this.GB_beeper.TabStop = false;
            this.GB_beeper.Text = "Beeper settings";
            // 
            // B_beeper_write
            // 
            this.B_beeper_write.Location = new System.Drawing.Point(83, 318);
            this.B_beeper_write.Name = "B_beeper_write";
            this.B_beeper_write.Size = new System.Drawing.Size(85, 23);
            this.B_beeper_write.TabIndex = 10;
            this.B_beeper_write.Text = "Write changes";
            this.B_beeper_write.UseVisualStyleBackColor = true;
            this.B_beeper_write.Click += new System.EventHandler(this.B_beeper_write_Click);
            // 
            // B_leds_write
            // 
            this.B_leds_write.Location = new System.Drawing.Point(82, 318);
            this.B_leds_write.Name = "B_leds_write";
            this.B_leds_write.Size = new System.Drawing.Size(85, 23);
            this.B_leds_write.TabIndex = 22;
            this.B_leds_write.Text = "Write changes";
            this.B_leds_write.UseVisualStyleBackColor = true;
            this.B_leds_write.Click += new System.EventHandler(this.B_leds_write_Click);
            // 
            // ConfigSignalization
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.GB_beeper);
            this.Controls.Add(this.GB_leds);
            this.Name = "ConfigSignalization";
            this.Size = new System.Drawing.Size(506, 350);
            this.GB_leds.ResumeLayout(false);
            this.GB_leds.PerformLayout();
            this.P_leds_mode_legacy.ResumeLayout(false);
            this.P_leds_mode_legacy.PerformLayout();
            this.P_leds_mode_combo.ResumeLayout(false);
            this.P_leds_mode_combo.PerformLayout();
            this.GB_beeper.ResumeLayout(false);
            this.GB_beeper.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.ComboBox CMB_beeper_pin;
        private System.Windows.Forms.Label L_beeper_pin;
        private System.Windows.Forms.Label L_beeper_functions;
        private System.Windows.Forms.CheckBox CB_beeper_0;
        private System.Windows.Forms.CheckBox CB_beeper_1;
        private System.Windows.Forms.CheckBox CB_beeper_2;
        private System.Windows.Forms.CheckBox CB_beeper_3;
        private System.Windows.Forms.CheckBox CB_beeper_4;
        private System.Windows.Forms.CheckBox CB_beeper_5;
        private System.Windows.Forms.CheckBox CB_beeper_6;
        private System.Windows.Forms.CheckBox CB_beeper_7;
        private System.Windows.Forms.GroupBox GB_leds;
        private System.Windows.Forms.GroupBox GB_beeper;
        private System.Windows.Forms.Label L_leds_type;
        private System.Windows.Forms.ComboBox CMB_leds_style;
        private Controls.MyButton B_leds_write;
        private Controls.MyButton B_beeper_write;
        private System.Windows.Forms.Panel P_leds_mode_combo;
        private System.Windows.Forms.Panel P_leds_mode_legacy;
        private System.Windows.Forms.Label L_led_1;
        private System.Windows.Forms.ComboBox CMB_led_1;
        private System.Windows.Forms.Label L_led_2;
        private System.Windows.Forms.ComboBox CMB_led_2;
        private System.Windows.Forms.Label L_led_3;
        private System.Windows.Forms.ComboBox CMB_led_3;
        private System.Windows.Forms.Label L_led_4;
        private System.Windows.Forms.ComboBox CMB_led_4;
        private System.Windows.Forms.Label L_led_5;
        private System.Windows.Forms.ComboBox CMB_led_5;
        private System.Windows.Forms.Label L_led_6;
        private System.Windows.Forms.ComboBox CMB_led_6;
        private System.Windows.Forms.Label L_led_7;
        private System.Windows.Forms.ComboBox CMB_led_7;
        private System.Windows.Forms.Label L_led_8;
        private System.Windows.Forms.ComboBox CMB_led_8;
        private System.Windows.Forms.CheckBox CB_leds_legacy_8;
        private System.Windows.Forms.CheckBox CB_leds_legacy_7;
        private System.Windows.Forms.CheckBox CB_leds_legacy_6;
        private System.Windows.Forms.CheckBox CB_leds_legacy_5;
        private System.Windows.Forms.CheckBox CB_leds_legacy_4;
        private System.Windows.Forms.CheckBox CB_leds_legacy_3;
        private System.Windows.Forms.CheckBox CB_leds_legacy_2;
        private System.Windows.Forms.CheckBox CB_leds_legacy_1;
    }
}

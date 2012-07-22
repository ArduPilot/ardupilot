namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigFlightModes
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(ConfigFlightModes));
            this.CB_simple6 = new System.Windows.Forms.CheckBox();
            this.CB_simple5 = new System.Windows.Forms.CheckBox();
            this.CB_simple4 = new System.Windows.Forms.CheckBox();
            this.CB_simple3 = new System.Windows.Forms.CheckBox();
            this.CB_simple2 = new System.Windows.Forms.CheckBox();
            this.CB_simple1 = new System.Windows.Forms.CheckBox();
            this.label14 = new System.Windows.Forms.Label();
            this.LBL_flightmodepwm = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.lbl_currentmode = new System.Windows.Forms.Label();
            this.currentStateBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.label12 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.CMB_fmode6 = new System.Windows.Forms.ComboBox();
            this.label5 = new System.Windows.Forms.Label();
            this.CMB_fmode5 = new System.Windows.Forms.ComboBox();
            this.label4 = new System.Windows.Forms.Label();
            this.CMB_fmode4 = new System.Windows.Forms.ComboBox();
            this.label3 = new System.Windows.Forms.Label();
            this.CMB_fmode3 = new System.Windows.Forms.ComboBox();
            this.label2 = new System.Windows.Forms.Label();
            this.CMB_fmode2 = new System.Windows.Forms.ComboBox();
            this.label1 = new System.Windows.Forms.Label();
            this.CMB_fmode1 = new System.Windows.Forms.ComboBox();
            this.BUT_SaveModes = new ArdupilotMega.Controls.MyButton();
            ((System.ComponentModel.ISupportInitialize)(this.currentStateBindingSource)).BeginInit();
            this.SuspendLayout();
            // 
            // CB_simple6
            // 
            resources.ApplyResources(this.CB_simple6, "CB_simple6");
            this.CB_simple6.Name = "CB_simple6";
            this.CB_simple6.UseVisualStyleBackColor = true;
            // 
            // CB_simple5
            // 
            resources.ApplyResources(this.CB_simple5, "CB_simple5");
            this.CB_simple5.Name = "CB_simple5";
            this.CB_simple5.UseVisualStyleBackColor = true;
            // 
            // CB_simple4
            // 
            resources.ApplyResources(this.CB_simple4, "CB_simple4");
            this.CB_simple4.Name = "CB_simple4";
            this.CB_simple4.UseVisualStyleBackColor = true;
            // 
            // CB_simple3
            // 
            resources.ApplyResources(this.CB_simple3, "CB_simple3");
            this.CB_simple3.Name = "CB_simple3";
            this.CB_simple3.UseVisualStyleBackColor = true;
            // 
            // CB_simple2
            // 
            resources.ApplyResources(this.CB_simple2, "CB_simple2");
            this.CB_simple2.Name = "CB_simple2";
            this.CB_simple2.UseVisualStyleBackColor = true;
            // 
            // CB_simple1
            // 
            resources.ApplyResources(this.CB_simple1, "CB_simple1");
            this.CB_simple1.Name = "CB_simple1";
            this.CB_simple1.UseVisualStyleBackColor = true;
            // 
            // label14
            // 
            resources.ApplyResources(this.label14, "label14");
            this.label14.Name = "label14";
            // 
            // LBL_flightmodepwm
            // 
            resources.ApplyResources(this.LBL_flightmodepwm, "LBL_flightmodepwm");
            this.LBL_flightmodepwm.Name = "LBL_flightmodepwm";
            // 
            // label13
            // 
            resources.ApplyResources(this.label13, "label13");
            this.label13.Name = "label13";
            // 
            // lbl_currentmode
            // 
            resources.ApplyResources(this.lbl_currentmode, "lbl_currentmode");
            this.lbl_currentmode.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.currentStateBindingSource, "mode", true));
            this.lbl_currentmode.Name = "lbl_currentmode";
            // 
            // currentStateBindingSource
            // 
            this.currentStateBindingSource.DataSource = typeof(ArdupilotMega.CurrentState);
            // 
            // label12
            // 
            resources.ApplyResources(this.label12, "label12");
            this.label12.Name = "label12";
            // 
            // label11
            // 
            resources.ApplyResources(this.label11, "label11");
            this.label11.Name = "label11";
            // 
            // label10
            // 
            resources.ApplyResources(this.label10, "label10");
            this.label10.Name = "label10";
            // 
            // label9
            // 
            resources.ApplyResources(this.label9, "label9");
            this.label9.Name = "label9";
            // 
            // label8
            // 
            resources.ApplyResources(this.label8, "label8");
            this.label8.Name = "label8";
            // 
            // label7
            // 
            resources.ApplyResources(this.label7, "label7");
            this.label7.Name = "label7";
            // 
            // label6
            // 
            resources.ApplyResources(this.label6, "label6");
            this.label6.Name = "label6";
            // 
            // CMB_fmode6
            // 
            this.CMB_fmode6.AutoCompleteMode = System.Windows.Forms.AutoCompleteMode.SuggestAppend;
            this.CMB_fmode6.AutoCompleteSource = System.Windows.Forms.AutoCompleteSource.ListItems;
            this.CMB_fmode6.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_fmode6.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_fmode6, "CMB_fmode6");
            this.CMB_fmode6.Name = "CMB_fmode6";
            // 
            // label5
            // 
            resources.ApplyResources(this.label5, "label5");
            this.label5.Name = "label5";
            // 
            // CMB_fmode5
            // 
            this.CMB_fmode5.AutoCompleteMode = System.Windows.Forms.AutoCompleteMode.SuggestAppend;
            this.CMB_fmode5.AutoCompleteSource = System.Windows.Forms.AutoCompleteSource.ListItems;
            this.CMB_fmode5.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_fmode5.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_fmode5, "CMB_fmode5");
            this.CMB_fmode5.Name = "CMB_fmode5";
            // 
            // label4
            // 
            resources.ApplyResources(this.label4, "label4");
            this.label4.Name = "label4";
            // 
            // CMB_fmode4
            // 
            this.CMB_fmode4.AutoCompleteMode = System.Windows.Forms.AutoCompleteMode.SuggestAppend;
            this.CMB_fmode4.AutoCompleteSource = System.Windows.Forms.AutoCompleteSource.ListItems;
            this.CMB_fmode4.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_fmode4.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_fmode4, "CMB_fmode4");
            this.CMB_fmode4.Name = "CMB_fmode4";
            // 
            // label3
            // 
            resources.ApplyResources(this.label3, "label3");
            this.label3.Name = "label3";
            // 
            // CMB_fmode3
            // 
            this.CMB_fmode3.AutoCompleteMode = System.Windows.Forms.AutoCompleteMode.SuggestAppend;
            this.CMB_fmode3.AutoCompleteSource = System.Windows.Forms.AutoCompleteSource.ListItems;
            this.CMB_fmode3.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_fmode3.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_fmode3, "CMB_fmode3");
            this.CMB_fmode3.Name = "CMB_fmode3";
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // CMB_fmode2
            // 
            this.CMB_fmode2.AutoCompleteMode = System.Windows.Forms.AutoCompleteMode.SuggestAppend;
            this.CMB_fmode2.AutoCompleteSource = System.Windows.Forms.AutoCompleteSource.ListItems;
            this.CMB_fmode2.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_fmode2.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_fmode2, "CMB_fmode2");
            this.CMB_fmode2.Name = "CMB_fmode2";
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            // 
            // CMB_fmode1
            // 
            this.CMB_fmode1.AutoCompleteMode = System.Windows.Forms.AutoCompleteMode.SuggestAppend;
            this.CMB_fmode1.AutoCompleteSource = System.Windows.Forms.AutoCompleteSource.ListItems;
            this.CMB_fmode1.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_fmode1.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_fmode1, "CMB_fmode1");
            this.CMB_fmode1.Name = "CMB_fmode1";
            // 
            // BUT_SaveModes
            // 
            resources.ApplyResources(this.BUT_SaveModes, "BUT_SaveModes");
            this.BUT_SaveModes.Name = "BUT_SaveModes";
            this.BUT_SaveModes.UseVisualStyleBackColor = true;
            this.BUT_SaveModes.Click += new System.EventHandler(this.BUT_SaveModes_Click);
            // 
            // ConfigFlightModes
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.CB_simple6);
            this.Controls.Add(this.CB_simple5);
            this.Controls.Add(this.CB_simple4);
            this.Controls.Add(this.CB_simple3);
            this.Controls.Add(this.CB_simple2);
            this.Controls.Add(this.CB_simple1);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.LBL_flightmodepwm);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.lbl_currentmode);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.CMB_fmode6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.CMB_fmode5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.CMB_fmode4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.CMB_fmode3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.CMB_fmode2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.CMB_fmode1);
            this.Controls.Add(this.BUT_SaveModes);
            this.Name = "ConfigFlightModes";
            ((System.ComponentModel.ISupportInitialize)(this.currentStateBindingSource)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.CheckBox CB_simple6;
        private System.Windows.Forms.CheckBox CB_simple5;
        private System.Windows.Forms.CheckBox CB_simple4;
        private System.Windows.Forms.CheckBox CB_simple3;
        private System.Windows.Forms.CheckBox CB_simple2;
        private System.Windows.Forms.CheckBox CB_simple1;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Label LBL_flightmodepwm;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label lbl_currentmode;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.ComboBox CMB_fmode6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.ComboBox CMB_fmode5;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.ComboBox CMB_fmode4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.ComboBox CMB_fmode3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.ComboBox CMB_fmode2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ComboBox CMB_fmode1;
        private ArdupilotMega.Controls.MyButton BUT_SaveModes;
        private System.Windows.Forms.BindingSource currentStateBindingSource;
    }
}

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class ConfigAccelerometerCalibrationPlane
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(ConfigAccelerometerCalibrationPlane));
            this.label28 = new System.Windows.Forms.Label();
            this.BUT_levelplane = new ArdupilotMega.Controls.MyButton();
            this.CHK_manuallevel = new System.Windows.Forms.CheckBox();
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.SuspendLayout();
            // 
            // label28
            // 
            resources.ApplyResources(this.label28, "label28");
            this.label28.Name = "label28";
            this.toolTip1.SetToolTip(this.label28, resources.GetString("label28.ToolTip"));
            // 
            // BUT_levelplane
            // 
            resources.ApplyResources(this.BUT_levelplane, "BUT_levelplane");
            this.BUT_levelplane.Name = "BUT_levelplane";
            this.toolTip1.SetToolTip(this.BUT_levelplane, resources.GetString("BUT_levelplane.ToolTip"));
            this.BUT_levelplane.UseVisualStyleBackColor = true;
            this.BUT_levelplane.Click += new System.EventHandler(this.BUT_levelplane_Click);
            // 
            // CHK_manuallevel
            // 
            resources.ApplyResources(this.CHK_manuallevel, "CHK_manuallevel");
            this.CHK_manuallevel.Name = "CHK_manuallevel";
            this.toolTip1.SetToolTip(this.CHK_manuallevel, resources.GetString("CHK_manuallevel.ToolTip"));
            this.CHK_manuallevel.UseVisualStyleBackColor = true;
            this.CHK_manuallevel.CheckedChanged += new System.EventHandler(this.CHK_manuallevel_CheckedChanged);
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            this.toolTip1.SetToolTip(this.label1, resources.GetString("label1.ToolTip"));
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            this.toolTip1.SetToolTip(this.label2, resources.GetString("label2.ToolTip"));
            // 
            // ConfigAccelerometerCalibrationPlane
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.CHK_manuallevel);
            this.Controls.Add(this.label28);
            this.Controls.Add(this.BUT_levelplane);
            this.Name = "ConfigAccelerometerCalibrationPlane";
            this.toolTip1.SetToolTip(this, resources.GetString("$this.ToolTip"));
            this.Load += new System.EventHandler(this.ConfigAccelerometerCalibration_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label label28;
        private ArdupilotMega.Controls.MyButton BUT_levelplane;
        private System.Windows.Forms.CheckBox CHK_manuallevel;
        private System.Windows.Forms.ToolTip toolTip1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
    }
}

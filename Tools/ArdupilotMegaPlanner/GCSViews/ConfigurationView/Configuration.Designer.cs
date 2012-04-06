namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class Configuration
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Configuration));
            this.backstageView = new ArdupilotMega.Controls.BackstageView.BackstageView();
            this.SuspendLayout();
            // 
            // backstageView
            // 
            this.backstageView.AutoSize = true;
            this.backstageView.Dock = System.Windows.Forms.DockStyle.Fill;
            this.backstageView.Location = new System.Drawing.Point(0, 0);
            this.backstageView.Name = "backstageView";
            this.backstageView.Size = new System.Drawing.Size(634, 336);
            this.backstageView.TabIndex = 0;
            // 
            // Configuration
            // 
            this.ClientSize = new System.Drawing.Size(634, 336);
            this.Controls.Add(this.backstageView);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "Configuration";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private Controls.BackstageView.BackstageView backstageView;
    }
}

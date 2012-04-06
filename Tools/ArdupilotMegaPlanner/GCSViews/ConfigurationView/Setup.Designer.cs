namespace ArdupilotMega.GCSViews.ConfigurationView
{
    partial class Setup
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Setup));
            this.lbl_pleaseconnect = new System.Windows.Forms.Label();
            this.backstageView = new ArdupilotMega.Controls.BackstageView.BackstageView();
            this.SuspendLayout();
            // 
            // lbl_pleaseconnect
            // 
            this.lbl_pleaseconnect.AutoSize = true;
            this.lbl_pleaseconnect.Location = new System.Drawing.Point(297, 284);
            this.lbl_pleaseconnect.Name = "lbl_pleaseconnect";
            this.lbl_pleaseconnect.Size = new System.Drawing.Size(104, 13);
            this.lbl_pleaseconnect.TabIndex = 1;
            this.lbl_pleaseconnect.Text = "Please Connect First";
            this.lbl_pleaseconnect.Visible = false;
            // 
            // backstageView
            // 
            this.backstageView.AutoSize = true;
            this.backstageView.Dock = System.Windows.Forms.DockStyle.Fill;
            this.backstageView.Location = new System.Drawing.Point(0, 0);
            this.backstageView.Name = "backstageView";
            this.backstageView.Size = new System.Drawing.Size(823, 468);
            this.backstageView.TabIndex = 0;
            // 
            // Setup
            // 
            this.ClientSize = new System.Drawing.Size(823, 468);
            this.Controls.Add(this.lbl_pleaseconnect);
            this.Controls.Add(this.backstageView);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.MinimumSize = new System.Drawing.Size(839, 506);
            this.Name = "Setup";
            this.Text = "Setup";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Setup_FormClosing);
            this.Load += new System.EventHandler(this.Setup_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private Controls.BackstageView.BackstageView backstageView;
        private System.Windows.Forms.Label lbl_pleaseconnect;
    }
}

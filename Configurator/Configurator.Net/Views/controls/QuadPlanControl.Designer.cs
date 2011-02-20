namespace ArducopterConfigurator.Views.controls
{
    partial class QuadPlanControl
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
            this.propControl1 = new ArducopterConfigurator.Views.controls.PropControl();
            this.SuspendLayout();
            // 
            // propControl1
            // 
            this.propControl1.Location = new System.Drawing.Point(49, 3);
            this.propControl1.Name = "propControl1";
            this.propControl1.Size = new System.Drawing.Size(50, 50);
            this.propControl1.TabIndex = 0;
            // 
            // QuadPlanControl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.propControl1);
            this.Name = "QuadPlanControl";
            this.ResumeLayout(false);

        }

        #endregion

        private PropControl propControl1;
    }
}

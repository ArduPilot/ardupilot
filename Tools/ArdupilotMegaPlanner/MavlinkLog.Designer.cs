namespace ArdupilotMega
{
    partial class MavlinkLog
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MavlinkLog));
            this.BUT_redokml = new ArdupilotMega.Controls.MyButton();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.BUT_humanreadable = new ArdupilotMega.Controls.MyButton();
            this.BUT_graphmavlog = new ArdupilotMega.Controls.MyButton();
            this.zg1 = new ZedGraph.ZedGraphControl();
            this.BUT_convertcsv = new ArdupilotMega.Controls.MyButton();
            this.SuspendLayout();
            // 
            // BUT_redokml
            // 
            resources.ApplyResources(this.BUT_redokml, "BUT_redokml");
            this.BUT_redokml.Name = "BUT_redokml";
            this.BUT_redokml.UseVisualStyleBackColor = true;
            this.BUT_redokml.Click += new System.EventHandler(this.BUT_redokml_Click);
            // 
            // progressBar1
            // 
            resources.ApplyResources(this.progressBar1, "progressBar1");
            this.progressBar1.Name = "progressBar1";
            // 
            // BUT_humanreadable
            // 
            resources.ApplyResources(this.BUT_humanreadable, "BUT_humanreadable");
            this.BUT_humanreadable.Name = "BUT_humanreadable";
            this.BUT_humanreadable.UseVisualStyleBackColor = true;
            this.BUT_humanreadable.Click += new System.EventHandler(this.BUT_humanreadable_Click);
            // 
            // BUT_graphmavlog
            // 
            resources.ApplyResources(this.BUT_graphmavlog, "BUT_graphmavlog");
            this.BUT_graphmavlog.Name = "BUT_graphmavlog";
            this.BUT_graphmavlog.UseVisualStyleBackColor = true;
            this.BUT_graphmavlog.Click += new System.EventHandler(this.BUT_graphmavlog_Click);
            // 
            // zg1
            // 
            resources.ApplyResources(this.zg1, "zg1");
            this.zg1.Name = "zg1";
            this.zg1.ScrollGrace = 0D;
            this.zg1.ScrollMaxX = 0D;
            this.zg1.ScrollMaxY = 0D;
            this.zg1.ScrollMaxY2 = 0D;
            this.zg1.ScrollMinX = 0D;
            this.zg1.ScrollMinY = 0D;
            this.zg1.ScrollMinY2 = 0D;
            // 
            // BUT_convertcsv
            // 
            resources.ApplyResources(this.BUT_convertcsv, "BUT_convertcsv");
            this.BUT_convertcsv.Name = "BUT_convertcsv";
            this.BUT_convertcsv.UseVisualStyleBackColor = true;
            this.BUT_convertcsv.Click += new System.EventHandler(this.BUT_convertcsv_Click);
            // 
            // MavlinkLog
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.BUT_convertcsv);
            this.Controls.Add(this.zg1);
            this.Controls.Add(this.BUT_graphmavlog);
            this.Controls.Add(this.BUT_humanreadable);
            this.Controls.Add(this.progressBar1);
            this.Controls.Add(this.BUT_redokml);
            this.Name = "MavlinkLog";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Log_FormClosing);
            this.ResumeLayout(false);

        }

        #endregion

        private ArdupilotMega.Controls.MyButton BUT_redokml;
        private System.Windows.Forms.ProgressBar progressBar1;
        private ArdupilotMega.Controls.MyButton BUT_humanreadable;
        private ArdupilotMega.Controls.MyButton BUT_graphmavlog;
        private ZedGraph.ZedGraphControl zg1;
        private Controls.MyButton BUT_convertcsv;
    }
}
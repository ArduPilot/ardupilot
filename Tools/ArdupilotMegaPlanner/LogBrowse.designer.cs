namespace ArdupilotMega
{
    partial class LogBrowse
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
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(LogBrowse));
            this.dataGridView1 = new System.Windows.Forms.DataGridView();
            this.zg1 = new ZedGraph.ZedGraphControl();
            this.Graphit = new ArdupilotMega.Controls.MyButton();
            this.BUT_cleargraph = new ArdupilotMega.Controls.MyButton();
            this.BUT_loadlog = new ArdupilotMega.Controls.MyButton();
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
            ((System.ComponentModel.ISupportInitialize)(this.dataGridView1)).BeginInit();
            this.SuspendLayout();
            // 
            // dataGridView1
            // 
            this.dataGridView1.AllowUserToAddRows = false;
            this.dataGridView1.AllowUserToDeleteRows = false;
            resources.ApplyResources(this.dataGridView1, "dataGridView1");
            this.dataGridView1.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.dataGridView1.MultiSelect = false;
            this.dataGridView1.Name = "dataGridView1";
            this.dataGridView1.ReadOnly = true;
            this.dataGridView1.SelectionMode = System.Windows.Forms.DataGridViewSelectionMode.CellSelect;
            this.dataGridView1.RowEnter += new System.Windows.Forms.DataGridViewCellEventHandler(this.dataGridView1_RowEnter);
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
            // Graphit
            // 
            resources.ApplyResources(this.Graphit, "Graphit");
            this.Graphit.Name = "Graphit";
            this.toolTip1.SetToolTip(this.Graphit, resources.GetString("Graphit.ToolTip"));
            this.Graphit.UseVisualStyleBackColor = true;
            this.Graphit.Click += new System.EventHandler(this.Graphit_Click);
            // 
            // BUT_cleargraph
            // 
            resources.ApplyResources(this.BUT_cleargraph, "BUT_cleargraph");
            this.BUT_cleargraph.Name = "BUT_cleargraph";
            this.toolTip1.SetToolTip(this.BUT_cleargraph, resources.GetString("BUT_cleargraph.ToolTip"));
            this.BUT_cleargraph.UseVisualStyleBackColor = true;
            this.BUT_cleargraph.Click += new System.EventHandler(this.BUT_cleargraph_Click);
            // 
            // BUT_loadlog
            // 
            resources.ApplyResources(this.BUT_loadlog, "BUT_loadlog");
            this.BUT_loadlog.Name = "BUT_loadlog";
            this.toolTip1.SetToolTip(this.BUT_loadlog, resources.GetString("BUT_loadlog.ToolTip"));
            this.BUT_loadlog.UseVisualStyleBackColor = true;
            this.BUT_loadlog.Click += new System.EventHandler(this.BUT_loadlog_Click);
            // 
            // LogBrowse
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.BUT_loadlog);
            this.Controls.Add(this.BUT_cleargraph);
            this.Controls.Add(this.Graphit);
            this.Controls.Add(this.zg1);
            this.Controls.Add(this.dataGridView1);
            this.Name = "LogBrowse";
            this.Load += new System.EventHandler(this.Form1_Load);
            ((System.ComponentModel.ISupportInitialize)(this.dataGridView1)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.DataGridView dataGridView1;
        private ZedGraph.ZedGraphControl zg1;
        private ArdupilotMega.Controls.MyButton Graphit;
        private ArdupilotMega.Controls.MyButton BUT_cleargraph;
        private ArdupilotMega.Controls.MyButton BUT_loadlog;
        private System.Windows.Forms.ToolTip toolTip1;
    }
}


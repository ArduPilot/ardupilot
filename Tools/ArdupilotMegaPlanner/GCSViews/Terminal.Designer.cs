namespace ArdupilotMega.GCSViews
{
    partial class Terminal
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Terminal));
            this.TXT_terminal = new System.Windows.Forms.RichTextBox();
            this.BUTsetupshow = new ArdupilotMega.Controls.MyButton();
            this.BUTradiosetup = new ArdupilotMega.Controls.MyButton();
            this.BUTtests = new ArdupilotMega.Controls.MyButton();
            this.Logs = new ArdupilotMega.Controls.MyButton();
            this.BUT_logbrowse = new ArdupilotMega.Controls.MyButton();
            this.SuspendLayout();
            // 
            // TXT_terminal
            // 
            resources.ApplyResources(this.TXT_terminal, "TXT_terminal");
            this.TXT_terminal.BackColor = System.Drawing.Color.Black;
            this.TXT_terminal.ForeColor = System.Drawing.Color.White;
            this.TXT_terminal.Name = "TXT_terminal";
            this.TXT_terminal.Click += new System.EventHandler(this.TXT_terminal_Click);
            this.TXT_terminal.KeyDown += new System.Windows.Forms.KeyEventHandler(this.TXT_terminal_KeyDown);
            this.TXT_terminal.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.TXT_terminal_KeyPress);
            // 
            // BUTsetupshow
            // 
            resources.ApplyResources(this.BUTsetupshow, "BUTsetupshow");
            this.BUTsetupshow.Name = "BUTsetupshow";
            this.BUTsetupshow.UseVisualStyleBackColor = true;
            this.BUTsetupshow.Click += new System.EventHandler(this.BUTsetupshow_Click);
            // 
            // BUTradiosetup
            // 
            resources.ApplyResources(this.BUTradiosetup, "BUTradiosetup");
            this.BUTradiosetup.Name = "BUTradiosetup";
            this.BUTradiosetup.UseVisualStyleBackColor = true;
            this.BUTradiosetup.Click += new System.EventHandler(this.BUTradiosetup_Click);
            // 
            // BUTtests
            // 
            resources.ApplyResources(this.BUTtests, "BUTtests");
            this.BUTtests.Name = "BUTtests";
            this.BUTtests.UseVisualStyleBackColor = true;
            this.BUTtests.Click += new System.EventHandler(this.BUTtests_Click);
            // 
            // Logs
            // 
            resources.ApplyResources(this.Logs, "Logs");
            this.Logs.Name = "Logs";
            this.Logs.UseVisualStyleBackColor = true;
            this.Logs.Click += new System.EventHandler(this.Logs_Click);
            // 
            // BUT_logbrowse
            // 
            resources.ApplyResources(this.BUT_logbrowse, "BUT_logbrowse");
            this.BUT_logbrowse.Name = "BUT_logbrowse";
            this.BUT_logbrowse.UseVisualStyleBackColor = true;
            this.BUT_logbrowse.Click += new System.EventHandler(this.BUT_logbrowse_Click);
            // 
            // Terminal
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.BUT_logbrowse);
            this.Controls.Add(this.Logs);
            this.Controls.Add(this.BUTtests);
            this.Controls.Add(this.BUTradiosetup);
            this.Controls.Add(this.BUTsetupshow);
            this.Controls.Add(this.TXT_terminal);
            this.Name = "Terminal";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Terminal_FormClosing);
            this.Load += new System.EventHandler(this.Terminal_Load);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.RichTextBox TXT_terminal;
        private ArdupilotMega.Controls.MyButton BUTsetupshow;
        private ArdupilotMega.Controls.MyButton BUTradiosetup;
        private ArdupilotMega.Controls.MyButton BUTtests;
        private ArdupilotMega.Controls.MyButton Logs;
        private ArdupilotMega.Controls.MyButton BUT_logbrowse;
    }
}

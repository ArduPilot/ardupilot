namespace ArdupilotMega.Controls
{
    partial class ConnectionControl
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
            this.cmb_Baud = new System.Windows.Forms.ComboBox();
            this.cmb_ConnectionType = new System.Windows.Forms.ComboBox();
            this.cmb_Connection = new System.Windows.Forms.ComboBox();
            this.linkLabel1 = new System.Windows.Forms.LinkLabel();
            this.SuspendLayout();
            // 
            // cmb_Baud
            // 
            this.cmb_Baud.FormattingEnabled = true;
            this.cmb_Baud.Items.AddRange(new object[] {
            "4800",
            "9600",
            "14400",
            "19200",
            "28800",
            "38400",
            "57600",
            "115200"});
            this.cmb_Baud.Location = new System.Drawing.Point(130, 12);
            this.cmb_Baud.Name = "cmb_Baud";
            this.cmb_Baud.Size = new System.Drawing.Size(70, 21);
            this.cmb_Baud.TabIndex = 0;
            // 
            // cmb_ConnectionType
            // 
            this.cmb_ConnectionType.FormattingEnabled = true;
            this.cmb_ConnectionType.Location = new System.Drawing.Point(79, 39);
            this.cmb_ConnectionType.Name = "cmb_ConnectionType";
            this.cmb_ConnectionType.Size = new System.Drawing.Size(121, 21);
            this.cmb_ConnectionType.TabIndex = 1;
            this.cmb_ConnectionType.Visible = false;
            // 
            // cmb_Connection
            // 
            this.cmb_Connection.FormattingEnabled = true;
            this.cmb_Connection.Location = new System.Drawing.Point(3, 12);
            this.cmb_Connection.Name = "cmb_Connection";
            this.cmb_Connection.Size = new System.Drawing.Size(121, 21);
            this.cmb_Connection.TabIndex = 2;
            // 
            // linkLabel1
            // 
            this.linkLabel1.AutoSize = true;
            this.linkLabel1.Image = global::ArdupilotMega.Properties.Resources.bg;
            this.linkLabel1.Location = new System.Drawing.Point(3, 55);
            this.linkLabel1.Name = "linkLabel1";
            this.linkLabel1.Size = new System.Drawing.Size(63, 13);
            this.linkLabel1.TabIndex = 3;
            this.linkLabel1.TabStop = true;
            this.linkLabel1.Text = "Link Stats...";
            this.linkLabel1.Visible = false;
            // 
            // ConnectionControl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackgroundImage = global::ArdupilotMega.Properties.Resources.bg;
            this.Controls.Add(this.linkLabel1);
            this.Controls.Add(this.cmb_Connection);
            this.Controls.Add(this.cmb_ConnectionType);
            this.Controls.Add(this.cmb_Baud);
            this.Name = "ConnectionControl";
            this.Size = new System.Drawing.Size(230, 76);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox cmb_Baud;
        private System.Windows.Forms.ComboBox cmb_ConnectionType;
        private System.Windows.Forms.ComboBox cmb_Connection;
        private System.Windows.Forms.LinkLabel linkLabel1;
    }
}

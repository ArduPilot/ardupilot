namespace _3DRRadio
{
    partial class Rssi
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
            this.zedGraphControl1 = new ZedGraph.ZedGraphControl();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.BUT_disconnect = new ArdupilotMega.Controls.MyButton();
            this.BUT_connect = new ArdupilotMega.Controls.MyButton();
            this.SuspendLayout();
            // 
            // zedGraphControl1
            // 
            this.zedGraphControl1.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.zedGraphControl1.Location = new System.Drawing.Point(4, 33);
            this.zedGraphControl1.Name = "zedGraphControl1";
            this.zedGraphControl1.ScrollGrace = 0D;
            this.zedGraphControl1.ScrollMaxX = 0D;
            this.zedGraphControl1.ScrollMaxY = 0D;
            this.zedGraphControl1.ScrollMaxY2 = 0D;
            this.zedGraphControl1.ScrollMinX = 0D;
            this.zedGraphControl1.ScrollMinY = 0D;
            this.zedGraphControl1.ScrollMinY2 = 0D;
            this.zedGraphControl1.Size = new System.Drawing.Size(485, 353);
            this.zedGraphControl1.TabIndex = 0;
            // 
            // timer1
            // 
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // BUT_disconnect
            // 
            this.BUT_disconnect.Anchor = System.Windows.Forms.AnchorStyles.Top;
            this.BUT_disconnect.Enabled = false;
            this.BUT_disconnect.Location = new System.Drawing.Point(250, 4);
            this.BUT_disconnect.Name = "BUT_disconnect";
            this.BUT_disconnect.Size = new System.Drawing.Size(75, 23);
            this.BUT_disconnect.TabIndex = 2;
            this.BUT_disconnect.Text = "Disconnect";
            this.BUT_disconnect.UseVisualStyleBackColor = true;
            this.BUT_disconnect.Click += new System.EventHandler(this.BUT_disconnect_Click);
            // 
            // BUT_connect
            // 
            this.BUT_connect.Anchor = System.Windows.Forms.AnchorStyles.Top;
            this.BUT_connect.Location = new System.Drawing.Point(169, 4);
            this.BUT_connect.Name = "BUT_connect";
            this.BUT_connect.Size = new System.Drawing.Size(75, 23);
            this.BUT_connect.TabIndex = 1;
            this.BUT_connect.Text = "Connect";
            this.BUT_connect.UseVisualStyleBackColor = true;
            this.BUT_connect.Click += new System.EventHandler(this.BUT_connect_Click);
            // 
            // Rssi
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.BUT_disconnect);
            this.Controls.Add(this.BUT_connect);
            this.Controls.Add(this.zedGraphControl1);
            this.Name = "Rssi";
            this.Size = new System.Drawing.Size(492, 386);
            this.ResumeLayout(false);

        }

        #endregion

        private ZedGraph.ZedGraphControl zedGraphControl1;
        private ArdupilotMega.Controls.MyButton BUT_connect;
        private ArdupilotMega.Controls.MyButton BUT_disconnect;
        private System.Windows.Forms.Timer timer1;
    }
}

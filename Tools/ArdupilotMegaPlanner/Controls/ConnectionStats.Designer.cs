namespace ArdupilotMega.Controls
{
    partial class ConnectionStats
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
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.txt_BytesReceived = new System.Windows.Forms.TextBox();
            this.txt_BytesPerSecondRx = new System.Windows.Forms.TextBox();
            this.txt_PacketsRx = new System.Windows.Forms.TextBox();
            this.txt_PacketsLost = new System.Windows.Forms.TextBox();
            this.txt_LinkQuality = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.txt_PacketsPerSecond = new System.Windows.Forms.TextBox();
            this.txt_BytesSent = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.txt_BytesPerSecondSent = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.label10 = new System.Windows.Forms.Label();
            this.txt_MaxPacketInterval = new System.Windows.Forms.TextBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.SuspendLayout();
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(10, 18);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(33, 13);
            this.label1.TabIndex = 0;
            this.label1.Text = "Bytes";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(135, 18);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(43, 13);
            this.label2.TabIndex = 1;
            this.label2.Text = "Bytes/s";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(6, 44);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(46, 13);
            this.label3.TabIndex = 2;
            this.label3.Text = "Packets";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(3, 70);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(51, 13);
            this.label4.TabIndex = 3;
            this.label4.Text = "Pkts Lost";
            // 
            // txt_BytesReceived
            // 
            this.txt_BytesReceived.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.txt_BytesReceived.Location = new System.Drawing.Point(58, 16);
            this.txt_BytesReceived.Name = "txt_BytesReceived";
            this.txt_BytesReceived.ReadOnly = true;
            this.txt_BytesReceived.Size = new System.Drawing.Size(64, 20);
            this.txt_BytesReceived.TabIndex = 4;
            // 
            // txt_BytesPerSecondRx
            // 
            this.txt_BytesPerSecondRx.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.txt_BytesPerSecondRx.Location = new System.Drawing.Point(184, 16);
            this.txt_BytesPerSecondRx.Name = "txt_BytesPerSecondRx";
            this.txt_BytesPerSecondRx.ReadOnly = true;
            this.txt_BytesPerSecondRx.Size = new System.Drawing.Size(51, 20);
            this.txt_BytesPerSecondRx.TabIndex = 5;
            // 
            // txt_PacketsRx
            // 
            this.txt_PacketsRx.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.txt_PacketsRx.Location = new System.Drawing.Point(58, 42);
            this.txt_PacketsRx.Name = "txt_PacketsRx";
            this.txt_PacketsRx.ReadOnly = true;
            this.txt_PacketsRx.Size = new System.Drawing.Size(64, 20);
            this.txt_PacketsRx.TabIndex = 6;
            // 
            // txt_PacketsLost
            // 
            this.txt_PacketsLost.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.txt_PacketsLost.Location = new System.Drawing.Point(58, 68);
            this.txt_PacketsLost.Name = "txt_PacketsLost";
            this.txt_PacketsLost.ReadOnly = true;
            this.txt_PacketsLost.Size = new System.Drawing.Size(64, 20);
            this.txt_PacketsLost.TabIndex = 7;
            // 
            // txt_LinkQuality
            // 
            this.txt_LinkQuality.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.txt_LinkQuality.Location = new System.Drawing.Point(184, 68);
            this.txt_LinkQuality.Name = "txt_LinkQuality";
            this.txt_LinkQuality.ReadOnly = true;
            this.txt_LinkQuality.Size = new System.Drawing.Size(51, 20);
            this.txt_LinkQuality.TabIndex = 9;
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(139, 68);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(39, 13);
            this.label5.TabIndex = 8;
            this.label5.Text = "Quality";
            // 
            // txt_PacketsPerSecond
            // 
            this.txt_PacketsPerSecond.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.txt_PacketsPerSecond.Location = new System.Drawing.Point(184, 42);
            this.txt_PacketsPerSecond.Name = "txt_PacketsPerSecond";
            this.txt_PacketsPerSecond.ReadOnly = true;
            this.txt_PacketsPerSecond.Size = new System.Drawing.Size(51, 20);
            this.txt_PacketsPerSecond.TabIndex = 10;
            // 
            // txt_BytesSent
            // 
            this.txt_BytesSent.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.txt_BytesSent.Location = new System.Drawing.Point(74, 16);
            this.txt_BytesSent.Name = "txt_BytesSent";
            this.txt_BytesSent.ReadOnly = true;
            this.txt_BytesSent.Size = new System.Drawing.Size(64, 20);
            this.txt_BytesSent.TabIndex = 12;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(6, 18);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(60, 13);
            this.label6.TabIndex = 11;
            this.label6.Text = "Total Bytes";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(23, 43);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(43, 13);
            this.label7.TabIndex = 13;
            this.label7.Text = "Bytes/s";
            // 
            // txt_BytesPerSecondSent
            // 
            this.txt_BytesPerSecondSent.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.txt_BytesPerSecondSent.Location = new System.Drawing.Point(74, 42);
            this.txt_BytesPerSecondSent.Name = "txt_BytesPerSecondSent";
            this.txt_BytesPerSecondSent.ReadOnly = true;
            this.txt_BytesPerSecondSent.Size = new System.Drawing.Size(64, 20);
            this.txt_BytesPerSecondSent.TabIndex = 14;
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(126, 43);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(56, 13);
            this.label8.TabIndex = 15;
            this.label8.Text = "Packets/s";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.label10);
            this.groupBox1.Controls.Add(this.txt_MaxPacketInterval);
            this.groupBox1.Controls.Add(this.label8);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.label4);
            this.groupBox1.Controls.Add(this.txt_BytesReceived);
            this.groupBox1.Controls.Add(this.txt_PacketsPerSecond);
            this.groupBox1.Controls.Add(this.txt_BytesPerSecondRx);
            this.groupBox1.Controls.Add(this.txt_LinkQuality);
            this.groupBox1.Controls.Add(this.txt_PacketsRx);
            this.groupBox1.Controls.Add(this.label5);
            this.groupBox1.Controls.Add(this.txt_PacketsLost);
            this.groupBox1.Location = new System.Drawing.Point(4, 4);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(245, 124);
            this.groupBox1.TabIndex = 16;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Download";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(22, 96);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(156, 13);
            this.label10.TabIndex = 16;
            this.label10.Text = "Max time between packets (ms)";
            // 
            // txt_MaxPacketInterval
            // 
            this.txt_MaxPacketInterval.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.txt_MaxPacketInterval.Location = new System.Drawing.Point(184, 94);
            this.txt_MaxPacketInterval.Name = "txt_MaxPacketInterval";
            this.txt_MaxPacketInterval.ReadOnly = true;
            this.txt_MaxPacketInterval.Size = new System.Drawing.Size(51, 20);
            this.txt_MaxPacketInterval.TabIndex = 17;
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.txt_BytesPerSecondSent);
            this.groupBox2.Controls.Add(this.label7);
            this.groupBox2.Controls.Add(this.label6);
            this.groupBox2.Controls.Add(this.txt_BytesSent);
            this.groupBox2.Location = new System.Drawing.Point(255, 4);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(144, 83);
            this.groupBox2.TabIndex = 17;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Upload";
            // 
            // ConnectionStats
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.groupBox2);
            this.Name = "ConnectionStats";
            this.Size = new System.Drawing.Size(408, 136);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox txt_BytesReceived;
        private System.Windows.Forms.TextBox txt_BytesPerSecondRx;
        private System.Windows.Forms.TextBox txt_PacketsRx;
        private System.Windows.Forms.TextBox txt_PacketsLost;
        private System.Windows.Forms.TextBox txt_LinkQuality;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox txt_PacketsPerSecond;
        private System.Windows.Forms.TextBox txt_BytesSent;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox txt_BytesPerSecondSent;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox txt_MaxPacketInterval;
    }
}

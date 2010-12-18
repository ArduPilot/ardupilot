namespace ArducopterConfigurator.Views
{
    partial class TransmitterChannelsView
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
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.progressBar3 = new System.Windows.Forms.ProgressBar();
            this.label9 = new System.Windows.Forms.Label();
            this.textBox6 = new System.Windows.Forms.TextBox();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.label3 = new System.Windows.Forms.Label();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.textBox3 = new System.Windows.Forms.TextBox();
            this.textBox4 = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.textBox5 = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.TransmitterChannelsBindingSource = new System.Windows.Forms.BindingSource(this.components);
            this.verticalProgressBar4 = new ArducopterConfigurator.VerticalProgressBar();
            this.verticalProgressBar3 = new ArducopterConfigurator.VerticalProgressBar();
            this.verticalProgressBar2 = new ArducopterConfigurator.VerticalProgressBar();
            this.verticalProgressBar1 = new ArducopterConfigurator.VerticalProgressBar();
            ((System.ComponentModel.ISupportInitialize)(this.TransmitterChannelsBindingSource)).BeginInit();
            this.SuspendLayout();
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(4, 11);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(43, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "Throttle";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(56, 11);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(31, 13);
            this.label2.TabIndex = 3;
            this.label2.Text = "Pitch";
            // 
            // progressBar3
            // 
            this.progressBar3.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Yaw", true));
            this.progressBar3.Location = new System.Drawing.Point(174, 30);
            this.progressBar3.Maximum = 2000;
            this.progressBar3.Minimum = 1000;
            this.progressBar3.Name = "progressBar3";
            this.progressBar3.Size = new System.Drawing.Size(100, 23);
            this.progressBar3.TabIndex = 31;
            this.progressBar3.Value = 1000;
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(171, 14);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(28, 13);
            this.label9.TabIndex = 30;
            this.label9.Text = "Yaw";
            // 
            // textBox6
            // 
            this.textBox6.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Yaw", true));
            this.textBox6.Location = new System.Drawing.Point(280, 30);
            this.textBox6.Name = "textBox6";
            this.textBox6.ReadOnly = true;
            this.textBox6.Size = new System.Drawing.Size(47, 20);
            this.textBox6.TabIndex = 29;
            // 
            // progressBar1
            // 
            this.progressBar1.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Roll", true));
            this.progressBar1.Location = new System.Drawing.Point(174, 80);
            this.progressBar1.Maximum = 2000;
            this.progressBar1.Minimum = 1000;
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Size = new System.Drawing.Size(100, 23);
            this.progressBar1.TabIndex = 34;
            this.progressBar1.Value = 1000;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(171, 64);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(25, 13);
            this.label3.TabIndex = 33;
            this.label3.Text = "Roll";
            // 
            // textBox1
            // 
            this.textBox1.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Roll", true));
            this.textBox1.Location = new System.Drawing.Point(280, 80);
            this.textBox1.Name = "textBox1";
            this.textBox1.ReadOnly = true;
            this.textBox1.Size = new System.Drawing.Size(47, 20);
            this.textBox1.TabIndex = 32;
            // 
            // textBox2
            // 
            this.textBox2.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Throttle", true));
            this.textBox2.Location = new System.Drawing.Point(7, 147);
            this.textBox2.Name = "textBox2";
            this.textBox2.ReadOnly = true;
            this.textBox2.Size = new System.Drawing.Size(47, 20);
            this.textBox2.TabIndex = 35;
            // 
            // textBox3
            // 
            this.textBox3.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Pitch", true));
            this.textBox3.Location = new System.Drawing.Point(59, 147);
            this.textBox3.Name = "textBox3";
            this.textBox3.ReadOnly = true;
            this.textBox3.Size = new System.Drawing.Size(47, 20);
            this.textBox3.TabIndex = 36;
            // 
            // textBox4
            // 
            this.textBox4.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Mode", true));
            this.textBox4.Location = new System.Drawing.Point(137, 210);
            this.textBox4.Name = "textBox4";
            this.textBox4.ReadOnly = true;
            this.textBox4.Size = new System.Drawing.Size(47, 20);
            this.textBox4.TabIndex = 39;
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(141, 115);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(34, 13);
            this.label4.TabIndex = 38;
            this.label4.Text = "Mode";
            // 
            // textBox5
            // 
            this.textBox5.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.TransmitterChannelsBindingSource, "Aux", true));
            this.textBox5.Location = new System.Drawing.Point(197, 210);
            this.textBox5.Name = "textBox5";
            this.textBox5.ReadOnly = true;
            this.textBox5.Size = new System.Drawing.Size(47, 20);
            this.textBox5.TabIndex = 42;
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(201, 115);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(25, 13);
            this.label5.TabIndex = 41;
            this.label5.Text = "Aux";
            // 
            // TransmitterChannelsBindingSource
            // 
            this.TransmitterChannelsBindingSource.DataSource = typeof(ArducopterConfigurator.PresentationModels.TransmitterChannelsVm);
            // 
            // verticalProgressBar4
            // 
            this.verticalProgressBar4.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Aux", true));
            this.verticalProgressBar4.Location = new System.Drawing.Point(205, 131);
            this.verticalProgressBar4.Maximum = 2000;
            this.verticalProgressBar4.Minimum = 1000;
            this.verticalProgressBar4.Name = "verticalProgressBar4";
            this.verticalProgressBar4.Size = new System.Drawing.Size(27, 73);
            this.verticalProgressBar4.TabIndex = 40;
            this.verticalProgressBar4.Value = 1000;
            // 
            // verticalProgressBar3
            // 
            this.verticalProgressBar3.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Mode", true));
            this.verticalProgressBar3.Location = new System.Drawing.Point(145, 131);
            this.verticalProgressBar3.Maximum = 2000;
            this.verticalProgressBar3.Minimum = 1000;
            this.verticalProgressBar3.Name = "verticalProgressBar3";
            this.verticalProgressBar3.Size = new System.Drawing.Size(27, 73);
            this.verticalProgressBar3.TabIndex = 37;
            this.verticalProgressBar3.Value = 1000;
            // 
            // verticalProgressBar2
            // 
            this.verticalProgressBar2.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Pitch", true));
            this.verticalProgressBar2.Location = new System.Drawing.Point(67, 30);
            this.verticalProgressBar2.Maximum = 2000;
            this.verticalProgressBar2.Minimum = 1000;
            this.verticalProgressBar2.Name = "verticalProgressBar2";
            this.verticalProgressBar2.Size = new System.Drawing.Size(27, 111);
            this.verticalProgressBar2.TabIndex = 2;
            this.verticalProgressBar2.Value = 1000;
            // 
            // verticalProgressBar1
            // 
            this.verticalProgressBar1.DataBindings.Add(new System.Windows.Forms.Binding("Value", this.TransmitterChannelsBindingSource, "Throttle", true));
            this.verticalProgressBar1.Location = new System.Drawing.Point(15, 30);
            this.verticalProgressBar1.Maximum = 2000;
            this.verticalProgressBar1.Minimum = 1000;
            this.verticalProgressBar1.Name = "verticalProgressBar1";
            this.verticalProgressBar1.Size = new System.Drawing.Size(27, 111);
            this.verticalProgressBar1.TabIndex = 0;
            this.verticalProgressBar1.Value = 1000;
            // 
            // TransmitterChannelsView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.textBox5);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.verticalProgressBar4);
            this.Controls.Add(this.textBox4);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.verticalProgressBar3);
            this.Controls.Add(this.textBox3);
            this.Controls.Add(this.textBox2);
            this.Controls.Add(this.progressBar1);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.progressBar3);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.textBox6);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.verticalProgressBar2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.verticalProgressBar1);
            this.Name = "TransmitterChannelsView";
            this.Size = new System.Drawing.Size(369, 244);
            ((System.ComponentModel.ISupportInitialize)(this.TransmitterChannelsBindingSource)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private VerticalProgressBar verticalProgressBar1;
        private System.Windows.Forms.BindingSource TransmitterChannelsBindingSource;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private VerticalProgressBar verticalProgressBar2;
        private System.Windows.Forms.ProgressBar progressBar3;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox textBox6;
        private System.Windows.Forms.ProgressBar progressBar1;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.TextBox textBox2;
        private System.Windows.Forms.TextBox textBox3;
        private System.Windows.Forms.TextBox textBox4;
        private System.Windows.Forms.Label label4;
        private VerticalProgressBar verticalProgressBar3;
        private System.Windows.Forms.TextBox textBox5;
        private System.Windows.Forms.Label label5;
        private VerticalProgressBar verticalProgressBar4;
    }
}

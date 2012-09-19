namespace ArdupilotMega.Controls
{
    partial class QuickView
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
            this.labelWithPseudoOpacity1 = new ArdupilotMega.Controls.LabelWithPseudoOpacity();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.labelWithPseudoOpacity2 = new ArdupilotMega.Controls.LabelWithPseudoOpacity();
            this.tableLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // labelWithPseudoOpacity1
            // 
            this.labelWithPseudoOpacity1.AutoSize = true;
            this.labelWithPseudoOpacity1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.labelWithPseudoOpacity1.DoubleBuffered = true;
            this.labelWithPseudoOpacity1.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelWithPseudoOpacity1.Location = new System.Drawing.Point(3, 0);
            this.labelWithPseudoOpacity1.Name = "labelWithPseudoOpacity1";
            this.labelWithPseudoOpacity1.Size = new System.Drawing.Size(118, 20);
            this.labelWithPseudoOpacity1.TabIndex = 0;
            this.labelWithPseudoOpacity1.Text = "Altitude:";
            this.labelWithPseudoOpacity1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 1;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel1.Controls.Add(this.labelWithPseudoOpacity2, 0, 1);
            this.tableLayoutPanel1.Controls.Add(this.labelWithPseudoOpacity1, 0, 0);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 2;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(122, 72);
            this.tableLayoutPanel1.TabIndex = 2;
            // 
            // labelWithPseudoOpacity2
            // 
            this.labelWithPseudoOpacity2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.labelWithPseudoOpacity2.DoubleBuffered = true;
            this.labelWithPseudoOpacity2.Font = new System.Drawing.Font("Microsoft Sans Serif", 21.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelWithPseudoOpacity2.Location = new System.Drawing.Point(3, 20);
            this.labelWithPseudoOpacity2.Name = "labelWithPseudoOpacity2";
            this.labelWithPseudoOpacity2.Size = new System.Drawing.Size(118, 52);
            this.labelWithPseudoOpacity2.TabIndex = 2;
            this.labelWithPseudoOpacity2.Text = "000.00";
            this.labelWithPseudoOpacity2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // QuickView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.tableLayoutPanel1);
            this.Name = "QuickView";
            this.Size = new System.Drawing.Size(122, 72);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private LabelWithPseudoOpacity labelWithPseudoOpacity1;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private LabelWithPseudoOpacity labelWithPseudoOpacity2;
    }
}

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
            this.labelWithPseudoOpacity2 = new ArdupilotMega.Controls.LabelWithPseudoOpacity();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.tableLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // labelWithPseudoOpacity1
            // 
            this.labelWithPseudoOpacity1.AutoSize = true;
            this.labelWithPseudoOpacity1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.labelWithPseudoOpacity1.Font = new System.Drawing.Font("Microsoft Sans Serif", 18F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelWithPseudoOpacity1.Location = new System.Drawing.Point(3, 0);
            this.labelWithPseudoOpacity1.Name = "labelWithPseudoOpacity1";
            this.labelWithPseudoOpacity1.Size = new System.Drawing.Size(161, 50);
            this.labelWithPseudoOpacity1.TabIndex = 0;
            this.labelWithPseudoOpacity1.Text = "Altitude:";
            this.labelWithPseudoOpacity1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // labelWithPseudoOpacity2
            // 
            this.labelWithPseudoOpacity2.AutoSize = true;
            this.labelWithPseudoOpacity2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.labelWithPseudoOpacity2.Font = new System.Drawing.Font("Microsoft Sans Serif", 36F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelWithPseudoOpacity2.Location = new System.Drawing.Point(170, 0);
            this.labelWithPseudoOpacity2.Name = "labelWithPseudoOpacity2";
            this.labelWithPseudoOpacity2.Size = new System.Drawing.Size(162, 50);
            this.labelWithPseudoOpacity2.TabIndex = 1;
            this.labelWithPseudoOpacity2.Text = "0.0";
            this.labelWithPseudoOpacity2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 2;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.Controls.Add(this.labelWithPseudoOpacity2, 1, 0);
            this.tableLayoutPanel1.Controls.Add(this.labelWithPseudoOpacity1, 0, 0);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 1;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(335, 50);
            this.tableLayoutPanel1.TabIndex = 2;
            // 
            // QuickView
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.tableLayoutPanel1);
            this.MinimumSize = new System.Drawing.Size(100, 27);
            this.Name = "QuickView";
            this.Size = new System.Drawing.Size(335, 50);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private LabelWithPseudoOpacity labelWithPseudoOpacity1;
        private LabelWithPseudoOpacity labelWithPseudoOpacity2;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
    }
}

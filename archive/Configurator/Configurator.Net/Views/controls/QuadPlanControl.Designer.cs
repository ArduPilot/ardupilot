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
            this.cirularIndicatorControl1 = new ArducopterConfigurator.Views.controls.CirularIndicatorControl();
            this.linearIndicatorControl1 = new ArducopterConfigurator.Views.controls.LinearIndicatorControl();
            this.SuspendLayout();
            // 
            // cirularIndicatorControl1
            // 
            this.cirularIndicatorControl1.BackColor = System.Drawing.SystemColors.Control;
            this.cirularIndicatorControl1.BarBackgroundDark = System.Drawing.Color.Transparent;
            this.cirularIndicatorControl1.BarBackgroundLight = System.Drawing.Color.Transparent;
            this.cirularIndicatorControl1.BarBorderColor = System.Drawing.Color.Black;
            this.cirularIndicatorControl1.BarBorderWidth = 3F;
            this.cirularIndicatorControl1.BarDark = System.Drawing.Color.Maroon;
            this.cirularIndicatorControl1.BarLight = System.Drawing.Color.Moccasin;
            this.cirularIndicatorControl1.IsReveresed = false;
            this.cirularIndicatorControl1.IsVertical = true;
            this.cirularIndicatorControl1.Location = new System.Drawing.Point(18, 71);
            this.cirularIndicatorControl1.Max = 100;
            this.cirularIndicatorControl1.Min = 0;
            this.cirularIndicatorControl1.Name = "cirularIndicatorControl1";
            this.cirularIndicatorControl1.Offset = 0;
            this.cirularIndicatorControl1.Size = new System.Drawing.Size(50, 50);
            this.cirularIndicatorControl1.TabIndex = 1;
            this.cirularIndicatorControl1.Value = 56;
            // 
            // linearIndicatorControl1
            // 
            this.linearIndicatorControl1.BarBackgroundDark = System.Drawing.Color.FromArgb(((int)(((byte)(199)))), ((int)(((byte)(200)))), ((int)(((byte)(201)))));
            this.linearIndicatorControl1.BarBackgroundLight = System.Drawing.Color.WhiteSmoke;
            this.linearIndicatorControl1.BarBorderColor = System.Drawing.Color.DarkGray;
            this.linearIndicatorControl1.BarDark = System.Drawing.Color.FromArgb(((int)(((byte)(40)))), ((int)(((byte)(68)))), ((int)(((byte)(202)))));
            this.linearIndicatorControl1.BarLight = System.Drawing.Color.FromArgb(((int)(((byte)(102)))), ((int)(((byte)(144)))), ((int)(((byte)(252)))));
            this.linearIndicatorControl1.IsVertical = false;
            this.linearIndicatorControl1.Location = new System.Drawing.Point(18, 26);
            this.linearIndicatorControl1.Max = 100;
            this.linearIndicatorControl1.MaxWaterMark = 0;
            this.linearIndicatorControl1.Min = 0;
            this.linearIndicatorControl1.MinWatermark = 0;
            this.linearIndicatorControl1.Name = "linearIndicatorControl1";
            this.linearIndicatorControl1.Offset = 0;
            this.linearIndicatorControl1.Size = new System.Drawing.Size(81, 24);
            this.linearIndicatorControl1.TabIndex = 2;
            this.linearIndicatorControl1.Value = 50;
            this.linearIndicatorControl1.WatermarkLineColor = System.Drawing.Color.DarkGray;
            // 
            // QuadPlanControl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.cirularIndicatorControl1);
            this.Controls.Add(this.linearIndicatorControl1);
            this.Name = "QuadPlanControl";
            this.ResumeLayout(false);

        }

        #endregion

        private CirularIndicatorControl cirularIndicatorControl1;
        private LinearIndicatorControl linearIndicatorControl1;
    }
}

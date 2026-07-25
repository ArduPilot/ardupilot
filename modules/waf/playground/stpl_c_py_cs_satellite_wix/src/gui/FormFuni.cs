using System;
using System.Windows.Forms;
using System.Threading;
using System.Resources;
using Funi;

namespace Funi
{
    public class FormFuni : Form
    {
        public FormFuni()
        {
            InitializeComponent();
        }

        private static System.Resources.ResourceManager res = new System.Resources.ResourceManager("funigui.Resources", typeof(FormFuni).Assembly);

        private void InitializeComponent()
        {
            this.text1 = new System.Windows.Forms.Label();
            this.SuspendLayout();
            string greeting = res.GetString("greeting");
            // text1
            this.text1.Dock = System.Windows.Forms.DockStyle.Top;
            this.text1.Location = new System.Drawing.Point(0, 0);
            this.text1.Name = "text1";
            this.text1.Text = greeting + Func.func3(4).ToString();
            this.text1.Size = new System.Drawing.Size(289, 369);
            this.text1.TabIndex = 0;
            /// text1
            // FormFuni
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(200, 200);
            this.Controls.Add(this.text1);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.Name = "FormFuni";
            this.Text = greeting;
            /// FormFuni
            this.ResumeLayout(false);

        }

        private System.Windows.Forms.Label text1;
    }
}

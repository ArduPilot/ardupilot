using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega.Controls
{
    public partial class QuickView : UserControl
    {
        [System.ComponentModel.Browsable(true)]
        public string desc { get { return labelWithPseudoOpacity1.Text; } set { if (labelWithPseudoOpacity1.Text == value) return; labelWithPseudoOpacity1.Text = value; } }
        [System.ComponentModel.Browsable(true)]
        public string number { get { return labelWithPseudoOpacity2.Text; } set { if (labelWithPseudoOpacity2.Text == value) return; labelWithPseudoOpacity2.Text = value; } }
        [System.ComponentModel.Browsable(true)]
        public Color numberColor { get { return labelWithPseudoOpacity2.ForeColor; } set { if (labelWithPseudoOpacity2.ForeColor == value) return; labelWithPseudoOpacity2.ForeColor = value; } }

        public QuickView()
        {
            InitializeComponent();

            labelWithPseudoOpacity1.DoubleClick += new EventHandler(labelWithPseudoOpacity1_DoubleClick);
            labelWithPseudoOpacity2.DoubleClick += new EventHandler(labelWithPseudoOpacity2_DoubleClick);
        }

        void labelWithPseudoOpacity2_DoubleClick(object sender, EventArgs e)
        {
            this.OnDoubleClick(e);
        }

        void labelWithPseudoOpacity1_DoubleClick(object sender, EventArgs e)
        {
            this.OnDoubleClick(e);
        }

        public override void Refresh()
        {
            if (this.Visible)
                base.Refresh();
        }

        protected override void OnInvalidated(InvalidateEventArgs e)
        {
            if (this.Visible)
                base.OnInvalidated(e);
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            if (this.Visible)
                base.OnPaint(e);
        }

        protected override void OnResize(EventArgs e)
        {
            if (this.Height > 20)
                labelWithPseudoOpacity2.Font = new Font(labelWithPseudoOpacity2.Font.FontFamily, this.Height * 0.7f);

            base.OnResize(e);
        }
    }
}

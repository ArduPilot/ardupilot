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
        public double number { get { return double.Parse(labelWithPseudoOpacity2.Text); } 
            set { 
                string ans = (value).ToString("0.00");
                if (labelWithPseudoOpacity2.Text == ans) 
                    return;
                labelWithPseudoOpacity2.Text = ans;
                GetFontSize();
            }
        }
        [System.ComponentModel.Browsable(true)]
        public Color numberColor { get { return labelWithPseudoOpacity2.ForeColor; } set { if (labelWithPseudoOpacity2.ForeColor == value) return; labelWithPseudoOpacity2.ForeColor = value; } }

        public QuickView()
        {
            InitializeComponent();

            labelWithPseudoOpacity1.DoubleClick += new EventHandler(labelWithPseudoOpacity1_DoubleClick);
            labelWithPseudoOpacity2.DoubleClick += new EventHandler(labelWithPseudoOpacity2_DoubleClick);

            labelWithPseudoOpacity2.DoubleBuffered = true;
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

        void GetFontSize()
        {

            Size extent = TextRenderer.MeasureText(labelWithPseudoOpacity2.Text, this.Font);

            float hRatio = (labelWithPseudoOpacity2.Height) / (float)(extent.Height);
            float wRatio = this.Width / (float)extent.Width;
            float ratio = (hRatio < wRatio) ? hRatio : wRatio;

            float newSize = this.Font.Size * ratio;

            if (newSize < 8)
                newSize = 8;

            //return newSize;

            labelWithPseudoOpacity2.Font = new Font(labelWithPseudoOpacity2.Font.FontFamily, newSize - 2, labelWithPseudoOpacity2.Font.Style);

            extent = TextRenderer.MeasureText(labelWithPseudoOpacity2.Text, labelWithPseudoOpacity2.Font);
        }

        protected override void OnResize(EventArgs e)
        {
            this.ResizeRedraw = true;

            GetFontSize();

            base.OnResize(e);
        }
    }
}

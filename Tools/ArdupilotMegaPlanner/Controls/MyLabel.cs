using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega
{
    /// <summary>
    /// profiling showed that the built in Label function was using alot ot call time.
    /// </summary>
    public partial class MyLabel : Control //: Label
    {
        string label = "";
        int noofchars = 0;
        bool autosize = false;

        [System.ComponentModel.Browsable(true)]
        public bool resize { get { return autosize; } set { autosize = value; } }

        public MyLabel()
        {
        }

        public override string Text
        {
            get
            {
                return label;
            }
            set
            {

                if (value == null)
                    return;

                if (label == value)
                    return;

                label = value;

                if (noofchars != label.Length && resize)
                {
                    noofchars = label.Length;
                    Size textSize = TextRenderer.MeasureText(value, this.Font);
                    this.Width = textSize.Width;
                }

                this.Invalidate();
            }
        }

        SolidBrush s = new SolidBrush(Color.White);
        SolidBrush b = new SolidBrush(Color.Black);

        StringFormat stringFormat = new StringFormat();

        protected override void OnPaint(PaintEventArgs e)
        {
            stringFormat.Alignment = StringAlignment.Near;
            stringFormat.LineAlignment = StringAlignment.Center;

            s = new SolidBrush(ForeColor);

            e.Graphics.DrawString(label, this.Font, s, new PointF(0, this.Height / 2.0f), stringFormat);
        }

        protected override void OnPaintBackground(PaintEventArgs pevent)
        {
            b = new SolidBrush(BackColor);

            pevent.Graphics.FillRectangle(b, this.ClientRectangle);

            base.OnPaintBackground(pevent);
        }
        
        protected override void WndProc(ref Message m) // seems to crash here on linux... so try ignore it
        {
            try
            {
                base.WndProc(ref m);
            }
            catch { }
        }
    }
}

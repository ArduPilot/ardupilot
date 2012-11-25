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
    /// <summary>
    /// profiling showed that the built in Label function was using alot of call time.
    /// </summary>
    public partial class MyLabel : Control //: Label
    {
        string label = "";
        int noofchars = 0;
        bool autosize = false;

        SolidBrush s = new SolidBrush(SystemColors.ControlText);
        SolidBrush b = new SolidBrush(SystemColors.Control);

        StringFormat stringFormat = new StringFormat();

        [System.ComponentModel.Browsable(true)]
        public bool resize { get { return autosize; } set { autosize = value; } }

        public MyLabel()
        {
            stringFormat.Alignment = StringAlignment.Near;
            stringFormat.LineAlignment = StringAlignment.Center;
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

                if (this.Visible && ThisReallyVisible())
                    this.Invalidate();
            }
        }

        /// <summary>
        /// this is to fix a mono off screen drawing issue
        /// </summary>
        /// <returns></returns>
        public bool ThisReallyVisible()
        {
            if (Parent != null)
                return this.Bounds.IntersectsWith(Parent.ClientRectangle);

            return true;
        }

        public override void Refresh()
        {
            base.Refresh();
        }

        protected override void OnParentBindingContextChanged(EventArgs e)
        {
            base.OnParentBindingContextChanged(e);
        }

        protected override void OnVisibleChanged(EventArgs e)
        {
            base.OnVisibleChanged(e);
        }

        public override Color BackColor
        {
            get
            {
                return base.BackColor;
            }
            set
            {
                base.BackColor = value;
                b = new SolidBrush(value);
                this.Invalidate();
            }
        }


        public override System.Drawing.Color ForeColor
        {
            get
            {
                return base.ForeColor;
            }
            set
            {
                base.ForeColor = value;
                s = new SolidBrush(value);
                this.Invalidate();
            }
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            
           // TextRenderer.DrawText(e.Graphics, label, this.Font, new Point(0, 0), ForeColor);

            e.Graphics.DrawString(label, this.Font, s, new PointF(0, this.Height / 2.0f), stringFormat);
        }

        protected override void OnPaintBackground(PaintEventArgs pevent)
        {
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

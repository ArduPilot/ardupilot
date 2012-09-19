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
    public partial class HSI : UserControl
    {
        Bitmap _headingimage;
        bool drawnheading = false;

        int _heading = 0;
        int _navbearing = 0;

        [System.ComponentModel.Browsable(true)]
        public int Heading
        {
            get { return _heading; }
            set { _heading = value; this.Invalidate(); }
        }

        [System.ComponentModel.Browsable(true)]
        public int NavHeading
        {
            get { return _navbearing; }
            set { _navbearing = value; }
        }

        /// <summary>
        /// Override to prevent offscreen drawing the control - mono mac
        /// </summary>
        public new void Invalidate()
        {
            if (Disposing)
                return;
            if (!ThisReallyVisible())
            {
                return;
            }

            base.Invalidate();
        }

        /// <summary>
        /// this is to fix a mono off screen drawing issue
        /// </summary>
        /// <returns></returns>
        public bool ThisReallyVisible()
        {
            //Control ctl = Control.FromHandle(this.Handle);
            return this.Visible;
        } 

        public HSI()
        {
            InitializeComponent();

            _headingimage = new Bitmap(this.Width, this.Height);
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);

            int _radiusinside = (int)(Width / 3.6f);
            int _radiusoutside = (int)(Width / 2.2f);

           // drawnheading = false;

            if (drawnheading == false || this.DesignMode)
            {
                _headingimage = new Bitmap(Width, Height);

                Graphics g = Graphics.FromImage(_headingimage);

                g.CompositingQuality = System.Drawing.Drawing2D.CompositingQuality.HighQuality;
                g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.Bicubic;
                g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
                //Graphics g = e.Graphics;

                g.TranslateTransform(this.Width/2,this.Height /2);

                int font = this.Width / 14;

                for (int a = 0; a <= 360; a += 5)
                {
                    if (a == 0) 
                    {
                        g.DrawString("N".PadLeft(2), new Font(FontFamily.GenericSansSerif, font), Brushes.White, new PointF(-font, -_radiusoutside));

                        g.DrawLine(Pens.White, 0, _radiusinside, 0, _radiusinside + 11);
                    }
                    else if (a == 90)
                    {
                        g.DrawString("E".PadLeft(2), new Font(FontFamily.GenericSansSerif, font), Brushes.White, new PointF(-font, -_radiusoutside));

                        g.DrawLine(Pens.White, 0, _radiusinside, 0, _radiusinside + 11);
                    }
                    else if (a == 180)
                    {
                        g.DrawString("S".PadLeft(2), new Font(FontFamily.GenericSansSerif, font), Brushes.White, new PointF(-font, -_radiusoutside));

                        g.DrawLine(Pens.White, 0, _radiusinside, 0, _radiusinside + 11);
                    }
                    else if (a == 270)
                    {
                        g.DrawString("W".PadLeft(2), new Font(FontFamily.GenericSansSerif, font), Brushes.White, new PointF(-font, -_radiusoutside));

                        g.DrawLine(Pens.White, 0, _radiusinside, 0, _radiusinside + 11);
                    }
                    else if (a == 360)
                    {
                        // ignore it, as we process it at 0
                    }
                    else if ((a % 30) == 0) // number labeled
                    {
                        g.DrawString((a / 10).ToString("0").PadLeft(2), new Font(FontFamily.GenericSansSerif, font), Brushes.White, new PointF(-font, -_radiusoutside));

                        g.DrawLine(Pens.White, 0, _radiusinside, 0, _radiusinside + 11);
                    }
                    else if (a % 10 == 0) // larger line
                    {
                        g.DrawLine(Pens.White, 0, _radiusinside, 0, _radiusinside + 7);
                    }
                    else if (a % 5 == 0) // small line
                    {
                        g.DrawLine(Pens.White, 0, _radiusinside, 0, _radiusinside + 4);
                    } 

                    g.RotateTransform(5);
                }

                g.ResetTransform();

                drawnheading = true;
            }

            e.Graphics.TranslateTransform(Width / 2, Height / 2);
            e.Graphics.RotateTransform(-Heading);

            e.Graphics.DrawImage(_headingimage, new Rectangle(-Width / 2, - Height/2,Width,Height));

            e.Graphics.RotateTransform(Heading);

            Pen or = new Pen(Color.DarkOrange,2);
            // body
            e.Graphics.DrawLine(or, 0, 30, 0, -10);
            // wing
            e.Graphics.DrawLine(or, -30, 0, 30, 0);
            //tail
            e.Graphics.DrawLine(or, -10, 25, 10, 25);

            e.Graphics.DrawLine(new Pen(Color.White,2),0,-_radiusoutside,0,-_radiusinside);

            e.Graphics.RotateTransform(NavHeading - Heading);

            Point[] headbug = new Point[7];
            headbug[0] = new Point(-5, -_radiusoutside + 0);
            headbug[1] = new Point(-5, -_radiusoutside + 4);
            headbug[2] = new Point(-3, -_radiusoutside + 4);
            headbug[3] = new Point(0, -_radiusoutside + 8);
            headbug[4] = new Point(3, -_radiusoutside + 4);
            headbug[5] = new Point(5, -_radiusoutside + 4);
            headbug[6] = new Point(5, -_radiusoutside + 0);

            e.Graphics.DrawLines(or, headbug);

          //  this.Invalidate();
        }

        protected override void OnResize(EventArgs e)
        {
            Width = Height;
            base.OnResize(e);
            this.Invalidate();
            drawnheading = false;
        }

    }
}

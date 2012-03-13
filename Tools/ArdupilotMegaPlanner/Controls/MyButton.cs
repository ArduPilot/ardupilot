using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;


using System.Drawing.Drawing2D;

namespace ArdupilotMega
{
    class MyButton : Button
    {
        bool mouseover = false;
        bool mousedown = false;

       bool inOnPaint = false;

       public Color BGGradTop = Color.FromArgb(0x94, 0xc1, 0x1f);

       public Color BGGradBot = Color.FromArgb(0xcd, 0xe2, 0x96);

        // i want to ignore forecolor
       public Color TextColor = Color.FromArgb(0x40, 0x57, 0x04);

       public Color Outline = Color.FromArgb(0x79, 0x94, 0x29);

        protected override void OnPaint(PaintEventArgs pevent)
        {
            //base.OnPaint(pevent);

            if (inOnPaint)
                return;

            inOnPaint = true;

            try
            {
                Graphics gr = pevent.Graphics;

                //  gr.SmoothingMode = SmoothingMode.AntiAlias;

                Rectangle outside = new Rectangle(0, 0, this.Width, this.Height);

                LinearGradientBrush linear = new LinearGradientBrush(outside, BGGradTop, BGGradBot, LinearGradientMode.Vertical);

                Pen mypen = new Pen(Outline, 2);
                /*
                gr.FillRectangle(new SolidBrush(Color.FromArgb(0x26, 0x27, 0x28)), outside);

                GraphicsPath outline = new GraphicsPath();

                float wid = this.Height / 5f;
                float widright = wid + 1;

                // tl
                outline.AddArc(0, 0, wid, wid, 180, 90);
                // top line
                outline.AddLine(wid, 0, this.Width - widright, 0);
                // tr
                outline.AddArc(this.Width - widright, 0, wid, wid, 270, 90);
                // br
                outline.AddArc(this.Width - widright, this.Height - widright, wid, wid, 0, 90);
                // bottom line
                outline.AddLine(wid, this.Height - 1, this.Width - widright, this.Height - 1);
                // bl
                outline.AddArc(0, this.Height - widright, wid, wid, 90, 90);


                gr.FillPath(linear, outline);

                gr.DrawPath(mypen, outline);

                */

                gr.FillRectangle(linear, outside);
                gr.DrawRectangle(mypen, outside);


                SolidBrush mybrush = new SolidBrush(TextColor);

                if (mouseover)
                {
                    SolidBrush brush = new SolidBrush(Color.FromArgb(73, 0x2b, 0x3a, 0x03));

                    gr.FillRectangle(brush, 0, 0, this.Width, this.Height);
                }
                if (mousedown)
                {
                    SolidBrush brush = new SolidBrush(Color.FromArgb(73, 0x2b, 0x3a, 0x03));

                    gr.FillRectangle(brush, 0, 0, this.Width, this.Height);
                }

                if (!this.Enabled)
                {
                    SolidBrush brush = new SolidBrush(Color.FromArgb(150, 0x2b, 0x3a, 0x03));

                    gr.FillRectangle(brush, 0, 0, this.Width, this.Height);
                }


                StringFormat stringFormat = new StringFormat();
                stringFormat.Alignment = StringAlignment.Center;
                stringFormat.LineAlignment = StringAlignment.Center;

                string display = this.Text;
                int amppos = display.IndexOf('&');
                if (amppos != -1)
                    display = display.Remove(amppos, 1);

                gr.DrawString(display, this.Font, mybrush, outside, stringFormat);
            }
            catch { }
            
            inOnPaint = false;
        }

        protected override void OnClick(EventArgs e)
        {
            base.OnClick(e);
        }

        protected override void OnPaintBackground(PaintEventArgs pevent)
        {
            //base.OnPaintBackground(pevent);
        }

        protected override void OnMouseEnter(EventArgs e)
        {
            mouseover = true;
            base.OnMouseEnter(e);
        }

        protected override void OnMouseLeave(EventArgs e)
        {
            mouseover = false;
            base.OnMouseLeave(e);
        }

        protected override void OnMouseDown(MouseEventArgs mevent)
        {
            mousedown = true;
            base.OnMouseDown(mevent);
        }

        protected override void OnMouseUp(MouseEventArgs mevent)
        {
            mousedown = false;
            base.OnMouseUp(mevent);
        }
    }
}

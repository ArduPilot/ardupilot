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

        protected override void OnPaint(PaintEventArgs pevent)
        {
            //base.OnPaint(pevent);

            Graphics gr = pevent.Graphics;

            Rectangle outside = new Rectangle(0,0,this.Width,this.Height);

            LinearGradientBrush linear = new LinearGradientBrush(outside,Color.FromArgb(0x94,0xc1,0x1f),Color.FromArgb(0xcd,0xe2,0x96),LinearGradientMode.Vertical);

            Pen mypen = new Pen(Color.FromArgb(0x79,0x94,0x29),2);

            gr.FillRectangle(linear,outside);

            gr.DrawRectangle(mypen,outside);

            SolidBrush mybrush = new SolidBrush(Color.FromArgb(0x40,0x57,0x04));

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
                display = display.Remove(amppos,1);

            gr.DrawString(display, this.Font, mybrush, outside, stringFormat);

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

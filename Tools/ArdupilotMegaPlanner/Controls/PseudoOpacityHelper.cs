using System;
using System.Drawing;
using System.Windows.Forms;

namespace ArdupilotMega.Controls
{
    internal static class PseudoOpacityHelper
    {
        public static void CoverWithRect(this Control c, Graphics g, float opacity)
        {
            var bgcolor = c.BackColor;
            int alpha = 255 - ((int)(opacity * 255));
            
            var opacityColor = Color.FromArgb(alpha, bgcolor.R, bgcolor.G, bgcolor.B);
            using (var brush = new SolidBrush(opacityColor))
            {
                g.FillRectangle(brush, 0, 0, c.Width, c.Height);
            }
        }
    }
}
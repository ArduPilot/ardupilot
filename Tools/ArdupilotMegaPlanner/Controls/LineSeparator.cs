using System;
using System.ComponentModel;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ArdupilotMega.Controls
{
    /// <summary>
    /// A seperator that is fundamentally made from two lines the top 'Primary' and the
    /// bottom 'Secondary' Thus the height is always 2 pixels
    /// In addition, the opacity can be modulated for the left, middle and right most
    /// points, and the color blended between. That way the seperator can be made to 
    /// fade out to the left or right or both, and more subtle UI effects made etc
    /// </summary>
    public partial class LineSeparator : UserControl
    {
        [Description("Primary Color of the secondary line"), Category("Appearance")]
        [DefaultValue(typeof(Color), "DarkGray")]
        public Color PrimaryColor { get; set; }

        [Description("Secondary Color of the secondary line"), Category("Appearance")]
        [DefaultValue(typeof(Color), "White")]
        public Color SecondaryColor { get; set; }

        [Description("Opacity at the left most point"), Category("Appearance")]
        [DefaultValue(typeof(float), "1.0")]
        public float Opacity1 { get; set; }

        [Description("Opacity at the mid point"), Category("Appearance")]
        [DefaultValue(typeof(float), "1.0")]
        public float Opacity2 { get; set; }

        [Description("Opacity at the right most point"), Category("Appearance")]
        [DefaultValue(typeof(float), "1.0")]
        public float Opacity3 { get; set; }

        public LineSeparator()
        {
            this.PrimaryColor = Color.DarkGray;
            this.SecondaryColor = Color.White;
            this.Opacity1 = Opacity2 = Opacity3 = 1f;

            this.Height = 2;
            this.Paint += LineSeparator_Paint;
            this.MaximumSize = new Size(2000, 2);
            this.MinimumSize = new Size(0, 2);
        }

        private void LineSeparator_Paint(object sender, PaintEventArgs e)
        {
            DrawLine(PrimaryColor, 0, e.Graphics);
            DrawLine(SecondaryColor, 1, e.Graphics);
        }

        private void DrawLine(Color baseColor, int y, Graphics g)
        {
            var c1 = Color.FromArgb((int)(Opacity1 * 255), baseColor.R, baseColor.G, baseColor.B);
            var c2 = Color.FromArgb((int)(Opacity2 * 255), baseColor.R, baseColor.G, baseColor.B);
            var c3 = Color.FromArgb((int)(Opacity3 * 255), baseColor.R, baseColor.G, baseColor.B);

            var point1 = new Point(0, y);
            var point2 = new Point(Width / 2, y);
            var point = new Point(Width, y);

            var b1 = new LinearGradientBrush(point1, point2, c1, c2);
            var b2 = new LinearGradientBrush(point2, point, c2, c3);

            g.DrawLine(new Pen(b1), point1, point2);
            g.DrawLine(new Pen(b2), point2, point);
        }
    }
}
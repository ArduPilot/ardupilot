using System.ComponentModel;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ArducopterConfigurator.Views.controls
{
    public partial class PropControl : Control
    {
        public PropControl()
        {
            InitializeComponent(); 
            
            SetStyle(ControlStyles.OptimizedDoubleBuffer |
                      ControlStyles.AllPaintingInWmPaint |
                      ControlStyles.UserPaint |
                      ControlStyles.ResizeRedraw, true);

            InitializeComponent();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.Clear(BackColor);

//            var rect =new RectangleF(x, y + height - (barStart + barSize), width, barSize)
//
//            using (var bg = new LinearGradientBrush(rect, _barLightColour, _barDarkColour, isVertical ? 0 : 90.0F, false))
//                g.FillRectangle(bg, rect);
//
//
//
//            using (Pen p = new Pen(_borderColor, _borderWidth))
//            {
//                p.Alignment = PenAlignment.Inset;
//                p.LineJoin = LineJoin.Round;
//                g.DrawLine(p, width, y, width, height);
//            } 
         //   var bmp = GenerateProcentBarBitmap(Width, Height, BackColor);
         //   g.DrawImage(bmp, 0, 0);
        }
    }
}

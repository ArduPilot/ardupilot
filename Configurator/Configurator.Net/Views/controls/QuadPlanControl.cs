using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

/*
 * Starting point for this control:
 * http://www.codeproject.com/KB/progress/iTunes_Bar.aspx
 * 
 */
namespace ArducopterConfigurator.Views.controls
{
    /// <summary>
    /// </summary>
    [ToolboxBitmap(typeof(System.Windows.Forms.ProgressBar))]
    public partial class QuadPlanControl : UserControl
    {
        public QuadPlanControl()
        {
            SetStyle(ControlStyles.OptimizedDoubleBuffer |
                      ControlStyles.AllPaintingInWmPaint |
                      ControlStyles.UserPaint |
                      ControlStyles.ResizeRedraw, true);

            InitializeComponent();
            Width = 50;
            Height = 50;
        }


        /// <summary>
        /// Raises the <see cref="E:System.Windows.Forms.Control.Paint"></see> event.
        /// </summary>
        /// <param name="e">A <see cref="T:System.Windows.Forms.PaintEventArgs"></see> that contains the event data.</param>
        protected override void OnPaint(PaintEventArgs e)
        {
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.Clear(BackColor);

            //var bmp = GenerateProcentBarBitmap(Width, Height, BackColor);
            //g.DrawImage(bmp, 0, 0);
        }


      


    
    }
}

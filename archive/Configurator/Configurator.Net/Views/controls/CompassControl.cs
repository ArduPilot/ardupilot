using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ArducopterConfigurator.Views.controls
{
    /// <summary>
    /// </summary>
    [ToolboxBitmap(typeof(System.Windows.Forms.ProgressBar))]
    public partial class CompassControl : UserControl
    {
        public static readonly Color preSweepLight = Color.FromArgb(102, 144, 252);
        public static readonly Color preSweepDark = Color.FromArgb(40, 68, 202);
        public static readonly Color PreBorderColor = Color.DarkGray;
        public static readonly Color PreBarBaseDark = Color.FromArgb(199, 200, 201);
        public static readonly Color PreBarBaseLight = Color.WhiteSmoke;

        private Color _sweepLightColour = preSweepLight;
        private Color _sweepDarkColour = preSweepDark;
        private Color _borderColor = PreBorderColor;
        private Color _barBgLightColour = PreBarBaseLight;
        private Color _barBgDarkColor = PreBarBaseDark;

        private float _borderWidth = 1.0F;

        private int _val = 50;

        public CompassControl()
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

            var bmp = GenerateBitmap(Width, Height);
            g.DrawImage(bmp, 0, 0);
        }


     private void GenerateCircle(Graphics g, float x, float y, float width, float height, int heading)
     {
         var outerRect = new RectangleF(x, y, width, height);

         // Fill the background of the whole control
         using (var bg = new SolidBrush(BackColor))
             g.FillRectangle(bg, outerRect);

         // Fill the background of the circle
         var circleRect = outerRect;
         circleRect.Inflate((_borderWidth/-2), (_borderWidth/-2));
         
         using (var white = new LinearGradientBrush(outerRect, _barBgLightColour, _barBgDarkColor, 0F))
                {
                  g.FillEllipse(white, circleRect);
                }
     
         var gp = new GraphicsPath();
         gp.AddEllipse(circleRect);

         var pgb = new PathGradientBrush(gp);

         pgb.CenterPoint = new PointF(circleRect.Width/2, circleRect.Height/2);
         pgb.CenterColor = _sweepDarkColour;
         pgb.SurroundColors = new[] {_sweepLightColour};

         heading = (heading + 270  -5)   %360;

         g.FillPie(pgb, circleRect.Left, circleRect.Top, circleRect.Width, circleRect.Height, heading, 10);

         pgb.Dispose();

         using (var borderPen = new Pen(_borderColor, _borderWidth))
         {
             g.DrawEllipse(borderPen, circleRect);
         }
     }


        private Bitmap GenerateBitmap(int width, int height)
         {
            var bmp = new Bitmap(width, height);
                     using (var g1 = Graphics.FromImage(bmp))
                     {
                         GenerateCircle(g1, 0.0F, 0.0F, width, height, _val);
                     }
            return bmp;
        }
     

        /// <summary>
        /// Gets or sets the width of the border.
        /// </summary>
        /// <value>The width of the border.</value>
        [System.ComponentModel.Description("Gets or sets the with of the borders")]
        [System.ComponentModel.DefaultValue(1.0f)]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public float BarBorderWidth
        {
            get { return _borderWidth; }
            set { _borderWidth = value; Refresh(); }
        }

        /// <summary>
        /// Gets or sets the bar background light.
        /// </summary>
        /// <value>The bar background light.</value>
        [System.ComponentModel.Description("Gets or sets the lighter background color")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public Color BarBackgroundLight
        {
            get { return _barBgLightColour; }
            set { _barBgLightColour = value; Refresh(); }
        }

        /// <summary>
        /// Gets or sets the bar background dark.
        /// </summary>
        /// <value>The bar background dark.</value>
        [System.ComponentModel.Description("Gets or sets the darker background color")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public Color BarBackgroundDark
        {
            get { return _barBgDarkColor; }
            set { _barBgDarkColor = value; Refresh(); }
        }

        /// <summary>
        /// Gets or sets the bar light.
        /// </summary>
        /// <value>The bar light.</value>
        [System.ComponentModel.Description("Gets or sets the light bar color")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public Color BarLight
        {
            get { return _sweepLightColour; }
            set { _sweepLightColour = value; Refresh(); }
        }

        /// <summary>
        /// Gets or sets the bar dark.
        /// </summary>
        /// <value>The bar dark.</value>
        [System.ComponentModel.Description("Gets or sets the dark bar color")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public Color BarDark
        {
            get { return _sweepDarkColour; }
            set { _sweepDarkColour = value; Refresh(); }
        }

        /// <summary>
        /// Gets or sets the color of the border.
        /// </summary>
        /// <value>The color of the border.</value>
        [System.ComponentModel.Description("The border color")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public Color BarBorderColor
        {
            get { return _borderColor; }
            set { _borderColor = value; Refresh(); }
        }


        [System.ComponentModel.Description("Gets or sets the Value")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public int Value
        {
            get { return _val; }
            set
            {
                if (_val != value)
                {
                    _val = value;
                    Refresh();
                }
            }
        }
    }
}

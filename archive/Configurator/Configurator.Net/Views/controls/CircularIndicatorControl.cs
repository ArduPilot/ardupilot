using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ArducopterConfigurator.Views.controls
{
    /// <summary>
    /// </summary>
    [ToolboxBitmap(typeof(System.Windows.Forms.ProgressBar))]
    public partial class CirularIndicatorControl : UserControl
    {
        public static readonly Color PreBarBaseDark = Color.FromArgb(199, 200, 201);
        public static readonly Color PreBarBaseLight = Color.WhiteSmoke;
        public static readonly Color PreBarLight = Color.FromArgb(102, 144, 252);
        public static readonly Color PreBarDark = Color.FromArgb(40, 68, 202);
        public static readonly Color PreBorderColor = Color.DarkGray;
     
        private Color _barBgLightColour = PreBarBaseLight;
        private Color _barBgDarkColor = PreBarBaseDark;
        private Color _barLightColour = PreBarLight;
        private Color _barDarkColour = PreBarDark;
        private Color _borderColor = PreBorderColor;

        private float _borderWidth = 1.0F;

        private int _min = 0;
        private int _max = 100;
        private int _val = 50;
        private int _offset = 0;

        private bool _isVertical = false;
        private bool _isReveresed;

        public CirularIndicatorControl()
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

            var bmp = GenerateBitmap(Width, Height, BackColor);
            g.DrawImage(bmp, 0, 0);
        }


        // Generate the bar image on graphics g
        // g is the graphics of the whole control
        private void GenerateCircle(Graphics g, float x, float y, float width, float height, bool isVertical, float fraction)
        {
            var outerRect = new RectangleF(x, y, width, height);
           
            // Fill the background of the whole control
            using (var bg = new  SolidBrush(Color.Transparent))
                g.FillRectangle(bg,outerRect);

            // Fill the background of the circle
            var circleRect = outerRect;
            circleRect.Inflate((_borderWidth/-2),(_borderWidth/-2));
            using (var white = new LinearGradientBrush(outerRect, _barBgLightColour, _barBgDarkColor, isVertical ? 0 : 90.0F, false))
                g.FillEllipse(white, circleRect);


            // We need to figure out how big the bar should be, where it starts and ends
            // This is so we can set the clip properly
            // note that what we clip is actually what is NOT the bar
            var clipSize = isVertical
                               ? circleRect.Height - circleRect.Height * fraction
                               : circleRect.Width - circleRect.Width * fraction;

            // The location of this clip rectangle is normally on the right for a left - to - right bar
            // and on the top for a bottom - to - top bar. However if the _isReversed is set, then we need to 
            // adjust the location to the 'other side'
            var clipBegin = isVertical
                ? _isReveresed  // vertical
                           ? circleRect.Top + circleRect.Height - clipSize
                           : circleRect.Top 
                : _isReveresed  // horizontal
                           ? circleRect.Left
                           : circleRect.Left + circleRect.Width - clipSize;

            var clipRectangle = new RectangleF(
                isVertical ? circleRect.Left : clipBegin,
                isVertical ? clipBegin : circleRect.Top,
                isVertical ? circleRect.Width : clipSize,
                isVertical ? clipSize : circleRect.Height);

            // We also need to work out the angle for the gradient brush, so that the griadient goes in the
            // right 'direction'

            var angle = !isVertical
                ? _isReveresed ? 0F : 180F
                : _isReveresed ? 270F : 90.0F;

            using (var bg = new LinearGradientBrush(circleRect, _barLightColour, _barDarkColour, angle, false))
            {
                // g.FillPie(bg, circleRect.Left, circleRect.Top, circleRect.Width, circleRect.Height, 270, fraction*360)
                g.SetClip(clipRectangle,  CombineMode.Exclude);
                g.FillEllipse(bg, circleRect.Left, circleRect.Top, circleRect.Width, circleRect.Height);
                g.ResetClip();
            }

            using (var borderPen = new Pen(_borderColor, _borderWidth))
            {
                g.DrawEllipse(borderPen, circleRect);
            }


            
           

            // Now do the progress bar that represents the value
//            var barSize = (float)Math.Abs(_val - _offset) / (_max - _min) * (isVertical ? height : width);
//            var barStart = (float)(Math.Min(_offset, _val) - _min) / (_max - _min) * (isVertical ? height : width);
//
//
//            if (barSize > 0.10F)
//            {
//                rect = isVertical 
//                    ? new RectangleF(x, y + height  - ( barStart + barSize), width, barSize)
//                    : new RectangleF(x + barStart, y, barSize, height);
//
//                using (var bg = new LinearGradientBrush(rect, _barLightColour, _barDarkColour, isVertical ? 0 : 90.0F, false))
//                    g.FillRectangle(bg, rect);
//
//                using (Pen p = new Pen(_borderColor, _borderWidth))
//                {
//                    p.Alignment = PenAlignment.Inset;
//                    p.LineJoin = LineJoin.Round;
//                    g.DrawLine(p, width, y, width, height);
//                } 
//            } 
//
//
//            // Now the line that represents the Offset/Origin
//            DrawLine(width, height, isVertical, g, _offset,_borderColor);
//
//            // Border around the whole thing
//            using (Pen p = new Pen(_borderColor, _borderWidth * 2))
//            {
//                p.Alignment = PenAlignment.Outset;
//                p.LineJoin = LineJoin.Round;
//                g.DrawRectangle(p, x, y, width, height);
//            }

        }


       
        private Bitmap GenerateBitmap(int width, int height,Color bgColor)
        {
            var theImage = new Bitmap(width, height);

            using (var g = Graphics.FromImage(theImage))
            {
                g.SmoothingMode = SmoothingMode.AntiAlias;
                g.Clear(bgColor);

                using (var bmp = new Bitmap(width, height))
                {
                    using (var g1 = Graphics.FromImage(bmp))
                    {
                        var fraction = ((float)(Value - Min))/(Max - Min);

                        GenerateCircle(g1, 0.0F, 0.0F, width, height, _isVertical, fraction);
                    }
                    g.DrawImage(bmp, 0, 0);
                }
            }
            return theImage;
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
            get { return _barLightColour; }
            set { _barLightColour = value; Refresh(); }
        }

        /// <summary>
        /// Gets or sets the bar dark.
        /// </summary>
        /// <value>The bar dark.</value>
        [System.ComponentModel.Description("Gets or sets the dark bar color")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public Color BarDark
        {
            get { return _barDarkColour; }
            set { _barDarkColour = value; Refresh(); }
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

     
       
        [System.ComponentModel.Description("Gets or sets the Minimum Value")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public int Min
        {
            get { return _min; }
            set { _min = value; Refresh(); }
        }

        [System.ComponentModel.Description("Gets or sets the Minimum Value")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public int Max
        {
            get { return _max; }
            set { _max = value; Refresh(); }
        }

        [System.ComponentModel.Description("Gets or sets the Offset Value")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public int Offset
        {
            get { return _offset; }
            set { _offset = value; Refresh(); }
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


        [System.ComponentModel.Description("Whether the indicator is vertical or horizontal")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public bool IsVertical
        {
            get { return _isVertical; }
            set { _isVertical = value; Refresh(); }
        }

        [System.ComponentModel.Description("If true, then for example control goes right to left when horizontal")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public bool IsReveresed
        {
            get { return _isReveresed; }
            set { _isReveresed = value; Refresh(); }
        }
    
    }
}

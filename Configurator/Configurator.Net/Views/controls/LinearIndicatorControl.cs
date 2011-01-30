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
    public partial class LinearIndicatorControl : UserControl
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
        private Color _watermarkLineColor = PreBorderColor;

        private float _borderWidth = 1.0F;

        private int _min = 0;
        private int _max = 100;
        private int _val = 50;
        private int _offset = 0;

        private int iNumDividers = 10;

        private bool _isVertical = false;
        private int _minWatermark;
        private int _maxWatermark;



        public LinearIndicatorControl()
        {
            SetStyle(ControlStyles.OptimizedDoubleBuffer |
                      ControlStyles.AllPaintingInWmPaint |
                      ControlStyles.UserPaint |
                      ControlStyles.ResizeRedraw, true);

            InitializeComponent();
            this.Width = 300;
            this.Height = 50;
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

            var bmp = GenerateProcentBarBitmap(Width, Height, BackColor);
            g.DrawImage(bmp, 0, 0);
        }


        // Generate the bar image on graphics g
        // g is the graphics of the whole control
        private void GenerateBar(Graphics g, float x, float y, float width, float height, bool isVertical)
        {
            var rect = new RectangleF(x, y, width, height);

            // Fill the background
            using (var white = new LinearGradientBrush(rect, _barBgLightColour, _barBgDarkColor, isVertical ? 0 : 90.0F, false))
                g.FillRectangle(white, rect);

         

            // Now do the progress bar that represents the value
            var barSize = (float)Math.Abs(_val - _offset) / (_max - _min) * (isVertical ? height : width);
            var barStart = (float)(Math.Min(_offset, _val) - _min) / (_max - _min) * (isVertical ? height : width);


            if (barSize > 0.10F)
            {
                rect = isVertical 
                    ? new RectangleF(x, y + height  - ( barStart + barSize), width, barSize)
                    : new RectangleF(x + barStart, y, barSize, height);

                using (var bg = new LinearGradientBrush(rect, _barLightColour, _barDarkColour, isVertical ? 0 : 90.0F, false))
                    g.FillRectangle(bg, rect);

                

                using (Pen p = new Pen(_borderColor, _borderWidth))
                {
                    p.Alignment = PenAlignment.Inset;
                    p.LineJoin = LineJoin.Round;
                    g.DrawLine(p, width, y, width, height);
                } 
            } 


            // Now the line that represents the Offset/Origin
            DrawLine(width, height, isVertical, g, _offset,_borderColor);

            // Draw the high/low watermarks
            if (_minWatermark!=_offset)
                DrawLine(width, height, isVertical, g, _minWatermark, _watermarkLineColor);
            
            if (_maxWatermark!=_offset)
                DrawLine(width, height, isVertical, g, _maxWatermark, _watermarkLineColor);


            // Border around the whole thing
            using (Pen p = new Pen(_borderColor, _borderWidth * 2))
            {
                p.Alignment = PenAlignment.Outset;
                p.LineJoin = LineJoin.Round;
                g.DrawRectangle(p, x, y, width, height);
            }

//            using (Pen p = new Pen(_barDarkColour, _borderWidth))
//            {
//                p.Alignment = PenAlignment.Inset;
//                p.LineJoin = LineJoin.Round;
//                using (Pen p2 = new Pen(_barLightColour, _borderWidth))
//                {
//                    p2.Alignment = PenAlignment.Inset;
//                    p2.LineJoin = LineJoin.Round;
//
//                    float procentMarkerWidth = (width / iNumDividers);
//
//                    for (float i = procentMarkerWidth; i < totalWidth; i += procentMarkerWidth)
//                    {
//                        if (i >= width)
//                        {
//                            p.Color = _barBgLightColour;
//                            p2.Color = _barBgDarkColor;
//                        } 
//
//                        g.DrawLine(p, i, 0, i, height);
//                        g.DrawLine(p2, i + _borderWidth, 0, i + _borderWidth, height);
//
//                    }
//                }
//            } 
        }

        private void DrawLine(float width, float height, bool isVertical, Graphics g, int position, Color color)
        {
            if (position > _min && _offset < _max)
                using (Pen p = new Pen(color, _borderWidth))
                {
                    var linePos = (float)(position - _min) / (_max - _min) * (isVertical ? height : width);

                    if (isVertical)
                        g.DrawLine(p, 0 ,height - linePos, width, height - linePos);
                    else
                        g.DrawLine(p, linePos, 0, linePos, height);
                }
        }


        private Bitmap GenerateProcentBarBitmap(int width, int height,Color bgColor)
        {
            var theImage = new Bitmap(width, height);

            using (var g = Graphics.FromImage(theImage))
            {
                g.SmoothingMode = SmoothingMode.AntiAlias;
                g.Clear(bgColor);

                //int height1 = height - (height / 3);
                int height1 = height;
                var bmp1 = new Bitmap(width, height1);
                var g1 = Graphics.FromImage(bmp1);

                GenerateBar(g1, 0.0F, 0.0F, width, height1, _isVertical);
                g1.Dispose();
                
                var bmp = bmp1;
                  
                g.DrawImage(bmp, 0, 0);
                g.Dispose();

                bmp.Dispose();
            }
            return theImage;
        } 

        /// <summary>
        /// Gets or sets the number of bar dividers to display.
        /// </summary>
        /// <value>The num bar dividers.</value>
        [System.ComponentModel.Description("Gets or sets how many dividers to display on the bar")]
        [System.ComponentModel.DefaultValue(10)]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public int BarDividersCount
        {
            get { return iNumDividers; }
            set { iNumDividers = value; Refresh(); }
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

        /// <summary>
        /// Gets or sets the color of the border.
        /// </summary>
        /// <value>The color of the border.</value>
        [System.ComponentModel.Description("The color for the watermark lines")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public Color WatermarkLineColor
        {
            get { return _watermarkLineColor; }
            set { _watermarkLineColor = value; Refresh(); }
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

        [System.ComponentModel.Description("The position of the Max watermark line. Set to Offset if not needed")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public int MaxWaterMark
        {
            get { return _maxWatermark; }
            set
            {
                if (_maxWatermark != value)
                {
                    _maxWatermark = value;
                    Refresh();
                }

            }
        }


        [System.ComponentModel.Description("The position of the Min watermark line. Set to Offset if not needed")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public int MinWatermark
        {
            get { return _minWatermark; }
            set
            {
                if (_minWatermark != value)
                {
                    _minWatermark = value;
                    Refresh();
                }

            }
        }

        [System.ComponentModel.Description("Gets or sets whether the indicator is vertical")]
        [System.ComponentModel.RefreshProperties(System.ComponentModel.RefreshProperties.Repaint)]
        public bool IsVertical
        {
            get { return _isVertical; }
            set { _isVertical = value; Refresh(); }
        }
    
    }
}

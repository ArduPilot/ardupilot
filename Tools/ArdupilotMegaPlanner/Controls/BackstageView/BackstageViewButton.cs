using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace ArdupilotMega.Controls.BackstageView
{
    public class BackstageViewButton : Control
    {
        private bool _isSelected;

        internal Color ContentPageColor = Color.Gray;
        internal Color PencilBorderColor = Color.White;
        internal Color SelectedTextColor = Color.White;
        internal Color UnSelectedTextColor = Color.Gray;
        internal Color HighlightColor1 = Color.DarkBlue;
        internal Color HighlightColor2 = Color.Blue;
        private bool _isMouseOver;

        //internal Color HighlightColor1 = Color.FromArgb(0x94, 0xc1, 0x1f);
        //internal Color HighlightColor2 = Color.FromArgb(0xcd, 0xe2, 0x96);

        public BackstageViewButton()
        {
            this.SuspendLayout();

            SetStyle(ControlStyles.ResizeRedraw, true);

            this.Width = 150;
            this.Height = 30;

            
            this.ResumeLayout(false);
        }

        /// <summary>
        /// Whether this button should show the selected style
        /// </summary>
        public bool IsSelected
        {
            get { return _isSelected; }
            set
            {
                if (_isSelected != value)
                {
                    _isSelected = value;

                    this.Invalidate();
                }
            }
        }

        protected override void OnPaintBackground(PaintEventArgs pevent)
        {
            base.OnPaintBackground(pevent);
        }

        protected override void OnResize(EventArgs e)
        {
            base.OnResize(e);
        }

        protected override void OnPaint(PaintEventArgs pevent)
        {
           ((BackStageViewMenuPanel)this.Parent).PaintBackground(pevent);

           Graphics g = pevent.Graphics;
         

            // Now the little 'arrow' thingy if we are selected and the selected bg grad
           if (_isSelected)
           {
               var rect1 = new Rectangle(0, 0, Width / 2, Height);
               var rect2 = new Rectangle(Width / 2, 0, Width, Height);

               g.FillRectangle(new LinearGradientBrush(rect1, HighlightColor1, HighlightColor2, LinearGradientMode.Horizontal), rect1);
               g.FillRectangle(new LinearGradientBrush(rect2, HighlightColor2, HighlightColor1, LinearGradientMode.Horizontal), rect2);

               var butPen = new Pen(HighlightColor1);
               g.DrawLine(butPen, 0, 0, Width, 0);
               g.DrawLine(butPen, 0, Height - 1, Width, Height - 1);

               var arrowBrush = new SolidBrush(this.ContentPageColor);

               var midheight = Height / 2;
               var arSize = 8;

               var arrowPoints = new[]
                                     {
                                         new Point(Width, midheight + arSize),
                                         new Point(Width - arSize, midheight),
                                         new Point(Width, midheight - arSize)
                                     };

               g.DrawString(Text, new Font(FontFamily.GenericSansSerif, 10), new SolidBrush(SelectedTextColor), 20, 6);

               g.FillPolygon(arrowBrush, arrowPoints);

               var pencilBrush = new Pen(this.PencilBorderColor);


               g.DrawPolygon(pencilBrush, arrowPoints);

               
           }
           else
           {
               if (_isMouseOver)
               {
                   var brush = new SolidBrush(Color.FromArgb(10, 0xA0, 0xA0, 0xA0));

                   g.FillRectangle(brush, this.ClientRectangle);

                   var butPen = new Pen(PencilBorderColor);
                   g.DrawLine(butPen, 0, 0, Width, 0);
                   g.DrawLine(butPen, 0, Height - 1, Width, Height - 1);
               }

               g.DrawString(Text, new Font(FontFamily.GenericSansSerif, 10), new SolidBrush(this.UnSelectedTextColor), 20, 6);
           }
        }


        protected override void OnMouseEnter(EventArgs e)
        {
            _isMouseOver = true;
            base.OnMouseEnter(e);
            this.Invalidate();
        }

        protected override void OnMouseLeave(EventArgs e)
        {
            _isMouseOver = false;
            base.OnMouseLeave(e);
            this.Invalidate();

        }

        /*
        // This IS necessary for transparency - windows only..... remove it
        protected override CreateParams CreateParams
        {
            get
            {
                const int WS_EX_TRANSPARENT = 0x20;
                CreateParams cp = base.CreateParams;
                cp.ExStyle |= WS_EX_TRANSPARENT;
                return cp;
            }
        }
         */
    }
}
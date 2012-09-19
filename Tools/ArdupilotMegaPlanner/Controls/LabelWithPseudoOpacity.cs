using System;
using System.ComponentModel;
using System.Windows.Forms;

namespace ArdupilotMega.Controls
{
    /// <summary>
    /// Label which uses a painting trick to simulate opacity
    /// </summary>
    /// <remarks>
    /// On Paint will put a rectangle with the same color as the background overtop
    /// of the imapge. The Alpha of the rectangle's color is adjusted according to the 
    /// (new) Opacity property. 
    /// For this to appear as translucency, the background property must be set to the 
    /// same color as the background of the form.
    /// </remarks>
    public class LabelWithPseudoOpacity : Label
    {
        private float _opacity = 1.0F;

        /// <summary>
        /// The (simulated) Opacity. 0 is fully transparent, 1.0 is normal
        /// </summary>
        [Description("(Simulated) Opacity of the image"), Category("Appearance")]
        [DefaultValue(typeof(float), "1.0")]
        public float Opacity
        {
            get { return _opacity; }
            set
            {
                if (_opacity == value)
                    return;

                if (value > 1.0F || value < 0.0F)
                    throw new ArgumentOutOfRangeException();

                _opacity = value;
                Invalidate();
            }
        }

        protected override void OnPaint(PaintEventArgs pe)
        {
            base.OnPaint(pe);
            this.CoverWithRect(pe.Graphics, Opacity);
        }

        public new bool DoubleBuffered
        {
            get
            {
                return base.DoubleBuffered;
            }
            set
            {
                base.DoubleBuffered = value;
            }
        }
    }
}
using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;

namespace ArducopterConfigurator
{
    /// <summary>
    /// This is a custom tweaking of the standard Progress bar 
    /// </summary>
    [ToolboxBitmap(typeof(ProgressBar))]
    public class LinearSensorIndicatorControl : ProgressBar
    {
        private bool m_IsVertical;

        protected override CreateParams CreateParams
        {
            get
            {
                CreateParams cp = base.CreateParams;
                if (m_IsVertical)
                    cp.Style |= 0x04;
                return cp;
            }
        }

        public new int Value { 
            get
            {
                return base.Value;
            } 
        set
            {
                if (value<Minimum)
                {
                    // don't do it
                    value = Minimum;
                }
                if (value > Maximum)
                {
                    // don't do it
                    value = Maximum;
                }
                base.Value = value;
            } 
        }

        public new int Maximum
        {
            get
            {
                return base.Maximum;
            }
            set
            {
                base.Maximum = value;
            }
        }


        public new int Minimum
        {
            get
            {
                return base.Minimum ;
            }
            set
            {
                base.Minimum = value;
            }
        } 


        [Description("Whether the display grows vertically")]
        [Category("LinearSensorIndicatorControl")]
        [DefaultValue(false)]
        [RefreshProperties(RefreshProperties.All)]
        public bool IsVertical
        {
            get
            {
                return m_IsVertical;
            }
            set
            {
                m_IsVertical = value;
                Invalidate();
            }
        }


//        [Description("An offset that will be added to every value applied")]
//        [Category("LinearSensorIndicatorControl")]
//        [DefaultValue(0)]
//        [RefreshProperties(RefreshProperties.All)]
//        public int Offset
//        {
//            get
//            {
//                return m_Offset;
//            }
//            set
//            {
//                m_Offset = value;
//                Invalidate();
//            }
//        }

     

    }
}
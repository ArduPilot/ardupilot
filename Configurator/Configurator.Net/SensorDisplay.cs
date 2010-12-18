using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;

namespace ArducopterConfigurator
{
    /// <summary>
    /// This is a custom tweaking of the standard Progress bar 
    /// </summary>
    /// <remarks>
    /// 
    /// </remarks>
    [ToolboxBitmap(typeof(ProgressBar))]
    public class SensorDisplay : ProgressBar
    {
        private bool m_IsVertical;
        private int m_Offset;

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
                return base.Value - m_Offset;
            } 
        set
            {
                base.Value = value + m_Offset;
            } 
        }

        public new int Maximum
        {
            get
            {
                return base.Maximum - m_Offset;
            }
            set
            {
                base.Maximum = value + m_Offset;
            }
        }


        public new int Minimum
        {
            get
            {
                return base.Minimum - m_Offset;
            }
            set
            {
                base.Minimum = value + m_Offset;
            }
        } 


        [Description("Whether the display grows vertically")]
        [Category("SensorDisplay")]
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


        [Description("An offset that will be added to every value applied")]
        [Category("SensorDisplay")]
        [DefaultValue(0)]
        [RefreshProperties(RefreshProperties.All)]
        public int Offset
        {
            get
            {
                return m_Offset;
            }
            set
            {
                m_Offset = value;
                Invalidate();
            }
        }

     

    }
}
using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;

namespace ArducopterConfigurator
{
    [Description("Vertical Progress Bar")]
    [ToolboxBitmap(typeof(ProgressBar))]
    public class VerticalProgressBar : ProgressBar
    {
        protected override CreateParams CreateParams
        {
            get
            {
                CreateParams cp = base.CreateParams;
                cp.Style |= 0x04;
                return cp;
            }
        }

       

    }
}
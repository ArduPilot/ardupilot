using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace System.Windows.Forms
{
    /// <summary>
    /// This is a mono fix, windows handles this error, mono crashs
    /// </summary>
    public class MyUserControl : System.Windows.Forms.UserControl
    {
        protected override void WndProc(ref Message m)
        {
            try
            {
                base.WndProc(ref m);
            }
            catch { }
        }
    }
}

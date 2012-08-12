using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace System.Windows.Forms
{
    /// <summary>
    /// This is a mono fix, windows handles this error, mono crashs
    /// </summary>
    public class MyUserControl : System.Windows.Forms.UserControl
    {
        /// <summary>
        /// implement an on closing event to tidy up enviroment. 
        /// Using preedefined refrence as can easerly change between form and user control this way.
        /// </summary>
        public event FormClosingEventHandler FormClosing;

        public void Close(object sender, FormClosingEventArgs e)
        {
            if (FormClosing != null)
                FormClosing(sender,e);
        }

        public void Close()
        {
            Close(this, new FormClosingEventArgs(CloseReason.UserClosing, false));
        }

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

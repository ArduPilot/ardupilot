using System;
using System.Collections.Generic;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega.Controls.BackstageView
{
    public class BackStageViewContentPanel : UserControl
    {
        public event FormClosingEventHandler FormClosing;

        public void Close()
        {
            if (FormClosing != null)
                FormClosing(this, new FormClosingEventArgs(CloseReason.UserClosing, false));
        }

        public void DoLoad(EventArgs e)
        {
            base.OnLoad(e);
        }

        public new void OnLoad(EventArgs e)
        {
            // this is now done on page load via parent control
           // base.OnLoad(e);
        }
    }
}

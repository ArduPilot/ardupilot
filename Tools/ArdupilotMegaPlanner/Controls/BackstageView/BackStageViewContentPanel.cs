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

        public new void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
        }
    }
}

using System;
using System.Collections.Generic;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega.Controls.BackstageView
{
    /// <summary>
    /// The implementor executes some logic on activation, for e.g when moving 
    /// from not selected to selected in a tab control
    /// </summary>
    public interface IActivate
    {
        // Should be idempotent
        void Activate();
    }

    /// <summary>
    /// The implementor executes some logic on deactivation, for e.g when moving 
    /// from selected to not selected in a tab control
    /// </summary>
    public interface IDeactivate
    {
        // Should be idempotent
        void Deactivate();
    }

    /// <summary>
    /// Base class for user controls that wish to participate as a backstage view 
    /// </summary>
    [Obsolete("This doesn't do much any more, and should be replaced by indicating lifecycle features with IActivate etc")]
    public class BackStageViewContentPanel : UserControl,  IActivate, IDeactivate
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

        public void Activate()
        {
            DoLoad(EventArgs.Empty);
        }

        public void Deactivate()
        {
            Close();
        }
    }
}

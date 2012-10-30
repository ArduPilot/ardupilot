using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Utilities;
using ArdupilotMega.Controls.BackstageView;

namespace ArdupilotMega.Controls
{
    public partial class MainSwitcher : UserControl
    {
        public List<Screen> screens = new List<Screen>();
        public Screen current;

        public MainSwitcher()
        {
            InitializeComponent();
        }

        public void AddScreen(Screen Screen)
        {
            // add to list
            screens.Add(Screen);

            // hide it
            Screen.Control.Visible = false;
        }

        public void ShowScreen(string name)
        {

            if (current != null)
            {
                // hide current screen
                current.Visible = false;

                // remove reference
                this.Controls.Remove(current.Control);

                if (current.Control is IDeactivate)
                {
                    ((IDeactivate)(current.Control)).Deactivate();
                }

                // check if we need to remove the current control
                if (!current.Persistent)
                {
                    // cleanup
                    current.Control.Close();

                    current.Control.Dispose();

                    current.Control = (MyUserControl)Activator.CreateInstance(current.Control.GetType());
                }
            }

            // find next screen
            Screen nextscreen = screens.Single(s => s.Name == name);

            nextscreen.Control.Location = new Point(0, 0);

            nextscreen.Control.SuspendLayout();
            nextscreen.Control.Dock = DockStyle.Fill;

            nextscreen.Control.Size = this.Size;

            nextscreen.Visible = true;

            if (nextscreen.Control is IActivate)
            {
                ((IActivate)(nextscreen.Control)).Activate();
            }

            this.Controls.Add(nextscreen.Control);

            nextscreen.Control.ResumeLayout();

            ThemeManager.ApplyThemeTo(nextscreen.Control);

            current = nextscreen;
        }

        protected override void OnPaintBackground(PaintEventArgs e)
        {
            base.OnPaintBackground(e);
        }

        public class Screen
        {
            public string Name;
            public MyUserControl Control;
            public bool Visible { get { return Control.Visible; } set { Control.Visible = value; } }
            public bool Persistent;

            public Screen(string Name, MyUserControl Control, bool Persistent = false)
            {
                this.Name = Name;
                this.Control = Control;
                this.Persistent = Persistent;
            }
        }
    }
}

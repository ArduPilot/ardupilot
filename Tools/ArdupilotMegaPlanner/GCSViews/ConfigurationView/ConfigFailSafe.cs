using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;
using ArdupilotMega.Controls;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigFailSafe : UserControl, IActivate, IDeactivate
    {
        Timer timer = new Timer();

        public ConfigFailSafe()
        {
            InitializeComponent();

            mavlinkCheckBox1.setup(1, 0, "THR_FAILSAFE", MainV2.comPort.param);
            mavlinkNumericUpDown1.setup(800, 1200, 1, 1, "THR_FS_VALUE", MainV2.comPort.param);

            // setup rc update
            timer.Tick += new EventHandler(timer_Tick);
        }

        public void Deactivate()
        {
            timer.Stop();
        }

        void timer_Tick(object sender, EventArgs e)
        {
            // update all linked controls - 10hz
            try
            {
                MainV2.cs.UpdateCurrentSettings(currentStateBindingSource);
            }
            catch { }
        }

        public void Activate()
        {
            timer.Enabled = true;
            timer.Interval = 100;
            timer.Start();

            CustomMessageBox.Show("Ensure your props are not on the Plane/Quad","FailSafe",MessageBoxButtons.OK,MessageBoxIcon.Exclamation);
        }
    }
}

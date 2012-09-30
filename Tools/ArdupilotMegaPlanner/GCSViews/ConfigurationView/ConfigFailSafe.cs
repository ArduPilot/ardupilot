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
using System.Diagnostics;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigFailSafe : UserControl, IActivate, IDeactivate
    {
        Timer timer = new Timer();
        //

        public ConfigFailSafe()
        {
            InitializeComponent();

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
            mavlinkCheckBoxthr_fs.setup(1, 0, "THR_FAILSAFE", MainV2.comPort.param, mavlinkNumericUpDownthr_fs_value);
            mavlinkNumericUpDownthr_fs_value.setup(800, 1200, 1, 1, "THR_FS_VALUE", MainV2.comPort.param);
            mavlinkCheckBoxthr_fs_action.setup(1, 0, "THR_FS_ACTION",MainV2.comPort.param);
            mavlinkCheckBoxgcs_fs.setup(1, 0, "FS_GCS_ENABL", MainV2.comPort.param);
            mavlinkCheckBoxshort_fs.setup(1, 0, "FS_SHORT_ACTN", MainV2.comPort.param);
            mavlinkCheckBoxlong_fs.setup(1, 0, "FS_LONG_ACTN", MainV2.comPort.param);

            timer.Enabled = true;
            timer.Interval = 100;
            timer.Start();

            CustomMessageBox.Show("Ensure your props are not on the Plane/Quad","FailSafe",MessageBoxButtons.OK,MessageBoxIcon.Exclamation);
        }

        private void LNK_wiki_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            Process.Start(new ProcessStartInfo("http://code.google.com/p/ardupilot-mega/wiki/APM2xFailsafe"));
        }
    }
}

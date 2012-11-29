using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;
using System.Collections;
using ArdupilotMega.Controls;
using System.Diagnostics;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigAP_Limits : UserControl, IActivate
    {
        public ConfigAP_Limits()
        {
            InitializeComponent();
        }

        private void LNK_wiki_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            Process.Start(new ProcessStartInfo("https://code.google.com/p/ardupilot-mega/wiki/APLimitsLibrary"));
        }

        private void ProcessChange(object sender, EventArgs e)
        {
            if (sender.GetType() == typeof(CheckBox))
            {
                CheckBox chk = ((CheckBox)sender);
                MainV2.comPort.setParam(chk.Name, chk.Checked ? 1 : 0);
            }
            else if (sender.GetType() == typeof(NumericUpDown))
            {
                NumericUpDown nud = ((NumericUpDown)sender);
                MainV2.comPort.setParam(nud.Name, (float)nud.Value);
            }
        }

        private void LIM_ENABLED_CheckedChanged(object sender, EventArgs e)
        {
            if (!MainV2.comPort.param.ContainsKey("LIM_ENABLED"))
            {
                CustomMessageBox.Show("This feature is not enabled in your firmware.");
                return;
            }
            
            if (LIM_ENABLED.Checked)
            {
                groupBox1.Enabled = true;
            }
            else
            {
                groupBox1.Enabled = false;
            }

            ProcessChange(sender, e);
        }

        private void LIM_REQUIRED_CheckedChanged(object sender, EventArgs e)
        {
            if (LIM_REQUIRED.Checked)
            {
                groupBox5.Enabled = true;
            }
            else
            {
                groupBox5.Enabled = false;
            }

            ProcessChange(sender, e);
        }

        public void Activate()
        {
            PopulateData();
        }

        void PopulateData()
        {
            Hashtable copy = new Hashtable(MainV2.comPort.param);

            foreach (string key in copy.Keys)
            {
                Control[] ctls = this.Controls.Find(key, true);
                if (ctls.Length > 0)
                {
                    if (ctls[0].GetType() == typeof(CheckBox))
                    {
                        CheckBox chk = ((CheckBox)ctls[0]);
                        Console.WriteLine(chk.Name + " "+  copy[key]);

                        chk.Checked = (float)copy[key] == 1 ? true: false;
                        chk.Enabled = true;
                    }
                    else if (ctls[0].GetType() == typeof(NumericUpDown))
                    {
                        NumericUpDown nud = ((NumericUpDown)ctls[0]);
                        Console.WriteLine(nud.Name + " " + copy[key]);
                        nud.Value = (decimal)(float)copy[key];
                        nud.Enabled = true;
                    }
                }
            }
        }
    }
}

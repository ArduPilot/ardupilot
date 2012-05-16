using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace _3DRRadio
{
    public partial class Config : Form
    {
        public Config()
        {
            InitializeComponent();

            loadSettings();

            CMB_SerialPort.Items.AddRange(ArdupilotMega.Comms.SerialPort.GetPortNames());
            if (CMB_SerialPort.Items.Count > 0)
                CMB_SerialPort.SelectedIndex = 0;

            // default
            CMB_Baudrate.SelectedIndex = CMB_Baudrate.Items.IndexOf("57600");
        }

        void loadSettings()
        {
            Terminal.threadrun = false;

            panel1.Controls.Clear();

            ArdupilotMega._3DRradio form = new ArdupilotMega._3DRradio();

            panel1.Controls.Add(form);

            ArdupilotMega.Utilities.ThemeManager.SetTheme(ArdupilotMega.Utilities.ThemeManager.Themes.None);

            ArdupilotMega.Utilities.ThemeManager.ApplyThemeTo(this);
        }

        void loadTerminal()
        {
            panel1.Controls.Clear();

            Terminal form = new Terminal();

            form.Dock = DockStyle.Fill;

            panel1.Controls.Add(form);

            ArdupilotMega.Utilities.ThemeManager.SetTheme(ArdupilotMega.Utilities.ThemeManager.Themes.None);

            ArdupilotMega.Utilities.ThemeManager.ApplyThemeTo(this);
        }

        void loadRssi()
        {
            Terminal.threadrun = false;

            panel1.Controls.Clear();

            Rssi form = new Rssi();

            form.Dock = DockStyle.Fill;

            panel1.Controls.Add(form);

            ArdupilotMega.Utilities.ThemeManager.SetTheme(ArdupilotMega.Utilities.ThemeManager.Themes.None);

            ArdupilotMega.Utilities.ThemeManager.ApplyThemeTo(this);
        }

        private void CMB_SerialPort_SelectedIndexChanged(object sender, EventArgs e)
        {
            ArdupilotMega.MainV2.comPort.BaseStream.PortName = CMB_SerialPort.Text;
        }

        private void CMB_Baudrate_SelectedIndexChanged(object sender, EventArgs e)
        {
            ArdupilotMega.MainV2.comPort.BaseStream.BaudRate = int.Parse(CMB_Baudrate.Text);
        }

        private void CMB_SerialPort_Click(object sender, EventArgs e)
        {
            CMB_SerialPort.Items.Clear();
            CMB_SerialPort.Items.AddRange(ArdupilotMega.Comms.SerialPort.GetPortNames());
        }

        private void helpToolStripMenuItem_Click(object sender, EventArgs e)
        {
            System.Diagnostics.Process.Start("http://code.google.com/p/ardupilot-mega/wiki/3DRadio");
        }

        private void projectPageToolStripMenuItem_Click(object sender, EventArgs e)
        {
            System.Diagnostics.Process.Start("https://github.com/tridge/SiK");
            
        }

        private void terminalToolStripMenuItem_Click(object sender, EventArgs e)
        {
            loadTerminal();
        }

        private void settingsToolStripMenuItem_Click(object sender, EventArgs e)
        {
            loadSettings();
        }

        private void rssiToolStripMenuItem_Click(object sender, EventArgs e)
        {
            loadRssi();
        }
    }
}

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

            ArdupilotMega._3DRradio form = new ArdupilotMega._3DRradio();

            panel1.Controls.Add(form);

            ArdupilotMega.Utilities.ThemeManager.SetTheme(ArdupilotMega.Utilities.ThemeManager.Themes.None);

            ArdupilotMega.Utilities.ThemeManager.ApplyThemeTo(this);

            CMB_SerialPort.Items.AddRange(ArdupilotMega.Comms.SerialPort.GetPortNames());
            if (CMB_SerialPort.Items.Count > 0)
                CMB_SerialPort.SelectedIndex = 0;

            // default
            CMB_Baudrate.SelectedIndex = CMB_Baudrate.Items.IndexOf("57600");
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
    }
}

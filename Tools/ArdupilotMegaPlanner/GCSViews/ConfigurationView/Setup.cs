using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class Setup : Form
    {
        public Setup()
        {
            InitializeComponent();

            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigRadioInput(), "Radio Calibration"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigFlightModes(), "Flight Modes"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigHardwareOptions(), "Hardware Options"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigBatteryMonitoring(), "Battery Monitor"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigAccelerometerCalibrationQuad(), "ArduCopter Level"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigAccelerometerCalibrationPlane(), "ArduPlane Level"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigTradHeli(), "Heli Setup"));

            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ArdupilotMega._3DRradio(), "3DR Radio"));

            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ArdupilotMega.Antenna.Tracker(), "Antenna Tracker"));

            this.backstageView.ActivatePage(backstageView.Pages[0]);

            if (!MainV2.comPort.BaseStream.IsOpen)
            {
                CustomMessageBox.Show("Please connect (click Connect Button) before using setup!!");
            }
        }

        private void Setup_Load(object sender, EventArgs e)
        {

        }

        private void Setup_FormClosing(object sender, FormClosingEventArgs e)
        {
            backstageView.Close();
        }
    }
}
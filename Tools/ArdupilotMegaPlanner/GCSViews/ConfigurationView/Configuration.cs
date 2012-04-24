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
    public partial class Configuration : Form
    {
        public Configuration()
        {
            InitializeComponent();

            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigRadioInput(), "Radio Calibration"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigFlightModes(), "Flight Modes"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigHardwareOptions(), "Hardware Options"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigBatteryMonitoring(), "Battery Monitor"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigAccelerometerCalibrationQuad(), "ArduCopter"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigAccelerometerCalibrationPlane(), "ArduPlane"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigArducopter(), "Arducopter Setup"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigArduplane(), "Arduplane Setup"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigTradHeli(), "Heli Setup"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigRawParams(), "Raw params (Advanced)"));
            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigPlanner(), "Planner"));
        }

        private void Configuration_FormClosing(object sender, FormClosingEventArgs e)
        {
            backstageView.Close();
        }
    }
}

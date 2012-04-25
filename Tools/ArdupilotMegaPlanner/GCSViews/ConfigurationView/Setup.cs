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
    public partial class Setup : MyUserControl
    {
        public Setup()
        {
            InitializeComponent();

            if (MainV2.comPort.BaseStream.IsOpen)
            {
                this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigRadioInput(), "Radio Calibration"));
                this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigFlightModes(), "Flight Modes"));
                this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigHardwareOptions(), "Hardware Options"));
                this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigBatteryMonitoring(), "Battery Monitor"));


                /******************************HELI **************************/
                if (MainV2.comPort.param["H_GYR_ENABLE"] != null) // heli
                {
                    this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigAccelerometerCalibrationQuad(), "ArduCopter Level"));

                    this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigTradHeli(), "Heli Setup"));

                    var configpanel = new Controls.ConfigPanel();
                    configpanel.LoadXML("ArduCopterConfig.xml");
                    this.backstageView.AddPage(new BackstageView.BackstageViewPage(configpanel, "ArduCopter Config"));

                    this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigArducopter(), "OLD ArduCopter Config"));
                }
                /****************************** ArduCopter **************************/
                else if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2)
                {
                    this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigAccelerometerCalibrationQuad(), "ArduCopter Level"));

                    var configpanel = new Controls.ConfigPanel();
                    configpanel.LoadXML("ArduCopterConfig.xml");
                    this.backstageView.AddPage(new BackstageView.BackstageViewPage(configpanel, "ArduCopter Config"));

                    this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigArducopter(), "OLD ArduCopter Config"));
                }
                /****************************** ArduPlane **************************/
                else if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
                {
                    this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigAccelerometerCalibrationPlane(), "ArduPlane Level"));
                    this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigArduplane(), "ArduPlane Config"));
                }

                this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigRawParams(), "Raw params (Adv)"));
            }

            

            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ArdupilotMega._3DRradio(), "3DR Radio"));

            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ArdupilotMega.Antenna.Tracker(), "Antenna Tracker"));

            this.backstageView.AddPage(new BackstageView.BackstageViewPage(new ConfigPlanner(), "Planner"));

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
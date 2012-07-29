using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;
using ArdupilotMega.Utilities;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class Setup : MyUserControl
    {
        public Setup()
        {
            InitializeComponent();
        }


        private void Setup_Load(object sender, EventArgs e)
        {

            if (MainV2.comPort.BaseStream.IsOpen)
            {
                AddPagesForConnectedState();
			//	backstageView.AddSpacer(20);
            }

            // These pages work when not connected to an APM
            AddBackstageViewPage(new ArdupilotMega._3DRradio(), "3DR Radio");
            AddBackstageViewPage(new ArdupilotMega.Antenna.Tracker(), "Antenna Tracker");
//backstageView.AddSpacer(15);
            AddBackstageViewPage(new ConfigPlanner(), "Planner");

            this.backstageView.ActivatePage(backstageView.Pages[0]);

            ThemeManager.ApplyThemeTo(this);

            if (!MainV2.comPort.BaseStream.IsOpen)
            {
                Common.MessageShowAgain("Config Connect", @"Please connect (click Connect Button) before using setup.
If you are just setting up 3DR radios, you may continue without connecting.");
            }
        }


        // Add the pages that can only be shown when we are connected to an APM
        private void AddPagesForConnectedState()
        {
            /****************************** Common  **************************/

            AddBackstageViewPage(new ConfigRadioInput(), "Radio Calibration");
            AddBackstageViewPage(new ConfigFlightModes(), "Flight Modes");
            AddBackstageViewPage(new ConfigHardwareOptions(), "Hardware Options");
            AddBackstageViewPage(new ConfigBatteryMonitoring(), "Battery Monitor");


            /******************************HELI **************************/
            if (MainV2.comPort.param["H_GYR_ENABLE"] != null) // heli
            {
                AddBackstageViewPage(new ConfigMount(), "Camera Gimbal");

                AddBackstageViewPage(new ConfigAccelerometerCalibrationQuad(), "ArduCopter Level");

                AddBackstageViewPage(new ConfigTradHeli(), "Heli Setup");

                var configpanel = new Controls.ConfigPanel(Application.StartupPath + System.IO.Path.DirectorySeparatorChar + "ArduCopterConfig.xml");
                AddBackstageViewPage(configpanel, "ArduCopter Pids");

                AddBackstageViewPage(new ConfigArducopter(), "ArduCopter Config");
                AddBackstageViewPage(new ConfigAP_Limits(), "GeoFence");
            }
                /****************************** ArduCopter **************************/
            else if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2)
            {
                //AddBackstageViewPage(new ConfigCameraStab(), "Camera Gimbal");

                AddBackstageViewPage(new ConfigMount(), "Camera Gimbal");

                AddBackstageViewPage(new ConfigAccelerometerCalibrationQuad(), "ArduCopter Level");

                var configpanel = new Controls.ConfigPanel(Application.StartupPath + System.IO.Path.DirectorySeparatorChar + "ArduCopterConfig.xml");
                AddBackstageViewPage(configpanel, "ArduCopter Pids");

                AddBackstageViewPage(new ConfigArducopter(), "ArduCopter Config");
                AddBackstageViewPage(new ConfigAP_Limits(), "GeoFence");
            }
                /****************************** ArduPlane **************************/
            else if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
                AddBackstageViewPage(new ConfigMount(), "Camera Gimbal");

                AddBackstageViewPage(new ConfigAccelerometerCalibrationPlane(), "ArduPlane Level");
                AddBackstageViewPage(new ConfigArduplane(), "ArduPlane Pids");
            }
                /****************************** ArduRover **************************/
            else if (MainV2.cs.firmware == MainV2.Firmwares.ArduRover)
            {
                //AddBackstageViewPage(new ConfigAccelerometerCalibrationPlane(), "ArduRover Level"));
                AddBackstageViewPage(new ConfigArdurover(), "ArduRover Pids");
            }

            AddBackstageViewPage(new ConfigFriendlyParams { ParameterMode = ParameterMetaDataConstants.Standard }, "Standard Params");
            AddBackstageViewPage(new ConfigFriendlyParams { ParameterMode = ParameterMetaDataConstants.Advanced }, "Advanced Params");
            AddBackstageViewPage(new ConfigRawParams(), "Parameter List");
        }

        private void AddBackstageViewPage(UserControl userControl, string headerText)
        {
            backstageView.AddPage(userControl, headerText);
        }


        private void Setup_FormClosing(object sender, FormClosingEventArgs e)
        {
            backstageView.Close();
        }
    }
}
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
    public partial class SetupFresh : MyUserControl
    {
        // remember the last page accessed
        static string lastpagename = "";

        public SetupFresh()
        {
            InitializeComponent();
            ThemeManager.ApplyThemeTo(this);
        }

        private void Setup_Load(object sender, EventArgs e)
        {
            if (MainV2.comPort.BaseStream.IsOpen)
            {
                AddPagesForConnectedState();
			//	backstageView.AddSpacer(20);
            }

            // remeber last page accessed
            foreach (BackstageView.BackstageViewPage page in backstageView.Pages) {
                if (page.LinkText == lastpagename)
                {
                    this.backstageView.ActivatePage(page);
                    break;
                }
            }


            //this.backstageView.ActivatePage(backstageView.Pages[0]);

            ThemeManager.ApplyThemeTo(this);

            if (!MainV2.comPort.BaseStream.IsOpen)
            {
                Common.MessageShowAgain("Config Connect", @"Please connect (click Connect Button) before using setup.");
            }
        }


        // Add the pages that can only be shown when we are connected to an APM
        private void AddPagesForConnectedState()
        {
            /****************************** Common  **************************/

            AddBackstageViewPage(new ConfigRadioInput(), "Step 1: Radio Calib");
            AddBackstageViewPage(new ConfigFlightModes(), "Step 2: Flight Modes");
            AddBackstageViewPage(new ConfigFailSafe(), "Step 3: FailSafe");
            BackstageView.BackstageViewPage hardware = AddBackstageViewPage(new ConfigHardwareOptions(), "Step 4: Hardware");
            AddBackstageViewPage(new ConfigBatteryMonitoring(), "Step 5: Battery");


            /******************************HELI **************************/
            if (MainV2.comPort.param["H_GYR_ENABLE"] != null) // heli
            {
                AddBackstageViewPage(new ConfigMount(), "Step 6: Gimbal");

                AddBackstageViewPage(new ConfigAccelerometerCalibrationQuad(), "Step 7: Level");

                AddBackstageViewPage(new ConfigTradHeli(), "Step 8: Heli Setup");
            }
                /****************************** ArduCopter **************************/
            else if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2)
            {
                AddBackstageViewPage(new ConfigMount(), "Step 6: Gimbal");

                AddBackstageViewPage(new ConfigAccelerometerCalibrationQuad(), "Step 7: Level");
            }
                /****************************** ArduPlane **************************/
            else if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
                AddBackstageViewPage(new ConfigMount(), "Step 6: Gimbal");

                AddBackstageViewPage(new ConfigAccelerometerCalibrationPlane(), "Step 7: Level");
            }
                /****************************** ArduRover **************************/
            else if (MainV2.cs.firmware == MainV2.Firmwares.ArduRover)
            {

            }
        }

        private BackstageView.BackstageViewPage AddBackstageViewPage(UserControl userControl, string headerText, BackstageView.BackstageViewPage Parent = null)
        {
            return backstageView.AddPage(userControl, headerText, Parent);
        }


        private void Setup_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (backstageView.SelectedPage != null)
                lastpagename = backstageView.SelectedPage.LinkText;

            backstageView.Close();
        }
    }
}
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega.Antenna
{
    public partial class Tracker : Form
    {
        System.Threading.Thread t12;
        static bool threadrun = false;
        static ITrackerOutput tracker;

        public Tracker()
        {
            InitializeComponent();

            ThemeManager.ApplyThemeTo(this);

            CMB_serialport.DataSource = SerialPort.GetPortNames();

            if (threadrun)
            {
                BUT_connect.Text = "Disconnect";
            }
        }

        private void BUT_connect_Click(object sender, EventArgs e)
        {
            if (threadrun)
            {
                threadrun = false;
                BUT_connect.Text = "Connect";
                tracker.Close();
                return;
            }

            if (tracker != null && tracker.ComPort != null && tracker.ComPort.IsOpen)
            {
                tracker.ComPort.Close();
            }

            tracker = new ArdupilotMega.Antenna.Maestro();

            try
            {
                tracker.ComPort = new SerialPort()
                {
                    PortName = CMB_serialport.Text,
                    BaudRate = int.Parse(CMB_baudrate.Text)
                };
            }
            catch (Exception ex) { MessageBox.Show("Bad Port settings " + ex.Message); return; }

            try
            {
                tracker.PanStartRange = int.Parse(TXT_panrange.Text) / 2 * -1;
                tracker.PanEndRange = int.Parse(TXT_panrange.Text) / 2;
                tracker.TrimPan = TRK_pantrim.Value;

                tracker.TiltStartRange = int.Parse(TXT_tiltrange.Text) / 2 * -1;
                tracker.TiltEndRange = int.Parse(TXT_tiltrange.Text) / 2;
                tracker.TrimTilt = TRK_tilttrim.Value;

            }
            catch (Exception ex) { MessageBox.Show("Bad User input " + ex.Message); return; }

            if (tracker.Init())
            {
                if (tracker.Setup())
                {
                    tracker.PanAndTilt(0, 0);

                    t12 = new System.Threading.Thread(new System.Threading.ThreadStart(mainloop))
                    {
                        IsBackground = true,
                        Name = "Antenna Tracker"
                    };
                    t12.Start();                     
                }
            }
        }

        void mainloop()
        {
            threadrun = true;
            while (threadrun)
            {
                try
                {
                    // 10 hz - position updates default to 3 hz on the stream rate
                    tracker.PanAndTilt(MainV2.cs.AZToMAV, MainV2.cs.ELToMAV);
                    System.Threading.Thread.Sleep(100);
                }
                catch { }
            }
        }

        private void TRK_pantrim_Scroll(object sender, EventArgs e)
        {
            if (tracker != null)
                tracker.TrimPan = TRK_pantrim.Value;
        }

        private void TRK_tilttrim_Scroll(object sender, EventArgs e)
        {
            if (tracker != null)
                tracker.TrimTilt = TRK_tilttrim.Value;
        }
    }
}

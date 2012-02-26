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

            MainV2.fixtheme(this);

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
                tracker.PanStartRange = int.Parse(TXT_panstart.Text);
                tracker.PanEndRange = int.Parse(TXT_panstop.Text);
                tracker.TrimPan = TRK_pantrim.Value;

                tracker.TiltStartRange = int.Parse(TXT_tiltstart.Text);
                tracker.TiltEndRange = int.Parse(TXT_tiltstop.Text);
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

                    /*
                    for (int a = ting.PanStartRange; a < ting.PanEndRange; a++)
                    {
                        System.Threading.Thread.Sleep(100);
                        ting.Pan(a);
                    }

                    for (int a = ting.TiltStartRange; a < ting.TiltEndRange; a++)
                    {
                        System.Threading.Thread.Sleep(100);
                        ting.Tilt(a);
                    }
                     */
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

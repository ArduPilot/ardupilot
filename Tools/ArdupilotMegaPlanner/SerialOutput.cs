using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;

namespace ArdupilotMega
{
    public partial class SerialOutput : Form
    {
        System.Threading.Thread t12;
        static bool threadrun = false;
        static internal SerialPort comPort = new SerialPort();
        static internal PointLatLngAlt HomeLoc = new PointLatLngAlt(0,0,0,"Home");

        public SerialOutput()
        {
            InitializeComponent();

            CMB_serialport.DataSource = SerialPort.GetPortNames();

            if (threadrun)
            {
                BUT_connect.Text = "Stop";
            }
        }

        private void BUT_connect_Click(object sender, EventArgs e)
        {
            if (comPort.IsOpen)
            {
                threadrun = false;
                comPort.Close();
                BUT_connect.Text = "Connect";
            }
            else
            {
                try
                {
                    comPort.PortName = CMB_serialport.Text;
                }
                catch { CustomMessageBox.Show("Invalid PortName"); return; }
                try {
                comPort.BaudRate = int.Parse(CMB_baudrate.Text);
                } catch {CustomMessageBox.Show("Invalid BaudRate"); return;}
                try {
                comPort.Open();
                } catch {CustomMessageBox.Show("Error Connecting\nif using com0com please rename the ports to COM??"); return;}

                t12 = new System.Threading.Thread(new System.Threading.ThreadStart(mainloop))
                {
                    IsBackground = true,
                    Name = "Nmea output"
                };
                t12.Start();
            }
        }

        void mainloop()
        {
            threadrun = true;
            int counter = 0;
            while (threadrun)
            {
                try
                {
                    double lat = (int)MainV2.cs.lat + ((MainV2.cs.lat - (int)MainV2.cs.lat) * .6f);
                    double lng = (int)MainV2.cs.lng + ((MainV2.cs.lng - (int)MainV2.cs.lng) * .6f);
                    string line = string.Format("$GP{0},{1:HHmmss},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},", "GGA", DateTime.Now.ToUniversalTime(), Math.Abs(lat * 100), MainV2.cs.lat < 0 ? "S" : "N", Math.Abs(lng * 100), MainV2.cs.lng < 0 ? "W" : "E", MainV2.cs.gpsstatus, MainV2.cs.satcount, MainV2.cs.gpshdop, MainV2.cs.alt, "M", 0, "M", "");

                    string checksum = GetChecksum(line);
                    comPort.WriteLine(line + "*" + checksum);

                    line = string.Format("$GP{0},{1:HHmmss},{2},{3},{4},{5},{6},{7},{8},{9:ddMMyy},{10},", "RMC", DateTime.Now.ToUniversalTime(), "A", Math.Abs(lat * 100), MainV2.cs.lat < 0 ? "S" : "N", Math.Abs(lng * 100), MainV2.cs.lng < 0 ? "W" : "E", MainV2.cs.groundspeed * 3.6, MainV2.cs.groundcourse, DateTime.Now, 0);

                    checksum = GetChecksum(line);
                    comPort.WriteLine(line + "*" + checksum);

                    if (counter % 5 == 0 && HomeLoc.Lat != 0 && HomeLoc.Lng != 0)
                    {
                        line = string.Format("$GP{0},{1:HHmmss},{2},{3},{4},{5},{6},{7},", "HOM", DateTime.Now.ToUniversalTime(), Math.Abs(HomeLoc.Lat * 100), HomeLoc.Lat < 0 ? "S" : "N", Math.Abs(HomeLoc.Lng * 100), HomeLoc.Lng < 0 ? "W" : "E", HomeLoc.Alt, "M");

                        checksum = GetChecksum(line);
                        comPort.WriteLine(line + "*" + checksum);
                    }

                    System.Threading.Thread.Sleep(500);
                    counter++;
                }
                catch { }
            }
        }

        private void SerialOutput_FormClosing(object sender, FormClosingEventArgs e)
        {
        }

        // Calculates the checksum for a sentence
        string GetChecksum(string sentence)
        {
            // Loop through all chars to get a checksum
            int Checksum = 0;
            foreach (char Character in sentence.ToCharArray())
            {
                switch (Character)
                {
                    case '$':
                        // Ignore the dollar sign
                        break;
                    case '*':
                        // Stop processing before the asterisk
                        continue;
                    default:
                        // Is this the first value for the checksum?
                        if (Checksum == 0)
                        {
                            // Yes. Set the checksum to the value
                            Checksum = Convert.ToByte(Character);
                        }
                        else
                        {
                            // No. XOR the checksum with this character's value
                            Checksum = Checksum ^ Convert.ToByte(Character);
                        }
                        break;
                }
            }
            // Return the checksum formatted as a two-character hexadecimal
            return Checksum.ToString("X2");
        }

    }
}

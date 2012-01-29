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
    public partial class SerialInput : Form
    {
        System.Threading.Thread t12;
        static bool threadrun = false;
        static internal SerialPort comPort = new SerialPort();
        static internal PointLatLngAlt lastgotolocation = new PointLatLngAlt(0, 0, 0, "Goto last");
        static internal PointLatLngAlt gotolocation = new PointLatLngAlt(0, 0, 0, "Goto");
        static internal int intalt = 100;

        public SerialInput()
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
                catch { MessageBox.Show("Invalid PortName"); return; }
                try {
                comPort.BaudRate = int.Parse(CMB_baudrate.Text);
                } catch {MessageBox.Show("Invalid BaudRate"); return;}
                try {
                comPort.Open();
                } catch {MessageBox.Show("Error Connecting\nif using com0com please rename the ports to COM??"); return;}


                string alt = "100";

                if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2)
                {
                    alt = (10 * MainV2.cs.multiplierdist).ToString("0");
                }
                else
                {
                    alt = (100 * MainV2.cs.multiplierdist).ToString("0");
                }
                if (DialogResult.Cancel == Common.InputBox("Enter Alt", "Enter Alt", ref alt))
                    return;

                intalt = (int)(100 * MainV2.cs.multiplierdist);
                if (!int.TryParse(alt, out intalt))
                {
                    MessageBox.Show("Bad Alt");
                    return;
                }

                t12 = new System.Threading.Thread(new System.Threading.ThreadStart(mainloop))
                {
                    IsBackground = true,
                    Name = "Nmea Input"
                };
                t12.Start();

                BUT_connect.Text = "Stop";
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
                    string line = comPort.ReadLine();


                    //string line = string.Format("$GP{0},{1:HHmmss},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},", "GGA", DateTime.Now.ToUniversalTime(), Math.Abs(lat * 100), MainV2.cs.lat < 0 ? "S" : "N", Math.Abs(lng * 100), MainV2.cs.lng < 0 ? "W" : "E", MainV2.cs.gpsstatus, MainV2.cs.satcount, MainV2.cs.gpshdop, MainV2.cs.alt, "M", 0, "M", "");
                    if (line.StartsWith("$GPGGA")) // 
                    {
                        int c1 = line.IndexOf(',',0)+ 1;
                        int c2 = line.IndexOf(',', c1) + 1;
                        int c3 = line.IndexOf(',', c2) + 1;
                        int c4 = line.IndexOf(',', c3 ) + 1;
                        int c5 = line.IndexOf(',', c4 ) + 1;
                        int c6 = line.IndexOf(',', c5 ) + 1;
                        int c7 = line.IndexOf(',', c6 ) + 1;
                        int c8 = line.IndexOf(',', c7 ) + 1;
                        int c9 = line.IndexOf(',', c8 ) + 1;
                        int c10 = line.IndexOf(',', c9 ) + 1;
                        int c11 = line.IndexOf(',', c10 ) + 1;
                        int c12 = line.IndexOf(',', c11) + 1;

                        gotolocation.Lat = double.Parse(line.Substring(c2, c3 - c2 - 1)) / 100.0;

                        gotolocation.Lat = (int)gotolocation.Lat + ((gotolocation.Lat - (int)gotolocation.Lat) / 0.60);

                        if (line.Substring(c3, c4 - c3 - 1) == "S")
                            gotolocation.Lat *= -1;

                        gotolocation.Lng = double.Parse(line.Substring(c4, c5 - c4 - 1)) / 100.0;

                        gotolocation.Lng = (int)gotolocation.Lng + ((gotolocation.Lng - (int)gotolocation.Lng) / 0.60);

                        if (line.Substring(c5, c6 - c5 - 1) == "W")
                            gotolocation.Lng *= -1;

                        gotolocation.Alt = intalt; // double.Parse(line.Substring(c9, c10 - c9 - 1)) +

                    }


                    if (counter % 10 == 0 && gotolocation.Lat != 0 && gotolocation.Lng != 0 && gotolocation.Alt != 0) // 200 * 10 = 2 sec /// lastgotolocation != gotolocation && 
                    {
                        Console.WriteLine("Sending follow wp " +DateTime.Now.ToString("h:MM:ss")+" "+ gotolocation.Lat + " " + gotolocation.Lng + " " +gotolocation.Alt);
                        lastgotolocation = new PointLatLngAlt(gotolocation);

                        Locationwp gotohere = new Locationwp();

                        gotohere.id = (byte)MAVLink.MAV_CMD.WAYPOINT;
                        gotohere.alt = (float)(gotolocation.Alt);
                        gotohere.lat = (float)(gotolocation.Lat);
                        gotohere.lng = (float)(gotolocation.Lng);

                        try
                        {
                            updateLocationLabel(gotohere);
                        }
                        catch { }

                        if (MainV2.comPort.BaseStream.IsOpen && MainV2.givecomport == false)
                        {
                            try
                            {
                                MainV2.givecomport = true;

                                MainV2.comPort.setWP(gotohere, 0, MAVLink.MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT, (byte)2);

                                MainV2.givecomport = false;
                            }
                            catch { MainV2.givecomport = false; }
                        }
                    }

                    System.Threading.Thread.Sleep(200);
                    counter++;
                }
                catch { System.Threading.Thread.Sleep(2000); }
            }
        }

        private void updateLocationLabel(Locationwp plla)
        {
             this.BeginInvoke((MethodInvoker)delegate
             {
                 LBL_location.Text = gotolocation.Lat + " " + gotolocation.Lng + " " + gotolocation.Alt;
            }
        );

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

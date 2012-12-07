using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Comms;
using System.Globalization;

namespace ArdupilotMega
{
    public partial class FollowMe : Form
    {
        System.Threading.Thread t12;
        static bool threadrun = false;
        static internal SerialPort comPort = new SerialPort();
        static internal PointLatLngAlt lastgotolocation = new PointLatLngAlt(0, 0, 0, "Goto last");
        static internal PointLatLngAlt gotolocation = new PointLatLngAlt(0, 0, 0, "Goto");
        static internal int intalt = 100;

        public FollowMe()
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
                } catch (Exception ex) {CustomMessageBox.Show("Error Connecting\nif using com0com please rename the ports to COM??\n" + ex.ToString()); return;}


                string alt = "100";

                if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduCopter2)
                {
                    alt = (10 * MainV2.comPort.MAV.cs.multiplierdist).ToString("0");
                }
                else
                {
                    alt = (100 * MainV2.comPort.MAV.cs.multiplierdist).ToString("0");
                }
                if (DialogResult.Cancel == Common.InputBox("Enter Alt", "Enter Alt (relative to home alt)", ref alt))
                    return;

                intalt = (int)(100 * MainV2.comPort.MAV.cs.multiplierdist);
                if (!int.TryParse(alt, out intalt))
                {
                    CustomMessageBox.Show("Bad Alt");
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
            DateTime nextsend = DateTime.Now;

            threadrun = true;
            while (threadrun)
            {
                try
                {
                    string line = comPort.ReadLine();

                    //string line = string.Format("$GP{0},{1:HHmmss},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},", "GGA", DateTime.Now.ToUniversalTime(), Math.Abs(lat * 100), MainV2.comPort.MAV.cs.lat < 0 ? "S" : "N", Math.Abs(lng * 100), MainV2.comPort.MAV.cs.lng < 0 ? "W" : "E", MainV2.comPort.MAV.cs.gpsstatus, MainV2.comPort.MAV.cs.satcount, MainV2.comPort.MAV.cs.gpshdop, MainV2.comPort.MAV.cs.alt, "M", 0, "M", "");
                    if (line.StartsWith("$GPGGA")) // 
                    {
                        string[] items = line.Trim().Split(',','*');

                        if (items[15] != GetChecksum(line.Trim()))
                        {
                            Console.WriteLine("Bad Nmea line " + items[15] + " vs " + GetChecksum(line.Trim()));
                            continue;
                        }

                        if (items[6] == "0")
                        {
                            Console.WriteLine("No Fix");
                            continue;
                        }

                        gotolocation.Lat = double.Parse(items[2], CultureInfo.InvariantCulture) / 100.0;

                        gotolocation.Lat = (int)gotolocation.Lat + ((gotolocation.Lat - (int)gotolocation.Lat) / 0.60);

                        if (items[3] == "S")
                            gotolocation.Lat *= -1;

                        gotolocation.Lng = double.Parse(items[4], CultureInfo.InvariantCulture) / 100.0;

                        gotolocation.Lng = (int)gotolocation.Lng + ((gotolocation.Lng - (int)gotolocation.Lng) / 0.60);

                        if (items[5] == "W")
                            gotolocation.Lng *= -1;

                        gotolocation.Alt = intalt; // double.Parse(line.Substring(c9, c10 - c9 - 1)) +

                        gotolocation.Tag = "Sats "+ items[7] + " hdop " + items[8] ;

                    }


                    if (DateTime.Now > nextsend && gotolocation.Lat != 0 && gotolocation.Lng != 0 && gotolocation.Alt != 0) // 200 * 10 = 2 sec /// lastgotolocation != gotolocation && 
                    {
                        nextsend = DateTime.Now.AddSeconds(2);
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

                        if (MainV2.comPort.BaseStream.IsOpen && MainV2.giveComport == false)
                        {
                            try
                            {
                                MainV2.giveComport = true;

                                MainV2.comPort.setGuidedModeWP(gotohere);

                                MainV2.giveComport = false;
                            }
                            catch { MainV2.giveComport = false; }
                        }
                    }
                }
                catch { System.Threading.Thread.Sleep(2000); }
            }
        }

        private void updateLocationLabel(Locationwp plla)
        {
             this.BeginInvoke((MethodInvoker)delegate
             {
                 LBL_location.Text = gotolocation.Lat + " " + gotolocation.Lng + " " + gotolocation.Alt +" "+ gotolocation.Tag;
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
                        return Checksum.ToString("X2");
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

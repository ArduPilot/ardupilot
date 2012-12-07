using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega.Comms;

namespace ArdupilotMega
{
    public partial class SerialOutput2 : Form
    {
        System.Threading.Thread t12;
        static bool threadrun = false;
        static internal SerialPort comPort = new SerialPort();
        static internal PointLatLngAlt HomeLoc = new PointLatLngAlt(0,0,0,"Home");

        public SerialOutput2()
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

        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        void mainloop()
        {
            threadrun = true;
            int counter = 0;
            while (threadrun)
            {
                try
                {
                    //http://www.microdrones.com/wiki/index.php/Downlink_Protocol

                    /*
...
#1,27,48,1,1,1,1,0,25343,8192,85
#2,1,0,0,0,-100,-100,0,-100,-100,50,0,0,0,100,167
#3,0,0,0,0,13
#4,9,111731750,1595,0,197
#5,398843292,55871844,492974923,2.240,9,5
#6,0.15,-0.06,0.21,1.0,197
#7,-0.042,-0.022,2.474,182
#8,0.000,-418.438,2158,159
#9,-139,-139,431,199
#10,1.496,-1.628,0.842,161
#11,0,0,0,58
#12,,33
#1,27,48,1,1,1,1,0,25343,8192,85
#2,1,0,0,0,-100,-100,0,-100,-100,50,0,0,0,100,167
#3,0,0,0,0,13
#4,9,111732000,1595,0,208
#5,398843288,55871840,492974925,2.240,9,2
#6,0.07,-0.02,0.16,0.9,188
#7,-0.042,-0.019,2.475,175
#8,0.000,-418.233,2158,166
#9,-139,-139,431,199
#10,1.354,-1.549,0.972,162
#11,0,0,0,58
#12,,33
...

                     */

                    writeline("#1,28,07,2,1,1,1,2,16000,0,2,");

                 //   writeline("#2,100,99,98,97,96,95,94,93,92,");

                   // writeline("#3,100,120,140,160,");

                    int week, seconds;
                    GetGPSTime(out week, out seconds);

                    writeline(string.Format("#4,{0},{1},{2},{3},", counter / 10, seconds, week, 25));

                    //    writeline("#4,counter/2,111732000,1595,0,");

                    double x,y,z;

                    GetGeo(out x, out y, out z, MainV2.comPort.MAV.cs.lat, MainV2.comPort.MAV.cs.lng, MainV2.comPort.MAV.cs.alt);

                    writeline(string.Format("#5,{0},{1},{2},{3},{4},", x * 100, y * 100, z*100, MainV2.comPort.MAV.cs.gpshdop + 0.01, MainV2.comPort.MAV.cs.satcount));

                    writeline(string.Format("#6,{0},{1},{2},{3},", MainV2.comPort.MAV.cs.groundspeed * Math.Sin(MainV2.comPort.MAV.cs.groundcourse * deg2rad),
                        MainV2.comPort.MAV.cs.groundspeed * Math.Cos(MainV2.comPort.MAV.cs.groundcourse * deg2rad),
                        MainV2.comPort.MAV.cs.verticalspeed,2));

                    writeline(string.Format("#7,{0},{1},{2},", MainV2.comPort.MAV.cs.roll * deg2rad, MainV2.comPort.MAV.cs.pitch * deg2rad, MainV2.comPort.MAV.cs.yaw * deg2rad));

                    writeline(string.Format("#8,{0},{1},{2},", MainV2.comPort.MAV.cs.alt , MainV2.comPort.MAV.cs.alt , MainV2.comPort.MAV.cs.press_temp));

                    writeline(string.Format("#9,{0},{1},{2},", MainV2.comPort.MAV.cs.mx, MainV2.comPort.MAV.cs.my, MainV2.comPort.MAV.cs.mz));

                  //  writeline(string.Format("#10,{0},{1},{2},", 1.354,-1.549,0.972));

                    System.Threading.Thread.Sleep(100);
                    counter++;
                }
                catch { }
            }
        }

        DateTime GetFromGps(int weeknumber, int seconds)
        {
            DateTime datum = new DateTime(1980, 1, 6, 0, 0, 0, DateTimeKind.Utc);
            DateTime week = datum.AddDays(weeknumber * 7);
            DateTime time = week.AddSeconds(seconds);
            return time;
        }

        void GetGPSTime(out int weeknumber, out int seconds)
        {
            DateTime datum = new DateTime(1980, 1, 6, 0, 0, 0,DateTimeKind.Utc);

            TimeSpan ts = DateTime.Now - datum;

            weeknumber = ((int)ts.TotalDays / 7);

            ts = DateTime.Now - GetFromGps(weeknumber, 0);

            seconds =  (int)ts.TotalSeconds;
        }


        void GetDegrees(double x, double y, double z, out double latitude, out double longitude)
        {
            double deg = 0.01745329252;
            double r = Math.Sqrt(x * x + y * y);
            longitude = Math.Asin(y / r) / deg;
            if (longitude > 180d)
                longitude = 180d - longitude;

            latitude = Math.Atan(z / r) / deg;
        }

        void GetGeo(out double x, out double y, out double z, double latitude, double longitude, double alt)
        {
            double wgs84a = 6378137;
            double wgs84f = 1.0 / 298.257223563;
            double wgs84b = wgs84a * (1.0 - wgs84f);

            double clat = Math.Cos(deg2rad * latitude);
            double slat = Math.Sin(deg2rad * latitude);
            double clon = Math.Cos(deg2rad * longitude);
            double slon = Math.Sin(deg2rad * longitude);

            var ecc = Math.Sqrt(2 * wgs84f - Math.Pow(wgs84f, 2));
            var esq = ecc * ecc;

            alt = alt * 0.0001;

                        // var rrnrm  = radcur (flat);
            var rn = wgs84a;/// (Math.Sqrt(1-esq)*(slat*slat));// rrnrm[1];
            var re = wgs84a;// rrnrm[0];

            x = (rn + alt) * clat * clon;
            y = (rn + alt) * clat * slon;
            z = ((1 - esq) * rn + (alt)) * slat;

        }

        void writeline(string line)
        {
            comPort.Write(line + checksum(line) + "\r\n");
        }

        byte checksum(string line)
        {
            byte ans = 0;
            foreach (char ch in line.ToCharArray())
            {
                ans += (byte)ch;
            }
            ans = (byte)(ans ^ 0xff);

            return ans;
        }

        private void SerialOutput_FormClosing(object sender, FormClosingEventArgs e)
        {
        }

   

    }
}

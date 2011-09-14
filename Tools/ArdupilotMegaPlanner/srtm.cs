using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace ArdupilotMega
{
    class srtm
    {
        public static string datadirectory;

        public static int getAltitude(double lat, double lng)
        {
            short alt = -32768;

            lat += 0.0008;
            //lng += 0.0008;

            int x = (int)Math.Floor(lng);
            int y = (int)Math.Floor(lat);

            string ns;
            if (y > 0)
                ns = "N";
            else
                ns = "S";

            string ew;
            if (x > 0)
                ew = "E";
            else
                ew = "W";

            string filename = ns+ Math.Abs(y).ToString("00")+ew+ Math.Abs(x).ToString("000")+".hgt";

            if (!File.Exists(datadirectory + Path.DirectorySeparatorChar + filename))
            {
                return alt;
            }

            FileStream fs = new FileStream(datadirectory + Path.DirectorySeparatorChar + filename, FileMode.Open,FileAccess.Read);

            float posx = 0;
            float row = 0;

            if (fs.Length <= (1201 * 1201 * 2)) {
                posx = (int)(((float)(lng - x)) * (1201 * 2));
                row = (int)(((float)(lat - y)) * 1201) * (1201 * 2);
                row = (1201 * 1201 * 2) - row;
            } else {
                posx = (int)(((float)(lng - x)) * (3601 * 2));
                row = (int)(((float)(lat - y)) * 3601) * (3601 * 2);
                row = (3601 * 3601 * 2) - row;
            }

            if (posx % 2 == 1)
            {
                posx--;
            }

            //Console.WriteLine(filename + " row " + row + " posx" + posx);

            byte[] data = new byte[2];

            fs.Seek((int)(row + posx), SeekOrigin.Begin);
            fs.Read(data, 0, data.Length);

            Array.Reverse(data);

            alt = BitConverter.ToInt16(data,0);

            return alt;
        }
    }
}

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

            lat += 0.00083333333333333;
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

            string filename = ns + Math.Abs(y).ToString("00") + ew + Math.Abs(x).ToString("000") + ".hgt";

            string filename2 = "srtm_" + Math.Round((lng + 2.5 + 180) / 5, 0).ToString("00") + "_" + Math.Round((60 - lat + 2.5) / 5, 0).ToString("00") + ".asc";

            if (File.Exists(datadirectory + Path.DirectorySeparatorChar + filename))
            { // srtm hgt files
                FileStream fs = new FileStream(datadirectory + Path.DirectorySeparatorChar + filename, FileMode.Open, FileAccess.Read);

                float posx = 0;
                float row = 0;

                if (fs.Length <= (1201 * 1201 * 2))
                {
                    posx = (int)(((float)(lng - x)) * (1201 * 2));
                    row = (int)(((float)(lat - y)) * 1201) * (1201 * 2);
                    row = (1201 * 1201 * 2) - row;
                }
                else
                {
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

                fs.Close();
                fs.Dispose();

                Array.Reverse(data);

                alt = BitConverter.ToInt16(data, 0);

                return alt;
            }
            else if (File.Exists(datadirectory + Path.DirectorySeparatorChar + filename2))
            {
                // this is way to slow - and cacheing it will chew memory 6001 * 6001 * 4 = 144048004 bytes
                FileStream fs = new FileStream(datadirectory + Path.DirectorySeparatorChar + filename2, FileMode.Open, FileAccess.Read);

                StreamReader sr = new StreamReader(fs);

                int nox = 0;
                int noy = 0;
                float left = 0;
                float top = 0;
                int nodata = -9999;
                float cellsize = 0;

                int rowcounter = 0;

                float wantrow = 0;
                float wantcol = 0;


                while (!sr.EndOfStream)
                {
                    string line = sr.ReadLine();

                    if (line.StartsWith("ncols"))
                    {
                        nox = int.Parse(line.Substring(line.IndexOf(' ')));

                        //hgtdata = new int[nox * noy];
                    }
                    else if (line.StartsWith("nrows"))
                    {
                        noy = int.Parse(line.Substring(line.IndexOf(' ')));

                        //hgtdata = new int[nox * noy];
                    }
                    else if (line.StartsWith("xllcorner"))
                    {
                        left = float.Parse(line.Substring(line.IndexOf(' ')));
                    }
                    else if (line.StartsWith("yllcorner"))
                    {
                        top = float.Parse(line.Substring(line.IndexOf(' ')));
                    }
                    else if (line.StartsWith("cellsize"))
                    {
                        cellsize = float.Parse(line.Substring(line.IndexOf(' ')));
                    }
                    else if (line.StartsWith("NODATA_value"))
                    {
                        nodata = int.Parse(line.Substring(line.IndexOf(' ')));
                    }
                    else
                    {
                        string[] data = line.Split(new char[] { ' ' });

                        if (data.Length == (nox + 1))
                        {



                            wantcol = (float)((lng - Math.Round(left,0)));

                            wantrow = (float)((lat - Math.Round(top,0)));

                            wantrow =(int)( wantrow / cellsize);
                            wantcol =(int)( wantcol / cellsize);

                            wantrow = noy - wantrow;

                            if (rowcounter == wantrow)
                            {
                                Console.WriteLine("{0} {1} {2} {3} ans {4} x {5}", lng, lat, left, top, data[(int)wantcol], (nox + wantcol * cellsize));

                                return int.Parse(data[(int)wantcol]);
                            }

                            rowcounter++;
                        }
                    }



                }

                return alt;
            }

            return alt;
        }
    }
}

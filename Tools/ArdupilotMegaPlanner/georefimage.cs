using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;

namespace ArdupilotMega
{
    class georefimage
    {

        public string logFile = "";
        public string dirWithImages = "";

        DateTime getPhotoTime(string fn)
        {
            DateTime dtaken = DateTime.MinValue;

            try
            {
                Image myImage = Image.FromFile(fn);
                PropertyItem propItem = myImage.GetPropertyItem(36867); // 36867  // 306

                //Convert date taken metadata to a DateTime object 
                string sdate = Encoding.UTF8.GetString(propItem.Value).Trim();
                string secondhalf = sdate.Substring(sdate.IndexOf(" "), (sdate.Length - sdate.IndexOf(" ")));
                string firsthalf = sdate.Substring(0, 10);
                firsthalf = firsthalf.Replace(":", "-");
                sdate = firsthalf + secondhalf;
                dtaken = DateTime.Parse(sdate);

                myImage.Dispose();
            }
            catch { }

            return dtaken;
        }

        List<string[]> readLog(string fn)
        {
            List<string[]> list = new List<string[]>();

            StreamReader sr = new StreamReader(fn);

            string lasttime = "0";

            while (!sr.EndOfStream)
            {
                string line = sr.ReadLine();

                if (line.ToLower().StartsWith("gps"))
                {
                    string[] vals = line.Split(new char[] {',',':'});

                    if (lasttime == vals[1])
                        continue;

                    lasttime = vals[1];

                    list.Add(vals);
                }
            }

            sr.Close();
            sr.Dispose();

            return list;
        }

        public void dowork(float offsetseconds)
        {
            DateTime localmin = DateTime.MaxValue;
            DateTime localmax = DateTime.MinValue;

            DateTime startTime = DateTime.MinValue;

            logFile = @"C:\temp\farm 1-10-2011\100SSCAM\2011-10-01 11-48 1.log";

            List<string[]> list = readLog(logFile);

            dirWithImages = @"C:\temp\farm 1-10-2011\100SSCAM";

            string[] files = Directory.GetFiles(dirWithImages);

            StreamWriter sw2 = new StreamWriter(dirWithImages + Path.DirectorySeparatorChar + "location.txt");

            StreamWriter sw = new StreamWriter(dirWithImages + Path.DirectorySeparatorChar + "location.tel");
            sw.WriteLine("version=1");
            sw.WriteLine("#longitude and latitude - in degrees");
            sw.WriteLine("#name	utc	longitude	latitude	height");

            foreach (string file in files)
            {
                if (file.ToLower().EndsWith(".jpg"))
                {
                    DateTime dt = getPhotoTime(file);

                    if (startTime == DateTime.MinValue)
                    {
                        startTime = new DateTime(dt.Year, dt.Month, dt.Day, 0, 0, 0, 0, DateTimeKind.Utc).ToLocalTime();

                        foreach (string[] arr in list)
                        {
                            DateTime crap = startTime.AddMilliseconds(int.Parse(arr[1])).AddSeconds(offsetseconds);

                            if (localmin > crap)
                                localmin = crap;
                            if (localmax < crap)
                                localmax = crap;
                        }

                        Console.WriteLine("min " + localmin + " max " + localmax);
                    }

                    

                    foreach (string[] arr in list)
                    {
                        DateTime crap = startTime.AddMilliseconds(int.Parse(arr[1])).AddSeconds(offsetseconds);

                        Console.Write("ph " + dt + " log " + crap + "         \r");

                        if (dt.Equals(crap))
                        {
                            sw2.WriteLine(Path.GetFileNameWithoutExtension(file) + " " + arr[5] + " " + arr[4] + " " + arr[7]);
                            sw.WriteLine(Path.GetFileNameWithoutExtension(file) + "\t" + crap.ToString("yyyy:MM:dd HH:mm:ss") +"\t"+ arr[5] + "\t" + arr[4] + "\t" + arr[7]);
                            sw.Flush();
                            sw2.Flush();
                            Console.WriteLine(Path.GetFileNameWithoutExtension(file) + " " + arr[5] + " " + arr[4] + " " + arr[7] + "           ");
                            break;
                        }
                        //Console.WriteLine(crap);
                    }
                }

                
            }

            sw2.Close();
            sw.Close();

        }
    }
}

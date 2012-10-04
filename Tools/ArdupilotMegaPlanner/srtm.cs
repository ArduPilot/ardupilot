using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Net;
using System.Text.RegularExpressions;
using ICSharpCode.SharpZipLib.Zip;
using System.Threading;
using System.Collections;

namespace ArdupilotMega
{
    class srtm
    {
        public static string datadirectory = "./srtm/";

        static List<string> allhgts = new List<string>();

        static object objlock = new object();

        static Thread requestThread;

        static List<string> queue = new List<string>();

        static Hashtable fnamecache = new Hashtable();

        static Hashtable filecache = new Hashtable();

        static Dictionary<string, short[,]> cache = new Dictionary<string, short[,]>();

        public static int getAltitude(double lat, double lng, double zoom)
        {
            short alt = 0;

            //lat += 1 / 1199.0;
            //lng -= 1 / 1201f;

            // 		lat	-35.115676879882812	double
            //		lng	117.94178754638671	double
            // 		alt	70	short

            int x = (lng < 0) ? (int)(lng-1) : (int)lng;//(int)Math.Floor(lng);
            int y = (lat < 0) ? (int)(lat - 1) : (int)lat; ;//(int)Math.Floor(lat);

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

            if (fnamecache[y] == null)
                fnamecache[y] = Math.Abs(y).ToString("00");
            if (fnamecache[1000 + x] == null)
                fnamecache[1000 + x] = Math.Abs(x).ToString("000");

            string filename = ns + fnamecache[y] + ew + fnamecache[1000 + x] + ".hgt";

            try
            {

                if (cache.ContainsKey(filename) || File.Exists(datadirectory + Path.DirectorySeparatorChar + filename))
                { // srtm hgt files

                    int size = -1;

                    if (!cache.ContainsKey(filename))
                    {
                        FileStream fs = new FileStream(datadirectory + Path.DirectorySeparatorChar + filename, FileMode.Open, FileAccess.Read, FileShare.Read);

                        if (fs.Length == (1201 * 1201 * 2))
                        {
                            size = 1201;
                        }
                        else
                        {
                            size = 3601;
                        }

                        byte[] altbytes = new byte[2];
                        short[,] altdata = new short[size, size];


                        int altlat = 0;
                        int altlng = 0;

                        while (fs.Read(altbytes, 0, 2) != 0)
                        {
                            altdata[altlat, altlng] = (short)((altbytes[0] << 8) + altbytes[1]);

                            altlat++;
                            if (altlat >= size)
                            {
                                altlng++;
                                altlat = 0;
                            }
                        }

                        fs.Close();

                        cache[filename] = altdata;
                    }

                    if (cache[filename].Length == (1201 * 1201))
                    {
                        size = 1201;
                    }
                    else
                    {
                        size = 3601;
                    }

                    int posx = 0;
                    int row = 0;

                    posx = (int)(((float)(lng - x)) * (size * 1));
                    row = (int)(((float)(lat - y)) * (size * 1));
                    row = size - row;

                    return cache[filename][posx, row];
                }

                string filename2 = "srtm_" + Math.Round((lng + 2.5 + 180) / 5, 0).ToString("00") + "_" + Math.Round((60 - lat + 2.5) / 5, 0).ToString("00") + ".asc";

                if (File.Exists(datadirectory + Path.DirectorySeparatorChar + filename2))
                {
                    StreamReader sr = new StreamReader(readFile(datadirectory + Path.DirectorySeparatorChar + filename2));

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



                                wantcol = (float)((lng - Math.Round(left, 0)));

                                wantrow = (float)((lat - Math.Round(top, 0)));

                                wantrow = (int)(wantrow / cellsize);
                                wantcol = (int)(wantcol / cellsize);

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

                    //sr.Close();

                    return alt;
                }
                else // get something
                {
                    if (zoom >= 15)
                    {
                        if (!Directory.Exists(datadirectory))
                            Directory.CreateDirectory(datadirectory);

                        if (requestThread == null)
                        {
                            Console.WriteLine("Getting " + filename);
                            queue.Add(filename);

                            requestThread = new Thread(requestRunner);
                            requestThread.IsBackground = true;
                            requestThread.Name = "SRTM request runner";
                            requestThread.Start();
                        }
                        else
                        {
                            lock (objlock)
                            {
                                if (!queue.Contains(filename))
                                {
                                    Console.WriteLine("Getting " + filename);
                                    queue.Add(filename);
                                }
                            }
                        }
                    }
                }

            }
            catch { alt = 0; }

            return alt;
        }

        static MemoryStream readFile(string filename)
        {
            if (filecache.ContainsKey(filename))
            {
                return (MemoryStream)filecache[filename];
            }
            else
            {
                FileStream fs = new FileStream(filename, FileMode.Open, FileAccess.Read);

                byte[] file = new byte[fs.Length];
                fs.Read(file, 0, (int)fs.Length);

                filecache[filename] = new MemoryStream(file);

                fs.Close();

                return (MemoryStream)filecache[filename];
            }
        }

        static void requestRunner()
        {
            while (true)
            {
                try
                {
                    string item = "";
                    lock (objlock)
                    {
                        if (queue.Count > 0)
                        {
                            item = queue[0];
                        }
                    }

                    if (item != "")
                    {
                        get3secfile(item);
                        lock (objlock)
                        {
                            queue.Remove(item);
                        }
                    }
                }
                catch { }
                Thread.Sleep(1000);
            }
        }

        static void get3secfile(object name)
        {
            string baseurl = "http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/";

            // check file doesnt already exist
            if (File.Exists(datadirectory + Path.DirectorySeparatorChar + (string)name))
            {
                FileInfo fi = new FileInfo(datadirectory + Path.DirectorySeparatorChar + (string)name);
                if (fi.Length != 0)
                    return;
            }

            List<string> list = getListing(baseurl);

            foreach (string item in list)
            {
                List<string> hgtfiles = new List<string>();

                hgtfiles = getListing(item);

                foreach (string hgt in hgtfiles)
                {
                    if (hgt.Contains((string)name))
                    {
                        // get file

                        gethgt(hgt, (string)name);
                        return;
                    }
                }
            }
        }

        static void gethgt(string url, string filename)
        {
            try
            {

                WebRequest req = HttpWebRequest.Create(url);

                WebResponse res = req.GetResponse();

                Stream resstream = res.GetResponseStream();

                BinaryWriter bw = new BinaryWriter(File.Create(datadirectory + Path.DirectorySeparatorChar + filename + ".zip"));

                byte[] buf1 = new byte[1024];

                while (resstream.CanRead)
                {

                    int len = resstream.Read(buf1, 0, 1024);
                    if (len == 0)
                        break;
                    bw.Write(buf1, 0, len);

                }

                bw.Close();

                FastZip fzip = new FastZip();

                fzip.ExtractZip(datadirectory + Path.DirectorySeparatorChar + filename + ".zip", datadirectory, "");
            }
            catch { }
        }

        static List<string> getListing(string url)
        {
            string name = new Uri(url).AbsolutePath;

            name = Path.GetFileName(name.TrimEnd('/'));

            List<string> list = new List<string>();

            if (File.Exists(datadirectory + Path.DirectorySeparatorChar + name))
            {
                StreamReader sr = new StreamReader(datadirectory + Path.DirectorySeparatorChar + name);

                while (!sr.EndOfStream)
                {
                    list.Add(sr.ReadLine());
                }

                sr.Close();

                return list;
            }


            try
            {
                StreamWriter sw = new StreamWriter(datadirectory + Path.DirectorySeparatorChar + name);

                WebRequest req = HttpWebRequest.Create(url);

                WebResponse res = req.GetResponse();

                StreamReader resstream = new StreamReader(res.GetResponseStream());

                string data = resstream.ReadToEnd();

                Regex regex = new Regex("href=\"([^\"]+)\"", RegexOptions.IgnoreCase);
                if (regex.IsMatch(data))
                {
                    MatchCollection matchs = regex.Matches(data);
                    for (int i = 0; i < matchs.Count; i++)
                    {
                        if (matchs[i].Groups[1].Value.ToString().Contains(".."))
                            continue;
                        if (matchs[i].Groups[1].Value.ToString().Contains("http"))
                            continue;

                        list.Add(url.TrimEnd(new char[] { '/', '\\' }) + "/" + matchs[i].Groups[1].Value.ToString());

                    }
                }

                list.ForEach(x =>
                {
                    sw.WriteLine((string)x);
                });

                sw.Close();
            }
            catch { }

            return list;
        }
    }
}
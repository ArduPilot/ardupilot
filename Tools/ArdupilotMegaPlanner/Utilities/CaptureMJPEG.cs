using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.IO;
using System.Drawing;
using System.Threading;
using log4net;

namespace ArdupilotMega.Utilities
{
    public class CaptureMJPEG
    {
        private static readonly ILog log =
    LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        static Thread asyncthread;
        static bool running = false;
        public static string URL = @"http://127.0.0.1:56781/map.jpg";

        public static event EventHandler OnNewImage;

        static DateTime lastimage = DateTime.Now;
        static int fps = 0;

        public static void runAsync()
        {
            while (asyncthread != null && asyncthread.IsAlive)
            {
                running = false;
            }

            asyncthread = new Thread(new ThreadStart(getUrl))
            {
                IsBackground = true,
                Priority = ThreadPriority.BelowNormal,
                Name = "mjpg stream reader"
            }; 
            
            asyncthread.Start();
        }

        public static void Stop()
        {
            running = false;
        }

        public static string ReadLine(BinaryReader br)
        {
            StringBuilder sb = new StringBuilder();

            while (true) {
                byte by = br.ReadByte();
                sb.Append((char)by);
                if (by == '\n')
                    break;
            }

            sb = sb.Replace("\r\n", "");

            return sb.ToString();
        }

        static void getUrl()
        {

            running = true;

            Bitmap frame = new Bitmap(640, 480);

            // Create a request using a URL that can receive a post. 
            WebRequest request = HttpWebRequest.Create(URL);
            // Set the Method property of the request to POST.
            request.Method = "GET";

            ((HttpWebRequest)request).AutomaticDecompression = DecompressionMethods.GZip | DecompressionMethods.Deflate;

            request.Headers.Add("Accept-Encoding", "gzip,deflate");

            // Get the response.
            WebResponse response = request.GetResponse();
            // Display the status.
            log.Debug(((HttpWebResponse)response).StatusDescription);
            // Get the stream containing content returned by the server.
            Stream dataStream = response.GetResponseStream();

            BinaryReader br = new BinaryReader(dataStream);

            // get boundary header
            
            string mpheader = response.Headers["Content-Type"];
            if (mpheader.IndexOf("boundary=") == -1) {
                ReadLine(br); // this is a blank line
                string line = "proxyline";
                while (line.Length > 2)
                {
                    line = ReadLine(br);
                    if (line.StartsWith("--")) {
                        mpheader = line;
                        break;
                    }
                }
            }
            else
            {
                int startboundary = mpheader.IndexOf("boundary=") + 9;
                int endboundary = mpheader.Length;

                mpheader = mpheader.Substring(startboundary, endboundary - startboundary);
            }

            dataStream.ReadTimeout = 30000; // 30 seconds
            br.BaseStream.ReadTimeout = 30000;

            while (running)
            {
                try
                {
                    // get the multipart start header
                    int length = int.Parse(getHeader(br)["Content-Length"]);

                    // read boundary header
                    if (length > 0)
                    {
                        byte[] buf1 = new byte[length];

                        dataStream.ReadTimeout = 3000;

                        int offset = 0;
                        int len = 0;

                        while ((len = br.Read(buf1, offset, length)) > 0)
                        {
                            offset += len;
                            length -= len;
                        
                        }
                        /*
                        BinaryWriter sw = new BinaryWriter(File.OpenWrite("test.jpg"));

                        sw.Write(buf1,0,buf1.Length);

                        sw.Close();
                        */
                        try
                        {

                            frame = new Bitmap(new MemoryStream(buf1));

                        }
                        catch {  }

                        fps++;

                        if (lastimage.Second != DateTime.Now.Second)
                        {
                            Console.WriteLine("MJPEG " + fps);
                            fps = 0;
                            lastimage = DateTime.Now;
                        }

                        if (OnNewImage != null)
                            OnNewImage(frame, new EventArgs());
                    }

                    // blank line at end of data
                    ReadLine(br);
                }
                catch (Exception ex) { log.Info(ex); break; }
            }

            // clear last image
            if (OnNewImage != null)
                OnNewImage(null, new EventArgs());

            dataStream.Close();
            response.Close();

            running = false;
        }

        static void getUrlRTSP()
        {

        
        }

        static Dictionary<string, string> getHeader(BinaryReader stream)
        {
            Dictionary<string, string> answer = new Dictionary<string, string>();

            string line;

            do
            {
                line = ReadLine(stream);

                string[] items = line.Split(new char[] { ':' }, StringSplitOptions.RemoveEmptyEntries);


                if (items.Length == 2)
                    answer.Add(items[0].Trim(), items[1].Trim());

            } while (line != "");

            return answer;
        }

    }
}
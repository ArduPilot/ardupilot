using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using AForge;
using AForge.Video;
using AForge.Video.DirectShow;

namespace ArdupilotMega.Utilities
{
    public class Video
    {
        private static FilterInfoCollection videoDevices;
        private static AsyncVideoSource asyncSource;

        public static bool isRunning { get { if (asyncSource == null) return false; return asyncSource.IsRunning; } }

        public static List<string> getDevices()
        {
            List<string> list = new List<string>();
            // Get the collection of video devices
            videoDevices = new FilterInfoCollection(FilterCategory.VideoInputDevice);

            foreach (FilterInfo dev in videoDevices)
            {
                list.Add(dev.Name);
            }

            return list;
        }

        public static void Start(VideoCaptureDevice videoSource)
        {
            videoDevices = new FilterInfoCollection(FilterCategory.VideoInputDevice);

            //VideoCaptureDevice videoSource = new VideoCaptureDevice(videoDevices[Device].MonikerString);
            videoSource.DesiredFrameRate = 25;


            asyncSource = new AsyncVideoSource(videoSource, true);

            asyncSource.NewFrame += new NewFrameEventHandler(asyncSource_NewFrame);
            asyncSource.Start();

        }

        static void asyncSource_NewFrame(object sender, NewFrameEventArgs eventArgs)
        {
            //GCSViews.FlightData.cam_camimage(eventArgs.Frame);
            if (MainV2.instance.IsDisposed)
                Dispose();
        }

        public static void Stop()
        {
            Dispose();
        }

        /// <summary> release everything. </summary>
        public static void Dispose()
        {
            try
            {
                asyncSource.Stop();

                asyncSource = null;
            }
            catch { }

            asyncSource_NewFrame(null, new NewFrameEventArgs(null));
        }

        ~Video()
        {
            Dispose();
        }
    }
}

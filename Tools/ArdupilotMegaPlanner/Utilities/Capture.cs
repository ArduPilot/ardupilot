/****************************************************************************
While the underlying libraries are covered by LGPL, this sample is released 
as public domain.  It is distributed in the hope that it will be useful, but 
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
or FITNESS FOR A PARTICULAR PURPOSE.  
*****************************************************************************/

using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Collections;
using System.Runtime.InteropServices;
using System.Threading;
using System.Diagnostics;
using System.Collections.Generic;

using DirectShowLib;


namespace WebCamService
{
    public delegate void CamImage(Image camimage);

    /// <summary> Summary description for MainForm. </summary>
    public class Capture : ISampleGrabberCB, IDisposable
    {
        #region Member variables

        /// <summary> graph builder interface. </summary>
        private IFilterGraph2 m_FilterGraph = null;
        private IMediaControl m_mediaCtrl = null;

        /// <summary> so we can wait for the async job to finish </summary>
        private ManualResetEvent m_PictureReady = null;

        /// <summary> Set by async routine when it captures an image </summary>
        private volatile bool m_bGotOne = false;

        /// <summary> Indicates the status of the graph </summary>
        private bool m_bRunning = false;

        /// <summary> Dimensions of the image, calculated once in constructor. </summary>
        private IntPtr m_handle = IntPtr.Zero;
        private int m_videoWidth;
        private int m_videoHeight;
        private int m_stride;
        public int m_Dropped = 0;

        public Image image = null;
        IntPtr ip = IntPtr.Zero;

        public event CamImage camimage;
        Thread timer1;

        #endregion

        #region API

        [DllImport("Kernel32.dll", EntryPoint="RtlMoveMemory")]
        private static extern void CopyMemory(IntPtr Destination, IntPtr Source, int Length);

        #endregion

        public Capture()
        {
        }

        /// <summary> Use capture with selected media caps</summary>
        public Capture(int iDeviceNum, AMMediaType media)
        {
            DsDevice[] capDevices;

            // Get the collection of video devices
            capDevices = DsDevice.GetDevicesOfCat(FilterCategory.VideoInputDevice);

            if (iDeviceNum + 1 > capDevices.Length)
            {
                throw new Exception("No video capture devices found at that index!");
            }

            try
            {
                // Set up the capture graph
                SetupGraph(capDevices[iDeviceNum], media);

                // tell the callback to ignore new images
                m_PictureReady = new ManualResetEvent(false);
                m_bGotOne = true;
                m_bRunning = false;

                //timer1.Interval = 1000 / 15; // 15 fps
                //timer1.Tick += new EventHandler(timer1_Tick);
                //timer1.Start();

                timer1 = new Thread(timer);
                timer1.IsBackground = true;
                timer1.Start();

            }
            catch
            {
                Dispose();
                throw;
            }
        }

        void timer()
        {
            DateTime last = DateTime.Now;

            while (true)
            {
                try
                {
                    System.Threading.Thread.Sleep(1000/25); // 25 fps

                    timer1_Tick(this, null);

                }
                catch (ThreadAbortException)
                {
                    break;
                }
                catch
                {

                }
            }
        }

        /// <summary> release everything. </summary>
        public void Dispose()
        {
            if (timer1 != null)
                timer1.Abort();

            if (camimage != null)
            {
                camimage(null); // clear last pic
            }
            CloseInterfaces();
            if (m_PictureReady != null)
            {
                m_PictureReady.Close();
                m_PictureReady = null;
            }
        }
        // Destructor
        ~Capture()
        {
            Dispose();
        }

        public int Width
        {
            get
            {
                return m_videoWidth;
            }
        }
        public int Height
        {
            get
            {
                return m_videoHeight;
            }
        }
        public int Stride
        {
            get
            {
                return m_stride;
            }
        }
        /// <summary> capture the next image </summary>
        public IntPtr GetBitMap()
        {
            if (m_handle == IntPtr.Zero)
                m_handle = Marshal.AllocCoTaskMem(m_stride * m_videoHeight);

            try
            {
                // get ready to wait for new image
                m_PictureReady.Reset();
                m_bGotOne = false;

                // If the graph hasn't been started, start it.
                Start();

                // Start waiting
                if ( ! m_PictureReady.WaitOne(5000, false) )
                {
                    //throw new Exception("Timeout waiting to get picture");
                }
                //Pause(); //- we are effectivly pulling at 15 fps, so no need to pause
            }
            catch
            {
                Marshal.FreeCoTaskMem(m_handle);
                throw;
            }
	
            // Got one
            return m_handle;
        }
        // Start the capture graph
        public void Start()
        {
            if (!m_bRunning)
            {
                int hr = m_mediaCtrl.Run();
                DsError.ThrowExceptionForHR( hr );

                m_bRunning = true;
            }
        }
        // Pause the capture graph.
        // Running the graph takes up a lot of resources.  Pause it when it
        // isn't needed.
        public void Pause()
        {
            if (m_bRunning)
            {
                int hr = m_mediaCtrl.Pause();
                DsError.ThrowExceptionForHR( hr );

                m_bRunning = false;
            }
        }

        public static List<string> getDevices()
        {
            List<string> list = new List<string>();
            DsDevice[] capDevices;

            // Get the collection of video devices
            capDevices = DsDevice.GetDevicesOfCat(FilterCategory.VideoInputDevice);

            foreach (DsDevice dev in capDevices)
            {
                list.Add(dev.Name);
            }

            return list;
        }

        void timer1_Tick(object sender, EventArgs e)
        {
            try
            {
                ip = this.GetBitMap();
                image = new Bitmap(this.Width, this.Height, this.Stride, PixelFormat.Format24bppRgb, ip);
                image.RotateFlip(RotateFlipType.RotateNoneFlipY);
                if (camimage != null)
                {
                    camimage(image);
                }
            }//System.Windows.Forms.CustomMessageBox.Show("Problem with capture device, grabbing frame took longer than 5 sec");
            catch { Console.WriteLine("Grab bmp failed"); }
        }

        /// <summary> build the capture graph for grabber. </summary>
        private void SetupGraph(DsDevice dev, AMMediaType media)
        {
            int hr;

            ISampleGrabber sampGrabber = null;
            IBaseFilter capFilter = null;
            ICaptureGraphBuilder2 capGraph = null;

            // Get the graphbuilder object
            m_FilterGraph = (IFilterGraph2) new FilterGraph();
            m_mediaCtrl = m_FilterGraph as IMediaControl;
            try
            {
                // Get the ICaptureGraphBuilder2
                capGraph = (ICaptureGraphBuilder2) new CaptureGraphBuilder2();

                // Get the SampleGrabber interface
                sampGrabber = (ISampleGrabber) new SampleGrabber();

                // Start building the graph
                hr = capGraph.SetFiltergraph( m_FilterGraph );
                DsError.ThrowExceptionForHR( hr );

                // Add the video device
                hr = m_FilterGraph.AddSourceFilterForMoniker(dev.Mon, null, "Video input", out capFilter);
                DsError.ThrowExceptionForHR( hr );

                // add video crossbar
                // thanks to Andrew Fernie - this is to get tv tuner cards working
                IAMCrossbar crossbar = null;
                object o;

                hr = capGraph.FindInterface(PinCategory.Capture, MediaType.Video, capFilter, typeof(IAMCrossbar).GUID, out o);
                if (hr >= 0)
                {
                    crossbar = (IAMCrossbar)o;
                    int oPin, iPin;
                    int ovLink, ivLink;
                    ovLink = ivLink = 0;

                    crossbar.get_PinCounts(out oPin, out iPin);
                    int pIdxRel;
                    PhysicalConnectorType tp;
                    for (int i = 0; i < iPin; i++)
                    {
                        crossbar.get_CrossbarPinInfo(true, i, out pIdxRel, out tp);
                        if (tp == PhysicalConnectorType.Video_Composite) ivLink = i;
                    }

                    for (int i = 0; i < oPin; i++)
                    {
                        crossbar.get_CrossbarPinInfo(false, i, out pIdxRel, out tp);
                        if (tp == PhysicalConnectorType.Video_VideoDecoder) ovLink = i;
                    }

                    try
                    {
                        crossbar.Route(ovLink, ivLink);
                        o = null;
                    }

                    catch
                    {
                        throw new Exception("Failed to get IAMCrossbar");
                    }
                }

                //add AVI Decompressor
                IBaseFilter pAVIDecompressor = (IBaseFilter)new AVIDec();
                hr = m_FilterGraph.AddFilter(pAVIDecompressor, "AVI Decompressor");
                DsError.ThrowExceptionForHR(hr);

                //
                IBaseFilter baseGrabFlt = (IBaseFilter)	sampGrabber;
                ConfigureSampleGrabber(sampGrabber);

                // Add the frame grabber to the graph
                hr = m_FilterGraph.AddFilter( baseGrabFlt, "Ds.NET Grabber" );
                DsError.ThrowExceptionForHR( hr );

                SetConfigParms(capGraph, capFilter, media);

                hr = capGraph.RenderStream(PinCategory.Capture, MediaType.Video, capFilter, pAVIDecompressor, baseGrabFlt);
                if (hr < 0)
                {
                    hr = capGraph.RenderStream(PinCategory.Capture, MediaType.Video, capFilter, null, baseGrabFlt);
                }

                DsError.ThrowExceptionForHR( hr );

                SaveSizeInfo(sampGrabber);
            }
            finally
            {
                if (capFilter != null)
                {
                    Marshal.ReleaseComObject(capFilter);
                    capFilter = null;
                }
                if (sampGrabber != null)
                {
                    Marshal.ReleaseComObject(sampGrabber);
                    sampGrabber = null;
                }
                if (capGraph != null)
                {
                    Marshal.ReleaseComObject(capGraph);
                    capGraph = null;
                }
            }
        }

        private void SaveSizeInfo(ISampleGrabber sampGrabber)
        {
            int hr;

            // Get the media type from the SampleGrabber
            AMMediaType media = new AMMediaType();
            hr = sampGrabber.GetConnectedMediaType( media );
            DsError.ThrowExceptionForHR( hr );

            if( (media.formatType != FormatType.VideoInfo) || (media.formatPtr == IntPtr.Zero) )
            {
                throw new NotSupportedException( "Unknown Grabber Media Format" );
            }

            // Grab the size info
            VideoInfoHeader videoInfoHeader = (VideoInfoHeader) Marshal.PtrToStructure( media.formatPtr, typeof(VideoInfoHeader) );
            m_videoWidth = videoInfoHeader.BmiHeader.Width;
            m_videoHeight = videoInfoHeader.BmiHeader.Height;
            m_stride = m_videoWidth * (videoInfoHeader.BmiHeader.BitCount / 8);

            DsUtils.FreeAMMediaType(media);
            media = null;
        }
        private void ConfigureSampleGrabber(ISampleGrabber sampGrabber)
        {
            AMMediaType media;
            int hr;

            // Set the media type to Video/RBG24
            media = new AMMediaType();
            media.majorType	= MediaType.Video;
            media.subType	= MediaSubType.RGB24;
            media.formatType = FormatType.VideoInfo;
            sampGrabber.SetBufferSamples(false);
            sampGrabber.SetOneShot(false);
            hr = sampGrabber.SetMediaType( media );
            DsError.ThrowExceptionForHR( hr );

            DsUtils.FreeAMMediaType(media);
            media = null;

            // Configure the samplegrabber
            hr = sampGrabber.SetCallback( this, 1 );
            DsError.ThrowExceptionForHR( hr );
        }

        // Set the Framerate, and video size
        private void SetConfigParms(ICaptureGraphBuilder2 capGraph, IBaseFilter capFilter, AMMediaType media)
        {
            int hr;
            object o;

            // Find the stream config interface
            hr = capGraph.FindInterface(
                PinCategory.Capture, MediaType.Video, capFilter, typeof(IAMStreamConfig).GUID, out o );

            IAMStreamConfig videoStreamConfig = o as IAMStreamConfig;
            if (videoStreamConfig == null)
            {
                throw new Exception("Failed to get IAMStreamConfig");
            }           

            // Set the new format
            try
            {
                hr = videoStreamConfig.SetFormat(media);
            }
            catch { }

                DsError.ThrowExceptionForHR(hr);


            DsUtils.FreeAMMediaType(media);
            media = null;
        }

        /// <summary> Shut down capture </summary>
        private void CloseInterfaces()
        {
            int hr;

            try
            {
                if( m_mediaCtrl != null )
                {
                    // Stop the graph
                    hr = m_mediaCtrl.Stop();
                    m_bRunning = false;
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine(ex);
            }

            if (m_FilterGraph != null)
            {
                Marshal.ReleaseComObject(m_FilterGraph);
                m_FilterGraph = null;
            }
        }

        /// <summary> sample callback, NOT USED. </summary>
        int ISampleGrabberCB.SampleCB( double SampleTime, IMediaSample pSample )
        {
            if (!m_bGotOne)
            {
                // Set bGotOne to prevent further calls until we
                // request a new bitmap.
                m_bGotOne = true;
                IntPtr pBuffer;

                pSample.GetPointer(out pBuffer);
                int iBufferLen = pSample.GetSize();

                if (pSample.GetSize() > m_stride * m_videoHeight)
                {
                    throw new Exception("Buffer is wrong size");
                }

                CopyMemory(m_handle, pBuffer, m_stride * m_videoHeight);

                // Picture is ready.
                m_PictureReady.Set();
            }

            Marshal.ReleaseComObject(pSample);
            return 0;
        }

        /// <summary> buffer callback, COULD BE FROM FOREIGN THREAD. </summary>
        int ISampleGrabberCB.BufferCB( double SampleTime, IntPtr pBuffer, int BufferLen )
        {
            if (!m_bGotOne)
            {
                // The buffer should be long enought
                if(BufferLen <= m_stride * m_videoHeight)
                {
                    // Copy the frame to the buffer
                    CopyMemory(m_handle, pBuffer, m_stride * m_videoHeight);
                }
                else
                {
                    throw new Exception("Buffer is wrong size");
                }

                // Set bGotOne to prevent further calls until we
                // request a new bitmap.
                m_bGotOne = true;

                // Picture is ready.
                m_PictureReady.Set();
            }
            else
            {
                m_Dropped++;
            }
            return 0;
        }
    }
}

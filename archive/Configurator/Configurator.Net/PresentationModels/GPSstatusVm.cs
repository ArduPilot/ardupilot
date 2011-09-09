using System;
using System.ComponentModel;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Net;

namespace ArducopterConfigurator.PresentationModels
{
    public class GpsStatusVm : NotifyProperyChangedBase, IPresentationModel
    {
        private const string SEND_GPS_DATA = "4";
        private const string STOP_UPDATES = "X";

        public ICommand GetMapCommand { get; private set; }

        private BackgroundWorker bg;

        public GpsStatusVm()
        {
            bg = new BackgroundWorker();
            bg.DoWork += DownloadGoogleMap;
            bg.RunWorkerCompleted += DownloadGoogleMapComplete;

            GetMapCommand = new DelegateCommand(_ => GetMap(), _=> HasFix && !bg.IsBusy);
        }

        private void GetMap()
        {
            var urlTemplate =
                "http://maps.google.com/maps/api/staticmap?&markers=color:blue|label:A|{0:0.00000},{1:0.00000}&zoom=13&size=250x200&maptype=roadmap&sensor=false";

            var url = string.Format(CultureInfo.InvariantCulture, urlTemplate, GpsLatitude, GpsLongitude);
        
            bg.RunWorkerAsync(url);
        }

        private void DownloadGoogleMapComplete(object sender, RunWorkerCompletedEventArgs e)
        {
            MapImage = e.Result as Image;
        }

        private void DownloadGoogleMap(object sender, DoWorkEventArgs e)
        {
            var url = e.Argument as string;
            var Client = new WebClient();
            using (var strm = Client.OpenRead(url))
            {
                var png = Image.FromStream(strm);
                e.Result = png;
            }
        }

        private void sendString(string str)
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs(str));
        }

        public void Activate()
        {
            sendString(SEND_GPS_DATA);
        }

        public void DeActivate()
        {
            sendString(STOP_UPDATES);
        }

        public event EventHandler updatedByApm;


        public  string Name
        {
            get { return "GPS Status"; }
        }

        public void handleLineOfText(string strRx)
        {
            // We don't bother with the automatic property population, as the received string is
            // tab delimeted and the values have crazy scale factors anyway.
            
            // hack for testing with no GPS
            // strRx = "73410000\t407021470\t-740157940\t9242\t0\t19831\t1";
            
            var parts = strRx.Split('\t');
            try
            {
                GpsTime = int.Parse(parts[0].Trim(), CultureInfo.InvariantCulture);
                GpsLatitude = (float)(int.Parse(parts[1].Trim(), CultureInfo.InvariantCulture)) / 10000000;
                GpsLongitude = (float)(int.Parse(parts[2].Trim(), CultureInfo.InvariantCulture)) / 10000000;
                GpsAltitude = int.Parse(parts[3].Trim(), CultureInfo.InvariantCulture) / 100;
                GpsGroundSpeed = (float)(int.Parse(parts[4].Trim(), CultureInfo.InvariantCulture)) / 100;
                GpsGroundCourse = int.Parse(parts[5].Trim(), CultureInfo.InvariantCulture) / 100;
                HasFix = parts[6].Trim() == "1";
            
                // Todo: the number of sats is actually a raw char, not an ascii char like '3'
            }
            catch (FormatException)
            {
                Console.WriteLine("Format Exception in GPS VM, string: " + strRx);
                
            }
          
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;

        #region bindables

        private Image _mapImage;

        public Image MapImage
        {
            get { return _mapImage; }
            set
            {
                if (_mapImage == value) return;
                _mapImage = value;
                FirePropertyChanged("MapImage");
            }
        }


        private int _gpsTime;

        public int GpsTime
        {
            get { return _gpsTime; }
            set
            {
                if (_gpsTime == value) return;
                _gpsTime = value;
                FirePropertyChanged("GpsTime");
            }
        }

        private float _gpsLatitude;

        public float GpsLatitude
        {
            get { return _gpsLatitude; }
            set
            {
                if (_gpsLatitude == value) return;
                _gpsLatitude = value;
                FirePropertyChanged("GpsLatitude");
            }
        }

        private float _gpsLongitude;

        public float GpsLongitude
        {
            get { return _gpsLongitude; }
            set
            {
                if (_gpsLongitude == value) return;
                _gpsLongitude = value;
                FirePropertyChanged("GpsLongitude");
            }
        }

        private float _gpsGroundSpeed;

        public float GpsGroundSpeed
        {
            get { return _gpsGroundSpeed; }
            set
            {
                if (_gpsGroundSpeed == value) return;
                _gpsGroundSpeed = value;
                FirePropertyChanged("GpsGroundSpeed");
            }
        }

        private int _gpsGroundCourse;

        public int GpsGroundCourse
        {
            get { return _gpsGroundCourse; }
            set
            {
                if (_gpsGroundCourse == value) return;
                _gpsGroundCourse = value;
                FirePropertyChanged("GpsGroundCourse");
            }
        }

        private int _gpsAltitude;

        public int GpsAltitude
        {
            get { return _gpsAltitude; }
            set
            {
                if (_gpsAltitude == value) return;
                _gpsAltitude = value;
                FirePropertyChanged("GpsAltitude");
            }
        }

        private bool _hasFix;

        public bool HasFix
        {
            get { return _hasFix; }
            set
            {
                if (_hasFix == value) return;
                _hasFix = value;
                FirePropertyChanged("HasFix");
            }
        }

        #endregion
    }
}
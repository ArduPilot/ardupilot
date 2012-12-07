using System;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using AGaugeApp;
using System.IO.Ports;
using System.Threading;
using ArdupilotMega.Attributes;
using GMap.NET;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;

using System.Security.Cryptography.X509Certificates;

using System.Net;
using System.Net.Sockets;
using System.Xml; // config file
using System.Runtime.InteropServices; // dll imports
using log4net;
using ZedGraph; // Graphs
using ArdupilotMega;
using System.Reflection;
using ArdupilotMega.Utilities;

using System.IO;

using System.Drawing.Drawing2D;

namespace ArdupilotMega
{
    /// <summary>
    /// Struct as used in Ardupilot
    /// </summary>
    public struct Locationwp
    {
        public byte id;				// command id
        public byte options;
        public float p1;				// param 1
        public float p2;				// param 2
        public float p3;				// param 3
        public float p4;				// param 4
        public float lat;				// Lattitude * 10**7
        public float lng;				// Longitude * 10**7
        public float alt;				// Altitude in centimeters (meters * 100)
    };


    /// <summary>
    /// used to override the drawing of the waypoint box bounding
    /// </summary>
    public class GMapMarkerRect : GMapMarker
    {
        public Pen Pen = new Pen(Brushes.White, 2);

        public Color Color { get { return Pen.Color; } set { Pen.Color = value; } }

        public GMapMarker InnerMarker;

        public int wprad = 0;
        public GMapControl MainMap;

        public GMapMarkerRect(PointLatLng p)
            : base(p)
        {
            Pen.DashStyle = DashStyle.Dash;

            // do not forget set Size of the marker
            // if so, you shall have no event on it ;}
            Size = new System.Drawing.Size(50, 50);
            Offset = new System.Drawing.Point(-Size.Width / 2, -Size.Height / 2 - 20);
        }

        public override void OnRender(Graphics g)
        {
            base.OnRender(g);

            if (wprad == 0 || MainMap == null)
                return;

            // undo autochange in mouse over
            if (Pen.Color == Color.Blue)
                Pen.Color = Color.White;

                double width = (MainMap.Manager.GetDistance(MainMap.FromLocalToLatLng(0, 0), MainMap.FromLocalToLatLng(MainMap.Width, 0)) * 1000.0);
                double height = (MainMap.Manager.GetDistance(MainMap.FromLocalToLatLng(0, 0), MainMap.FromLocalToLatLng(MainMap.Height, 0)) * 1000.0);
                double m2pixelwidth = MainMap.Width / width;
                double m2pixelheight = MainMap.Height / height;

            GPoint loc = new GPoint((int)(LocalPosition.X - (m2pixelwidth * wprad * 2)), LocalPosition.Y);// MainMap.FromLatLngToLocal(wpradposition);

            g.DrawArc(Pen, new System.Drawing.Rectangle(LocalPosition.X - Offset.X - (Math.Abs(loc.X - LocalPosition.X) / 2), LocalPosition.Y - Offset.Y - Math.Abs(loc.X - LocalPosition.X) / 2, Math.Abs(loc.X - LocalPosition.X), Math.Abs(loc.X - LocalPosition.X)), 0, 360);
       
        }
    }

    public class GMapMarkerRover : GMapMarker
    {
        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        static readonly System.Drawing.Size SizeSt = new System.Drawing.Size(global::ArdupilotMega.Properties.Resources.rover.Width, global::ArdupilotMega.Properties.Resources.rover.Height);
        float heading = 0;
        float cog = -1;
        float target = -1;
        float nav_bearing = -1;
        public GMapControl MainMap;

        public GMapMarkerRover(PointLatLng p, float heading, float cog, float nav_bearing, float target, GMapControl map)
            : base(p)
        {
            this.heading = heading;
            this.cog = cog;
            this.target = target;
            this.nav_bearing = nav_bearing;
            Size = SizeSt;
            MainMap = map;
        }

        public override void OnRender(Graphics g)
        {
            Matrix temp = g.Transform;
            g.TranslateTransform(LocalPosition.X, LocalPosition.Y);

            g.RotateTransform(-MainMap.Bearing);

            int length = 500;
            // anti NaN
            try
            {
                g.DrawLine(new Pen(Color.Red, 2), 0.0f, 0.0f, (float)Math.Cos((heading - 90) * deg2rad) * length, (float)Math.Sin((heading - 90) * deg2rad) * length);
            }
            catch { }
            g.DrawLine(new Pen(Color.Green, 2), 0.0f, 0.0f, (float)Math.Cos((nav_bearing - 90) * deg2rad) * length, (float)Math.Sin((nav_bearing - 90) * deg2rad) * length);
            g.DrawLine(new Pen(Color.Black, 2), 0.0f, 0.0f, (float)Math.Cos((cog - 90) * deg2rad) * length, (float)Math.Sin((cog - 90) * deg2rad) * length);
            g.DrawLine(new Pen(Color.Orange, 2), 0.0f, 0.0f, (float)Math.Cos((target - 90) * deg2rad) * length, (float)Math.Sin((target - 90) * deg2rad) * length);
            // anti NaN

            try
            {
                g.RotateTransform(heading);
            }
            catch { }
            g.DrawImageUnscaled(global::ArdupilotMega.Properties.Resources.rover, global::ArdupilotMega.Properties.Resources.rover.Width / -2, global::ArdupilotMega.Properties.Resources.rover.Height / -2);

            g.Transform = temp;
        }
    }


    public class GMapMarkerPlane : GMapMarker
    {
        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        static readonly System.Drawing.Size SizeSt = new System.Drawing.Size(global::ArdupilotMega.Properties.Resources.planeicon.Width, global::ArdupilotMega.Properties.Resources.planeicon.Height);
        float heading = 0;
        float cog = -1;
        float target = -1;
        float nav_bearing = -1;
        public GMapControl MainMap;

        public GMapMarkerPlane(PointLatLng p, float heading, float cog, float nav_bearing,float target, GMapControl map)
            : base(p)
        {
            this.heading = heading;
            this.cog = cog;
            this.target = target;
            this.nav_bearing = nav_bearing;
            Size = SizeSt;
            MainMap = map;
        }

        public override void OnRender(Graphics g)
        {
            Matrix temp = g.Transform;
            g.TranslateTransform(LocalPosition.X, LocalPosition.Y);

            g.RotateTransform(-MainMap.Bearing);

            int length = 500;
// anti NaN
            try
            {
                g.DrawLine(new Pen(Color.Red, 2), 0.0f, 0.0f, (float)Math.Cos((heading - 90) * deg2rad) * length, (float)Math.Sin((heading - 90) * deg2rad) * length);
            }
            catch { }
            g.DrawLine(new Pen(Color.Green, 2), 0.0f, 0.0f, (float)Math.Cos((nav_bearing - 90) * deg2rad) * length, (float)Math.Sin((nav_bearing - 90) * deg2rad) * length);
            g.DrawLine(new Pen(Color.Black, 2), 0.0f, 0.0f, (float)Math.Cos((cog - 90) * deg2rad) * length, (float)Math.Sin((cog - 90) * deg2rad) * length);
            g.DrawLine(new Pen(Color.Orange, 2), 0.0f, 0.0f, (float)Math.Cos((target - 90) * deg2rad) * length, (float)Math.Sin((target - 90) * deg2rad) * length);
// anti NaN
            try
            {

                float desired_lead_dist = 100;


                double width = (MainMap.Manager.GetDistance(MainMap.FromLocalToLatLng(0, 0), MainMap.FromLocalToLatLng(MainMap.Width, 0)) * 1000.0);
                double m2pixelwidth = MainMap.Width / width;

                float alpha = ((desired_lead_dist * (float)m2pixelwidth) / MainV2.comPort.MAV.cs.radius) * rad2deg;

                if (MainV2.comPort.MAV.cs.radius < -1)
                {
                    // fixme 

                    float p1 = (float)Math.Cos((cog) * deg2rad) * MainV2.comPort.MAV.cs.radius + MainV2.comPort.MAV.cs.radius;

                    float p2 = (float)Math.Sin((cog) * deg2rad) * MainV2.comPort.MAV.cs.radius + MainV2.comPort.MAV.cs.radius;

                    g.DrawArc(new Pen(Color.HotPink, 2), p1, p2, Math.Abs(MainV2.comPort.MAV.cs.radius) * 2, Math.Abs(MainV2.comPort.MAV.cs.radius) * 2, cog, alpha);

                }

                else if (MainV2.comPort.MAV.cs.radius > 1)
                {
                    // correct

                    float p1 = (float)Math.Cos((cog - 180) * deg2rad) * MainV2.comPort.MAV.cs.radius + MainV2.comPort.MAV.cs.radius;

                    float p2 = (float)Math.Sin((cog - 180) * deg2rad) * MainV2.comPort.MAV.cs.radius + MainV2.comPort.MAV.cs.radius;

                    g.DrawArc(new Pen(Color.HotPink, 2), -p1, -p2, MainV2.comPort.MAV.cs.radius * 2, MainV2.comPort.MAV.cs.radius * 2, cog - 180, alpha);
                }

            }

            catch { }


            try {
            g.RotateTransform(heading);
            } catch{}
            g.DrawImageUnscaled(global::ArdupilotMega.Properties.Resources.planeicon, global::ArdupilotMega.Properties.Resources.planeicon.Width / -2, global::ArdupilotMega.Properties.Resources.planeicon.Height / -2);

            g.Transform = temp;
        }
    }


    public class GMapMarkerQuad : GMapMarker
    {
        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        static readonly System.Drawing.Size SizeSt = new System.Drawing.Size(global::ArdupilotMega.Properties.Resources.quadicon.Width, global::ArdupilotMega.Properties.Resources.quadicon.Height);
        float heading = 0;
        float cog = -1;
        float target = -1;

        public GMapMarkerQuad(PointLatLng p, float heading, float cog, float target)
            : base(p)
        {
            this.heading = heading;
            this.cog = cog;
            this.target = target;
            Size = SizeSt;
        }

        public override void OnRender(Graphics g)
        {
            Matrix temp = g.Transform;
            g.TranslateTransform(LocalPosition.X, LocalPosition.Y);

            int length = 500;
// anti NaN
            try
            {
                g.DrawLine(new Pen(Color.Red, 2), 0.0f, 0.0f, (float)Math.Cos((heading - 90) * deg2rad) * length, (float)Math.Sin((heading - 90) * deg2rad) * length);
            }
            catch { }
            //g.DrawLine(new Pen(Color.Green, 2), 0.0f, 0.0f, (float)Math.Cos((nav_bearing - 90) * deg2rad) * length, (float)Math.Sin((nav_bearing - 90) * deg2rad) * length);
            g.DrawLine(new Pen(Color.Black, 2), 0.0f, 0.0f, (float)Math.Cos((cog - 90) * deg2rad) * length, (float)Math.Sin((cog - 90) * deg2rad) * length);
            g.DrawLine(new Pen(Color.Orange, 2), 0.0f, 0.0f, (float)Math.Cos((target - 90) * deg2rad) * length, (float)Math.Sin((target - 90) * deg2rad) * length);
// anti NaN
            try
            {
                g.RotateTransform(heading);
            }
            catch { }
            g.DrawImageUnscaled(global::ArdupilotMega.Properties.Resources.quadicon, global::ArdupilotMega.Properties.Resources.quadicon.Width / -2 + 2, global::ArdupilotMega.Properties.Resources.quadicon.Height / -2);

            g.Transform = temp;
        }
    }

    public class PointLatLngAlt
    {
        public double Lat = 0;
        public double Lng = 0;
        public double Alt = 0;
        public string Tag = "";
        public Color color = Color.White;

        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        public PointLatLngAlt(double lat, double lng, double alt, string tag)
        {
            this.Lat = lat;
            this.Lng = lng;
            this.Alt = alt;
            this.Tag = tag;
        }

        public PointLatLngAlt()
        {

        }

        public PointLatLngAlt(GMap.NET.PointLatLng pll)
        {
            this.Lat = pll.Lat;
            this.Lng = pll.Lng;
        }

        public PointLatLngAlt(Locationwp locwp)
        {
            this.Lat = locwp.lat;
            this.Lng = locwp.lng;
            this.Alt = locwp.alt;
        }

        public PointLatLngAlt(PointLatLngAlt plla)
        {
            this.Lat = plla.Lat;
            this.Lng = plla.Lng;
            this.Alt = plla.Alt;
            this.color = plla.color;
            this.Tag = plla.Tag;
        }

        public PointLatLng Point()
        {
            return new PointLatLng(Lat, Lng);
        }

        public override bool Equals(Object pllaobj)
         {
             PointLatLngAlt plla = (PointLatLngAlt)pllaobj;

             if (plla == null)
                 return false;

             if (this.Lat == plla.Lat &&
             this.Lng == plla.Lng &&
             this.Alt == plla.Alt &&
             this.color == plla.color &&
             this.Tag == plla.Tag)
             {
                 return true;
             }
             return false;
         }

        public override int GetHashCode()
        {
            return (int)((Lat + Lng + Alt) * 100);
        }

        public override string ToString()
        {
            return Lat + "," + Lng + "," + Alt;
        }

        /// <summary>
        /// Calc Distance in M
        /// </summary>
        /// <param name="p2"></param>
        /// <returns>Distance in M</returns>
        public double GetDistance(PointLatLngAlt p2)
        {
            double d = Lat * 0.017453292519943295;
            double num2 = Lng * 0.017453292519943295;
            double num3 = p2.Lat * 0.017453292519943295;
            double num4 = p2.Lng * 0.017453292519943295;
            double num5 = num4 - num2;
            double num6 = num3 - d;
            double num7 = Math.Pow(Math.Sin(num6 / 2.0), 2.0) + ((Math.Cos(d) * Math.Cos(num3)) * Math.Pow(Math.Sin(num5 / 2.0), 2.0));
            double num8 = 2.0 * Math.Atan2(Math.Sqrt(num7), Math.Sqrt(1.0 - num7));
            return (6371 * num8) * 1000.0; // M
        }

        public double GetDistance2(PointLatLngAlt p2)
        {
            //http://www.movable-type.co.uk/scripts/latlong.html
            var R = 6371; // 6371 km
            var dLat = (p2.Lat - Lat) * deg2rad;
            var dLon = (p2.Lng - Lng) * deg2rad;
            var lat1 = Lat * deg2rad;
            var lat2 = p2.Lat * deg2rad;

            var a = Math.Sin(dLat / 2) * Math.Sin(dLat / 2) +
                    Math.Sin(dLon / 2) * Math.Sin(dLon / 2) * Math.Cos(lat1) * Math.Cos(lat2);
            var c = 2.0 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
            var d = R * c * 1000.0; // M

            return d;
        }
    }

    class NoCheckCertificatePolicy : ICertificatePolicy
    {
        public bool CheckValidationResult(ServicePoint srvPoint, X509Certificate certificate, WebRequest request, int certificateProblem)
        {
            return true;
        }
    } 


    public class Common
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        public enum distances
        {
            Meters,
            Feet
        }

        public enum speeds
        {
            ms,
            fps,
            kph,
            mph,
            knots
        }

        public enum ap_product
        {
            [DisplayText("HIL")]
            AP_PRODUCT_ID_NONE = 0x00,	// Hardware in the loop
            [DisplayText("APM1 1280")]
            AP_PRODUCT_ID_APM1_1280 = 0x01,// APM1 with 1280 CPUs
            [DisplayText("APM1 2560")]
            AP_PRODUCT_ID_APM1_2560 = 0x02,// APM1 with 2560 CPUs
            [DisplayText("SITL")]
            AP_PRODUCT_ID_SITL = 0x03,// Software in the loop
            [DisplayText("APM2 ES C4")]
            AP_PRODUCT_ID_APM2ES_REV_C4 = 0x14,// APM2 with MPU6000ES_REV_C4
            [DisplayText("APM2 ES C5")]
            AP_PRODUCT_ID_APM2ES_REV_C5 = 0x15,	// APM2 with MPU6000ES_REV_C5
            [DisplayText("APM2 ES D6")]
            AP_PRODUCT_ID_APM2ES_REV_D6 = 0x16,	// APM2 with MPU6000ES_REV_D6
            [DisplayText("APM2 ES D7")]
            AP_PRODUCT_ID_APM2ES_REV_D7 = 0x17,	// APM2 with MPU6000ES_REV_D7
            [DisplayText("APM2 ES D8")]
            AP_PRODUCT_ID_APM2ES_REV_D8 = 0x18,	// APM2 with MPU6000ES_REV_D8	
            [DisplayText("APM2 C4")]
            AP_PRODUCT_ID_APM2_REV_C4 = 0x54,// APM2 with MPU6000_REV_C4 	
            [DisplayText("APM2 C5")]
            AP_PRODUCT_ID_APM2_REV_C5 = 0x55,	// APM2 with MPU6000_REV_C5 	
            [DisplayText("APM2 D6")]
            AP_PRODUCT_ID_APM2_REV_D6 = 0x56,	// APM2 with MPU6000_REV_D6 		
            [DisplayText("APM2 D7")]
            AP_PRODUCT_ID_APM2_REV_D7 = 0x57,	// APM2 with MPU6000_REV_D7 	
            [DisplayText("APM2 D8")]
            AP_PRODUCT_ID_APM2_REV_D8 = 0x58,	// APM2 with MPU6000_REV_D8 	
            [DisplayText("APM2 D9")]
            AP_PRODUCT_ID_APM2_REV_D9 = 0x59	// APM2 with MPU6000_REV_D9 
        }

        public enum apmmodes
        {
            [DisplayText("Manual")]
            MANUAL = 0,
            [DisplayText("Circle")]
            CIRCLE = 1,
            [DisplayText("Stabilize")]
            STABILIZE = 2,
            [DisplayText("FBW A")]
            FLY_BY_WIRE_A = 5,
            [DisplayText("FBW B")]
            FLY_BY_WIRE_B = 6,
            [DisplayText("Auto")]
            AUTO = 10,
            [DisplayText("RTL")]
            RTL = 11,
            [DisplayText("Loiter")]
            LOITER = 12,
            [DisplayText("Guided")]
            GUIDED = 15
        }

        public enum aprovermodes
        {
            [DisplayText("Manual")]
            MANUAL = 0,
            [DisplayText("Circle")]
            CIRCLE = 1,
            [DisplayText("Learning")]
            LEARNING = 2,
            [DisplayText("FBW A")]
            FLY_BY_WIRE_A = 5,
            [DisplayText("FBW B")]
            FLY_BY_WIRE_B = 6,
            [DisplayText("Auto")]
            AUTO = 10,
            [DisplayText("RTL")]
            RTL = 11,
            [DisplayText("Loiter")]
            LOITER = 12,
            [DisplayText("Guided")]
            GUIDED = 15,
            HEADALT = 17,
            SARSEC = 18,
            SARGRID = 19,
            THERMAL = 20,
            LAND = 21
        }

        public enum ac2modes
        {
            [DisplayText("Stabilize")]
            STABILIZE = 0,			// hold level position
            [DisplayText("Acro")]
            ACRO = 1,			// rate control
            [DisplayText("Alt Hold")]
            ALT_HOLD = 2,		// AUTO control
            [DisplayText("Auto")]
            AUTO = 3,			// AUTO control
            [DisplayText("Guided")]
            GUIDED = 4,		// AUTO control
            [DisplayText("Loiter")]
            LOITER = 5,		// Hold a single location
            [DisplayText("RTL")]
            RTL = 6,				// AUTO control
            [DisplayText("Circle")]
            CIRCLE = 7,
            [DisplayText("Pos Hold")]
            POSITION = 8,
            [DisplayText("Land")]
            LAND = 9,				// AUTO control
            OF_LOITER = 10,
            [DisplayText("Toy")]
			TOY = 11
        }

        public enum ac2ch7modes
        {
            [DisplayText("Do Nothing")]
            CH7_DO_NOTHING = 0,
            [DisplayText("Flip")]
            CH7_FLIP = 2,
            [DisplayText("Simple Mode")]
            CH7_SIMPLE_MODE = 3,
            [DisplayText("Return to Launch")]
            CH7_RTL = 4,
            [DisplayText("Save Trim")]
            CH7_AUTO_TRIM = 5,
            [DisplayText("Save Waypoint")]
            CH7_SAVE_WP = 7,
             [DisplayText("Camera Trigger")]
            CH7_CAMERA_TRIGGER = 9
        }

        public enum ac2ch6modes
        {
            // CH_6 Tuning
            // -----------
            CH6_NONE = 0,
            // Attitude
            CH6_STABILIZE_KP = 1,
            CH6_STABILIZE_KI = 2,
            CH6_YAW_KP = 3,
            // Rate
            CH6_RATE_KP = 4,
            CH6_RATE_KI = 5,
            CH6_RATE_KD = 21,
            CH6_YAW_RATE_KP = 6,
            // Altitude rate controller
            CH6_THROTTLE_KP = 7,
            // Extras
            CH6_TOP_BOTTOM_RATIO = 8,
            CH6_RELAY = 9,
            CH6_TRAVERSE_SPEED = 10,

            CH6_NAV_P = 11,
            CH6_LOITER_P = 12,
            CH6_HELI_EXTERNAL_GYRO = 13,

            // altitude controller
            CH6_THR_HOLD_KP = 14,
            CH6_Z_GAIN = 15,
            //CH6_DAMP = 16,

            // optical flow controller
            CH6_OPTFLOW_KP = 17,
            CH6_OPTFLOW_KI = 18,
            CH6_OPTFLOW_KD = 19,

            CH6_NAV_I = 20,
            CH6_LOITER_RATE_P = 22,
            CH6_LOITER_RATE_D = 23,
            CH6_YAW_KI = 24,
            CH6_ACRO_KP = 25,
            CH6_YAW_RATE_KD = 26,
            CH6_LOITER_KI = 27,
            CH6_LOITER_RATE_KI = 28,
            CH6_STABILIZE_KD = 29
        }

        public static void linearRegression()
        {
            double[] values = { 4.8, 4.8, 4.5, 3.9, 4.4, 3.6, 3.6, 2.9, 3.5, 3.0, 2.5, 2.2, 2.6, 2.1, 2.2 };
            
            double xAvg = 0;
            double yAvg = 0;

            for (int x = 0; x < values.Length; x++)
            {
                xAvg += x;
                yAvg += values[x];
            }

            xAvg = xAvg / values.Length;
            yAvg = yAvg / values.Length;


            double v1 = 0;
            double v2 = 0;

            for (int x = 0; x < values.Length; x++)
            {
                v1 += (x - xAvg) * (values[x] - yAvg);
                v2 += Math.Pow(x - xAvg, 2);
            }

            double a = v1 / v2;
            double b = yAvg - a * xAvg;

            log.Debug("y = ax + b");
            log.DebugFormat("a = {0}, the slope of the trend line.", Math.Round(a, 2));
            log.DebugFormat("b = {0}, the intercept of the trend line.", Math.Round(b, 2));

            //Console.ReadLine();
        }
       


        
        public static bool getFilefromNet(string url,string saveto) {
            try
            {
                // this is for mono to a ssl server
                //ServicePointManager.CertificatePolicy = new NoCheckCertificatePolicy(); 

                ServicePointManager.ServerCertificateValidationCallback =
    new System.Net.Security.RemoteCertificateValidationCallback((sender, certificate, chain, policyErrors) => { return true; });

                // Create a request using a URL that can receive a post. 
                WebRequest request = WebRequest.Create(url);
                request.Timeout = 10000;
                // Set the Method property of the request to POST.
                request.Method = "GET";
                // Get the response.
                WebResponse response = request.GetResponse();
                // Display the status.
                log.Info(((HttpWebResponse)response).StatusDescription);
                if (((HttpWebResponse)response).StatusCode != HttpStatusCode.OK)
                    return false;
                // Get the stream containing content returned by the server.
                Stream dataStream = response.GetResponseStream();

                long bytes = response.ContentLength;
                long contlen = bytes;

                byte[] buf1 = new byte[1024];

                FileStream fs = new FileStream(saveto + ".new", FileMode.Create);

                DateTime dt = DateTime.Now;

                while (dataStream.CanRead && bytes > 0)
                {
                    Application.DoEvents();
                    log.Debug(saveto + " " + bytes);
                    int len = dataStream.Read(buf1, 0, buf1.Length);
                    bytes -= len;
                    fs.Write(buf1, 0, len);
                }

                fs.Close();
                dataStream.Close();
                response.Close();

                File.Delete(saveto);
                File.Move(saveto + ".new", saveto);

                return true;
            }
            catch (Exception ex) { log.Info("getFilefromNet(): " + ex.ToString()); return false; }
        }
        
        public static Type getModes()
        {
            if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
                return typeof(apmmodes);
            }
            else if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduCopter2)
            {
                return typeof(ac2modes);
            }
            else if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduRover)
            {
                return typeof(aprovermodes);
            }

            return null;
        }

        public static List<KeyValuePair<int,string>> getModesList()
        {
            if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
                var flightModes = EnumTranslator.Translate<apmmodes>();
                return flightModes.ToList();
            }
            else if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduCopter2)
            {
                var flightModes = EnumTranslator.Translate<ac2modes>();
                return flightModes.ToList();
            }
            else if (MainV2.comPort.MAV.cs.firmware == MainV2.Firmwares.ArduRover)
            {
                var flightModes = EnumTranslator.Translate<aprovermodes>();
                return flightModes.ToList();
            }

            return null;
        }

        public static Form LoadingBox(string title, string promptText)
        {
            Form form = new Form();
            System.Windows.Forms.Label label = new System.Windows.Forms.Label();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainV2));
            form.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));

            form.Text = title;
            label.Text = promptText;

            label.SetBounds(9, 50, 372, 13);

            label.AutoSize = true;

            form.ClientSize = new Size(396, 107);
            form.Controls.AddRange(new Control[] { label });
            form.ClientSize = new Size(Math.Max(300, label.Right + 10), form.ClientSize.Height);
            form.FormBorderStyle = FormBorderStyle.FixedDialog;
            form.StartPosition = FormStartPosition.CenterScreen;
            form.MinimizeBox = false;
            form.MaximizeBox = false;

            ThemeManager.ApplyThemeTo(form);

            form.Show();
            form.Refresh();
            label.Refresh();
            Application.DoEvents();
            return form;
        }

        public static DialogResult MessageShowAgain(string title, string promptText)
        {
            Form form = new Form();
            System.Windows.Forms.Label label = new System.Windows.Forms.Label();
            CheckBox chk = new CheckBox();
            ArdupilotMega.Controls.MyButton buttonOk = new ArdupilotMega.Controls.MyButton();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainV2));
            form.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));

            form.Text = title;
            label.Text = promptText;

            chk.Tag = ("SHOWAGAIN_" + title.Replace(" ","_"));
            chk.AutoSize = true;
            chk.Text = "Show me again?";
            chk.Checked = true;
            chk.Location = new Point(9,80);

            if (MainV2.config[(string)chk.Tag] != null && (string)MainV2.config[(string)chk.Tag] == "False") // skip it
            {
                form.Dispose();
                chk.Dispose();
                buttonOk.Dispose();
                label.Dispose();
                return DialogResult.OK;
            }

            chk.CheckStateChanged += new EventHandler(chk_CheckStateChanged);

            buttonOk.Text = "OK";
            buttonOk.DialogResult = DialogResult.OK;
            buttonOk.Location = new Point(form.Right - 100 ,80);

            label.SetBounds(9, 40, 372, 13);

            label.AutoSize = true;

            form.ClientSize = new Size(396, 107);
            form.Controls.AddRange(new Control[] { label, chk, buttonOk });
            form.ClientSize = new Size(Math.Max(300, label.Right + 10), form.ClientSize.Height);
            form.FormBorderStyle = FormBorderStyle.FixedDialog;
            form.StartPosition = FormStartPosition.CenterScreen;
            form.MinimizeBox = false;
            form.MaximizeBox = false;

            ThemeManager.ApplyThemeTo(form);

            DialogResult dialogResult =form.ShowDialog();

            form.Dispose();

            form = null;

            return dialogResult;
        }

        static void chk_CheckStateChanged(object sender, EventArgs e)
        {
            MainV2.config[(string)((CheckBox)(sender)).Tag] = ((CheckBox)(sender)).Checked.ToString();
        }

        //from http://www.csharp-examples.net/inputbox/
        public static DialogResult InputBox(string title, string promptText, ref string value)
        {
            Form form = new Form();
            System.Windows.Forms.Label label = new System.Windows.Forms.Label();
            TextBox textBox = new TextBox();
            ArdupilotMega.Controls.MyButton buttonOk = new ArdupilotMega.Controls.MyButton();
            ArdupilotMega.Controls.MyButton buttonCancel = new ArdupilotMega.Controls.MyButton();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainV2));
            form.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));

            form.TopMost = true;
            form.TopLevel = true;

            form.Text = title;
            label.Text = promptText;
            textBox.Text = value;

            buttonOk.Text = "OK";
            buttonCancel.Text = "Cancel";
            buttonOk.DialogResult = DialogResult.OK;
            buttonCancel.DialogResult = DialogResult.Cancel;

            label.SetBounds(9, 20, 372, 13);
            textBox.SetBounds(12, 36, 372, 20);
            buttonOk.SetBounds(228, 72, 75, 23);
            buttonCancel.SetBounds(309, 72, 75, 23);

            label.AutoSize = true;
            textBox.Anchor = textBox.Anchor | AnchorStyles.Right;
            buttonOk.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
            buttonCancel.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;

            form.ClientSize = new Size(396, 107);
            form.Controls.AddRange(new Control[] { label, textBox, buttonOk, buttonCancel });
            form.ClientSize = new Size(Math.Max(300, label.Right + 10), form.ClientSize.Height);
            form.FormBorderStyle = FormBorderStyle.FixedSingle;
            form.StartPosition = FormStartPosition.CenterScreen;
            form.MinimizeBox = false;
            form.MaximizeBox = false;
            form.AcceptButton = buttonOk;
            form.CancelButton = buttonCancel;

            ThemeManager.ApplyThemeTo(form);

            DialogResult dialogResult = DialogResult.Cancel;

            Console.WriteLine("Input Box");

                form.ShowDialog();

            Console.WriteLine("Input Box 2");

            dialogResult = form.DialogResult;

            if (dialogResult == DialogResult.OK)
            {
                value = textBox.Text;
            }

            form.Dispose();

            form = null;
            
            return dialogResult;
        }

        public static string speechConversion(string input)
        {
            if (MainV2.comPort.MAV.cs.wpno == 0)
            {
                input = input.Replace("{wpn}", "Home");
            }
            else
            {
                input = input.Replace("{wpn}", MainV2.comPort.MAV.cs.wpno.ToString());
            }

            input = input.Replace("{asp}", MainV2.comPort.MAV.cs.airspeed.ToString("0"));

            input = input.Replace("{alt}", MainV2.comPort.MAV.cs.alt.ToString("0"));

            input = input.Replace("{wpa}", MainV2.comPort.MAV.cs.targetalt.ToString("0"));

            input = input.Replace("{gsp}", MainV2.comPort.MAV.cs.groundspeed.ToString("0"));

            input = input.Replace("{mode}", MainV2.comPort.MAV.cs.mode.ToString());

            input = input.Replace("{batv}", MainV2.comPort.MAV.cs.battery_voltage.ToString("0.00"));

            return input;
        }
    }

    public class VerticalProgressBar : HorizontalProgressBar
    {
        protected override CreateParams CreateParams
        {
            get
            {
                CreateParams cp = base.CreateParams;
                cp.Style |= 0x04;
                return cp;
            }
        }
    }

    public class VerticalProgressBar2 : HorizontalProgressBar2
    {
        protected override void OnPaint(PaintEventArgs e)
        {
            e.Graphics.TranslateTransform(0, e.Graphics.ClipBounds.Height);
            e.Graphics.RotateTransform(270);
            e.Graphics.ScaleTransform((float)this.Height / (float)this.Width, (float)this.Width / (float)this.Height);
            base.OnPaint(e);
        }
    }

    public class HorizontalProgressBar2 : BSE.Windows.Forms.ProgressBar
    {
        private string m_Text;
        int offset = 0;
        int _min = 0;
        int _max = 0;
        int _value = 0;
        System.Windows.Forms.Label lbl1 = new System.Windows.Forms.Label();
        System.Windows.Forms.Label lbl = new System.Windows.Forms.Label();
        public bool reverse = false;
        int displayvalue = 0;

        public HorizontalProgressBar2()
            : base()
        {
        }

        public new int Value
        {
            get { return _value; }
            set
            {
                if (_value == value)
                    return;
                _value = value;
                displayvalue = _value;

                if (reverse)
                {
                    int dif = _value - Minimum;
                    _value = Maximum - dif;
                }

                int ans = _value + offset;
                if (ans <= base.Minimum)
                {
                    ans = base.Minimum + 1; // prevent an exception for the -1 hack
                }
                else if (ans >= base.Maximum)
                {
                    ans = base.Maximum;
                }
                
                base.Value = ans;

                if (this.DesignMode) return;

                if (this.Parent != null)
                {
                    this.Parent.Controls.Add(lbl);
                    this.Parent.Controls.Add(lbl1);
                }
            }
        }

        public new int Minimum
        {
            get { return _min; }
            set
            {
                _min = value;
                if (_min < 0)
                {
                    base.Minimum = 0; offset = (_max - value) / 2; base.Maximum = _max - value;
                }
                else
                {
                    base.Minimum = value;
                }
            }
        }

        public new int Maximum { get { return _max; } set { _max = value; base.Maximum = value; } }

        [System.ComponentModel.Browsable(true),
System.ComponentModel.Category("Mine"),
System.ComponentModel.Description("Text under Bar")]
        public string Label
        {
            get
            {
                return m_Text;
            }
            set
            {
                if (m_Text != value)
                {
                    m_Text = value;
                }
            }
        }

        private void drawlbl(Graphics e)
        {
            lbl.Location = new Point(this.Location.X, this.Location.Y + this.Height + 2);
            lbl.ClientSize = new Size(this.Width, 13);
            lbl.TextAlign = ContentAlignment.MiddleCenter;
            lbl.Text = m_Text;

            lbl1.Location = new Point(this.Location.X, this.Location.Y + this.Height + 15);
            lbl1.ClientSize = new Size(this.Width, 13);
            lbl1.TextAlign = ContentAlignment.MiddleCenter;
            lbl1.Text = displayvalue.ToString();

            if (minline != 0 && maxline != 0)
            {
                float range = this.Maximum - this.Minimum;
                float range2 = this.Width;
                Pen redPen = new Pen(Color.Red, 2);

                SolidBrush mybrush = new SolidBrush(Color.FromArgb(0x40, 0x57, 0x04));

                if ((Type)this.GetType() == typeof(VerticalProgressBar2)) {
                    e.ResetTransform();
                    range2 = this.Height;
                    if (reverse)
                    {
                        e.DrawLine(redPen, 0, (maxline - this.Minimum) / range * range2 + 0, this.Width, (maxline - this.Minimum) / range * range2 + 0);
                        e.DrawLine(redPen, 0, (minline - this.Minimum) / range * range2 + 6, this.Width, (minline - this.Minimum) / range * range2 + 6);
                        e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, mybrush, 5, (maxline - this.Minimum) / range * range2 + 2);
                        e.DrawString(minline.ToString(), SystemFonts.DefaultFont, Brushes.White, 5, (minline - this.Minimum) / range * range2 - 10);
                    }
                    else
                    {
                        e.DrawLine(redPen, 0, (this.Maximum - minline) / range * range2 + 0, this.Width, (this.Maximum - minline) / range * range2 + 0);
                        e.DrawLine(redPen, 0, (this.Maximum - maxline) / range * range2 + 6, this.Width, (this.Maximum - maxline) / range * range2 + 6);
                        e.DrawString(minline.ToString(), SystemFonts.DefaultFont, mybrush, 5, (this.Maximum - minline) / range * range2 + 2);
                        e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, Brushes.White, 5, (this.Maximum - maxline) / range * range2 - 10);
                    }
                } else {
                    if (reverse)
                    {
                        e.DrawLine(redPen, (this.Maximum - minline) / range * range2 - 0, 0, (this.Maximum - minline) / range * range2 - 0, this.Height);
                        e.DrawLine(redPen, (this.Maximum - maxline) / range * range2 - 0, 0, (this.Maximum - maxline) / range * range2 - 0, this.Height);
                        e.DrawString(minline.ToString(), SystemFonts.DefaultFont, mybrush, (this.Maximum - minline) / range * range2 - 30, 5);
                        e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, Brushes.White, (this.Maximum - maxline) / range * range2 - 0, 5);
                    }
                    else
                    {
                        e.DrawLine(redPen, (minline - this.Minimum) / range * range2 - 0, 0, (minline - this.Minimum) / range * range2 - 0, this.Height);
                        e.DrawLine(redPen, (maxline - this.Minimum) / range * range2 - 0, 0, (maxline - this.Minimum) / range * range2 - 0, this.Height);
                        e.DrawString(minline.ToString(), SystemFonts.DefaultFont, mybrush, (minline - this.Minimum) / range * range2 - 30, 5);
                        e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, Brushes.White, (maxline - this.Minimum) / range * range2 - 0, 5);
                    }
                }
            }
        }

        public int minline { get; set; }
        public int maxline { get; set; }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            drawlbl(e.Graphics);
        }

    }

    public class HorizontalProgressBar : ProgressBar
    {
        private string m_Text;
        int offset = 0;
        int _min = 0;
        int _max = 0;
        int _value = 0;
        bool ctladded = false;
        System.Windows.Forms.Label lbl1 = new System.Windows.Forms.Label();
        System.Windows.Forms.Label lbl = new System.Windows.Forms.Label();


        public HorizontalProgressBar()
            : base()
        {
            drawlbl();
            //this.Parent.Controls.AddRange(new Control[] { lbl, lbl1 });
        }

        public new int Value
        {
            get { return _value; }
            set
            {
                _value = value;
                int ans = value + offset;
                if (ans <= base.Minimum)
                {
                    ans = base.Minimum + 1; // prevent an exception for the -1 hack
                }
                else if (ans >= base.Maximum)
                {
                    ans = base.Maximum;
                }
                base.Value = ans;
                drawlbl();
                base.Value = ans - 1;
                drawlbl();
                base.Value = ans;
                drawlbl();

                if (this.Parent != null && ctladded == false)
                {
                    this.Parent.Controls.Add(lbl);
                    this.Parent.Controls.Add(lbl1);
                    ctladded = true;
                }
            }
        }

        public new int Minimum
        {
            get { return _min; }
            set
            {
                _min = value;
                if (_min < 0)
                {
                    base.Minimum = 0; offset = (_max - value) / 2; base.Maximum = _max - value;
                }
                else
                {
                    base.Minimum = value;
                }

                if (this.DesignMode) return;

                if (this.Parent != null && ctladded == false)
                {
                    this.Parent.Controls.Add(lbl);
                    this.Parent.Controls.Add(lbl1);
                    ctladded = true;
                }
            }
        }

        public new int Maximum { get { return _max; } set { _max = value; base.Maximum = value; } }

        [System.ComponentModel.Browsable(true),
System.ComponentModel.Category("Mine"),
System.ComponentModel.Description("Text under Bar")]
        public string Label
        {
            get
            {
                return m_Text;
            }
            set
            {
                if (m_Text != value)
                {
                    m_Text = value;
                }
            }
        }

        private void drawlbl()
        {
            lbl.Location = new Point(this.Location.X, this.Location.Y + this.Height + 2);
            lbl.ClientSize = new Size(this.Width, 13);
            lbl.TextAlign = ContentAlignment.MiddleCenter;
            lbl.Text = m_Text;

            lbl1.Location = new Point(this.Location.X, this.Location.Y + this.Height + 15);
            lbl1.ClientSize = new Size(this.Width, 13);
            lbl1.TextAlign = ContentAlignment.MiddleCenter;
            lbl1.Text = Value.ToString();

            if (minline != 0 && maxline != 0)
            {
                float range = this.Maximum - this.Minimum;
                float range2 = this.Width;
                Graphics e = this.CreateGraphics();
                Pen redPen = new Pen(Color.Red, 2);

                if ((Type)this.GetType() == typeof(VerticalProgressBar)) {
                    range2 = this.Height;
                    e.DrawLine(redPen, 0, (this.Maximum - minline) / range * range2 + 0, this.Width, (this.Maximum - minline) / range * range2 + 0);
                    e.DrawLine(redPen, 0, (this.Maximum - maxline) / range * range2 + 0, this.Width, (this.Maximum - maxline) / range * range2 + 0);
                    e.DrawString(minline.ToString(), SystemFonts.DefaultFont, Brushes.Black, 5, (this.Maximum - minline) / range * range2 + 2);
                    e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, Brushes.Black, 5, (this.Maximum - maxline) / range * range2 - 15);
                } else {
                    e.DrawLine(redPen, (minline - this.Minimum) / range * range2 - 3, 0, (minline - this.Minimum) / range * range2 - 3, this.Height);
                    e.DrawLine(redPen, (maxline - this.Minimum) / range * range2 - 3, 0, (maxline - this.Minimum) / range * range2 - 3, this.Height);
                    e.DrawString(minline.ToString(), SystemFonts.DefaultFont, Brushes.Black, (minline - this.Minimum) / range * range2 - 35, 5);
                    e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, Brushes.Black, (maxline - this.Minimum) / range * range2 - 3, 5);
                }
            }
        }

        public int minline { get; set; }
        public int maxline { get; set; }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            drawlbl();
        }
    }
}

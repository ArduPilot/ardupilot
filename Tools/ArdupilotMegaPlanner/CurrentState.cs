using System;
using System.Collections.Generic;
using System.Reflection;
using System.Text;
using System.ComponentModel;
using ArdupilotMega.Utilities;
using log4net;
using ArdupilotMega.Attributes;

namespace ArdupilotMega
{
    public class CurrentState : ICloneable
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        // multipliers
        public float multiplierdist = 1;
        internal string DistanceUnit = "";
        public float multiplierspeed = 1;
        internal string SpeedUnit = "";

        // orientation - rads
        [DisplayText("Roll (deg)")]
        public float roll { get; set; }
        [DisplayText("Pitch (deg)")]
        public float pitch { get; set; }
        [DisplayText("Yaw (deg)")]
        public float yaw { get { return _yaw; } set { if (value < 0) { _yaw = value + 360; } else { _yaw = value; } } }
        private float _yaw = 0;

        [DisplayText("GroundCourse (deg)")]
        public float groundcourse { get { return _groundcourse; } set { if (value < 0) { _groundcourse = value + 360; } else { _groundcourse = value; } } }
        private float _groundcourse = 0;

        /// <summary>
        /// time over target in seconds
        /// </summary>
        [DisplayText("Time over Target (sec)")]
        public int tot { get { if (groundspeed <= 0) return 0; return (int)(wp_dist / groundspeed); } }
        [DisplayText("Dist Traveled (dist)")]
        public float distTraveled { get; set; }
        [DisplayText("Time in Air (sec)")]
        public float timeInAir { get; set; }

        // speeds
        [DisplayText("AirSpeed (speed)")]
        public float airspeed { get { return _airspeed * multiplierspeed; } set { _airspeed = value; } }
        [DisplayText("GroundSpeed (speed)")]
        public float groundspeed { get { return _groundspeed * multiplierspeed; } set { _groundspeed = value; } }
        float _airspeed;
        float _groundspeed;
        float _verticalspeed;
        [DisplayText("Vertical Speed (speed)")]
        public float verticalspeed { get { if (float.IsNaN(_verticalspeed)) _verticalspeed = 0; return _verticalspeed * multiplierspeed; } set { _verticalspeed = _verticalspeed * 0.4f + value * 0.6f; } }
        [DisplayText("Wind Direction (Deg)")]
        public float wind_dir { get; set; }
        [DisplayText("Wind Velocity (speed)")]
        public float wind_vel { get; set; }
        /// <summary>
        /// used in wind calc
        /// </summary>
        double Wn_fgo;
        /// <summary>
        /// used for wind calc
        /// </summary>
        double We_fgo;

        //(alt_now - alt_then)/(time_now-time_then)

        // position
        [DisplayText("Latitude (dd)")]
        public float lat { get; set; }
        [DisplayText("Longitude (dd)")]
        public float lng { get; set; }
        [DisplayText("Altitude (dist)")]
        public float alt { get { return (_alt - altoffsethome) * multiplierdist; } set { _alt = value; } }
        DateTime lastalt = DateTime.Now;
        float oldalt = 0;
        [DisplayText("Alt Home Offset (dist)")]
        public float altoffsethome { get; set; }
        private float _alt = 0;
        [DisplayText("Gps Status")]
        public float gpsstatus { get; set; }
        [DisplayText("Gps HDOP")]
        public float gpshdop { get; set; }
        [DisplayText("Sat Count")]
        public float satcount { get; set; }

        public float altd1000 { get { return (alt / 1000) % 10; } }
        public float altd100 { get { return (alt / 100) % 10; } }

        // accel
        [DisplayText("Accel X")]
        public float ax { get; set; }
        [DisplayText("Accel Y")]
        public float ay { get; set; }
        [DisplayText("Accel Z")]
        public float az { get; set; }
        // gyro
        [DisplayText("Gyro X")]
        public float gx { get; set; }
        [DisplayText("Gyro Y")]
        public float gy { get; set; }
        [DisplayText("Gyro Z")]
        public float gz { get; set; }
        // mag
        [DisplayText("Mag X")]
        public float mx { get; set; }
        [DisplayText("Mag Y")]
        public float my { get; set; }
        [DisplayText("Mag Z")]
        public float mz { get; set; }

        [DisplayText("Mag Field")]
        public float magfield { get { return (float)Math.Sqrt(Math.Pow(mx, 2) + Math.Pow(my, 2) + Math.Pow(mz, 2)); } }
        [DisplayText("Accel Strength")]
        public float accelsq { get { return (float)Math.Sqrt(Math.Pow(ax, 2) + Math.Pow(ay, 2) + Math.Pow(az, 2)) / 1000.0f /*980.665f*/; } }

        // calced turn rate
        [DisplayText("Turn Rate (speed)")]
        public float turnrate { get { if (groundspeed <= 1) return 0; return (roll * 9.8f) / groundspeed; } }
        // turn radius
        [DisplayText("Turn Radius (dist)")]
        public float radius { get { if (groundspeed <= 1) return 0; return ((groundspeed * groundspeed)/(float)(9.8f*Math.Tan(roll * deg2rad))); } }

        [DisplayText("RX Rssi")]
        public float rxrssi { get; set; }
        //radio
        public float ch1in { get; set; }
        public float ch2in { get; set; }
        public float ch3in { get; set; }
        public float ch4in { get; set; }
        public float ch5in { get; set; }
        public float ch6in { get; set; }
        public float ch7in { get; set; }
        public float ch8in { get; set; }

        // motors
        public float ch1out { get; set; }
        public float ch2out { get; set; }
        public float ch3out { get; set; }
        public float ch4out { get; set; }
        public float ch5out { get; set; }
        public float ch6out { get; set; }
        public float ch7out { get; set; }
        public float ch8out { get; set; }
        public float ch3percent
        {
            get
            {
                if (_ch3percent != -1)
                    return _ch3percent;
                try
                {
                    if (MainV2.comPort.MAV.param.ContainsKey("RC3_MIN"))
                    {
                        return (int)((ch3out - float.Parse(MainV2.comPort.MAV.param["RC3_MIN"].ToString())) / (float.Parse(MainV2.comPort.MAV.param["RC3_MAX"].ToString()) - float.Parse(MainV2.comPort.MAV.param["RC3_MIN"].ToString())) * 100);
                    }
                    else
                    {
                        return (int)((ch3out - 1100) / (1900 - 1100) * 100);
                    }
                }
                catch
                {
                    return 0;
                }
            }

            set { _ch3percent = value; }
        }

        float _ch3percent = -1;

        //nav state
        [DisplayText("Roll Target (deg)")]
        public float nav_roll { get; set; }
        [DisplayText("Pitch Target (deg)")]
        public float nav_pitch { get; set; }
        [DisplayText("Bearing Target (deg)")]
        public float nav_bearing { get; set; }
        [DisplayText("Bearing Target (deg)")]
        public float target_bearing { get; set; }
        [DisplayText("Dist to WP (dist)")]
        public float wp_dist { get { return (_wpdist * multiplierdist); } set { _wpdist = value; } }
        [DisplayText("Altitude Error (dist)")]
        public float alt_error { get { return _alt_error * multiplierdist; } set { if (_alt_error == value) return; _alt_error = value; _targetalt = _targetalt * 0.5f + (float)Math.Round(alt + alt_error, 0) * 0.5f; } }
        [DisplayText("Bearing Error (deg)")]
        public float ber_error { get { return (target_bearing - yaw); } set { } }
        [DisplayText("Airspeed Error (speed)")]
        public float aspd_error { get { return _aspd_error * multiplierspeed; } set { if (_aspd_error == value) return; _aspd_error = value; _targetairspeed = _targetairspeed * 0.5f + (float)Math.Round(airspeed + aspd_error, 0) * 0.5f; } }
        [DisplayText("Xtrack Error (m)")]
        public float xtrack_error { get; set; }
        [DisplayText("WP No")]
        public float wpno { get; set; }
        [DisplayText("Mode")]
        public string mode { get; set; }
        [DisplayText("ClimbRate (speed)")]
        public float climbrate { get { return _climbrate * multiplierspeed; } set {_climbrate = value;} }
        float _wpdist;
        float _aspd_error;
        float _alt_error;
        float _targetalt;
        float _targetairspeed;
        float _climbrate;

        public float targetaltd100 { get { return (_targetalt / 100) % 10; } }
        public float targetalt { get { return _targetalt; } }

        //airspeed_error = (airspeed_error - airspeed);
        [DisplayText("Airspeed Target (speed)")]
        public float targetairspeed { get { return _targetairspeed; } }


        //message
        internal List<string> messages { get; set; }
        internal string message { get { if (messages.Count == 0) return ""; return messages[messages.Count - 1]; } set { } }

        //battery
        [DisplayText("Bat Voltage (V)")]
        public float battery_voltage { get { return _battery_voltage; } set { _battery_voltage = value / 1000; } }
        private float _battery_voltage;
        [DisplayText("Bat Remaining (%)")]
        public float battery_remaining { get { return _battery_remaining; } set { _battery_remaining = value / 100; if (_battery_remaining < 0 || _battery_remaining > 1) _battery_remaining = 0; } }
        private float _battery_remaining;
        [DisplayText("Bat Current (Amps)")]
        public float current { get { return _current; } set { _current = value / 100; } }
        private float _current;

        public float HomeAlt { get { return (float)HomeLocation.Alt; } set { } }
        internal PointLatLngAlt HomeLocation = new PointLatLngAlt();

        PointLatLngAlt _trackerloc = new PointLatLngAlt();
        internal PointLatLngAlt TrackerLocation { get { if (_trackerloc.Lng != 0) return _trackerloc; return HomeLocation; } set { _trackerloc = value; } }

        [DisplayText("Distance to Mav (dist)")]
        public float DistToMAV
        {
            get
            {
                // shrinking factor for longitude going to poles direction
                double rads = Math.Abs(TrackerLocation.Lat) * 0.0174532925;
                double scaleLongDown = Math.Cos(rads);
                double scaleLongUp = 1.0f / Math.Cos(rads);

                //DST to Home
                double dstlat = Math.Abs(TrackerLocation.Lat - lat) * 111319.5;
                double dstlon = Math.Abs(TrackerLocation.Lng - lng) * 111319.5 * scaleLongDown;
                return (float)Math.Sqrt((dstlat * dstlat) + (dstlon * dstlon)) * multiplierdist;
            }
        }

     [DisplayText("Elevation to Mav (deg)")]
        public float ELToMAV
        {
            get
            {
                float dist = DistToMAV / multiplierdist;

                if (dist < 5)
                    return 0;

                float altdiff = (float)(alt - TrackerLocation.Alt);

                float angle = (float)Math.Atan(altdiff / dist) * rad2deg;

                return angle;
            }
        }

         [DisplayText("Bearing to Mav (deg)")]
        public float AZToMAV
        {
            get
            {
                // shrinking factor for longitude going to poles direction
                double rads = Math.Abs(TrackerLocation.Lat) * 0.0174532925;
                double scaleLongDown = Math.Cos(rads);
                double scaleLongUp = 1.0f / Math.Cos(rads);

                //DIR to Home
                double dstlon = (TrackerLocation.Lng - lng); //OffSet_X
                double dstlat = (TrackerLocation.Lat - lat) * scaleLongUp; //OffSet Y
                double bearing = 90 + (Math.Atan2(dstlat, -dstlon) * 57.295775); //absolut home direction
                if (bearing < 0) bearing += 360;//normalization
                //bearing = bearing - 180;//absolut return direction
                //if (bearing < 0) bearing += 360;//normalization

                float dist = DistToMAV / multiplierdist;

                if (dist < 5)
                    return 0;

                return (float)bearing;
            }
        }


        // pressure
        public float press_abs { get; set; }
        public int press_temp { get; set; }

        // sensor offsets
        public int mag_ofs_x { get; set; }
        public int mag_ofs_y { get; set; }
        public int mag_ofs_z { get; set; }
        public float mag_declination { get; set; }
        public int raw_press { get; set; }
        public int raw_temp { get; set; }
        public float gyro_cal_x { get; set; }
        public float gyro_cal_y { get; set; }
        public float gyro_cal_z { get; set; }
        public float accel_cal_x { get; set; }
        public float accel_cal_y { get; set; }
        public float accel_cal_z { get; set; }

        // HIL
        public int hilch1 { get; set; }
        public int hilch2 { get; set; }
        public int hilch3 { get; set; }
        public int hilch4 { get; set; }
        public int hilch5;
        public int hilch6;
        public int hilch7;
        public int hilch8;

        // rc override
        public ushort rcoverridech1 { get; set; }
        public ushort rcoverridech2 { get; set; }
        public ushort rcoverridech3 { get; set; }
        public ushort rcoverridech4 { get; set; }
        public ushort rcoverridech5 { get; set; }
        public ushort rcoverridech6 { get; set; }
        public ushort rcoverridech7 { get; set; }
        public ushort rcoverridech8 { get; set; }


        // current firmware
        public MainV2.Firmwares firmware = MainV2.Firmwares.ArduPlane;
        public float freemem { get; set; }
        public float brklevel { get; set; }
        public bool armed { get; set; }

        // 3dr radio
        public float rssi { get; set; }
        public float remrssi { get; set; }
        public byte txbuffer { get; set; }
        public float noise { get; set; }
        public float remnoise { get; set; }
        public ushort rxerrors { get; set; }
        public ushort fixedp { get; set; }
        private float _localsnrdb = 0;
        private float _remotesnrdb = 0;
        private DateTime lastrssi = DateTime.Now;
        private DateTime lastremrssi = DateTime.Now;
        public float localsnrdb { get { if (lastrssi.AddSeconds(1) > DateTime.Now) { return _localsnrdb; } lastrssi = DateTime.Now; _localsnrdb = ((rssi - noise) / 1.9f) * 0.5f + _localsnrdb * 0.5f; return _localsnrdb; } }
        public float remotesnrdb { get { if (lastremrssi.AddSeconds(1) > DateTime.Now) { return _remotesnrdb; } lastremrssi = DateTime.Now; _remotesnrdb = ((remrssi - remnoise) / 1.9f) * 0.5f + _remotesnrdb * 0.5f; return _remotesnrdb; } }
        public float DistRSSIRemain {
            get
            {
                float work = 0;
                if (localsnrdb == 0)
                {
                    return 0;
                }
                if (localsnrdb > remotesnrdb)
                {
                    // remote
                    // minus fade margin
                    work = remotesnrdb - 5;
                }
                else
                {
                    // local
                    // minus fade margin
                    work = localsnrdb - 5;
                }

                {
                    float dist = DistToMAV / multiplierdist;

                    work = dist * (float)Math.Pow(2.0, work / 6.0);
                }

                return work;
            }
        }

        // stats
        public ushort packetdropremote { get; set; }
        public ushort linkqualitygcs { get; set; }
        public ushort hwvoltage { get; set; }
        public ushort i2cerrors { get; set; }

        // requested stream rates
        public byte rateattitude { get; set; }
        public byte rateposition { get; set; }
        public byte ratestatus { get; set; }
        public byte ratesensors { get; set; }
        public byte raterc { get; set; }

        // reference
        public DateTime datetime { get; set; }

        private object locker = new object();
        bool useLocation = false;
        bool gotwind = false;
        internal bool batterymonitoring = false;

        public CurrentState()
        {
            mode = "";
            messages = new List<string>();
            rateattitude = 10;
            rateposition = 3;
            ratestatus = 2;
            ratesensors = 2;
            raterc = 2;
            datetime = DateTime.MinValue;
        }

        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        private DateTime lastupdate = DateTime.Now;

        private DateTime lastsecondcounter = DateTime.Now;
        private PointLatLngAlt lastpos = new PointLatLngAlt();

        public string GetNameandUnit(string name)
        {
            string desc = name;
            try
            {
                desc = ((Attributes.DisplayTextAttribute)typeof(CurrentState).GetProperty(name).GetCustomAttributes(false)[0]).Text;
            }
            catch { }

            if (desc.Contains("(dist)"))
            {
                desc = desc.Replace("(dist)", "(" + MainV2.comPort.MAV.cs.DistanceUnit + ")");
            }
            else if (desc.Contains("(speed)"))
            {
                desc = desc.Replace("(speed)", "(" + MainV2.comPort.MAV.cs.SpeedUnit + ")");
            }

            return desc;
        }

        public void UpdateCurrentSettings(System.Windows.Forms.BindingSource bs)
        {
            UpdateCurrentSettings(bs, false, MainV2.comPort);
        }
        /*
        public void UpdateCurrentSettings(System.Windows.Forms.BindingSource bs, bool updatenow)
        {
            UpdateCurrentSettings(bs, false, MainV2.comPort);
        }
        */
        public void UpdateCurrentSettings(System.Windows.Forms.BindingSource bs, bool updatenow, MAVLink mavinterface)
        {
            lock (locker)
            {
                if (DateTime.Now > lastupdate.AddMilliseconds(19) || updatenow) // 50 hz
                {
                    lastupdate = DateTime.Now;

                    if (DateTime.Now.Second != lastsecondcounter.Second)
                    {
                        lastsecondcounter = DateTime.Now;

                        if (lastpos.Lat != 0 && lastpos.Lng != 0)
                        {
                            if (!MainV2.comPort.BaseStream.IsOpen && !MainV2.comPort.logreadmode)
                                distTraveled = 0;

                            distTraveled += (float)lastpos.GetDistance(new PointLatLngAlt(lat, lng, 0, "")) * multiplierdist;
                            lastpos = new PointLatLngAlt(lat, lng, 0, "");
                        }
                        else
                        {
                            lastpos = new PointLatLngAlt(lat, lng, 0, "");
                        }

                        // cant use gs, cant use alt, 
                        if (ch3percent > 12) 
                            timeInAir++;

                        if (!gotwind)
                            dowindcalc();
                    }

                    if (mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT] != null) // status text 
                    {

                        string logdata = DateTime.Now + " " + Encoding.ASCII.GetString(mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT], 6, mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT].Length - 6);

                        int ind = logdata.IndexOf('\0');
                        if (ind != -1)
                            logdata = logdata.Substring(0, ind);

                        try
                        {
                            while (messages.Count > 5)
                            {
                                messages.RemoveAt(0);
                            }
                            messages.Add(logdata + "\n");

                        }
                        catch { }
                        mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT] = null;
                    }

                    byte[] bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_SCALED];

                    if (bytearray != null) // hil mavlink 0.9
                    {
                        var hil = bytearray.ByteArrayToStructure<MAVLink.mavlink_rc_channels_scaled_t>(6);

                        hilch1 = hil.chan1_scaled;
                        hilch2 = hil.chan2_scaled;
                        hilch3 = hil.chan3_scaled;
                        hilch4 = hil.chan4_scaled;
                        hilch5 = hil.chan5_scaled;
                        hilch6 = hil.chan6_scaled;
                        hilch7 = hil.chan7_scaled;
                        hilch8 = hil.chan8_scaled;

                        //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_SCALED] = null;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_HIL_CONTROLS];

                    if (bytearray != null) // hil mavlink 0.9 and 1.0
                    {
                        var hil = bytearray.ByteArrayToStructure<MAVLink.mavlink_hil_controls_t>(6);

                        hilch1 = (int)(hil.roll_ailerons * 10000);
                        hilch2 = (int)(hil.pitch_elevator * 10000);
                        hilch3 = (int)(hil.throttle * 10000);
                        hilch4 = (int)(hil.yaw_rudder * 10000);

                        //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_HIL_CONTROLS] = null;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_HWSTATUS];

                    if (bytearray != null)
                    {
                        var hwstatus = bytearray.ByteArrayToStructure<MAVLink.mavlink_hwstatus_t>(6);

                        hwvoltage = hwstatus.Vcc;
                        i2cerrors = hwstatus.I2Cerr;

                        //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_HWSTATUS] = null;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_WIND];
                    if (bytearray != null)
                    {
                        var wind = bytearray.ByteArrayToStructure<MAVLink.mavlink_wind_t>(6);

                        gotwind = true;

                        wind_dir = (wind.direction + 360) % 360;
                        wind_vel = wind.speed * multiplierspeed;

                        //MAVLink.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS] = null;
                    }

#if MAVLINK10


                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_HEARTBEAT];
                    if (bytearray != null)
                    {
                        var hb = bytearray.ByteArrayToStructure<MAVLink.mavlink_heartbeat_t>(6);

                        armed = (hb.base_mode & (byte)MAVLink.MAV_MODE_FLAG.SAFETY_ARMED) == (byte)MAVLink.MAV_MODE_FLAG.SAFETY_ARMED;

                        string oldmode = mode;

                        mode = "Unknown";

                        if ((hb.base_mode & (byte)MAVLink.MAV_MODE_FLAG.CUSTOM_MODE_ENABLED) != 0)
                        {
                            if (hb.type == (byte)MAVLink.MAV_TYPE.FIXED_WING)
                            {
                                switch (hb.custom_mode)
                                {
                                    case (byte)(Common.apmmodes.MANUAL):
                                        mode = "Manual";
                                        break;
                                    case (byte)(Common.apmmodes.GUIDED):
                                        mode = "Guided";
                                        break;
                                    case (byte)(Common.apmmodes.STABILIZE):
                                        mode = "Stabilize";
                                        break;
                                    case (byte)(Common.apmmodes.FLY_BY_WIRE_A):
                                        mode = "FBW A";
                                        break;
                                    case (byte)(Common.apmmodes.FLY_BY_WIRE_B):
                                        mode = "FBW B";
                                        break;
                                    case (byte)(Common.apmmodes.AUTO):
                                        mode = "Auto";
                                        break;
                                    case (byte)(Common.apmmodes.RTL):
                                        mode = "RTL";
                                        break;
                                    case (byte)(Common.apmmodes.LOITER):
                                        mode = "Loiter";
                                        break;
                                    case (byte)(Common.apmmodes.CIRCLE):
                                        mode = "Circle";
                                        break;
                                    case 16:
                                        mode = "Initialising";
                                        break;
                                    default:
                                        mode = "Unknown";
                                        break;
                                }
                            }
                            else if (hb.type == (byte)MAVLink.MAV_TYPE.QUADROTOR)
                            {
                                switch (hb.custom_mode)
                                {
                                    case (byte)(Common.ac2modes.STABILIZE):
                                        mode = "Stabilize";
                                        break;
                                    case (byte)(Common.ac2modes.ACRO):
                                        mode = "Acro";
                                        break;
                                    case (byte)(Common.ac2modes.ALT_HOLD):
                                        mode = "Alt Hold";
                                        break;
                                    case (byte)(Common.ac2modes.AUTO):
                                        mode = "Auto";
                                        break;
                                    case (byte)(Common.ac2modes.GUIDED):
                                        mode = "Guided";
                                        break;
                                    case (byte)(Common.ac2modes.LOITER):
                                        mode = "Loiter";
                                        break;
                                    case (byte)(Common.ac2modes.RTL):
                                        mode = "RTL";
                                        break;
                                    case (byte)(Common.ac2modes.CIRCLE):
                                        mode = "Circle";
                                        break;
                                    case (byte)(Common.ac2modes.LAND):
                                        mode = "Land";
                                        break;
                                    default:
                                        mode = "Unknown";
                                        break;
                                }
                            }
                        }

                        if (oldmode != mode && MainV2.speechEnable && MainV2.getConfig("speechmodeenabled") == "True")
                        {
                            MainV2.speechEngine.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechmode")));
                        }
                    }


                    bytearray = mavinterface.MAV.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS];
                    if (bytearray != null)
                    {
                        var sysstatus = bytearray.ByteArrayToStructure<MAVLink.mavlink_sys_status_t>(6);

                        battery_voltage = sysstatus.voltage_battery;
                        battery_remaining = sysstatus.battery_remaining;
                        current = sysstatus.current_battery;

                        packetdropremote = sysstatus.drop_rate_comm;

                        //MAVLink.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS] = null;
                    }
#else

                bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_SYS_STATUS];

                if (bytearray != null)
                {
                    var sysstatus = bytearray.ByteArrayToStructure<MAVLink.mavlink_sys_status_t>(6);

                    armed = sysstatus.status;

                    string oldmode = mode;

                    switch (sysstatus.mode)
                    {
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_UNINIT:
                            switch (sysstatus.nav_mode)
                            {
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_GROUNDED:
                                    mode = "Initialising";
                                    break;
                            }
                            break;
                        case (byte)(100 + Common.ac2modes.STABILIZE):
                            mode = EnumTranslator.GetDisplayText(Common.ac2modes.STABILIZE);
                            break;
                        case (byte)(100 + Common.ac2modes.ACRO):
                            mode = EnumTranslator.GetDisplayText(Common.ac2modes.ACRO);
                            break;
                        case (byte)(100 + Common.ac2modes.ALT_HOLD):
                            mode = EnumTranslator.GetDisplayText(Common.ac2modes.ALT_HOLD);
                            break;
                        case (byte)(100 + Common.ac2modes.AUTO):
                            mode = EnumTranslator.GetDisplayText(Common.ac2modes.AUTO);
                            break;
                        case (byte)(100 + Common.ac2modes.GUIDED):
                            mode = EnumTranslator.GetDisplayText(Common.ac2modes.GUIDED);
                            break;
                        case (byte)(100 + Common.ac2modes.LOITER):
                            mode = EnumTranslator.GetDisplayText(Common.ac2modes.LOITER);
                            break;
                        case (byte)(100 + Common.ac2modes.RTL):
                            mode = EnumTranslator.GetDisplayText(Common.ac2modes.RTL);
                            break;
                        case (byte)(100 + Common.ac2modes.CIRCLE):
                            mode = EnumTranslator.GetDisplayText(Common.ac2modes.CIRCLE);
                            break;
                        case (byte)(100 + Common.ac2modes.LAND):
                            mode = EnumTranslator.GetDisplayText(Common.ac2modes.LAND);
                            break;
                        case (byte)(100 + Common.ac2modes.POSITION):
                            mode = EnumTranslator.GetDisplayText(Common.ac2modes.POSITION);
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_MANUAL:
                            mode = "Manual";
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_GUIDED:
                            mode = "Guided";
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_TEST1:
                            mode = "Stabilize";
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_TEST2:
                            mode = "FBW A"; // fall though  old
                            switch (sysstatus.nav_mode)
                            {
                                case (byte)1:
                                    mode = "FBW A";
                                    break;
                                case (byte)2:
                                    mode = "FBW B";
                                    break;
                                case (byte)3:
                                    mode = "FBW C";
                                    break;
                            }
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_TEST3:
                            mode = "Circle";
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_AUTO:
                            switch (sysstatus.nav_mode)
                            {
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_WAYPOINT:
                                    mode = "Auto";
                                    break;
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_RETURNING:
                                    mode = "RTL";
                                    break;
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_HOLD:
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_LOITER:
                                    mode = "Loiter";
                                    break;
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_LIFTOFF:
                                    mode = "Takeoff";
                                    break;
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_LANDING:
                                    mode = "Land";
                                    break;
                            }

                            break;
                        default:
                            mode = "Unknown";
                            break;
                    }

                    battery_voltage = sysstatus.vbat;
                    battery_remaining = sysstatus.battery_remaining;

                    packetdropremote = sysstatus.packet_drop;

                    if (oldmode != mode && MainV2.speechEnable && MainV2.speechEngine != null && MainV2.getConfig("speechmodeenabled") == "True")
                    {
                        MainV2.speechEngine.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechmode")));
                    }

                    //MAVLink.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS] = null;
                }
#endif

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_SCALED_PRESSURE];
                    if (bytearray != null)
                    {
                        var pres = bytearray.ByteArrayToStructure<MAVLink.mavlink_scaled_pressure_t>(6);
                        press_abs = pres.press_abs;
                        press_temp = pres.temperature;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_SENSOR_OFFSETS];
                    if (bytearray != null)
                    {
                        var sensofs = bytearray.ByteArrayToStructure<MAVLink.mavlink_sensor_offsets_t>(6);

                        mag_ofs_x = sensofs.mag_ofs_x;
                        mag_ofs_y = sensofs.mag_ofs_y;
                        mag_ofs_z = sensofs.mag_ofs_z;
                        mag_declination = sensofs.mag_declination;

                        raw_press = sensofs.raw_press;
                        raw_temp = sensofs.raw_temp;

                        gyro_cal_x = sensofs.gyro_cal_x;
                        gyro_cal_y = sensofs.gyro_cal_y;
                        gyro_cal_z = sensofs.gyro_cal_z;

                        accel_cal_x = sensofs.accel_cal_x;
                        accel_cal_y = sensofs.accel_cal_y;
                        accel_cal_z = sensofs.accel_cal_z;

                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_ATTITUDE];

                    if (bytearray != null)
                    {
                        var att = bytearray.ByteArrayToStructure<MAVLink.mavlink_attitude_t>(6);

                        roll = att.roll * rad2deg;
                        pitch = att.pitch * rad2deg;
                        yaw = att.yaw * rad2deg;

                        //                    Console.WriteLine(roll + " " + pitch + " " + yaw);

                        //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_ATTITUDE] = null;
                    }
                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW_INT];
                    if (bytearray != null)
                    {
                        var gps = bytearray.ByteArrayToStructure<MAVLink.mavlink_gps_raw_int_t>(6);

                        if (!useLocation)
                        {
                            lat = gps.lat * 1.0e-7f;
                            lng = gps.lon * 1.0e-7f;
                        }
                        //                alt = gps.alt; // using vfr as includes baro calc

                        gpsstatus = gps.fix_type;
                        //                    Console.WriteLine("gpsfix {0}",gpsstatus);

                        gpshdop = (float)Math.Round((double)gps.eph / 100.0,2);

                        satcount = gps.satellites_visible;

                        groundspeed = gps.vel * 1.0e-2f;
                        groundcourse = gps.cog * 1.0e-2f;

                        //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW] = null;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_GPS_STATUS];
                    if (bytearray != null)
                    {
                        var gps = bytearray.ByteArrayToStructure<MAVLink.mavlink_gps_status_t>(6);
                        satcount = gps.satellites_visible;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_RADIO];
                    if (bytearray != null)
                    {
                        var radio = bytearray.ByteArrayToStructure<MAVLink.mavlink_radio_t>(6);
                        rssi = radio.rssi;
                        remrssi = radio.remrssi;
                        txbuffer = radio.txbuf;
                        rxerrors = radio.rxerrors;
                        noise = radio.noise;
                        remnoise = radio.remnoise;
                        fixedp = radio.fixedp;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT];
                    if (bytearray != null)
                    {
                        var loc = bytearray.ByteArrayToStructure<MAVLink.mavlink_global_position_int_t>(6);

                        // the new arhs deadreckoning may send 0 alt and 0 long. check for and undo

                        useLocation = true;
                        if (loc.lat == 0 && loc.lon == 0)
                        {
                            useLocation = false;
                        }
                        else
                        {
                            //alt = loc.alt / 1000.0f;
                            lat = loc.lat / 10000000.0f;
                            lng = loc.lon / 10000000.0f;
                        }
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_MISSION_CURRENT];
                    if (bytearray != null)
                    {
                        var wpcur = bytearray.ByteArrayToStructure<MAVLink.mavlink_mission_current_t>(6);

                        int oldwp = (int)wpno;

                        wpno = wpcur.seq;

                        if (oldwp != wpno && MainV2.speechEnable && MainV2.getConfig("speechwaypointenabled") == "True")
                        {
                            MainV2.speechEngine.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechwaypoint")));
                        }

                        //MAVLink.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_WAYPOINT_CURRENT] = null;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT];

                    if (bytearray != null)
                    {
                        var nav = bytearray.ByteArrayToStructure<MAVLink.mavlink_nav_controller_output_t>(6);

                        nav_roll = nav.nav_roll;
                        nav_pitch = nav.nav_pitch;
                        nav_bearing = nav.nav_bearing;
                        target_bearing = nav.target_bearing;
                        wp_dist = nav.wp_dist;
                        alt_error = nav.alt_error;
                        aspd_error = nav.aspd_error / 100.0f;
                        xtrack_error = nav.xtrack_error;

                        //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT] = null;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_RAW];
                    if (bytearray != null)
                    {
                        var rcin = bytearray.ByteArrayToStructure<MAVLink.mavlink_rc_channels_raw_t>(6);

                        ch1in = rcin.chan1_raw;
                        ch2in = rcin.chan2_raw;
                        ch3in = rcin.chan3_raw;
                        ch4in = rcin.chan4_raw;
                        ch5in = rcin.chan5_raw;
                        ch6in = rcin.chan6_raw;
                        ch7in = rcin.chan7_raw;
                        ch8in = rcin.chan8_raw;

                        //percent
                        rxrssi = (float)((rcin.rssi / 255.0) * 100.0);

                        //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_RAW] = null;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW];
                    if (bytearray != null)
                    {
                        var servoout = bytearray.ByteArrayToStructure<MAVLink.mavlink_servo_output_raw_t>(6);

                        ch1out = servoout.servo1_raw;
                        ch2out = servoout.servo2_raw;
                        ch3out = servoout.servo3_raw;
                        ch4out = servoout.servo4_raw;
                        ch5out = servoout.servo5_raw;
                        ch6out = servoout.servo6_raw;
                        ch7out = servoout.servo7_raw;
                        ch8out = servoout.servo8_raw;

                        //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW] = null;
                    }


                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_RAW_IMU];
                    if (bytearray != null)
                    {
                        var imu = bytearray.ByteArrayToStructure<MAVLink.mavlink_raw_imu_t>(6);

                        gx = imu.xgyro;
                        gy = imu.ygyro;
                        gz = imu.zgyro;

                        ax = imu.xacc;
                        ay = imu.yacc;
                        az = imu.zacc;

                        mx = imu.xmag;
                        my = imu.ymag;
                        mz = imu.zmag;

                        //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_RAW_IMU] = null;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_SCALED_IMU];
                    if (bytearray != null)
                    {
                        var imu = bytearray.ByteArrayToStructure<MAVLink.mavlink_scaled_imu_t>(6);

                        gx = imu.xgyro;
                        gy = imu.ygyro;
                        gz = imu.zgyro;

                        ax = imu.xacc;
                        ay = imu.yacc;
                        az = imu.zacc;

                        //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_RAW_IMU] = null;
                    }


                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_VFR_HUD];
                    if (bytearray != null)
                    {
                        var vfr = bytearray.ByteArrayToStructure<MAVLink.mavlink_vfr_hud_t>(6);

                        groundspeed = vfr.groundspeed;
                        airspeed = vfr.airspeed;

                        alt = vfr.alt; // this might include baro

                        ch3percent = vfr.throttle;

                        //Console.WriteLine(alt);

                        //climbrate = vfr.climb;

                        if ((DateTime.Now - lastalt).TotalSeconds >= 0.2 && oldalt != alt)
                        {
                            climbrate = (alt - oldalt) / (float)(DateTime.Now - lastalt).TotalSeconds;
                            verticalspeed = (alt - oldalt) / (float)(DateTime.Now - lastalt).TotalSeconds;
                            if (float.IsInfinity(_verticalspeed))
                                _verticalspeed = 0;
                            lastalt = DateTime.Now;
                            oldalt = alt;
                        }

                        //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_VFR_HUD] = null;
                    }

                    bytearray = mavinterface.MAV.packets[MAVLink.MAVLINK_MSG_ID_MEMINFO];
                    if (bytearray != null)
                    {
                        var mem = bytearray.ByteArrayToStructure<MAVLink.mavlink_meminfo_t>(6);
                        freemem = mem.freemem;
                        brklevel = mem.brkval;
                    }
                }

                //Console.WriteLine(DateTime.Now.Millisecond + " start ");
                // update form
                try
                {
                    if (bs != null)
                    {
                        //System.Diagnostics.Debug.WriteLine(DateTime.Now.Millisecond);
                        //Console.WriteLine(DateTime.Now.Millisecond);
                        bs.DataSource = this;
                        // Console.WriteLine(DateTime.Now.Millisecond + " 1 " + updatenow + " " + System.Threading.Thread.CurrentThread.Name);
                        bs.ResetBindings(false);
                        //Console.WriteLine(DateTime.Now.Millisecond + " done ");
                    }
                }
                catch { log.InfoFormat("CurrentState Binding error"); }
            }
        }

        public object Clone()
        {
            return this.MemberwiseClone();
        }

        public void dowindcalc()
        {
            //Wind Fixed gain Observer
            //Ryan Beall 
            //8FEB10

            double Kw = 0.010; // 0.01 // 0.10

            if (airspeed < 1 || groundspeed < 1)
                return;

            double Wn_error = airspeed * Math.Cos((yaw) * deg2rad) * Math.Cos(pitch * deg2rad) - groundspeed * Math.Cos((groundcourse) * deg2rad) - Wn_fgo;
            double We_error = airspeed * Math.Sin((yaw) * deg2rad) * Math.Cos(pitch * deg2rad) - groundspeed * Math.Sin((groundcourse) * deg2rad) - We_fgo;

            Wn_fgo = Wn_fgo + Kw * Wn_error;
            We_fgo = We_fgo + Kw * We_error;

            double wind_dir = (Math.Atan2(We_fgo, Wn_fgo) * (180 / Math.PI));
            double wind_vel = (Math.Sqrt(Math.Pow(We_fgo, 2) + Math.Pow(Wn_fgo, 2)));

            wind_dir = (wind_dir + 360) % 360;

            this.wind_dir = (float)wind_dir;// (float)(wind_dir * 0.5 + this.wind_dir * 0.5);
            this.wind_vel = (float)wind_vel;// (float)(wind_vel * 0.5 + this.wind_vel * 0.5);

            //Console.WriteLine("Wn_error = {0}\nWe_error = {1}\nWn_fgo =    {2}\nWe_fgo =  {3}\nWind_dir =    {4}\nWind_vel =    {5}\n",Wn_error,We_error,Wn_fgo,We_fgo,wind_dir,wind_vel);

            //Console.WriteLine("wind_dir: {0} wind_vel: {1}    as {4} yaw {5} pitch {6} gs {7} cog {8}", wind_dir, wind_vel, Wn_fgo, We_fgo , airspeed,yaw,pitch,groundspeed,groundcourse);

            //low pass the outputs for better results!
        }
    }
}
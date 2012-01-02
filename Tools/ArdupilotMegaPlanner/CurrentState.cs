using System;
using System.Collections.Generic;
using System.Text;
using System.ComponentModel;

namespace ArdupilotMega
{
    public class CurrentState: ICloneable
    {
        // multipliers
        public float multiplierdist = 1;
        public float multiplierspeed = 1;

        // orientation - rads
        public float roll { get; set; }
        public float pitch { get; set; }
        public float yaw { get { return _yaw; } set { if (value < 0) { _yaw = value + 360; } else { _yaw = value; } } }
        private float _yaw = 0;

        public float groundcourse { get { return _groundcourse; } set { if (value < 0) { _groundcourse = value + 360; } else { _groundcourse = value; } } }
        private float _groundcourse = 0;

        /// <summary>
        /// time over target in seconds
        /// </summary>
        public int tot { get { if (groundspeed <= 0) return 0; return (int)(wp_dist / groundspeed); } }

        // speeds
        public float airspeed { get { return _airspeed * multiplierspeed; } set { _airspeed = value; } }
        public float groundspeed { get { return _groundspeed * multiplierspeed; } set { _groundspeed = value; } }
        float _airspeed;
        float _groundspeed;
        float _verticalspeed;
        public float verticalspeed { get { if (float.IsNaN(_verticalspeed)) _verticalspeed = 0; return _verticalspeed; } set { _verticalspeed = _verticalspeed * 0.8f + value * 0.2f; } }
        public float wind_dir { get; set; }
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
        public float lat { get; set; }
        public float lng { get; set; }
        public float alt { get { return (_alt - altoffsethome) * multiplierdist; } set { _alt = value; } }
        DateTime lastalt = DateTime.Now;
        float oldalt = 0;
        public float altoffsethome { get; set; }
        private float _alt = 0;
        public float gpsstatus { get; set; }
        public float gpshdop { get; set; }
        public float satcount { get; set; }

        public float altd1000 { get { return (alt / 1000) % 10; } }
        public float altd100 { get { return (alt / 100) % 10; } }

        // accel
        public  float ax { get; set; }
        public  float ay { get; set; }
        public  float az { get; set; }
        // gyro
        public  float gx { get; set; }
        public  float gy { get; set; }
        public  float gz { get; set; }
        // mag
        public float mx { get; set; }
        public float my { get; set; }
        public float mz { get; set; }

        // calced turn rate
        public float turnrate { get { if (groundspeed <= 1) return 0; return (roll * 9.8f) / groundspeed; } }

        //radio
        public  float ch1in { get; set; }
        public  float ch2in { get; set; }
        public  float ch3in { get; set; }
        public  float ch4in { get; set; }
        public  float ch5in { get; set; }
        public  float ch6in { get; set; }
        public  float ch7in { get; set; }
        public  float ch8in { get; set; }

        // motors
        public float ch1out { get; set; }
        public float ch2out { get; set; }
        public float ch3out { get; set; }
        public float ch4out { get; set; }
        public float ch5out { get; set; }
        public float ch6out { get; set; }
        public float ch7out { get; set; }
        public float ch8out { get; set; }
        public float ch3percent { 
            get {
                try
                {
                    return (int)((ch3out - float.Parse(MainV2.comPort.param["RC3_MIN"].ToString())) / (float.Parse(MainV2.comPort.param["RC3_MAX"].ToString()) - float.Parse(MainV2.comPort.param["RC3_MIN"].ToString())) * 100);
                }
                catch {
                    return 0;
                }
            } 
        }

        //nav state
        public float nav_roll { get; set; }
        public float nav_pitch { get; set; }
        public float nav_bearing { get; set; }
        public float target_bearing { get; set; }
        public float wp_dist { get { return (_wpdist * multiplierdist); } set { _wpdist = value; } }
        public float alt_error { get { return _alt_error * multiplierdist; } set { _alt_error = value; } }
        public float ber_error { get { return (target_bearing - yaw); } set {  } }
        public float aspd_error { get { return _aspd_error * multiplierspeed; } set { _aspd_error = value; } }
        public float xtrack_error { get; set; }
        public float wpno { get; set; }
        public string mode { get; set; }
        public float climbrate { get; set; }
        float _wpdist;
        float _aspd_error;
        float _alt_error;

        public float targetaltd100 { get { return ((alt + alt_error) / 100) % 10; } }
        public float targetalt { get { return (float)Math.Round(alt + alt_error,0); } }

        //airspeed_error = (airspeed_error - airspeed);
        public float targetairspeed { get { return (float)Math.Round(airspeed + aspd_error / 100,0); } }


        //message
        public List<string> messages { get; set; }
        public string message { get { if (messages.Count == 0) return ""; return messages[messages.Count - 1]; } set { } }

        //battery
        public float battery_voltage { get { return _battery_voltage; } set { _battery_voltage = value / 1000; } }
        private float _battery_voltage;
        public float battery_remaining { get { return _battery_remaining; } set { _battery_remaining = value / 1000; if (_battery_remaining < 0) _battery_remaining = 0; } }
        private float _battery_remaining;

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

        // stats
        public ushort packetdropremote { get; set; }
        public ushort linkqualitygcs { get; set; }

        // requested stream rates
        public byte rateattitude { get; set; }
        public byte rateposition { get; set; }
        public byte ratestatus { get; set; }
        public byte ratesensors { get; set; }
        public byte raterc { get; set; }

        // reference
        public DateTime datetime { get; set; }

        public CurrentState()
        {
            mode = "";
            messages = new List<string>();
            rateattitude = 10;
            rateposition = 3;
            ratestatus = 1;
            ratesensors = 3;
            raterc = 3;
        }

        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        private DateTime lastupdate = DateTime.Now;

        private DateTime lastwindcalc = DateTime.Now;
        
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
            if (DateTime.Now > lastupdate.AddMilliseconds(19) || updatenow) // 50 hz
            {
                lastupdate = DateTime.Now;

                if (DateTime.Now.Second != lastwindcalc.Second)
                {
                    lastwindcalc = DateTime.Now;
                    dowindcalc();
                }

                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT] != null) // status text 
                {

                    string logdata = DateTime.Now + " " + Encoding.ASCII.GetString(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT], 6, mavinterface.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT].Length - 6);

                    int ind = logdata.IndexOf('\0');
                    if (ind != -1)
                        logdata = logdata.Substring(0, ind);
                    if (messages.Count > 5)
                    {
                        messages.RemoveAt(0);
                    }
                    messages.Add(logdata + "\n");

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT] = null;
                }

                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_SCALED] != null) // hil
                {
                    var hil = new ArdupilotMega.MAVLink.__mavlink_rc_channels_scaled_t();

                    object temp = hil;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_SCALED], ref temp, 6);

                    hil = (MAVLink.__mavlink_rc_channels_scaled_t)(temp);

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

                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT] != null)
                {
                    MAVLink.__mavlink_nav_controller_output_t nav = new MAVLink.__mavlink_nav_controller_output_t();

                    object temp = nav;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT], ref temp, 6);

                    nav = (MAVLink.__mavlink_nav_controller_output_t)(temp);

                    nav_roll = nav.nav_roll;
                    nav_pitch = nav.nav_pitch;
                    nav_bearing = nav.nav_bearing;
                    target_bearing = nav.target_bearing;
                    wp_dist = nav.wp_dist;
                    alt_error = nav.alt_error;
                    aspd_error = nav.aspd_error;
                    xtrack_error = nav.xtrack_error;

                    //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT] = null;
                }
#if MAVLINK10
                if (mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_HEARTBEAT] != null)
                {
                    ArdupilotMega.MAVLink.__mavlink_heartbeat_t hb = new ArdupilotMega.MAVLink.__mavlink_heartbeat_t();

                    object temp = hb;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_HEARTBEAT], ref temp, 6);

                    hb = (ArdupilotMega.MAVLink.__mavlink_heartbeat_t)(temp);

                    string oldmode = mode;

                    mode = "Unknown";

                    if ((hb.base_mode & (byte)MAVLink.MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) != 0)
                    {
                        if (hb.type == (byte)MAVLink.MAV_TYPE.MAV_TYPE_FIXED_WING)
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
                                default:
                                    mode = "Unknown";
                                    break;
                            }
                        }
                        else if (hb.type == (byte)MAVLink.MAV_TYPE.MAV_TYPE_QUADROTOR) 
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
                                default:
                                    mode = "Unknown";
                                    break;
                            }
                        }
                    }

                    if (oldmode != mode && MainV2.speechenable && MainV2.getConfig("speechmodeenabled") == "True")
                    {
                        MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechmode")));
                    }
                }


                if (mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS] != null)
                {
                    ArdupilotMega.MAVLink.__mavlink_sys_status_t sysstatus = new ArdupilotMega.MAVLink.__mavlink_sys_status_t();

                    object temp = sysstatus;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS], ref temp, 6);

                    sysstatus = (ArdupilotMega.MAVLink.__mavlink_sys_status_t)(temp);

                    battery_voltage = sysstatus.voltage_battery;
                    battery_remaining = sysstatus.battery_remaining;

                    packetdropremote = sysstatus.drop_rate_comm;

                    //MAVLink.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS] = null;
                }
				#else

                if (mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS] != null)
                {
                    ArdupilotMega.MAVLink.__mavlink_sys_status_t sysstatus = new ArdupilotMega.MAVLink.__mavlink_sys_status_t();

                    object temp = sysstatus;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS], ref temp, 6);

                    sysstatus = (ArdupilotMega.MAVLink.__mavlink_sys_status_t)(temp);

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
                            mode = "Stabilize";
                            break;
                        case (byte)(100 + Common.ac2modes.ACRO):
                            mode = "Acro";
                            break;
                        case (byte)(100 + Common.ac2modes.ALT_HOLD):
                            mode = "Alt Hold";
                            break;
                        case (byte)(100 + Common.ac2modes.AUTO):
                            mode = "Auto";
                            break;
                        case (byte)(100 + Common.ac2modes.GUIDED):
                            mode = "Guided";
                            break;
                        case (byte)(100 + Common.ac2modes.LOITER):
                            mode = "Loiter";
                            break;
                        case (byte)(100 + Common.ac2modes.RTL):
                            mode = "RTL";
                            break;
                        case (byte)(100 + Common.ac2modes.CIRCLE):
                            mode = "Circle";
                            break;
                        case (byte)(100 + Common.ac2modes.POSITION):
                            mode = "Position";
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

                    if (oldmode != mode && MainV2.speechenable && MainV2.getConfig("speechmodeenabled") == "True")
                    {
                        MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechmode")));
                    }

                    //MAVLink.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS] = null;
                }				
#endif
                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_ATTITUDE] != null)
                {
                    var att = new ArdupilotMega.MAVLink.__mavlink_attitude_t();

                    object temp = att;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_ATTITUDE], ref temp, 6);

                    att = (MAVLink.__mavlink_attitude_t)(temp);

                    roll = att.roll * rad2deg;
                    pitch = att.pitch * rad2deg;
                    yaw = att.yaw * rad2deg;

                    //                    Console.WriteLine(roll + " " + pitch + " " + yaw);

                    //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_ATTITUDE] = null;
                }
#if MAVLINK10
                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW_INT] != null)
                {
                    var gps = new MAVLink.__mavlink_gps_raw_int_t();

                    object temp = gps;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW_INT], ref temp, 6);

                    gps = (MAVLink.__mavlink_gps_raw_int_t)(temp);

                    lat = gps.lat * 1.0e-7f;
                    lng = gps.lon * 1.0e-7f;
                    //                alt = gps.alt; // using vfr as includes baro calc

                    gpsstatus = gps.fix_type;
                    //                    Console.WriteLine("gpsfix {0}",gpsstatus);

                    gpshdop = gps.eph;

                    groundspeed = gps.vel * 1.0e-2f;
                    groundcourse = gps.cog * 1.0e-2f;

                    //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW] = null;
                }
				#else

                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW] != null)
                {
                    var gps = new MAVLink.__mavlink_gps_raw_t();

                    object temp = gps;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW], ref temp, 6);

                    gps = (MAVLink.__mavlink_gps_raw_t)(temp);

                    lat = gps.lat;
                    lng = gps.lon;
                    //                alt = gps.alt; // using vfr as includes baro calc

                    gpsstatus = gps.fix_type;
                    //                    Console.WriteLine("gpsfix {0}",gpsstatus);

                    gpshdop = gps.eph;

                    groundspeed = gps.v;
                    groundcourse = gps.hdg;

                    //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW] = null;
                }				
#endif
                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GPS_STATUS] != null)
                {
                    var gps = new MAVLink.__mavlink_gps_status_t();

                    object temp = gps;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GPS_STATUS], ref temp, 6);

                    gps = (MAVLink.__mavlink_gps_status_t)(temp);

                    satcount = gps.satellites_visible;
                }
                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT] != null)
                {
                    var loc = new MAVLink.__mavlink_global_position_int_t();

                    object temp = loc;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT], ref temp, 6);

                    loc = (MAVLink.__mavlink_global_position_int_t)(temp);

                    //alt = loc.alt / 1000.0f;
                    lat = loc.lat / 10000000.0f;
                    lng = loc.lon / 10000000.0f;

                }
#if MAVLINK10
                if (mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_MISSION_CURRENT] != null)
                {
                    ArdupilotMega.MAVLink.__mavlink_mission_current_t wpcur = new ArdupilotMega.MAVLink.__mavlink_mission_current_t();

                    object temp = wpcur;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_MISSION_CURRENT], ref temp, 6);

                    wpcur = (ArdupilotMega.MAVLink.__mavlink_mission_current_t)(temp);

                    int oldwp = (int)wpno;

                    wpno = wpcur.seq;

                    if (oldwp != wpno && MainV2.speechenable && MainV2.getConfig("speechwaypointenabled") == "True")
                    {
                        MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechwaypoint")));
                    }

                    //MAVLink.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_WAYPOINT_CURRENT] = null;
                }
				#else
	               if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GLOBAL_POSITION] != null)
                {
                    var loc = new MAVLink.__mavlink_global_position_t();

                    object temp = loc;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GLOBAL_POSITION], ref temp, 6);

                    loc = (MAVLink.__mavlink_global_position_t)(temp);

                    alt = loc.alt;
                    lat = loc.lat;
                    lng = loc.lon;

                }
                if (mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_WAYPOINT_CURRENT] != null)
                {
                    ArdupilotMega.MAVLink.__mavlink_waypoint_current_t wpcur = new ArdupilotMega.MAVLink.__mavlink_waypoint_current_t();

                    object temp = wpcur;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_WAYPOINT_CURRENT], ref temp, 6);

                    wpcur = (ArdupilotMega.MAVLink.__mavlink_waypoint_current_t)(temp);

                    int oldwp = (int)wpno;

                    wpno = wpcur.seq;

                    if (oldwp != wpno && MainV2.speechenable && MainV2.getConfig("speechwaypointenabled") == "True")
                    {
                        MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechwaypoint")));
                    }

                    //MAVLink.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_WAYPOINT_CURRENT] = null;
                }			
				
#endif
                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_RAW] != null)
                {
                    var rcin = new MAVLink.__mavlink_rc_channels_raw_t();

                    object temp = rcin;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_RAW], ref temp, 6);

                    rcin = (MAVLink.__mavlink_rc_channels_raw_t)(temp);

                    ch1in = rcin.chan1_raw;
                    ch2in = rcin.chan2_raw;
                    ch3in = rcin.chan3_raw;
                    ch4in = rcin.chan4_raw;
                    ch5in = rcin.chan5_raw;
                    ch6in = rcin.chan6_raw;
                    ch7in = rcin.chan7_raw;
                    ch8in = rcin.chan8_raw;

                    //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_RAW] = null;
                }

                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW] != null)
                {
                    var servoout = new MAVLink.__mavlink_servo_output_raw_t();

                    object temp = servoout;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW], ref temp, 6);

                    servoout = (MAVLink.__mavlink_servo_output_raw_t)(temp);

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

                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RAW_IMU] != null)
                {
                    var imu = new MAVLink.__mavlink_raw_imu_t();

                    object temp = imu;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RAW_IMU], ref temp, 6);

                    imu = (MAVLink.__mavlink_raw_imu_t)(temp);

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

                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SCALED_IMU] != null)
                {
                    var imu = new MAVLink.__mavlink_scaled_imu_t();

                    object temp = imu;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SCALED_IMU], ref temp, 6);

                    imu = (MAVLink.__mavlink_scaled_imu_t)(temp);

                    gx = imu.xgyro;
                    gy = imu.ygyro;
                    gz = imu.zgyro;

                    ax = imu.xacc;
                    ay = imu.yacc;
                    az = imu.zacc;

                    //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_RAW_IMU] = null;
                }

                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_VFR_HUD] != null)
                {
                    MAVLink.__mavlink_vfr_hud_t vfr = new MAVLink.__mavlink_vfr_hud_t();

                    object temp = vfr;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_VFR_HUD], ref temp, 6);

                    vfr = (MAVLink.__mavlink_vfr_hud_t)(temp);

                    groundspeed = vfr.groundspeed;
                    airspeed = vfr.airspeed;

                    alt = vfr.alt; // this might include baro

                    //climbrate = vfr.climb;

                    if ((DateTime.Now - lastalt).TotalSeconds >= 0.1 && oldalt != alt)
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

                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_MEMINFO] != null) // hil
                {
                    var mem = new ArdupilotMega.MAVLink.__mavlink_meminfo_t();

                    object temp = mem;

                    MAVLink.ByteArrayToStructure(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_MEMINFO], ref temp, 6);

                    mem = (MAVLink.__mavlink_meminfo_t)(temp);

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
                    //Console.WriteLine(DateTime.Now.Millisecond);
                    bs.DataSource = this;
                    //Console.WriteLine(DateTime.Now.Millisecond + " 1 " + updatenow);
                    bs.ResetBindings(false);
                    //Console.WriteLine(DateTime.Now.Millisecond + " done ");
                }
            }
            catch { Console.WriteLine("CurrentState Binding error"); }
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
            double wind_vel = (Math.Sqrt(Math.Pow(We_fgo,2) + Math.Pow(Wn_fgo,2)));

            wind_dir = (wind_dir + 360) % 360;

            this.wind_dir = (float)wind_dir;// (float)(wind_dir * 0.5 + this.wind_dir * 0.5);
            this.wind_vel = (float)wind_vel;// (float)(wind_vel * 0.5 + this.wind_vel * 0.5);

            //Console.WriteLine("Wn_error = {0}\nWe_error = {1}\nWn_fgo =    {2}\nWe_fgo =  {3}\nWind_dir =    {4}\nWind_vel =    {5}\n",Wn_error,We_error,Wn_fgo,We_fgo,wind_dir,wind_vel);

            //Console.WriteLine("wind_dir: {0} wind_vel: {1}    as {4} yaw {5} pitch {6} gs {7} cog {8}", wind_dir, wind_vel, Wn_fgo, We_fgo , airspeed,yaw,pitch,groundspeed,groundcourse);

            //low pass the outputs for better results!
        }
    }
}

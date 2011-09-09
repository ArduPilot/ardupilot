using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace ArdupilotMega
{
    partial class MAVLink
    {
		public const byte MAVLINK_MSG_ID_SENSOR_OFFSETS = 150;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_sensor_offsets_t
		{
			public short mag_ofs_x; ///< magnetometer X offset
			public short mag_ofs_y; ///< magnetometer Y offset
			public short mag_ofs_z; ///< magnetometer Z offset
			public float mag_declination; ///< magnetic declination (radians)
			public int raw_press; ///< raw pressure from barometer
			public int raw_temp; ///< raw temperature from barometer
			public float gyro_cal_x; ///< gyro X calibration
			public float gyro_cal_y; ///< gyro Y calibration
			public float gyro_cal_z; ///< gyro Z calibration
			public float accel_cal_x; ///< accel X calibration
			public float accel_cal_y; ///< accel Y calibration
			public float accel_cal_z; ///< accel Z calibration
		};

		public const byte MAVLINK_MSG_ID_SET_MAG_OFFSETS = 151;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_mag_offsets_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public short mag_ofs_x; ///< magnetometer X offset
			public short mag_ofs_y; ///< magnetometer Y offset
			public short mag_ofs_z; ///< magnetometer Z offset
		};

		public const byte MAVLINK_MSG_ID_MEMINFO = 152;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_meminfo_t
		{
			public ushort brkval; ///< heap top
			public ushort freemem; ///< free memory
		};

		public enum MAV_CMD
		{ 
			WAYPOINT = 16,
			LOITER_UNLIM = 17,
			LOITER_TURNS = 18,
			LOITER_TIME = 19,
			RETURN_TO_LAUNCH = 20,
			LAND = 21,
			TAKEOFF = 22,
			ROI = 80,
			PATHPLANNING = 81,
			LAST = 95,
			CONDITION_DELAY = 112,
			CONDITION_CHANGE_ALT = 113,
			CONDITION_DISTANCE = 114,
			CONDITION_YAW = 115,
			CONDITION_LAST = 159,
			DO_SET_MODE = 176,
			DO_JUMP = 177,
			DO_CHANGE_SPEED = 178,
			DO_SET_HOME = 179,
			DO_SET_PARAMETER = 180,
			DO_SET_RELAY = 181,
			DO_REPEAT_RELAY = 182,
			DO_SET_SERVO = 183,
			DO_REPEAT_SERVO = 184,
			DO_CONTROL_VIDEO = 200,
			DO_SET_ROI = 201,
			DO_LAST = 240,
			PREFLIGHT_CALIBRATION = 241,
			PREFLIGHT_STORAGE = 245,
		};

		public enum MAV_DATA_STREAM
		{ 
			MAV_DATA_STREAM_ALL = 0,
			MAV_DATA_STREAM_RAW_SENSORS = 1,
			MAV_DATA_STREAM_EXTENDED_STATUS = 2,
			MAV_DATA_STREAM_RC_CHANNELS = 3,
			MAV_DATA_STREAM_RAW_CONTROLLER = 4,
			MAV_DATA_STREAM_POSITION = 6,
			MAV_DATA_STREAM_EXTRA1 = 10,
			MAV_DATA_STREAM_EXTRA2 = 11,
			MAV_DATA_STREAM_EXTRA3 = 12,
		};

		public enum MAV_ROI
		{ 
			MAV_ROI_NONE = 0,
			MAV_ROI_WPNEXT = 1,
			MAV_ROI_WPINDEX = 2,
			MAV_ROI_LOCATION = 3,
			MAV_ROI_TARGET = 4,
		};

		public const byte MAVLINK_MSG_ID_HEARTBEAT = 0;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_heartbeat_t
		{
			public byte type; ///< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
			public byte autopilot; ///< Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
			public byte mavlink_version; ///< MAVLink version
		};

		public const byte MAVLINK_MSG_ID_BOOT = 1;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_boot_t
		{
			public uint version; ///< The onboard software version
		};

		public const byte MAVLINK_MSG_ID_SYSTEM_TIME = 2;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_system_time_t
		{
			public ulong time_usec; ///< Timestamp of the master clock in microseconds since UNIX epoch.
		};

		public const byte MAVLINK_MSG_ID_PING = 3;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_ping_t
		{
			public uint seq; ///< PING sequence
			public byte target_system; ///< 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
			public byte target_component; ///< 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
			public ulong time; ///< Unix timestamp in microseconds
		};

		public const byte MAVLINK_MSG_ID_SYSTEM_TIME_UTC = 4;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_system_time_utc_t
		{
			public uint utc_date; ///< GPS UTC date ddmmyy
			public uint utc_time; ///< GPS UTC time hhmmss
		};

		public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL = 5;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_change_operator_control_t
		{
			public byte target_system; ///< System the GCS requests control for
			public byte control_request; ///< 0: request control of this MAV, 1: Release control of this MAV
			public byte version; ///< 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=25)] 
			char passkey; ///< Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
		};

		public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK = 6;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_change_operator_control_ack_t
		{
			public byte gcs_system_id; ///< ID of the GCS this message 
			public byte control_request; ///< 0: request control of this MAV, 1: Release control of this MAV
			public byte ack; ///< 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
		};

		public const byte MAVLINK_MSG_ID_AUTH_KEY = 7;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_auth_key_t
		{
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=32)] 
			char key; ///< key
		};

		public const byte MAVLINK_MSG_ID_ACTION_ACK = 9;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_action_ack_t
		{
			public byte action; ///< The action id
			public byte result; ///< 0: Action DENIED, 1: Action executed
		};

		public const byte MAVLINK_MSG_ID_ACTION = 10;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_action_t
		{
			public byte target; ///< The system executing the action
			public byte target_component; ///< The component executing the action
			public byte action; ///< The action id
		};

		public const byte MAVLINK_MSG_ID_SET_MODE = 11;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_mode_t
		{
			public byte target; ///< The system setting the mode
			public byte mode; ///< The new mode
		};

		public const byte MAVLINK_MSG_ID_SET_NAV_MODE = 12;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_nav_mode_t
		{
			public byte target; ///< The system setting the mode
			public byte nav_mode; ///< The new navigation mode
		};

		public const byte MAVLINK_MSG_ID_PARAM_REQUEST_READ = 20;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_param_request_read_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=15)] 
			public byte[] param_id; ///< Onboard parameter id
			public short param_index; ///< Parameter index. Send -1 to use the param ID field as identifier
		};

		public const byte MAVLINK_MSG_ID_PARAM_REQUEST_LIST = 21;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_param_request_list_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
		};

		public const byte MAVLINK_MSG_ID_PARAM_VALUE = 22;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_param_value_t
		{
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=15)] 
			public byte[] param_id; ///< Onboard parameter id
			public float param_value; ///< Onboard parameter value
			public ushort param_count; ///< Total number of onboard parameters
			public ushort param_index; ///< Index of this onboard parameter
		};

		public const byte MAVLINK_MSG_ID_PARAM_SET = 23;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_param_set_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=15)] 
			public byte[] param_id; ///< Onboard parameter id
			public float param_value; ///< Onboard parameter value
		};

		public const byte MAVLINK_MSG_ID_GPS_RAW_INT = 25;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_raw_int_t
		{
			public ulong usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			public byte fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
			public int lat; ///< Latitude in 1E7 degrees
			public int lon; ///< Longitude in 1E7 degrees
			public int alt; ///< Altitude in 1E3 meters (millimeters)
			public float eph; ///< GPS HDOP
			public float epv; ///< GPS VDOP
			public float v; ///< GPS ground speed (m/s)
			public float hdg; ///< Compass heading in degrees, 0..360 degrees
		};

		public const byte MAVLINK_MSG_ID_SCALED_IMU = 26;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_scaled_imu_t
		{
			public ulong usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			public short xacc; ///< X acceleration (mg)
			public short yacc; ///< Y acceleration (mg)
			public short zacc; ///< Z acceleration (mg)
			public short xgyro; ///< Angular speed around X axis (millirad /sec)
			public short ygyro; ///< Angular speed around Y axis (millirad /sec)
			public short zgyro; ///< Angular speed around Z axis (millirad /sec)
			public short xmag; ///< X Magnetic field (milli tesla)
			public short ymag; ///< Y Magnetic field (milli tesla)
			public short zmag; ///< Z Magnetic field (milli tesla)
		};

		public const byte MAVLINK_MSG_ID_GPS_STATUS = 27;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_status_t
		{
			public byte satellites_visible; ///< Number of satellites visible
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=20)] 
			public byte[] satellite_prn; ///< Global satellite ID
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=20)] 
			public byte[] satellite_used; ///< 0: Satellite not used, 1: used for localization
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=20)] 
			public byte[] satellite_elevation; ///< Elevation (0: right on top of receiver, 90: on the horizon) of satellite
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=20)] 
			public byte[] satellite_azimuth; ///< Direction of satellite, 0: 0 deg, 255: 360 deg.
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=20)] 
			public byte[] satellite_snr; ///< Signal to noise ratio of satellite
		};

		public const byte MAVLINK_MSG_ID_RAW_IMU = 28;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_raw_imu_t
		{
			public ulong usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			public short xacc; ///< X acceleration (raw)
			public short yacc; ///< Y acceleration (raw)
			public short zacc; ///< Z acceleration (raw)
			public short xgyro; ///< Angular speed around X axis (raw)
			public short ygyro; ///< Angular speed around Y axis (raw)
			public short zgyro; ///< Angular speed around Z axis (raw)
			public short xmag; ///< X Magnetic field (raw)
			public short ymag; ///< Y Magnetic field (raw)
			public short zmag; ///< Z Magnetic field (raw)
		};

		public const byte MAVLINK_MSG_ID_RAW_PRESSURE = 29;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_raw_pressure_t
		{
			public ulong usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			public short press_abs; ///< Absolute pressure (raw)
			public short press_diff1; ///< Differential pressure 1 (raw)
			public short press_diff2; ///< Differential pressure 2 (raw)
			public short temperature; ///< Raw Temperature measurement (raw)
		};

		public const byte MAVLINK_MSG_ID_SCALED_PRESSURE = 38;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_scaled_pressure_t
		{
			public ulong usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			public float press_abs; ///< Absolute pressure (hectopascal)
			public float press_diff; ///< Differential pressure 1 (hectopascal)
			public short temperature; ///< Temperature measurement (0.01 degrees celsius)
		};

		public const byte MAVLINK_MSG_ID_ATTITUDE = 30;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_attitude_t
		{
			public ulong usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			public float roll; ///< Roll angle (rad)
			public float pitch; ///< Pitch angle (rad)
			public float yaw; ///< Yaw angle (rad)
			public float rollspeed; ///< Roll angular speed (rad/s)
			public float pitchspeed; ///< Pitch angular speed (rad/s)
			public float yawspeed; ///< Yaw angular speed (rad/s)
		};

		public const byte MAVLINK_MSG_ID_LOCAL_POSITION = 31;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_local_position_t
		{
			public ulong usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			public float x; ///< X Position
			public float y; ///< Y Position
			public float z; ///< Z Position
			public float vx; ///< X Speed
			public float vy; ///< Y Speed
			public float vz; ///< Z Speed
		};

		public const byte MAVLINK_MSG_ID_GLOBAL_POSITION = 33;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_global_position_t
		{
			public ulong usec; ///< Timestamp (microseconds since unix epoch)
			public float lat; ///< Latitude, in degrees
			public float lon; ///< Longitude, in degrees
			public float alt; ///< Absolute altitude, in meters
			public float vx; ///< X Speed (in Latitude direction, positive: going north)
			public float vy; ///< Y Speed (in Longitude direction, positive: going east)
			public float vz; ///< Z Speed (in Altitude direction, positive: going up)
		};

		public const byte MAVLINK_MSG_ID_GPS_RAW = 32;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_raw_t
		{
			public ulong usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			public byte fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
			public float lat; ///< Latitude in degrees
			public float lon; ///< Longitude in degrees
			public float alt; ///< Altitude in meters
			public float eph; ///< GPS HDOP
			public float epv; ///< GPS VDOP
			public float v; ///< GPS ground speed
			public float hdg; ///< Compass heading in degrees, 0..360 degrees
		};

		public const byte MAVLINK_MSG_ID_SYS_STATUS = 34;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_sys_status_t
		{
			public byte mode; ///< System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
			public byte nav_mode; ///< Navigation mode, see MAV_NAV_MODE ENUM
			public byte status; ///< System status flag, see MAV_STATUS ENUM
			public ushort load; ///< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
			public ushort vbat; ///< Battery voltage, in millivolts (1 = 1 millivolt)
			public ushort battery_remaining; ///< Remaining battery energy: (0%: 0, 100%: 1000)
			public ushort packet_drop; ///< Dropped packets (packets that were corrupted on reception on the MAV)
		};

		public const byte MAVLINK_MSG_ID_RC_CHANNELS_RAW = 35;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_rc_channels_raw_t
		{
			public ushort chan1_raw; ///< RC channel 1 value, in microseconds
			public ushort chan2_raw; ///< RC channel 2 value, in microseconds
			public ushort chan3_raw; ///< RC channel 3 value, in microseconds
			public ushort chan4_raw; ///< RC channel 4 value, in microseconds
			public ushort chan5_raw; ///< RC channel 5 value, in microseconds
			public ushort chan6_raw; ///< RC channel 6 value, in microseconds
			public ushort chan7_raw; ///< RC channel 7 value, in microseconds
			public ushort chan8_raw; ///< RC channel 8 value, in microseconds
			public byte rssi; ///< Receive signal strength indicator, 0: 0%, 255: 100%
		};

		public const byte MAVLINK_MSG_ID_RC_CHANNELS_SCALED = 36;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_rc_channels_scaled_t
		{
			public short chan1_scaled; ///< RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
			public short chan2_scaled; ///< RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
			public short chan3_scaled; ///< RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
			public short chan4_scaled; ///< RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
			public short chan5_scaled; ///< RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
			public short chan6_scaled; ///< RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
			public short chan7_scaled; ///< RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
			public short chan8_scaled; ///< RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
			public byte rssi; ///< Receive signal strength indicator, 0: 0%, 255: 100%
		};

		public const byte MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 37;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_servo_output_raw_t
		{
			public ushort servo1_raw; ///< Servo output 1 value, in microseconds
			public ushort servo2_raw; ///< Servo output 2 value, in microseconds
			public ushort servo3_raw; ///< Servo output 3 value, in microseconds
			public ushort servo4_raw; ///< Servo output 4 value, in microseconds
			public ushort servo5_raw; ///< Servo output 5 value, in microseconds
			public ushort servo6_raw; ///< Servo output 6 value, in microseconds
			public ushort servo7_raw; ///< Servo output 7 value, in microseconds
			public ushort servo8_raw; ///< Servo output 8 value, in microseconds
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT = 39;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public ushort seq; ///< Sequence
			public byte frame; ///< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
			public byte command; ///< The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
			public byte current; ///< false:0, true:1
			public byte autocontinue; ///< autocontinue to next wp
			public float param1; ///< PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
			public float param2; ///< PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
			public float param3; ///< PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
			public float param4; ///< PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
			public float x; ///< PARAM5 / local: x position, global: latitude
			public float y; ///< PARAM6 / y position: global: longitude
			public float z; ///< PARAM7 / z position: global: altitude
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST = 40;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_request_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public ushort seq; ///< Sequence
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT = 41;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_set_current_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public ushort seq; ///< Sequence
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_CURRENT = 42;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_current_t
		{
			public ushort seq; ///< Sequence
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST = 43;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_request_list_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_COUNT = 44;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_count_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public ushort count; ///< Number of Waypoints in the Sequence
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL = 45;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_clear_all_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_REACHED = 46;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_reached_t
		{
			public ushort seq; ///< Sequence
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_ACK = 47;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_ack_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public byte type; ///< 0: OK, 1: Error
		};

		public const byte MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN = 48;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_set_global_origin_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public int latitude; ///< global position * 1E7
			public int longitude; ///< global position * 1E7
			public int altitude; ///< global position * 1000
		};

		public const byte MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET = 49;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_local_origin_set_t
		{
			public int latitude; ///< Latitude (WGS84), expressed as * 1E7
			public int longitude; ///< Longitude (WGS84), expressed as * 1E7
			public int altitude; ///< Altitude(WGS84), expressed as * 1000
		};

		public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET = 50;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_local_position_setpoint_set_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public float x; ///< x position
			public float y; ///< y position
			public float z; ///< z position
			public float yaw; ///< Desired yaw angle
		};

		public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT = 51;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_local_position_setpoint_t
		{
			public float x; ///< x position
			public float y; ///< y position
			public float z; ///< z position
			public float yaw; ///< Desired yaw angle
		};

		public const byte MAVLINK_MSG_ID_CONTROL_STATUS = 52;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_control_status_t
		{
			public byte position_fix; ///< Position fix: 0: lost, 2: 2D position fix, 3: 3D position fix 
			public byte vision_fix; ///< Vision position fix: 0: lost, 1: 2D local position hold, 2: 2D global position fix, 3: 3D global position fix 
			public byte gps_fix; ///< GPS position fix: 0: no reception, 1: Minimum 1 satellite, but no position fix, 2: 2D position fix, 3: 3D position fix 
			public byte ahrs_health; ///< Attitude estimation health: 0: poor, 255: excellent
			public byte control_att; ///< 0: Attitude control disabled, 1: enabled
			public byte control_pos_xy; ///< 0: X, Y position control disabled, 1: enabled
			public byte control_pos_z; ///< 0: Z position control disabled, 1: enabled
			public byte control_pos_yaw; ///< 0: Yaw angle control disabled, 1: enabled
		};

		public const byte MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA = 53;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_safety_set_allowed_area_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public byte frame; ///< Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
			public float p1x; ///< x position 1 / Latitude 1
			public float p1y; ///< y position 1 / Longitude 1
			public float p1z; ///< z position 1 / Altitude 1
			public float p2x; ///< x position 2 / Latitude 2
			public float p2y; ///< y position 2 / Longitude 2
			public float p2z; ///< z position 2 / Altitude 2
		};

		public const byte MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA = 54;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_safety_allowed_area_t
		{
			public byte frame; ///< Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
			public float p1x; ///< x position 1 / Latitude 1
			public float p1y; ///< y position 1 / Longitude 1
			public float p1z; ///< z position 1 / Altitude 1
			public float p2x; ///< x position 2 / Latitude 2
			public float p2y; ///< y position 2 / Longitude 2
			public float p2z; ///< z position 2 / Altitude 2
		};

		public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST = 55;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_roll_pitch_yaw_thrust_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public float roll; ///< Desired roll angle in radians
			public float pitch; ///< Desired pitch angle in radians
			public float yaw; ///< Desired yaw angle in radians
			public float thrust; ///< Collective thrust, normalized to 0 .. 1
		};

		public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST = 56;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_roll_pitch_yaw_speed_thrust_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public float roll_speed; ///< Desired roll angular speed in rad/s
			public float pitch_speed; ///< Desired pitch angular speed in rad/s
			public float yaw_speed; ///< Desired yaw angular speed in rad/s
			public float thrust; ///< Collective thrust, normalized to 0 .. 1
		};

		public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT = 57;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_roll_pitch_yaw_thrust_setpoint_t
		{
			public uint time_ms; ///< Timestamp in milliseconds since system boot
			public float roll; ///< Desired roll angle in radians
			public float pitch; ///< Desired pitch angle in radians
			public float yaw; ///< Desired yaw angle in radians
			public float thrust; ///< Collective thrust, normalized to 0 .. 1
		};

		public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT = 58;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_roll_pitch_yaw_speed_thrust_setpoint_t
		{
			public uint time_ms; ///< Timestamp in milliseconds since system boot
			public float roll_speed; ///< Desired roll angular speed in rad/s
			public float pitch_speed; ///< Desired pitch angular speed in rad/s
			public float yaw_speed; ///< Desired yaw angular speed in rad/s
			public float thrust; ///< Collective thrust, normalized to 0 .. 1
		};

		public const byte MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT = 62;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_nav_controller_output_t
		{
			public float nav_roll; ///< Current desired roll in degrees
			public float nav_pitch; ///< Current desired pitch in degrees
			public short nav_bearing; ///< Current desired heading in degrees
			public short target_bearing; ///< Bearing to current waypoint/target in degrees
			public ushort wp_dist; ///< Distance to active waypoint in meters
			public float alt_error; ///< Current altitude error in meters
			public float aspd_error; ///< Current airspeed error in meters/second
			public float xtrack_error; ///< Current crosstrack error on x-y plane in meters
		};

		public const byte MAVLINK_MSG_ID_POSITION_TARGET = 63;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_position_target_t
		{
			public float x; ///< x position
			public float y; ///< y position
			public float z; ///< z position
			public float yaw; ///< yaw orientation in radians, 0 = NORTH
		};

		public const byte MAVLINK_MSG_ID_STATE_CORRECTION = 64;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_state_correction_t
		{
			public float xErr; ///< x position error
			public float yErr; ///< y position error
			public float zErr; ///< z position error
			public float rollErr; ///< roll error (radians)
			public float pitchErr; ///< pitch error (radians)
			public float yawErr; ///< yaw error (radians)
			public float vxErr; ///< x velocity
			public float vyErr; ///< y velocity
			public float vzErr; ///< z velocity
		};

		public const byte MAVLINK_MSG_ID_SET_ALTITUDE = 65;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_altitude_t
		{
			public byte target; ///< The system setting the altitude
			public uint mode; ///< The new altitude in meters
		};

		public const byte MAVLINK_MSG_ID_REQUEST_DATA_STREAM = 66;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_request_data_stream_t
		{
			public byte target_system; ///< The target requested to send the message stream.
			public byte target_component; ///< The target requested to send the message stream.
			public byte req_stream_id; ///< The ID of the requested message type
			public ushort req_message_rate; ///< Update rate in Hertz
			public byte start_stop; ///< 1 to start sending, 0 to stop sending.
		};

		public const byte MAVLINK_MSG_ID_HIL_STATE = 67;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_hil_state_t
		{
			public ulong usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
			public float roll; ///< Roll angle (rad)
			public float pitch; ///< Pitch angle (rad)
			public float yaw; ///< Yaw angle (rad)
			public float rollspeed; ///< Roll angular speed (rad/s)
			public float pitchspeed; ///< Pitch angular speed (rad/s)
			public float yawspeed; ///< Yaw angular speed (rad/s)
			public int lat; ///< Latitude, expressed as * 1E7
			public int lon; ///< Longitude, expressed as * 1E7
			public int alt; ///< Altitude in meters, expressed as * 1000 (millimeters)
			public short vx; ///< Ground X Speed (Latitude), expressed as m/s * 100
			public short vy; ///< Ground Y Speed (Longitude), expressed as m/s * 100
			public short vz; ///< Ground Z Speed (Altitude), expressed as m/s * 100
			public short xacc; ///< X acceleration (mg)
			public short yacc; ///< Y acceleration (mg)
			public short zacc; ///< Z acceleration (mg)
		};

		public const byte MAVLINK_MSG_ID_HIL_CONTROLS = 68;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_hil_controls_t
		{
		};

		public const byte MAVLINK_MSG_ID_MANUAL_CONTROL = 69;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_manual_control_t
		{
			public byte target; ///< The system to be controlled
			public float roll; ///< roll
			public float pitch; ///< pitch
			public float yaw; ///< yaw
			public float thrust; ///< thrust
			public byte roll_manual; ///< roll control enabled auto:0, manual:1
			public byte pitch_manual; ///< pitch auto:0, manual:1
			public byte yaw_manual; ///< yaw auto:0, manual:1
			public byte thrust_manual; ///< thrust auto:0, manual:1
		};

		public const byte MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = 70;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_rc_channels_override_t
		{
			public byte target_system; ///< System ID
			public byte target_component; ///< Component ID
			public ushort chan1_raw; ///< RC channel 1 value, in microseconds
			public ushort chan2_raw; ///< RC channel 2 value, in microseconds
			public ushort chan3_raw; ///< RC channel 3 value, in microseconds
			public ushort chan4_raw; ///< RC channel 4 value, in microseconds
			public ushort chan5_raw; ///< RC channel 5 value, in microseconds
			public ushort chan6_raw; ///< RC channel 6 value, in microseconds
			public ushort chan7_raw; ///< RC channel 7 value, in microseconds
			public ushort chan8_raw; ///< RC channel 8 value, in microseconds
		};

		public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 73;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_global_position_int_t
		{
			public int lat; ///< Latitude, expressed as * 1E7
			public int lon; ///< Longitude, expressed as * 1E7
			public int alt; ///< Altitude in meters, expressed as * 1000 (millimeters)
			public short vx; ///< Ground X Speed (Latitude), expressed as m/s * 100
			public short vy; ///< Ground Y Speed (Longitude), expressed as m/s * 100
			public short vz; ///< Ground Z Speed (Altitude), expressed as m/s * 100
		};

		public const byte MAVLINK_MSG_ID_VFR_HUD = 74;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_vfr_hud_t
		{
			public float airspeed; ///< Current airspeed in m/s
			public float groundspeed; ///< Current ground speed in m/s
			public short heading; ///< Current heading in degrees, in compass units (0..360, 0=north)
			public ushort throttle; ///< Current throttle setting in integer percent, 0 to 100
			public float alt; ///< Current altitude (MSL), in meters
			public float climb; ///< Current climb rate in meters/second
		};

		public const byte MAVLINK_MSG_ID_COMMAND = 75;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_command_t
		{
			public byte target_system; ///< System which should execute the command
			public byte target_component; ///< Component which should execute the command, 0 for all components
			public byte command; ///< Command ID, as defined by MAV_CMD enum.
			public byte confirmation; ///< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
			public float param1; ///< Parameter 1, as defined by MAV_CMD enum.
			public float param2; ///< Parameter 2, as defined by MAV_CMD enum.
			public float param3; ///< Parameter 3, as defined by MAV_CMD enum.
			public float param4; ///< Parameter 4, as defined by MAV_CMD enum.
		};

		public const byte MAVLINK_MSG_ID_COMMAND_ACK = 76;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_command_ack_t
		{
			public float command; ///< Current airspeed in m/s
			public float result; ///< 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
		};

		public const byte MAVLINK_MSG_ID_DEBUG_VECT = 251;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_debug_vect_t
		{
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=10)] 
			char name; ///< Name
			public ulong usec; ///< Timestamp
			public float x; ///< x
			public float y; ///< y
			public float z; ///< z
		};

		public const byte MAVLINK_MSG_ID_NAMED_VALUE_FLOAT = 252;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_named_value_float_t
		{
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=10)] 
			char name; ///< Name of the debug variable
			public float value; ///< Floating point value
		};

		public const byte MAVLINK_MSG_ID_NAMED_VALUE_INT = 253;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_named_value_int_t
		{
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=10)] 
			char name; ///< Name of the debug variable
			public int value; ///< Signed integer value
		};

		public const byte MAVLINK_MSG_ID_STATUSTEXT = 254;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_statustext_t
		{
			public byte severity; ///< Severity of status, 0 = info message, 255 = critical fault
			[MarshalAs(UnmanagedType.ByValArray, SizeConst=50)] 
			public byte[] text; ///< Status text message, without null termination character
		};

		public const byte MAVLINK_MSG_ID_DEBUG = 255;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_debug_t
		{
			public byte ind; ///< index of debug variable
			public float value; ///< DEBUG value
		};

Type[] mavstructs = new Type[] {typeof(__mavlink_heartbeat_t) ,typeof(__mavlink_boot_t) ,typeof(__mavlink_system_time_t) ,typeof(__mavlink_ping_t) ,typeof(__mavlink_system_time_utc_t) ,typeof(__mavlink_change_operator_control_t) ,typeof(__mavlink_change_operator_control_ack_t) ,typeof(__mavlink_auth_key_t) ,null ,typeof(__mavlink_action_ack_t) ,typeof(__mavlink_action_t) ,typeof(__mavlink_set_mode_t) ,typeof(__mavlink_set_nav_mode_t) ,null ,null ,null ,null ,null ,null ,null ,typeof(__mavlink_param_request_read_t) ,typeof(__mavlink_param_request_list_t) ,typeof(__mavlink_param_value_t) ,typeof(__mavlink_param_set_t) ,null ,typeof(__mavlink_gps_raw_int_t) ,typeof(__mavlink_scaled_imu_t) ,typeof(__mavlink_gps_status_t) ,typeof(__mavlink_raw_imu_t) ,typeof(__mavlink_raw_pressure_t) ,typeof(__mavlink_attitude_t) ,typeof(__mavlink_local_position_t) ,typeof(__mavlink_gps_raw_t) ,typeof(__mavlink_global_position_t) ,typeof(__mavlink_sys_status_t) ,typeof(__mavlink_rc_channels_raw_t) ,typeof(__mavlink_rc_channels_scaled_t) ,typeof(__mavlink_servo_output_raw_t) ,typeof(__mavlink_scaled_pressure_t) ,typeof(__mavlink_waypoint_t) ,typeof(__mavlink_waypoint_request_t) ,typeof(__mavlink_waypoint_set_current_t) ,typeof(__mavlink_waypoint_current_t) ,typeof(__mavlink_waypoint_request_list_t) ,typeof(__mavlink_waypoint_count_t) ,typeof(__mavlink_waypoint_clear_all_t) ,typeof(__mavlink_waypoint_reached_t) ,typeof(__mavlink_waypoint_ack_t) ,typeof(__mavlink_gps_set_global_origin_t) ,typeof(__mavlink_gps_local_origin_set_t) ,typeof(__mavlink_local_position_setpoint_set_t) ,typeof(__mavlink_local_position_setpoint_t) ,typeof(__mavlink_control_status_t) ,typeof(__mavlink_safety_set_allowed_area_t) ,typeof(__mavlink_safety_allowed_area_t) ,typeof(__mavlink_set_roll_pitch_yaw_thrust_t) ,typeof(__mavlink_set_roll_pitch_yaw_speed_thrust_t) ,typeof(__mavlink_roll_pitch_yaw_thrust_setpoint_t) ,typeof(__mavlink_roll_pitch_yaw_speed_thrust_setpoint_t) ,null ,null ,null ,typeof(__mavlink_nav_controller_output_t) ,typeof(__mavlink_position_target_t) ,typeof(__mavlink_state_correction_t) ,typeof(__mavlink_set_altitude_t) ,typeof(__mavlink_request_data_stream_t) ,typeof(__mavlink_hil_state_t) ,typeof(__mavlink_hil_controls_t) ,typeof(__mavlink_manual_control_t) ,typeof(__mavlink_rc_channels_override_t) ,null ,null ,typeof(__mavlink_global_position_int_t) ,typeof(__mavlink_vfr_hud_t) ,typeof(__mavlink_command_t) ,typeof(__mavlink_command_ack_t) ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,typeof(__mavlink_sensor_offsets_t) ,typeof(__mavlink_set_mag_offsets_t) ,typeof(__mavlink_meminfo_t) ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,typeof(__mavlink_debug_vect_t) ,typeof(__mavlink_named_value_float_t) ,typeof(__mavlink_named_value_int_t) ,typeof(__mavlink_statustext_t) ,typeof(__mavlink_debug_t) ,null ,};

	}
}


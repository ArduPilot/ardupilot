using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace ArdupilotMega
{
#if !MAVLINK10
    partial class MAVLink
    {
		public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {3, 4, 8, 14, 8, 28, 3, 32, 0, 2, 3, 2, 2, 0, 0, 0, 0, 0, 0, 0, 19, 2, 23, 21, 0, 37, 26, 101, 26, 16, 32, 32, 37, 32, 11, 17, 17, 16, 18, 36, 4, 4, 2, 2, 4, 2, 2, 3, 14, 12, 18, 16, 8, 27, 25, 18, 18, 24, 24, 0, 0, 0, 26, 16, 36, 5, 6, 56, 26, 21, 18, 0, 0, 18, 20, 20, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 42, 8, 4, 12, 15, 13, 6, 15, 14, 0, 12, 3, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 14, 14, 51, 5};
		public byte[] MAVLINK_MESSAGE_CRCS = new byte[] {72, 39, 190, 92, 191, 217, 104, 119, 0, 219, 60, 186, 10, 0, 0, 0, 0, 0, 0, 0, 89, 159, 162, 121, 0, 149, 222, 110, 179, 136, 66, 126, 185, 147, 112, 252, 162, 215, 229, 128, 9, 106, 101, 213, 4, 229, 21, 214, 215, 14, 206, 50, 157, 126, 108, 213, 95, 5, 127, 0, 0, 0, 57, 126, 130, 119, 193, 191, 236, 158, 143, 0, 0, 104, 123, 131, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 174, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 155, 0, 0, 0, 0, 0, 0, 0, 0, 0, 143, 29, 208, 188, 118, 242, 19, 97, 233, 0, 18, 68, 136, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 178, 224, 60, 106, 7};
		public 		enum MAV_MOUNT_MODE
		{
			MAV_MOUNT_MODE_RETRACT=0, /* Load and keep safe position (Roll,Pitch,Yaw) from EEPROM and stop stabilization | */
			MAV_MOUNT_MODE_NEUTRAL=1, /* Load and keep neutral position (Roll,Pitch,Yaw) from EEPROM. | */
			MAV_MOUNT_MODE_MAVLINK_TARGETING=2, /* Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | */
			MAV_MOUNT_MODE_RC_TARGETING=3, /* Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | */
			MAV_MOUNT_MODE_GPS_POINT=4, /* Load neutral position and start to point to Lat,Lon,Alt | */
			MAV_MOUNT_MODE_ENUM_END=5, /*  | */
		};

		public 		enum MAV_CMD
		{
			WAYPOINT=16, /* Navigate to waypoint. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at waypoint (rotary wing)| Latitude| Longitude| Altitude|  */
			LOITER_UNLIM=17, /* Loiter around this waypoint an unlimited amount of time |Empty| Empty| Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			LOITER_TURNS=18, /* Loiter around this waypoint for X turns |Turns| Empty| Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			LOITER_TIME=19, /* Loiter around this waypoint for X seconds |Seconds (decimal)| Empty| Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
			LAND=21, /* Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			TAKEOFF=22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */
			ROI=80, /* Sets the region of interest (ROI) for a sensor set or the
		            vehicle itself. This can then be used by the vehicles control
		            system to control the vehicle attitude and the attitude of various
		            sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
			PATHPLANNING=81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
			LAST=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
			CONDITION_DELAY=112, /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
			CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
			CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
			CONDITION_YAW=115, /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
			CONDITION_LAST=159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
			DO_SET_MODE=176, /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|  */
			DO_JUMP=177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
			DO_CHANGE_SPEED=178, /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|  */
			DO_SET_HOME=179, /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
			DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
			DO_SET_RELAY=181, /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
			DO_REPEAT_RELAY=182, /* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
			DO_SET_SERVO=183, /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
			DO_REPEAT_SERVO=184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
			DO_CONTROL_VIDEO=200, /* Control onboard camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
			DO_SET_ROI=201, /* Sets the region of interest (ROI) for a sensor set or the
		                    vehicle itself. This can then be used by the vehicles control
		                    system to control the vehicle attitude and the attitude of various
		                    devices such as cameras.
		                 |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
			DO_DIGICAM_CONFIGURE=202, /* Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  */
			DO_DIGICAM_CONTROL=203, /* Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|  */
			DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  */
			DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|  */
			DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
			PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Empty| Empty| Empty|  */
			PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Reserved| Reserved| Empty| Empty| Empty|  */
			ENUM_END=246, /*  | */
		};

		public const byte MAVLINK_MSG_ID_AP_ADC = 153;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_ap_adc_t
		{
		 public ushort adc1; /// ADC output 1
		 public ushort adc2; /// ADC output 2
		 public ushort adc3; /// ADC output 3
		 public ushort adc4; /// ADC output 4
		 public ushort adc5; /// ADC output 5
		 public ushort adc6; /// ADC output 6
		};

		public const byte MAVLINK_MSG_ID_AP_ADC_LEN = 12;
		public const byte MAVLINK_MSG_ID_DIGICAM_CONFIGURE = 154;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_digicam_configure_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte mode; /// Mode enumeration from 1 to N //P, TV, AV, M, Etc (0 means ignore)
		 public ushort shutter_speed; /// Divisor number //e.g. 1000 means 1/1000 (0 means ignore)
		 public byte aperture; /// F stop number x 10 //e.g. 28 means 2.8 (0 means ignore)
		 public byte iso; /// ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore)
		 public byte exposure_type; /// Exposure type enumeration from 1 to N (0 means ignore)
		 public byte command_id; /// Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
		 public byte engine_cut_off; /// Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
		 public byte extra_param; /// Extra parameters enumeration (0 means ignore)
		public float extra_value; /// Correspondent value to given extra_param
		};

		public const byte MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN = 15;
		public const byte MAVLINK_MSG_ID_DIGICAM_CONTROL = 155;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_digicam_control_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte session; /// 0: stop, 1: start or keep it up //Session control e.g. show/hide lens
		 public byte zoom_pos; /// 1 to N //Zoom's absolute position (0 means ignore)
		 public byte zoom_step; /// -100 to 100 //Zooming step value to offset zoom from the current position
		 public byte focus_lock; /// 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
		 public byte shot; /// 0: ignore, 1: shot or start filming
		 public byte command_id; /// Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
		 public byte extra_param; /// Extra parameters enumeration (0 means ignore)
		public float extra_value; /// Correspondent value to given extra_param
		};

		public const byte MAVLINK_MSG_ID_DIGICAM_CONTROL_LEN = 13;
		public const byte MAVLINK_MSG_ID_FENCED_FETCH_POINT = 161;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_fenced_fetch_point_t
		{
		 public byte idx; /// point index (first point is 1, 0 is for return point)
		};

		public const byte MAVLINK_MSG_ID_FENCED_FETCH_POINT_LEN = 1;
		public const byte MAVLINK_MSG_ID_FENCED_POINT = 160;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_fenced_point_t
		{
		 public byte idx; /// point index (first point is 1, 0 is for return point)
		 public byte count; /// total number of points (for sanity checking)
		public float lat; /// Latitude of point
		public float lng; /// Longitude of point
		};

		public const byte MAVLINK_MSG_ID_FENCED_POINT_LEN = 10;
		public const byte MAVLINK_MSG_ID_FENCE_FETCH_POINT = 161;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_fence_fetch_point_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte idx; /// point index (first point is 1, 0 is for return point)
		};

		public const byte MAVLINK_MSG_ID_FENCE_FETCH_POINT_LEN = 3;
		public const byte MAVLINK_MSG_ID_FENCE_POINT = 160;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_fence_point_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte idx; /// point index (first point is 1, 0 is for return point)
		 public byte count; /// total number of points (for sanity checking)
		public float lat; /// Latitude of point
		public float lng; /// Longitude of point
		};

		public const byte MAVLINK_MSG_ID_FENCE_POINT_LEN = 12;
		public const byte MAVLINK_MSG_ID_FENCE_STATUS = 162;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_fence_status_t
		{
		 public byte breach_status; /// 0 if currently inside fence, 1 if outside
		 public ushort breach_count; /// number of fence breaches
		 public byte breach_type; /// last breach type (see FENCE_BREACH_* enum)
		 public uint breach_time; /// time of last breach in milliseconds since boot
		};

		public const byte MAVLINK_MSG_ID_FENCE_STATUS_LEN = 8;
		public const byte MAVLINK_MSG_ID_MEMINFO = 152;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_meminfo_t
		{
		 public ushort brkval; /// heap top
		 public ushort freemem; /// free memory
		};

		public const byte MAVLINK_MSG_ID_MEMINFO_LEN = 4;
		public const byte MAVLINK_MSG_ID_MOUNT_CONFIGURE = 156;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mount_configure_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte mount_mode; /// mount operating mode (see MAV_MOUNT_MODE enum)
		 public byte stab_roll; /// (1 = yes, 0 = no)
		 public byte stab_pitch; /// (1 = yes, 0 = no)
		 public byte stab_yaw; /// (1 = yes, 0 = no)
		};

		public const byte MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN = 6;
		public const byte MAVLINK_MSG_ID_MOUNT_CONTROL = 157;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mount_control_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public int input_a; /// pitch(deg*100) or lat, depending on mount mode
		 public int input_b; /// roll(deg*100) or lon depending on mount mode
		 public int input_c; /// yaw(deg*100) or alt (in cm) depending on mount mode
		 public byte save_position; /// if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
		};

		public const byte MAVLINK_MSG_ID_MOUNT_CONTROL_LEN = 15;
		public const byte MAVLINK_MSG_ID_MOUNT_STATUS = 158;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mount_status_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public int pointing_a; /// pitch(deg*100) or lat, depending on mount mode
		 public int pointing_b; /// roll(deg*100) or lon depending on mount mode
		 public int pointing_c; /// yaw(deg*100) or alt (in cm) depending on mount mode
		};

		public const byte MAVLINK_MSG_ID_MOUNT_STATUS_LEN = 14;
		public const byte MAVLINK_MSG_ID_SENSOR_OFFSETS = 150;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_sensor_offsets_t
		{
		 public short mag_ofs_x; /// magnetometer X offset
		 public short mag_ofs_y; /// magnetometer Y offset
		 public short mag_ofs_z; /// magnetometer Z offset
		public float mag_declination; /// magnetic declination (radians)
		 public int raw_press; /// raw pressure from barometer
		 public int raw_temp; /// raw temperature from barometer
		public float gyro_cal_x; /// gyro X calibration
		public float gyro_cal_y; /// gyro Y calibration
		public float gyro_cal_z; /// gyro Z calibration
		public float accel_cal_x; /// accel X calibration
		public float accel_cal_y; /// accel Y calibration
		public float accel_cal_z; /// accel Z calibration
		};

		public const byte MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN = 42;
		public const byte MAVLINK_MSG_ID_SET_MAG_OFFSETS = 151;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_mag_offsets_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public short mag_ofs_x; /// magnetometer X offset
		 public short mag_ofs_y; /// magnetometer Y offset
		 public short mag_ofs_z; /// magnetometer Z offset
		};

		public const byte MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN = 8;
		public 		enum MAV_DATA_STREAM
		{
			MAV_DATA_STREAM_ALL=0, /* Enable all data streams | */
			MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. | */
			MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS | */
			MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW | */
			MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. | */
			MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. | */
			MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot | */
			MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot | */
			MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot | */
			MAV_DATA_STREAM_ENUM_END=13, /*  | */
		};

		public 		enum MAV_ROI
		{
			MAV_ROI_NONE=0, /* No region of interest. | */
			MAV_ROI_WPNEXT=1, /* Point toward next waypoint. | */
			MAV_ROI_WPINDEX=2, /* Point toward given waypoint. | */
			MAV_ROI_LOCATION=3, /* Point toward fixed location. | */
			MAV_ROI_TARGET=4, /* Point toward of given id. | */
			MAV_ROI_ENUM_END=5, /*  | */
		};

		public const byte MAVLINK_MSG_ID_ACTION = 10;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_action_t
		{
		 public byte target; /// The system executing the action
		 public byte target_component; /// The component executing the action
		 public byte action; /// The action id
		};

		public const byte MAVLINK_MSG_ID_ACTION_LEN = 3;
		public const byte MAVLINK_MSG_ID_ACTION_ACK = 9;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_action_ack_t
		{
		 public byte action; /// The action id
		 public byte result; /// 0: Action DENIED, 1: Action executed
		};

		public const byte MAVLINK_MSG_ID_ACTION_ACK_LEN = 2;
		public const byte MAVLINK_MSG_ID_ATTITUDE = 30;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_attitude_t
		{
		 public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float roll; /// Roll angle (rad)
		public float pitch; /// Pitch angle (rad)
		public float yaw; /// Yaw angle (rad)
		public float rollspeed; /// Roll angular speed (rad/s)
		public float pitchspeed; /// Pitch angular speed (rad/s)
		public float yawspeed; /// Yaw angular speed (rad/s)
		};

		public const byte MAVLINK_MSG_ID_ATTITUDE_LEN = 32;
		public const byte MAVLINK_MSG_ID_AUTH_KEY = 7;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_auth_key_t
		{
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=32)] 
		 public byte[] key; /// key
		};

		public const byte MAVLINK_MSG_ID_AUTH_KEY_LEN = 32;
		public const byte MAVLINK_MSG_ID_BOOT = 1;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_boot_t
		{
		 public uint version; /// The onboard software version
		};

		public const byte MAVLINK_MSG_ID_BOOT_LEN = 4;
		public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL = 5;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_change_operator_control_t
		{
		 public byte target_system; /// System the GCS requests control for
		 public byte control_request; /// 0: request control of this MAV, 1: Release control of this MAV
		 public byte version; /// 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=25)] 
		 public byte[] passkey; /// Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
		};

		public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN = 28;
		public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK = 6;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_change_operator_control_ack_t
		{
		 public byte gcs_system_id; /// ID of the GCS this message 
		 public byte control_request; /// 0: request control of this MAV, 1: Release control of this MAV
		 public byte ack; /// 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
		};

		public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_LEN = 3;
		public const byte MAVLINK_MSG_ID_COMMAND = 75;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_command_t
		{
		 public byte target_system; /// System which should execute the command
		 public byte target_component; /// Component which should execute the command, 0 for all components
		 public byte command; /// Command ID, as defined by MAV_CMD enum.
		 public byte confirmation; /// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
		public float param1; /// Parameter 1, as defined by MAV_CMD enum.
		public float param2; /// Parameter 2, as defined by MAV_CMD enum.
		public float param3; /// Parameter 3, as defined by MAV_CMD enum.
		public float param4; /// Parameter 4, as defined by MAV_CMD enum.
		};

		public const byte MAVLINK_MSG_ID_COMMAND_LEN = 20;
		public const byte MAVLINK_MSG_ID_COMMAND_ACK = 76;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_command_ack_t
		{
		public float command; /// Current airspeed in m/s
		public float result; /// 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
		};

		public const byte MAVLINK_MSG_ID_COMMAND_ACK_LEN = 8;
		public const byte MAVLINK_MSG_ID_CONTROL_STATUS = 52;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_control_status_t
		{
		 public byte position_fix; /// Position fix: 0: lost, 2: 2D position fix, 3: 3D position fix 
		 public byte vision_fix; /// Vision position fix: 0: lost, 1: 2D local position hold, 2: 2D global position fix, 3: 3D global position fix 
		 public byte gps_fix; /// GPS position fix: 0: no reception, 1: Minimum 1 satellite, but no position fix, 2: 2D position fix, 3: 3D position fix 
		 public byte ahrs_health; /// Attitude estimation health: 0: poor, 255: excellent
		 public byte control_att; /// 0: Attitude control disabled, 1: enabled
		 public byte control_pos_xy; /// 0: X, Y position control disabled, 1: enabled
		 public byte control_pos_z; /// 0: Z position control disabled, 1: enabled
		 public byte control_pos_yaw; /// 0: Yaw angle control disabled, 1: enabled
		};

		public const byte MAVLINK_MSG_ID_CONTROL_STATUS_LEN = 8;
		public const byte MAVLINK_MSG_ID_DEBUG = 255;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_debug_t
		{
		 public byte ind; /// index of debug variable
		public float value; /// DEBUG value
		};

		public const byte MAVLINK_MSG_ID_DEBUG_LEN = 5;
		public const byte MAVLINK_MSG_ID_DEBUG_VECT = 251;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_debug_vect_t
		{
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=10)] 
		 public byte[] name; /// Name
		 public ulong usec; /// Timestamp
		public float x; /// x
		public float y; /// y
		public float z; /// z
		};

		public const byte MAVLINK_MSG_ID_DEBUG_VECT_LEN = 30;
		public const byte MAVLINK_MSG_ID_GLOBAL_POSITION = 33;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_global_position_t
		{
		 public ulong usec; /// Timestamp (microseconds since unix epoch)
		public float lat; /// Latitude, in degrees
		public float lon; /// Longitude, in degrees
		public float alt; /// Absolute altitude, in meters
		public float vx; /// X Speed (in Latitude direction, positive: going north)
		public float vy; /// Y Speed (in Longitude direction, positive: going east)
		public float vz; /// Z Speed (in Altitude direction, positive: going up)
		};

		public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_LEN = 32;
		public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 73;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_global_position_int_t
		{
		 public int lat; /// Latitude, expressed as * 1E7
		 public int lon; /// Longitude, expressed as * 1E7
		 public int alt; /// Altitude in meters, expressed as * 1000 (millimeters)
		 public short vx; /// Ground X Speed (Latitude), expressed as m/s * 100
		 public short vy; /// Ground Y Speed (Longitude), expressed as m/s * 100
		 public short vz; /// Ground Z Speed (Altitude), expressed as m/s * 100
		};

		public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN = 18;
		public const byte MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET = 49;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_local_origin_set_t
		{
		 public int latitude; /// Latitude (WGS84), expressed as * 1E7
		 public int longitude; /// Longitude (WGS84), expressed as * 1E7
		 public int altitude; /// Altitude(WGS84), expressed as * 1000
		};

		public const byte MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET_LEN = 12;
		public const byte MAVLINK_MSG_ID_GPS_RAW = 32;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_raw_t
		{
		 public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		 public byte fix_type; /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
		public float lat; /// Latitude in degrees
		public float lon; /// Longitude in degrees
		public float alt; /// Altitude in meters
		public float eph; /// GPS HDOP
		public float epv; /// GPS VDOP
		public float v; /// GPS ground speed
		public float hdg; /// Compass heading in degrees, 0..360 degrees
		};

		public const byte MAVLINK_MSG_ID_GPS_RAW_LEN = 37;
		public const byte MAVLINK_MSG_ID_GPS_RAW_INT = 25;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_raw_int_t
		{
		 public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		 public byte fix_type; /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
		 public int lat; /// Latitude in 1E7 degrees
		 public int lon; /// Longitude in 1E7 degrees
		 public int alt; /// Altitude in 1E3 meters (millimeters)
		public float eph; /// GPS HDOP
		public float epv; /// GPS VDOP
		public float v; /// GPS ground speed (m/s)
		public float hdg; /// Compass heading in degrees, 0..360 degrees
		};

		public const byte MAVLINK_MSG_ID_GPS_RAW_INT_LEN = 37;
		public const byte MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN = 48;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_set_global_origin_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public int latitude; /// global position * 1E7
		 public int longitude; /// global position * 1E7
		 public int altitude; /// global position * 1000
		};

		public const byte MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN_LEN = 14;
		public const byte MAVLINK_MSG_ID_GPS_STATUS = 27;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_status_t
		{
		 public byte satellites_visible; /// Number of satellites visible
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=20)] 
		 public byte[] satellite_prn; /// Global satellite ID
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=20)] 
		 public byte[] satellite_used; /// 0: Satellite not used, 1: used for localization
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=20)] 
		 public byte[] satellite_elevation; /// Elevation (0: right on top of receiver, 90: on the horizon) of satellite
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=20)] 
		 public byte[] satellite_azimuth; /// Direction of satellite, 0: 0 deg, 255: 360 deg.
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=20)] 
		 public byte[] satellite_snr; /// Signal to noise ratio of satellite
		};

		public const byte MAVLINK_MSG_ID_GPS_STATUS_LEN = 101;
		public const byte MAVLINK_MSG_ID_HEARTBEAT = 0;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_heartbeat_t
		{
		 public byte type; /// Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
		 public byte autopilot; /// Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
		 public byte mavlink_version; /// MAVLink version
		};

		public const byte MAVLINK_MSG_ID_HEARTBEAT_LEN = 3;
		public const byte MAVLINK_MSG_ID_HIL_CONTROLS = 68;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_hil_controls_t
		{
		 public ulong time_us; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float roll_ailerons; /// Control output -3 .. 1
		public float pitch_elevator; /// Control output -1 .. 1
		public float yaw_rudder; /// Control output -1 .. 1
		public float throttle; /// Throttle 0 .. 1
		 public byte mode; /// System mode (MAV_MODE)
		 public byte nav_mode; /// Navigation mode (MAV_NAV_MODE)
		};

		public const byte MAVLINK_MSG_ID_HIL_CONTROLS_LEN = 26;
		public const byte MAVLINK_MSG_ID_HIL_STATE = 67;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_hil_state_t
		{
		 public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float roll; /// Roll angle (rad)
		public float pitch; /// Pitch angle (rad)
		public float yaw; /// Yaw angle (rad)
		public float rollspeed; /// Roll angular speed (rad/s)
		public float pitchspeed; /// Pitch angular speed (rad/s)
		public float yawspeed; /// Yaw angular speed (rad/s)
		 public int lat; /// Latitude, expressed as * 1E7
		 public int lon; /// Longitude, expressed as * 1E7
		 public int alt; /// Altitude in meters, expressed as * 1000 (millimeters)
		 public short vx; /// Ground X Speed (Latitude), expressed as m/s * 100
		 public short vy; /// Ground Y Speed (Longitude), expressed as m/s * 100
		 public short vz; /// Ground Z Speed (Altitude), expressed as m/s * 100
		 public short xacc; /// X acceleration (mg)
		 public short yacc; /// Y acceleration (mg)
		 public short zacc; /// Z acceleration (mg)
		};

		public const byte MAVLINK_MSG_ID_HIL_STATE_LEN = 56;
		public const byte MAVLINK_MSG_ID_LOCAL_POSITION = 31;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_local_position_t
		{
		 public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float x; /// X Position
		public float y; /// Y Position
		public float z; /// Z Position
		public float vx; /// X Speed
		public float vy; /// Y Speed
		public float vz; /// Z Speed
		};

		public const byte MAVLINK_MSG_ID_LOCAL_POSITION_LEN = 32;
		public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT = 51;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_local_position_setpoint_t
		{
		public float x; /// x position
		public float y; /// y position
		public float z; /// z position
		public float yaw; /// Desired yaw angle
		};

		public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_LEN = 16;
		public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET = 50;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_local_position_setpoint_set_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		public float x; /// x position
		public float y; /// y position
		public float z; /// z position
		public float yaw; /// Desired yaw angle
		};

		public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET_LEN = 18;
		public const byte MAVLINK_MSG_ID_MANUAL_CONTROL = 69;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_manual_control_t
		{
		 public byte target; /// The system to be controlled
		public float roll; /// roll
		public float pitch; /// pitch
		public float yaw; /// yaw
		public float thrust; /// thrust
		 public byte roll_manual; /// roll control enabled auto:0, manual:1
		 public byte pitch_manual; /// pitch auto:0, manual:1
		 public byte yaw_manual; /// yaw auto:0, manual:1
		 public byte thrust_manual; /// thrust auto:0, manual:1
		};

		public const byte MAVLINK_MSG_ID_MANUAL_CONTROL_LEN = 21;
		public const byte MAVLINK_MSG_ID_NAMED_VALUE_FLOAT = 252;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_named_value_float_t
		{
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=10)] 
		 public byte[] name; /// Name of the debug variable
		public float value; /// Floating point value
		};

		public const byte MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN = 14;
		public const byte MAVLINK_MSG_ID_NAMED_VALUE_INT = 253;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_named_value_int_t
		{
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=10)] 
		 public byte[] name; /// Name of the debug variable
		 public int value; /// Signed integer value
		};

		public const byte MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN = 14;
		public const byte MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT = 62;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_nav_controller_output_t
		{
		public float nav_roll; /// Current desired roll in degrees
		public float nav_pitch; /// Current desired pitch in degrees
		 public short nav_bearing; /// Current desired heading in degrees
		 public short target_bearing; /// Bearing to current waypoint/target in degrees
		 public ushort wp_dist; /// Distance to active waypoint in meters
		public float alt_error; /// Current altitude error in meters
		public float aspd_error; /// Current airspeed error in meters/second
		public float xtrack_error; /// Current crosstrack error on x-y plane in meters
		};

		public const byte MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN = 26;
		public const byte MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT = 140;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_object_detection_event_t
		{
		 public uint time; /// Timestamp in milliseconds since system boot
		 public ushort object_id; /// Object ID
		 public byte type; /// Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=20)] 
		 public byte[] name; /// Name of the object as defined by the detector
		 public byte quality; /// Detection quality / confidence. 0: bad, 255: maximum confidence
		public float bearing; /// Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
		public float distance; /// Ground distance in meters
		};

		public const byte MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN = 36;
		public const byte MAVLINK_MSG_ID_OPTICAL_FLOW = 100;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_optical_flow_t
		{
		 public ulong time; /// Timestamp (UNIX)
		 public byte sensor_id; /// Sensor ID
		 public short flow_x; /// Flow in pixels in x-sensor direction
		 public short flow_y; /// Flow in pixels in y-sensor direction
		 public byte quality; /// Optical flow quality / confidence. 0: bad, 255: maximum quality
		public float ground_distance; /// Ground distance in meters
		};

		public const byte MAVLINK_MSG_ID_OPTICAL_FLOW_LEN = 18;
		public const byte MAVLINK_MSG_ID_PARAM_REQUEST_LIST = 21;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_param_request_list_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_PARAM_REQUEST_LIST_LEN = 2;
		public const byte MAVLINK_MSG_ID_PARAM_REQUEST_READ = 20;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_param_request_read_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=15)] 
		 public byte[] param_id; /// Onboard parameter id
		 public short param_index; /// Parameter index. Send -1 to use the param ID field as identifier
		};

		public const byte MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN = 19;
		public const byte MAVLINK_MSG_ID_PARAM_SET = 23;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_param_set_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=15)] 
		 public byte[] param_id; /// Onboard parameter id
		public float param_value; /// Onboard parameter value
		};

		public const byte MAVLINK_MSG_ID_PARAM_SET_LEN = 21;
		public const byte MAVLINK_MSG_ID_PARAM_VALUE = 22;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_param_value_t
		{
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=15)] 
		 public byte[] param_id; /// Onboard parameter id
		public float param_value; /// Onboard parameter value
		 public ushort param_count; /// Total number of onboard parameters
		 public ushort param_index; /// Index of this onboard parameter
		};

		public const byte MAVLINK_MSG_ID_PARAM_VALUE_LEN = 23;
		public const byte MAVLINK_MSG_ID_PING = 3;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_ping_t
		{
		 public uint seq; /// PING sequence
		 public byte target_system; /// 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
		 public byte target_component; /// 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
		 public ulong time; /// Unix timestamp in microseconds
		};

		public const byte MAVLINK_MSG_ID_PING_LEN = 14;
		public const byte MAVLINK_MSG_ID_POSITION_TARGET = 63;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_position_target_t
		{
		public float x; /// x position
		public float y; /// y position
		public float z; /// z position
		public float yaw; /// yaw orientation in radians, 0 = NORTH
		};

		public const byte MAVLINK_MSG_ID_POSITION_TARGET_LEN = 16;
		public const byte MAVLINK_MSG_ID_RAW_IMU = 28;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_raw_imu_t
		{
		 public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		 public short xacc; /// X acceleration (raw)
		 public short yacc; /// Y acceleration (raw)
		 public short zacc; /// Z acceleration (raw)
		 public short xgyro; /// Angular speed around X axis (raw)
		 public short ygyro; /// Angular speed around Y axis (raw)
		 public short zgyro; /// Angular speed around Z axis (raw)
		 public short xmag; /// X Magnetic field (raw)
		 public short ymag; /// Y Magnetic field (raw)
		 public short zmag; /// Z Magnetic field (raw)
		};

		public const byte MAVLINK_MSG_ID_RAW_IMU_LEN = 26;
		public const byte MAVLINK_MSG_ID_RAW_PRESSURE = 29;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_raw_pressure_t
		{
		 public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		 public short press_abs; /// Absolute pressure (raw)
		 public short press_diff1; /// Differential pressure 1 (raw)
		 public short press_diff2; /// Differential pressure 2 (raw)
		 public short temperature; /// Raw Temperature measurement (raw)
		};

		public const byte MAVLINK_MSG_ID_RAW_PRESSURE_LEN = 16;
		public const byte MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = 70;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_rc_channels_override_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public ushort chan1_raw; /// RC channel 1 value, in microseconds
		 public ushort chan2_raw; /// RC channel 2 value, in microseconds
		 public ushort chan3_raw; /// RC channel 3 value, in microseconds
		 public ushort chan4_raw; /// RC channel 4 value, in microseconds
		 public ushort chan5_raw; /// RC channel 5 value, in microseconds
		 public ushort chan6_raw; /// RC channel 6 value, in microseconds
		 public ushort chan7_raw; /// RC channel 7 value, in microseconds
		 public ushort chan8_raw; /// RC channel 8 value, in microseconds
		};

		public const byte MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN = 18;
		public const byte MAVLINK_MSG_ID_RC_CHANNELS_RAW = 35;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_rc_channels_raw_t
		{
		 public ushort chan1_raw; /// RC channel 1 value, in microseconds
		 public ushort chan2_raw; /// RC channel 2 value, in microseconds
		 public ushort chan3_raw; /// RC channel 3 value, in microseconds
		 public ushort chan4_raw; /// RC channel 4 value, in microseconds
		 public ushort chan5_raw; /// RC channel 5 value, in microseconds
		 public ushort chan6_raw; /// RC channel 6 value, in microseconds
		 public ushort chan7_raw; /// RC channel 7 value, in microseconds
		 public ushort chan8_raw; /// RC channel 8 value, in microseconds
		 public byte rssi; /// Receive signal strength indicator, 0: 0%, 255: 100%
		};

		public const byte MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN = 17;
		public const byte MAVLINK_MSG_ID_RC_CHANNELS_SCALED = 36;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_rc_channels_scaled_t
		{
		 public short chan1_scaled; /// RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan2_scaled; /// RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan3_scaled; /// RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan4_scaled; /// RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan5_scaled; /// RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan6_scaled; /// RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan7_scaled; /// RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan8_scaled; /// RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public byte rssi; /// Receive signal strength indicator, 0: 0%, 255: 100%
		};

		public const byte MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN = 17;
		public const byte MAVLINK_MSG_ID_REQUEST_DATA_STREAM = 66;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_request_data_stream_t
		{
		 public byte target_system; /// The target requested to send the message stream.
		 public byte target_component; /// The target requested to send the message stream.
		 public byte req_stream_id; /// The ID of the requested message type
		 public ushort req_message_rate; /// Update rate in Hertz
		 public byte start_stop; /// 1 to start sending, 0 to stop sending.
		};

		public const byte MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN = 6;
		public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT = 58;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_roll_pitch_yaw_speed_thrust_setpoint_t
		{
		 public ulong time_us; /// Timestamp in micro seconds since unix epoch
		public float roll_speed; /// Desired roll angular speed in rad/s
		public float pitch_speed; /// Desired pitch angular speed in rad/s
		public float yaw_speed; /// Desired yaw angular speed in rad/s
		public float thrust; /// Collective thrust, normalized to 0 .. 1
		};

		public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT_LEN = 24;
		public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT = 57;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_roll_pitch_yaw_thrust_setpoint_t
		{
		 public ulong time_us; /// Timestamp in micro seconds since unix epoch
		public float roll; /// Desired roll angle in radians
		public float pitch; /// Desired pitch angle in radians
		public float yaw; /// Desired yaw angle in radians
		public float thrust; /// Collective thrust, normalized to 0 .. 1
		};

		public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT_LEN = 24;
		public const byte MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA = 54;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_safety_allowed_area_t
		{
		 public byte frame; /// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
		public float p1x; /// x position 1 / Latitude 1
		public float p1y; /// y position 1 / Longitude 1
		public float p1z; /// z position 1 / Altitude 1
		public float p2x; /// x position 2 / Latitude 2
		public float p2y; /// y position 2 / Longitude 2
		public float p2z; /// z position 2 / Altitude 2
		};

		public const byte MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA_LEN = 25;
		public const byte MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA = 53;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_safety_set_allowed_area_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte frame; /// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
		public float p1x; /// x position 1 / Latitude 1
		public float p1y; /// y position 1 / Longitude 1
		public float p1z; /// z position 1 / Altitude 1
		public float p2x; /// x position 2 / Latitude 2
		public float p2y; /// y position 2 / Longitude 2
		public float p2z; /// z position 2 / Altitude 2
		};

		public const byte MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN = 27;
		public const byte MAVLINK_MSG_ID_SCALED_IMU = 26;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_scaled_imu_t
		{
		 public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		 public short xacc; /// X acceleration (mg)
		 public short yacc; /// Y acceleration (mg)
		 public short zacc; /// Z acceleration (mg)
		 public short xgyro; /// Angular speed around X axis (millirad /sec)
		 public short ygyro; /// Angular speed around Y axis (millirad /sec)
		 public short zgyro; /// Angular speed around Z axis (millirad /sec)
		 public short xmag; /// X Magnetic field (milli tesla)
		 public short ymag; /// Y Magnetic field (milli tesla)
		 public short zmag; /// Z Magnetic field (milli tesla)
		};

		public const byte MAVLINK_MSG_ID_SCALED_IMU_LEN = 26;
		public const byte MAVLINK_MSG_ID_SCALED_PRESSURE = 38;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_scaled_pressure_t
		{
		 public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float press_abs; /// Absolute pressure (hectopascal)
		public float press_diff; /// Differential pressure 1 (hectopascal)
		 public short temperature; /// Temperature measurement (0.01 degrees celsius)
		};

		public const byte MAVLINK_MSG_ID_SCALED_PRESSURE_LEN = 18;
		public const byte MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 37;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_servo_output_raw_t
		{
		 public ushort servo1_raw; /// Servo output 1 value, in microseconds
		 public ushort servo2_raw; /// Servo output 2 value, in microseconds
		 public ushort servo3_raw; /// Servo output 3 value, in microseconds
		 public ushort servo4_raw; /// Servo output 4 value, in microseconds
		 public ushort servo5_raw; /// Servo output 5 value, in microseconds
		 public ushort servo6_raw; /// Servo output 6 value, in microseconds
		 public ushort servo7_raw; /// Servo output 7 value, in microseconds
		 public ushort servo8_raw; /// Servo output 8 value, in microseconds
		};

		public const byte MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN = 16;
		public const byte MAVLINK_MSG_ID_SET_ALTITUDE = 65;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_altitude_t
		{
		 public byte target; /// The system setting the altitude
		 public uint mode; /// The new altitude in meters
		};

		public const byte MAVLINK_MSG_ID_SET_ALTITUDE_LEN = 5;
		public const byte MAVLINK_MSG_ID_SET_MODE = 11;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_mode_t
		{
		 public byte target; /// The system setting the mode
		 public byte mode; /// The new mode
		};

		public const byte MAVLINK_MSG_ID_SET_MODE_LEN = 2;
		public const byte MAVLINK_MSG_ID_SET_NAV_MODE = 12;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_nav_mode_t
		{
		 public byte target; /// The system setting the mode
		 public byte nav_mode; /// The new navigation mode
		};

		public const byte MAVLINK_MSG_ID_SET_NAV_MODE_LEN = 2;
		public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST = 56;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_roll_pitch_yaw_speed_thrust_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		public float roll_speed; /// Desired roll angular speed in rad/s
		public float pitch_speed; /// Desired pitch angular speed in rad/s
		public float yaw_speed; /// Desired yaw angular speed in rad/s
		public float thrust; /// Collective thrust, normalized to 0 .. 1
		};

		public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST_LEN = 18;
		public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST = 55;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_roll_pitch_yaw_thrust_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		public float roll; /// Desired roll angle in radians
		public float pitch; /// Desired pitch angle in radians
		public float yaw; /// Desired yaw angle in radians
		public float thrust; /// Collective thrust, normalized to 0 .. 1
		};

		public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST_LEN = 18;
		public const byte MAVLINK_MSG_ID_STATE_CORRECTION = 64;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_state_correction_t
		{
		public float xErr; /// x position error
		public float yErr; /// y position error
		public float zErr; /// z position error
		public float rollErr; /// roll error (radians)
		public float pitchErr; /// pitch error (radians)
		public float yawErr; /// yaw error (radians)
		public float vxErr; /// x velocity
		public float vyErr; /// y velocity
		public float vzErr; /// z velocity
		};

		public const byte MAVLINK_MSG_ID_STATE_CORRECTION_LEN = 36;
		public const byte MAVLINK_MSG_ID_STATUSTEXT = 254;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_statustext_t
		{
		 public byte severity; /// Severity of status, 0 = info message, 255 = critical fault
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=50)] 
		 public byte[] text; /// Status text message, without null termination public byteacter
		};

		public const byte MAVLINK_MSG_ID_STATUSTEXT_LEN = 51;
		public const byte MAVLINK_MSG_ID_SYSTEM_TIME = 2;
		public const byte MAVLINK_MSG_ID_SYSTEM_TIME_UTC = 4;
		public const byte MAVLINK_MSG_ID_SYS_STATUS = 34;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_sys_status_t
		{
		 public byte mode; /// System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
		 public byte nav_mode; /// Navigation mode, see MAV_NAV_MODE ENUM
		 public byte status; /// System status flag, see MAV_STATUS ENUM
		 public ushort load; /// Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
		 public ushort vbat; /// Battery voltage, in millivolts (1 = 1 millivolt)
		 public ushort battery_remaining; /// Remaining battery energy: (0%: 0, 100%: 1000)
		 public ushort packet_drop; /// Dropped packets (packets that were corrupted on reception on the MAV)
		};

		public const byte MAVLINK_MSG_ID_SYS_STATUS_LEN = 11;
		public const byte MAVLINK_MSG_ID_VFR_HUD = 74;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_vfr_hud_t
		{
		public float airspeed; /// Current airspeed in m/s
		public float groundspeed; /// Current ground speed in m/s
		 public short heading; /// Current heading in degrees, in compass units (0..360, 0=north)
		 public ushort throttle; /// Current throttle setting in integer percent, 0 to 100
		public float alt; /// Current altitude (MSL), in meters
		public float climb; /// Current climb rate in meters/second
		};

		public const byte MAVLINK_MSG_ID_VFR_HUD_LEN = 20;
		public const byte MAVLINK_MSG_ID_WAYPOINT = 39;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public ushort seq; /// Sequence
		 public byte frame; /// The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
		 public byte command; /// The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
		 public byte current; /// false:0, true:1
		 public byte autocontinue; /// autocontinue to next wp
		public float param1; /// PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
		public float param2; /// PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
		public float param3; /// PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
		public float param4; /// PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
		public float x; /// PARAM5 / local: x position, global: latitude
		public float y; /// PARAM6 / y position: global: longitude
		public float z; /// PARAM7 / z position: global: altitude
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_LEN = 36;
		public const byte MAVLINK_MSG_ID_WAYPOINT_ACK = 47;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_ack_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte type; /// 0: OK, 1: Error
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_ACK_LEN = 3;
		public const byte MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL = 45;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_clear_all_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL_LEN = 2;
		public const byte MAVLINK_MSG_ID_WAYPOINT_COUNT = 44;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_count_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public ushort count; /// Number of Waypoints in the Sequence
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_COUNT_LEN = 4;
		public const byte MAVLINK_MSG_ID_WAYPOINT_CURRENT = 42;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_current_t
		{
		 public ushort seq; /// Sequence
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_CURRENT_LEN = 2;
		public const byte MAVLINK_MSG_ID_WAYPOINT_REACHED = 46;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_reached_t
		{
		 public ushort seq; /// Sequence
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_REACHED_LEN = 2;
		public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST = 40;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_request_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public ushort seq; /// Sequence
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST_LEN = 4;
		public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST = 43;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_request_list_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST_LEN = 2;
		public const byte MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT = 41;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_waypoint_set_current_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public ushort seq; /// Sequence
		};

		public const byte MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT_LEN = 4;
		public 		enum MAV_CLASS
		{
		    MAV_CLASS_GENERIC = 0,        /// Generic autopilot, full support for everything
		    MAV_CLASS_PIXHAWK = 1,        /// PIXHAWK autopilot, http://pixhawk.ethz.ch
		    MAV_CLASS_SLUGS = 2,          /// SLUGS autopilot, http://slugsuav.soe.ucsc.edu
		    MAV_CLASS_ARDUPILOTMEGA = 3,  /// ArduPilotMega / ArduCopter, http://diydrones.com
		    MAV_CLASS_OPENPILOT = 4,      /// OpenPilot, http://openpilot.org
		    MAV_CLASS_GENERIC_MISSION_WAYPOINTS_ONLY = 5,  /// Generic autopilot only supporting simple waypoints
		    MAV_CLASS_GENERIC_MISSION_NAVIGATION_ONLY = 6, /// Generic autopilot supporting waypoints and other simple navigation commands
		    MAV_CLASS_GENERIC_MISSION_FULL = 7,            /// Generic autopilot supporting the full mission command set
		    MAV_CLASS_NONE = 8,           /// No valid autopilot
		    MAV_CLASS_NB                  /// Number of autopilot classes
		};

		public 		enum MAV_ACTION
		{
		    MAV_ACTION_HOLD = 0,
		    MAV_ACTION_MOTORS_START = 1,
		    MAV_ACTION_LAUNCH = 2,
		    MAV_ACTION_RETURN = 3,
		    MAV_ACTION_EMCY_LAND = 4,
		    MAV_ACTION_EMCY_KILL = 5,
		    MAV_ACTION_CONFIRM_KILL = 6,
		    MAV_ACTION_CONTINUE = 7,
		    MAV_ACTION_MOTORS_STOP = 8,
		    MAV_ACTION_HALT = 9,
		    MAV_ACTION_SHUTDOWN = 10,
		    MAV_ACTION_REBOOT = 11,
		    MAV_ACTION_SET_MANUAL = 12,
		    MAV_ACTION_SET_AUTO = 13,
		    MAV_ACTION_STORAGE_READ = 14,
		    MAV_ACTION_STORAGE_WRITE = 15,
		    MAV_ACTION_CALIBRATE_RC = 16,
		    MAV_ACTION_CALIBRATE_GYRO = 17,
		    MAV_ACTION_CALIBRATE_MAG = 18,
		    MAV_ACTION_CALIBRATE_ACC = 19,
		    MAV_ACTION_CALIBRATE_PRESSURE = 20,
		    MAV_ACTION_REC_START = 21,
		    MAV_ACTION_REC_PAUSE = 22,
		    MAV_ACTION_REC_STOP = 23,
		    MAV_ACTION_TAKEOFF = 24,
		    MAV_ACTION_NAVIGATE = 25,
		    MAV_ACTION_LAND = 26,
		    MAV_ACTION_LOITER = 27,
		    MAV_ACTION_SET_ORIGIN = 28,
		    MAV_ACTION_RELAY_ON = 29,
		    MAV_ACTION_RELAY_OFF = 30,
		    MAV_ACTION_GET_IMAGE = 31,
		    MAV_ACTION_VIDEO_START = 32,
		    MAV_ACTION_VIDEO_STOP = 33,
		    MAV_ACTION_RESET_MAP = 34,
		    MAV_ACTION_RESET_PLAN = 35,
		    MAV_ACTION_DELAY_BEFORE_COMMAND = 36,
		    MAV_ACTION_ASCEND_AT_RATE = 37,
		    MAV_ACTION_CHANGE_MODE = 38,
		    MAV_ACTION_LOITER_MAX_TURNS = 39,
		    MAV_ACTION_LOITER_MAX_TIME = 40,
		    MAV_ACTION_START_HILSIM = 41,
		    MAV_ACTION_STOP_HILSIM = 42,    
		    MAV_ACTION_NB        /// Number of MAV actions
		};

		public 		enum MAV_MODE
		{
		    MAV_MODE_UNINIT = 0,     /// System is in undefined state
		    MAV_MODE_LOCKED = 1,     /// Motors are blocked, system is safe
		    MAV_MODE_MANUAL = 2,     /// System is allowed to be active, under manual (RC) control
		    MAV_MODE_GUIDED = 3,     /// System is allowed to be active, under autonomous control, manual setpoint
		    MAV_MODE_AUTO =   4,     /// System is allowed to be active, under autonomous control and navigation
		    MAV_MODE_TEST1 =  5,     /// Generic test mode, for custom use
		    MAV_MODE_TEST2 =  6,     /// Generic test mode, for custom use
		    MAV_MODE_TEST3 =  7,     /// Generic test mode, for custom use
		    MAV_MODE_READY =  8,     /// System is ready, motors are unblocked, but controllers are inactive
		    MAV_MODE_RC_TRAINING = 9 /// System is blocked, only RC valued are read and reported back
		};

		public 		enum MAV_STATE
		{
		    MAV_STATE_UNINIT = 0,
		    MAV_STATE_BOOT,
		    MAV_STATE_CALIBRATING,
		    MAV_STATE_STANDBY,
		    MAV_STATE_ACTIVE,
		    MAV_STATE_CRITICAL,
		    MAV_STATE_EMERGENCY,
		    MAV_STATE_HILSIM,
		    MAV_STATE_POWEROFF
		};

		public 		enum MAV_NAV
		{
		    MAV_NAV_GROUNDED = 0,
		    MAV_NAV_LIFTOFF,
		    MAV_NAV_HOLD,
		    MAV_NAV_WAYPOINT,
		    MAV_NAV_VECTOR,
		    MAV_NAV_RETURNING,
		    MAV_NAV_LANDING,
		    MAV_NAV_LOST,
		    MAV_NAV_LOITER,
		    MAV_NAV_FREE_DRIFT
		};

		public 		enum MAV_TYPE
		{
		    MAV_GENERIC = 0,
		    MAV_FIXED_WING = 1,
		    MAV_QUADROTOR = 2,
		    MAV_COAXIAL = 3,
		    MAV_HELICOPTER = 4,
		    MAV_GROUND = 5,
		    OCU = 6,
		    MAV_AIRSHIP = 7,
		    MAV_FREE_BALLOON = 8,
		    MAV_ROCKET = 9,
		    UGV_GROUND_ROVER = 10,
		    UGV_SURFACE_SHIP = 11
		};

		public 		enum MAV_AUTOPILOT_TYPE
		{
		    MAV_AUTOPILOT_GENERIC = 0,
		    MAV_AUTOPILOT_PIXHAWK = 1,
		    MAV_AUTOPILOT_SLUGS = 2,
		    MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
		    MAV_AUTOPILOT_NONE = 4
		};

		public 		enum MAV_COMPONENT
		{
		    MAV_COMP_ID_GPS,
		    MAV_COMP_ID_WAYPOINTPLANNER,
		    MAV_COMP_ID_BLOBTRACKER,
		    MAV_COMP_ID_PATHPLANNER,
		    MAV_COMP_ID_AIRSLAM,
		    MAV_COMP_ID_MAPPER,
		    MAV_COMP_ID_CAMERA,
		    MAV_COMP_ID_IMU = 200,
		    MAV_COMP_ID_IMU_2 = 201,
		    MAV_COMP_ID_IMU_3 = 202,
		    MAV_COMP_ID_UDP_BRIDGE = 240,
		    MAV_COMP_ID_UART_BRIDGE = 241,
		    MAV_COMP_ID_SYSTEM_CONTROL = 250
		};

		public 		enum MAV_FRAME
		{
		    MAV_FRAME_GLOBAL = 0,
		    MAV_FRAME_LOCAL = 1,
		    MAV_FRAME_MISSION = 2,
		    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
		    MAV_FRAME_LOCAL_ENU = 4
		};

Type[] mavstructs = new Type[] {typeof( __mavlink_heartbeat_t) ,typeof( __mavlink_boot_t) ,null ,typeof( __mavlink_ping_t) ,null ,typeof( __mavlink_change_operator_control_t) ,typeof( __mavlink_change_operator_control_ack_t) ,typeof( __mavlink_auth_key_t) ,null ,typeof( __mavlink_action_ack_t) ,typeof( __mavlink_action_t) ,typeof( __mavlink_set_mode_t) ,typeof( __mavlink_set_nav_mode_t) ,null ,null ,null ,null ,null ,null ,null ,typeof( __mavlink_param_request_read_t) ,typeof( __mavlink_param_request_list_t) ,typeof( __mavlink_param_value_t) ,typeof( __mavlink_param_set_t) ,null ,typeof( __mavlink_gps_raw_int_t) ,typeof( __mavlink_scaled_imu_t) ,typeof( __mavlink_gps_status_t) ,typeof( __mavlink_raw_imu_t) ,typeof( __mavlink_raw_pressure_t) ,typeof( __mavlink_attitude_t) ,typeof( __mavlink_local_position_t) ,typeof( __mavlink_gps_raw_t) ,typeof( __mavlink_global_position_t) ,typeof( __mavlink_sys_status_t) ,typeof( __mavlink_rc_channels_raw_t) ,typeof( __mavlink_rc_channels_scaled_t) ,typeof( __mavlink_servo_output_raw_t) ,typeof( __mavlink_scaled_pressure_t) ,typeof( __mavlink_waypoint_t) ,typeof( __mavlink_waypoint_request_t) ,typeof( __mavlink_waypoint_set_current_t) ,typeof( __mavlink_waypoint_current_t) ,typeof( __mavlink_waypoint_request_list_t) ,typeof( __mavlink_waypoint_count_t) ,typeof( __mavlink_waypoint_clear_all_t) ,typeof( __mavlink_waypoint_reached_t) ,typeof( __mavlink_waypoint_ack_t) ,typeof( __mavlink_gps_set_global_origin_t) ,typeof( __mavlink_gps_local_origin_set_t) ,typeof( __mavlink_local_position_setpoint_set_t) ,typeof( __mavlink_local_position_setpoint_t) ,typeof( __mavlink_control_status_t) ,typeof( __mavlink_safety_set_allowed_area_t) ,typeof( __mavlink_safety_allowed_area_t) ,typeof( __mavlink_set_roll_pitch_yaw_thrust_t) ,typeof( __mavlink_set_roll_pitch_yaw_speed_thrust_t) ,typeof( __mavlink_roll_pitch_yaw_thrust_setpoint_t) ,typeof( __mavlink_roll_pitch_yaw_speed_thrust_setpoint_t) ,null ,null ,null ,typeof( __mavlink_nav_controller_output_t) ,typeof( __mavlink_position_target_t) ,typeof( __mavlink_state_correction_t) ,typeof( __mavlink_set_altitude_t) ,typeof( __mavlink_request_data_stream_t) ,typeof( __mavlink_hil_state_t) ,typeof( __mavlink_hil_controls_t) ,typeof( __mavlink_manual_control_t) ,typeof( __mavlink_rc_channels_override_t) ,null ,null ,typeof( __mavlink_global_position_int_t) ,typeof( __mavlink_vfr_hud_t) ,typeof( __mavlink_command_t) ,typeof( __mavlink_command_ack_t) ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,typeof( __mavlink_optical_flow_t) ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,typeof( __mavlink_object_detection_event_t) ,null ,null ,null ,null ,null ,null ,null ,null ,null ,typeof( __mavlink_sensor_offsets_t) ,typeof( __mavlink_set_mag_offsets_t) ,typeof( __mavlink_meminfo_t) ,typeof( __mavlink_ap_adc_t) ,typeof( __mavlink_digicam_configure_t) ,typeof( __mavlink_digicam_control_t) ,typeof( __mavlink_mount_configure_t) ,typeof( __mavlink_mount_control_t) ,typeof( __mavlink_mount_status_t) ,null ,typeof( __mavlink_fence_point_t) ,typeof( __mavlink_fence_fetch_point_t) ,typeof( __mavlink_fence_status_t) ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,typeof( __mavlink_debug_vect_t) ,typeof( __mavlink_named_value_float_t) ,typeof( __mavlink_named_value_int_t) ,typeof( __mavlink_statustext_t) ,typeof( __mavlink_debug_t) ,null ,};

	}
	#endif
}


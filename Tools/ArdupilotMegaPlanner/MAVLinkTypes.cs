using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace ArdupilotMega
{
#if MAVLINK10
    partial class MAVLink
    {

        public const byte MAVLINK_MSG_ID_FENCED_POINT_LEN = 10;
        public const byte MAVLINK_MSG_ID_FENCE_FETCH_POINT = 161;
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct __mavlink_fence_fetch_point_t
        {
            public byte target_system; /// System ID
            public byte target_component; /// Component ID
            public byte idx; /// point index (first point is 1, 0 is for return point)
        };

        public const byte MAVLINK_MSG_ID_FENCE_FETCH_POINT_LEN = 3;
        public const byte MAVLINK_MSG_ID_FENCE_POINT = 160;
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct __mavlink_fence_point_t
        {
            public byte target_system; /// System ID
            public byte target_component; /// Component ID
            public byte idx; /// point index (first point is 1, 0 is for return point)
            public byte count; /// total number of points (for sanity checking)
            public float lat; /// Latitude of point
            public float lng; /// Longitude of point
        };

		public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {9, 31, 12, 0, 14, 28, 3, 32, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 20, 2, 25, 23, 30, 101, 22, 26, 16, 14, 28, 32, 28, 28, 22, 22, 21, 6, 6, 37, 4, 4, 2, 2, 4, 2, 2, 3, 13, 12, 19, 17, 15, 15, 27, 25, 18, 18, 20, 20, 0, 0, 26, 0, 36, 0, 6, 4, 0, 21, 18, 0, 0, 0, 20, 0, 33, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 42, 33, 0, 0, 0, 0, 0, 0, 0, 18, 32, 32, 20, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 42, 8, 4, 12, 15, 13, 6, 15, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 30, 18, 18, 51, 9, 3};
		public byte[] MAVLINK_MESSAGE_CRCS = new byte[] {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 214, 223, 141, 33, 15, 3, 100, 24, 239, 238, 0, 0, 183, 0, 130, 0, 148, 21, 0, 52, 124, 0, 0, 0, 20, 0, 152, 143, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 183, 63, 54, 0, 0, 0, 0, 0, 0, 0, 19, 102, 158, 208, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 134, 219, 208, 188, 84, 22, 19, 21, 134, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 204, 49, 170, 44, 83, 46, 247};
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
			WAYPOINT=16, /* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|  */
			LOITER_UNLIM=17, /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			LOITER_TURNS=18, /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			LOITER_TIME=19, /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
			LAND=21, /* Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|  */
			TAKEOFF=22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */
			ROI=80, /* Sets the region of interest (ROI) for a sensor set or the
		            vehicle itself. This can then be used by the vehicles control
		            system to control the vehicle attitude and the attitude of various
		            sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
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
			DO_CONTROL_VIDEO=200, /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
			DO_DIGICAM_CONFIGURE=202, /* Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  */
			DO_DIGICAM_CONTROL=203, /* Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|  */
			DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  */
			DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|  */
			DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
			PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Empty| Empty| Empty|  */
			PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
			PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Reserved| Reserved| Empty| Empty| Empty|  */
			PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.| Reserved| Reserved| Empty| Empty| Empty|  */
			OVERRIDE_GOTO=252, /* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
			MISSION_START=300, /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
			ENUM_END=301, /*  | */
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
		public float extra_value; /// Correspondent value to given extra_param
		 public ushort shutter_speed; /// Divisor number //e.g. 1000 means 1/1000 (0 means ignore)
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte mode; /// Mode enumeration from 1 to N //P, TV, AV, M, Etc (0 means ignore)
		 public byte aperture; /// F stop number x 10 //e.g. 28 means 2.8 (0 means ignore)
		 public byte iso; /// ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore)
		 public byte exposure_type; /// Exposure type enumeration from 1 to N (0 means ignore)
		 public byte command_id; /// Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
		 public byte engine_cut_off; /// Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
		 public byte extra_param; /// Extra parameters enumeration (0 means ignore)
		};

		public const byte MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN = 15;
		public const byte MAVLINK_MSG_ID_DIGICAM_CONTROL = 155;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_digicam_control_t
		{
		public float extra_value; /// Correspondent value to given extra_param
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte session; /// 0: stop, 1: start or keep it up //Session control e.g. show/hide lens
		 public byte zoom_pos; /// 1 to N //Zoom's absolute position (0 means ignore)
		 public byte zoom_step; /// -100 to 100 //Zooming step value to offset zoom from the current position
		 public byte focus_lock; /// 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
		 public byte shot; /// 0: ignore, 1: shot or start filming
		 public byte command_id; /// Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
		 public byte extra_param; /// Extra parameters enumeration (0 means ignore)
		};

		public const byte MAVLINK_MSG_ID_DIGICAM_CONTROL_LEN = 13;
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
		 public int input_a; /// pitch(deg*100) or lat, depending on mount mode
		 public int input_b; /// roll(deg*100) or lon depending on mount mode
		 public int input_c; /// yaw(deg*100) or alt (in cm) depending on mount mode
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte save_position; /// if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
		};

		public const byte MAVLINK_MSG_ID_MOUNT_CONTROL_LEN = 15;
		public const byte MAVLINK_MSG_ID_MOUNT_STATUS = 158;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mount_status_t
		{
		 public int pointing_a; /// pitch(deg*100) or lat, depending on mount mode
		 public int pointing_b; /// roll(deg*100) or lon depending on mount mode
		 public int pointing_c; /// yaw(deg*100) or alt (in cm) depending on mount mode
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_MOUNT_STATUS_LEN = 14;
		public const byte MAVLINK_MSG_ID_SENSOR_OFFSETS = 150;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_sensor_offsets_t
		{
		public float mag_declination; /// magnetic declination (radians)
		 public int raw_press; /// raw pressure from barometer
		 public int raw_temp; /// raw temperature from barometer
		public float gyro_cal_x; /// gyro X calibration
		public float gyro_cal_y; /// gyro Y calibration
		public float gyro_cal_z; /// gyro Z calibration
		public float accel_cal_x; /// accel X calibration
		public float accel_cal_y; /// accel Y calibration
		public float accel_cal_z; /// accel Z calibration
		 public short mag_ofs_x; /// magnetometer X offset
		 public short mag_ofs_y; /// magnetometer Y offset
		 public short mag_ofs_z; /// magnetometer Z offset
		};

		public const byte MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN = 42;
		public const byte MAVLINK_MSG_ID_SET_MAG_OFFSETS = 151;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_mag_offsets_t
		{
		 public short mag_ofs_x; /// magnetometer X offset
		 public short mag_ofs_y; /// magnetometer Y offset
		 public short mag_ofs_z; /// magnetometer Z offset
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN = 8;
		public 		enum MAV_AUTOPILOT
		{
			MAV_AUTOPILOT_GENERIC=0, /* Generic autopilot, full support for everything | */
			MAV_AUTOPILOT_PIXHAWK=1, /* PIXHAWK autopilot, http://pixhawk.ethz.ch | */
			MAV_AUTOPILOT_SLUGS=2, /* SLUGS autopilot, http://slugsuav.soe.ucsc.edu | */
			MAV_AUTOPILOT_ARDUPILOTMEGA=3, /* ArduPilotMega / ArduCopter, http://diydrones.com | */
			MAV_AUTOPILOT_OPENPILOT=4, /* OpenPilot, http://openpilot.org | */
			MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY=5, /* Generic autopilot only supporting simple waypoints | */
			MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY=6, /* Generic autopilot supporting waypoints and other simple navigation commands | */
			MAV_AUTOPILOT_GENERIC_MISSION_FULL=7, /* Generic autopilot supporting the full mission command set | */
			MAV_AUTOPILOT_INVALID=8, /* No valid autopilot, e.g. a GCS or other MAVLink component | */
			MAV_AUTOPILOT_PPZ=9, /* PPZ UAV - http://nongnu.org/paparazzi | */
			MAV_AUTOPILOT_UDB=10, /* UAV Dev Board | */
			MAV_AUTOPILOT_FP=11, /* FlexiPilot | */
			MAV_AUTOPILOT_ENUM_END=12, /*  | */
		};

		public 		enum MAV_MODE_FLAG
		{
			MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, /* 0b00000001 Reserved for future use. | */
			MAV_MODE_FLAG_TEST_ENABLED=2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
			MAV_MODE_FLAG_AUTO_ENABLED=4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
			MAV_MODE_FLAG_GUIDED_ENABLED=8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
			MAV_MODE_FLAG_STABILIZE_ENABLED=16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
			MAV_MODE_FLAG_HIL_ENABLED=32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
			MAV_MODE_FLAG_MANUAL_INPUT_ENABLED=64, /* 0b01000000 remote control input is enabled. | */
			MAV_MODE_FLAG_SAFETY_ARMED=128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
			MAV_MODE_FLAG_ENUM_END=129, /*  | */
		};

		public 		enum MAV_MODE_FLAG_DECODE_POSITION
		{
			MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE=1, /* Eighth bit: 00000001 | */
			MAV_MODE_FLAG_DECODE_POSITION_TEST=2, /* Seventh bit: 00000010 | */
			MAV_MODE_FLAG_DECODE_POSITION_AUTO=4, /* Sixt bit:   00000100 | */
			MAV_MODE_FLAG_DECODE_POSITION_GUIDED=8, /* Fifth bit:  00001000 | */
			MAV_MODE_FLAG_DECODE_POSITION_STABILIZE=16, /* Fourth bit: 00010000 | */
			MAV_MODE_FLAG_DECODE_POSITION_HIL=32, /* Third bit:  00100000 | */
			MAV_MODE_FLAG_DECODE_POSITION_MANUAL=64, /* Second bit: 01000000 | */
			MAV_MODE_FLAG_DECODE_POSITION_SAFETY=128, /* First bit:  10000000 | */
			MAV_MODE_FLAG_DECODE_POSITION_ENUM_END=129, /*  | */
		};

		public 		enum MAV_GOTO
		{
			MAV_GOTO_DO_HOLD=0, /* Hold at the current position. | */
			MAV_GOTO_DO_CONTINUE=1, /* Continue with the next item in mission execution. | */
			MAV_GOTO_HOLD_AT_CURRENT_POSITION=2, /* Hold at the current position of the system | */
			MAV_GOTO_HOLD_AT_SPECIFIED_POSITION=3, /* Hold at the position specified in the parameters of the DO_HOLD action | */
			MAV_GOTO_ENUM_END=4, /*  | */
		};

		public 		enum MAV_MODE
		{
			MAV_MODE_PREFLIGHT=0, /* System is not ready to fly, booting, calibrating, etc. No flag is set. | */
			MAV_MODE_MANUAL_DISARMED=64, /* System is allowed to be active, under manual (RC) control, no stabilization | */
			MAV_MODE_TEST_DISARMED=66, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
			MAV_MODE_STABILIZE_DISARMED=80, /* System is allowed to be active, under assisted RC control. | */
			MAV_MODE_GUIDED_DISARMED=88, /* System is allowed to be active, under autonomous control, manual setpoint | */
			MAV_MODE_AUTO_DISARMED=92, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | */
			MAV_MODE_MANUAL_ARMED=192, /* System is allowed to be active, under manual (RC) control, no stabilization | */
			MAV_MODE_TEST_ARMED=194, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
			MAV_MODE_STABILIZE_ARMED=208, /* System is allowed to be active, under assisted RC control. | */
			MAV_MODE_GUIDED_ARMED=216, /* System is allowed to be active, under autonomous control, manual setpoint | */
			MAV_MODE_AUTO_ARMED=220, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | */
			MAV_MODE_ENUM_END=221, /*  | */
		};

		public 		enum MAV_STATE
		{
			MAV_STATE_UNINIT=0, /* Uninitialized system, state is unknown. | */
			MAV_STATE_BOOT=1, /* System is booting up. | */
			MAV_STATE_CALIBRATING=2, /* System is calibrating and not flight-ready. | */
			MAV_STATE_STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
			MAV_STATE_ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
			MAV_STATE_CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
			MAV_STATE_EMERGENCY=6, /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
			MAV_STATE_POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
			MAV_STATE_ENUM_END=8, /*  | */
		};

		public 		enum MAV_TYPE
		{
			MAV_TYPE_GENERIC=0, /* Generic micro air vehicle. | */
			MAV_TYPE_FIXED_WING=1, /* Fixed wing aircraft. | */
			MAV_TYPE_QUADROTOR=2, /* Quadrotor | */
			MAV_TYPE_COAXIAL=3, /* Coaxial helicopter | */
			MAV_TYPE_HELICOPTER=4, /* Normal helicopter with tail rotor. | */
			MAV_TYPE_ANTENNA_TRACKER=5, /* Ground installation | */
			MAV_TYPE_GCS=6, /* Operator control unit / ground control station | */
			MAV_TYPE_AIRSHIP=7, /* Airship, controlled | */
			MAV_TYPE_FREE_BALLOON=8, /* Free balloon, uncontrolled | */
			MAV_TYPE_ROCKET=9, /* Rocket | */
			MAV_TYPE_GROUND_ROVER=10, /* Ground rover | */
			MAV_TYPE_SURFACE_BOAT=11, /* Surface vessel, boat, ship | */
			MAV_TYPE_SUBMARINE=12, /* Submarine | */
			MAV_TYPE_HEXAROTOR=13, /* Hexarotor | */
			MAV_TYPE_OCTOROTOR=14, /* Octorotor | */
			MAV_TYPE_TRICOPTER=15, /* Octorotor | */
			MAV_TYPE_FLAPPING_WING=16, /* Flapping wing | */
			MAV_TYPE_ENUM_END=17, /*  | */
		};

		public 		enum MAV_COMPONENT
		{
			MAV_COMP_ID_ALL=0, /*  | */
			MAV_COMP_ID_CAMERA=100, /*  | */
			MAV_COMP_ID_SERVO1=140, /*  | */
			MAV_COMP_ID_SERVO2=141, /*  | */
			MAV_COMP_ID_SERVO3=142, /*  | */
			MAV_COMP_ID_SERVO4=143, /*  | */
			MAV_COMP_ID_SERVO5=144, /*  | */
			MAV_COMP_ID_SERVO6=145, /*  | */
			MAV_COMP_ID_SERVO7=146, /*  | */
			MAV_COMP_ID_SERVO8=147, /*  | */
			MAV_COMP_ID_SERVO9=148, /*  | */
			MAV_COMP_ID_SERVO10=149, /*  | */
			MAV_COMP_ID_SERVO11=150, /*  | */
			MAV_COMP_ID_SERVO12=151, /*  | */
			MAV_COMP_ID_SERVO13=152, /*  | */
			MAV_COMP_ID_SERVO14=153, /*  | */
			MAV_COMP_ID_MAPPER=180, /*  | */
			MAV_COMP_ID_MISSIONPLANNER=190, /*  | */
			MAV_COMP_ID_PATHPLANNER=195, /*  | */
			MAV_COMP_ID_IMU=200, /*  | */
			MAV_COMP_ID_IMU_2=201, /*  | */
			MAV_COMP_ID_IMU_3=202, /*  | */
			MAV_COMP_ID_GPS=220, /*  | */
			MAV_COMP_ID_UDP_BRIDGE=240, /*  | */
			MAV_COMP_ID_UART_BRIDGE=241, /*  | */
			MAV_COMP_ID_SYSTEM_CONTROL=250, /*  | */
			MAV_COMPONENT_ENUM_END=251, /*  | */
		};

		public 		enum MAV_FRAME
		{
			MAV_FRAME_GLOBAL=0, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */
			MAV_FRAME_LOCAL_NED=1, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */
			MAV_FRAME_MISSION=2, /* NOT a coordinate frame, indicates a mission command. | */
			MAV_FRAME_GLOBAL_RELATIVE_ALT=3, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */
			MAV_FRAME_LOCAL_ENU=4, /* Local coordinate frame, Z-down (x: east, y: north, z: up) | */
			MAV_FRAME_ENUM_END=5, /*  | */
		};

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
			MAV_ROI_WPNEXT=1, /* Point toward next MISSION. | */
			MAV_ROI_WPINDEX=2, /* Point toward given MISSION. | */
			MAV_ROI_LOCATION=3, /* Point toward fixed location. | */
			MAV_ROI_TARGET=4, /* Point toward of given id. | */
			MAV_ROI_ENUM_END=5, /*  | */
		};

		public 		enum ACK
		{
			ACK_OK=1, /* Command / mission item is ok. | */
			ACK_ERR_FAIL=2, /* Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. | */
			ACK_ERR_ACCESS_DENIED=3, /* The system is refusing to accept this command from this source / communication partner. | */
			ACK_ERR_NOT_SUPPORTED=4, /* Command or mission item is not supported, other commands would be accepted. | */
			ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED=5, /* The coordinate frame of this command / mission item is not supported. | */
			ACK_ERR_COORDINATES_OUT_OF_RANGE=6, /* The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible. | */
			ACK_ERR_X_LAT_OUT_OF_RANGE=7, /* The X or latitude value is out of range. | */
			ACK_ERR_Y_LON_OUT_OF_RANGE=8, /* The Y or longitude value is out of range. | */
			ACK_ERR_Z_ALT_OUT_OF_RANGE=9, /* The Z or altitude value is out of range. | */
			ACK_ENUM_END=10, /*  | */
		};

		public 		enum MAV_VAR
		{
			MAV_VAR_FLOAT=0, /* 32 bit float | */
			MAV_VAR_UINT8=1, /* 8 bit unsigned integer | */
			MAV_VAR_INT8=2, /* 8 bit signed integer | */
			MAV_VAR_UINT16=3, /* 16 bit unsigned integer | */
			MAV_VAR_INT16=4, /* 16 bit signed integer | */
			MAV_VAR_UINT32=5, /* 32 bit unsigned integer | */
			MAV_VAR_INT32=6, /* 32 bit signed integer | */
			MAV_VAR_ENUM_END=7, /*  | */
		};

		public 		enum MAV_RESULT
		{
			MAV_RESULT_ACCEPTED=0, /* Command ACCEPTED and EXECUTED | */
			MAV_RESULT_TEMPORARILY_REJECTED=1, /* Command TEMPORARY REJECTED/DENIED | */
			MAV_RESULT_DENIED=2, /* Command PERMANENTLY DENIED | */
			MAV_RESULT_UNSUPPORTED=3, /* Command UNKNOWN/UNSUPPORTED | */
			MAV_RESULT_FAILED=4, /* Command executed, but failed | */
			MAV_RESULT_ENUM_END=5, /*  | */
		};

		public 		enum MAV_MISSION_RESULT
		{
			MAV_MISSION_ACCEPTED=0, /* mission accepted OK | */
			MAV_MISSION_ERROR=1, /* generic error / not accepting mission commands at all right now | */
			MAV_MISSION_UNSUPPORTED_FRAME=2, /* coordinate frame is not supported | */
			MAV_MISSION_UNSUPPORTED=3, /* command is not supported | */
			MAV_MISSION_NO_SPACE=4, /* mission item exceeds storage space | */
			MAV_MISSION_INVALID=5, /* one of the parameters has an invalid value | */
			MAV_MISSION_INVALID_PARAM1=6, /* param1 has an invalid value | */
			MAV_MISSION_INVALID_PARAM2=7, /* param2 has an invalid value | */
			MAV_MISSION_INVALID_PARAM3=8, /* param3 has an invalid value | */
			MAV_MISSION_INVALID_PARAM4=9, /* param4 has an invalid value | */
			MAV_MISSION_INVALID_PARAM5_X=10, /* x/param5 has an invalid value | */
			MAV_MISSION_INVALID_PARAM6_Y=11, /* y/param6 has an invalid value | */
			MAV_MISSION_INVALID_PARAM7=12, /* param7 has an invalid value | */
			MAV_MISSION_INVALID_SEQUENCE=13, /* received waypoint out of sequence | */
			MAV_MISSION_DENIED=14, /* not accepting any mission commands from this communication partner | */
			MAV_MISSION_RESULT_ENUM_END=15, /*  | */
		};

		public const byte MAVLINK_MSG_ID_ATTITUDE = 30;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_attitude_t
		{
		 public uint time_boot_ms; /// Timestamp (milliseconds since system boot)
		public float roll; /// Roll angle (rad)
		public float pitch; /// Pitch angle (rad)
		public float yaw; /// Yaw angle (rad)
		public float rollspeed; /// Roll angular speed (rad/s)
		public float pitchspeed; /// Pitch angular speed (rad/s)
		public float yawspeed; /// Yaw angular speed (rad/s)
		};

		public const byte MAVLINK_MSG_ID_ATTITUDE_LEN = 28;
		public const byte MAVLINK_MSG_ID_ATTITUDE_QUATERNION = 31;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_attitude_quaternion_t
		{
		 public uint time_boot_ms; /// Timestamp (milliseconds since system boot)
		public float q1; /// Quaternion component 1
		public float q2; /// Quaternion component 2
		public float q3; /// Quaternion component 3
		public float q4; /// Quaternion component 4
		public float rollspeed; /// Roll angular speed (rad/s)
		public float pitchspeed; /// Pitch angular speed (rad/s)
		public float yawspeed; /// Yaw angular speed (rad/s)
		};

		public const byte MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN = 32;
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
		public const byte MAVLINK_MSG_ID_COMMAND_ACK = 77;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_command_ack_t
		{
		 public ushort command; /// Command ID, as defined by MAV_CMD enum.
		 public byte result; /// See MAV_RESULT enum
		};

		public const byte MAVLINK_MSG_ID_COMMAND_ACK_LEN = 3;
		public const byte MAVLINK_MSG_ID_COMMAND_LONG = 76;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_command_long_t
		{
		public float param1; /// Parameter 1, as defined by MAV_CMD enum.
		public float param2; /// Parameter 2, as defined by MAV_CMD enum.
		public float param3; /// Parameter 3, as defined by MAV_CMD enum.
		public float param4; /// Parameter 4, as defined by MAV_CMD enum.
		public float param5; /// Parameter 5, as defined by MAV_CMD enum.
		public float param6; /// Parameter 6, as defined by MAV_CMD enum.
		public float param7; /// Parameter 7, as defined by MAV_CMD enum.
		 public ushort command; /// Command ID, as defined by MAV_CMD enum.
		 public byte target_system; /// System which should execute the command
		 public byte target_component; /// Component which should execute the command, 0 for all components
		 public byte confirmation; /// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
		};

		public const byte MAVLINK_MSG_ID_COMMAND_LONG_LEN = 33;
		public const byte MAVLINK_MSG_ID_DATA_STREAM = 67;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_data_stream_t
		{
		 public ushort message_rate; /// The requested interval between two messages of this type
		 public byte stream_id; /// The ID of the requested data stream
		 public byte on_off; /// 1 stream is enabled, 0 stream is stopped.
		};

		public const byte MAVLINK_MSG_ID_DATA_STREAM_LEN = 4;
		public const byte MAVLINK_MSG_ID_DEBUG = 254;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_debug_t
		{
		 public uint time_boot_ms; /// Timestamp (milliseconds since system boot)
		public float value; /// DEBUG value
		 public byte ind; /// index of debug variable
		};

		public const byte MAVLINK_MSG_ID_DEBUG_LEN = 9;
		public const byte MAVLINK_MSG_ID_DEBUG_VECT = 250;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_debug_vect_t
		{
		 public ulong time_usec; /// Timestamp
		public float x; /// x
		public float y; /// y
		public float z; /// z
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=10)] 
		 public byte[] name; /// Name
		};

		public const byte MAVLINK_MSG_ID_DEBUG_VECT_LEN = 30;
		public const byte MAVLINK_MSG_ID_EXTENDED_MESSAGE = 255;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_extended_message_t
		{
		 public byte target_system; /// System which should execute the command
		 public byte target_component; /// Component which should execute the command, 0 for all components
		 public byte protocol_flags; /// Retransmission / ACK flags
		};

		public const byte MAVLINK_MSG_ID_EXTENDED_MESSAGE_LEN = 3;
		public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_global_position_int_t
		{
		 public uint time_boot_ms; /// Timestamp (milliseconds since system boot)
		 public int lat; /// Latitude, expressed as * 1E7
		 public int lon; /// Longitude, expressed as * 1E7
		 public int alt; /// Altitude in meters, expressed as * 1000 (millimeters), above MSL
		 public int relative_alt; /// Altitude above ground in meters, expressed as * 1000 (millimeters)
		 public short vx; /// Ground X Speed (Latitude), expressed as m/s * 100
		 public short vy; /// Ground Y Speed (Longitude), expressed as m/s * 100
		 public short vz; /// Ground Z Speed (Altitude), expressed as m/s * 100
		 public ushort hdg; /// Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
		};

		public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN = 28;
		public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT = 52;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_global_position_setpoint_int_t
		{
		 public int latitude; /// WGS84 Latitude position in degrees * 1E7
		 public int longitude; /// WGS84 Longitude position in degrees * 1E7
		 public int altitude; /// WGS84 Altitude in meters * 1000 (positive for up)
		 public short yaw; /// Desired yaw angle in degrees * 100
		 public byte coordinate_frame; /// Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
		};

		public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT_LEN = 15;
		public const byte MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE = 101;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_global_vision_position_estimate_t
		{
		 public ulong usec; /// Timestamp (milliseconds)
		public float x; /// Global X position
		public float y; /// Global Y position
		public float z; /// Global Z position
		public float roll; /// Roll angle in rad
		public float pitch; /// Pitch angle in rad
		public float yaw; /// Yaw angle in rad
		};

		public const byte MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE_LEN = 32;
		public const byte MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN = 49;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_global_origin_t
		{
		 public int latitude; /// Latitude (WGS84), expressed as * 1E7
		 public int longitude; /// Longitude (WGS84), expressed as * 1E7
		 public int altitude; /// Altitude(WGS84), expressed as * 1000
		};

		public const byte MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN = 12;
		public const byte MAVLINK_MSG_ID_GPS_RAW_INT = 24;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_gps_raw_int_t
		{
		 public ulong time_usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		 public int lat; /// Latitude in 1E7 degrees
		 public int lon; /// Longitude in 1E7 degrees
		 public int alt; /// Altitude in 1E3 meters (millimeters) above MSL
		 public ushort eph; /// GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
		 public ushort epv; /// GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
		 public ushort vel; /// GPS ground speed (m/s * 100). If unknown, set to: 65535
		 public ushort cog; /// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
		 public byte fix_type; /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
		 public byte satellites_visible; /// Number of satellites visible. If unknown, set to 255
		};

		public const byte MAVLINK_MSG_ID_GPS_RAW_INT_LEN = 30;
		public const byte MAVLINK_MSG_ID_GPS_STATUS = 25;
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
		 public uint custom_mode; /// Navigation mode bitfield, see MAV_AUTOPILOT_CUSTOM_MODE ENUM for some examples. This field is autopilot-specific.
		 public byte type; /// Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
		 public byte autopilot; /// Autopilot type / class. defined in MAV_CLASS ENUM
		 public byte base_mode; /// System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
		 public byte system_status; /// System status flag, see MAV_STATUS ENUM
		 public byte mavlink_version; /// MAVLink version
		};

		public const byte MAVLINK_MSG_ID_HEARTBEAT_LEN = 9;
		public const byte MAVLINK_MSG_ID_HIL_CONTROLS = 91;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_hil_controls_t
		{
		 public ulong time_usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float roll_ailerons; /// Control output -1 .. 1
		public float pitch_elevator; /// Control output -1 .. 1
		public float yaw_rudder; /// Control output -1 .. 1
		public float throttle; /// Throttle 0 .. 1
		public float aux1; /// Aux 1, -1 .. 1
		public float aux2; /// Aux 2, -1 .. 1
		public float aux3; /// Aux 3, -1 .. 1
		public float aux4; /// Aux 4, -1 .. 1
		 public byte mode; /// System mode (MAV_MODE)
		 public byte nav_mode; /// Navigation mode (MAV_NAV_MODE)
		};

		public const byte MAVLINK_MSG_ID_HIL_CONTROLS_LEN = 42;
		public const byte MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW = 92;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_hil_rc_inputs_raw_t
		{
		 public ulong time_usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		 public ushort chan1_raw; /// RC channel 1 value, in microseconds
		 public ushort chan2_raw; /// RC channel 2 value, in microseconds
		 public ushort chan3_raw; /// RC channel 3 value, in microseconds
		 public ushort chan4_raw; /// RC channel 4 value, in microseconds
		 public ushort chan5_raw; /// RC channel 5 value, in microseconds
		 public ushort chan6_raw; /// RC channel 6 value, in microseconds
		 public ushort chan7_raw; /// RC channel 7 value, in microseconds
		 public ushort chan8_raw; /// RC channel 8 value, in microseconds
		 public ushort chan9_raw; /// RC channel 9 value, in microseconds
		 public ushort chan10_raw; /// RC channel 10 value, in microseconds
		 public ushort chan11_raw; /// RC channel 11 value, in microseconds
		 public ushort chan12_raw; /// RC channel 12 value, in microseconds
		 public byte rssi; /// Receive signal strength indicator, 0: 0%, 255: 100%
		};

		public const byte MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN = 33;
		public const byte MAVLINK_MSG_ID_HIL_STATE = 90;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_hil_state_t
		{
		 public ulong time_usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
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
		public const byte MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_local_position_ned_t
		{
		 public uint time_boot_ms; /// Timestamp (milliseconds since system boot)
		public float x; /// X Position
		public float y; /// Y Position
		public float z; /// Z Position
		public float vx; /// X Speed
		public float vy; /// Y Speed
		public float vz; /// Z Speed
		};

		public const byte MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN = 28;
		public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT = 51;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_local_position_setpoint_t
		{
		public float x; /// x position
		public float y; /// y position
		public float z; /// z position
		public float yaw; /// Desired yaw angle
		 public byte coordinate_frame; /// Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
		};

		public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_LEN = 17;
		public const byte MAVLINK_MSG_ID_MANUAL_CONTROL = 69;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_manual_control_t
		{
		public float roll; /// roll
		public float pitch; /// pitch
		public float yaw; /// yaw
		public float thrust; /// thrust
		 public byte target; /// The system to be controlled
		 public byte roll_manual; /// roll control enabled auto:0, manual:1
		 public byte pitch_manual; /// pitch auto:0, manual:1
		 public byte yaw_manual; /// yaw auto:0, manual:1
		 public byte thrust_manual; /// thrust auto:0, manual:1
		};

		public const byte MAVLINK_MSG_ID_MANUAL_CONTROL_LEN = 21;
		public const byte MAVLINK_MSG_ID_MEMORY_VECT = 249;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_memory_vect_t
		{
		 public ushort address; /// Starting address of the debug variables
		 public byte ver; /// Version code of the type variable. 0=unknown, type ignored and assumed public short. 1=as below
		 public byte type; /// Type code of the memory variables. for ver = 1: 0=16 x public short, 1=16 x public ushort, 2=16 x Q15, 3=16 x 1Q14
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=32)] 
		 public byte[] value; /// Memory contents at specified address
		};

		public const byte MAVLINK_MSG_ID_MEMORY_VECT_LEN = 36;
		public const byte MAVLINK_MSG_ID_MISSION_ACK = 47;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mission_ack_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte type; /// See MAV_MISSION_RESULT enum
		};

		public const byte MAVLINK_MSG_ID_MISSION_ACK_LEN = 3;
		public const byte MAVLINK_MSG_ID_MISSION_CLEAR_ALL = 45;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mission_clear_all_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN = 2;
		public const byte MAVLINK_MSG_ID_MISSION_COUNT = 44;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mission_count_t
		{
		 public ushort count; /// Number of mission items in the sequence
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_MISSION_COUNT_LEN = 4;
		public const byte MAVLINK_MSG_ID_MISSION_CURRENT = 42;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mission_current_t
		{
		 public ushort seq; /// Sequence
		};

		public const byte MAVLINK_MSG_ID_MISSION_CURRENT_LEN = 2;
		public const byte MAVLINK_MSG_ID_MISSION_ITEM = 39;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mission_item_t
		{
		public float param1; /// PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
		public float param2; /// PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
		public float param3; /// PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
		public float param4; /// PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
		public float x; /// PARAM5 / local: x position, global: latitude
		public float y; /// PARAM6 / y position: global: longitude
		public float z; /// PARAM7 / z position: global: altitude
		 public ushort seq; /// Sequence
		 public ushort command; /// The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte frame; /// The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
		 public byte current; /// false:0, true:1
		 public byte autocontinue; /// autocontinue to next wp
		};

		public const byte MAVLINK_MSG_ID_MISSION_ITEM_LEN = 37;
		public const byte MAVLINK_MSG_ID_MISSION_ITEM_REACHED = 46;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mission_item_reached_t
		{
		 public ushort seq; /// Sequence
		};

		public const byte MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN = 2;
		public const byte MAVLINK_MSG_ID_MISSION_REQUEST = 40;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mission_request_t
		{
		 public ushort seq; /// Sequence
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_MISSION_REQUEST_LEN = 4;
		public const byte MAVLINK_MSG_ID_MISSION_REQUEST_LIST = 43;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mission_request_list_t
		{
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN = 2;
		public const byte MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST = 37;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mission_request_partial_list_t
		{
		 public short start_index; /// Start index, 0 by default
		 public short end_index; /// End index, -1 by default (-1: send list to end). Else a valid index of the list
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN = 6;
		public const byte MAVLINK_MSG_ID_MISSION_SET_CURRENT = 41;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mission_set_current_t
		{
		 public ushort seq; /// Sequence
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_MISSION_SET_CURRENT_LEN = 4;
		public const byte MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST = 38;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_mission_write_partial_list_t
		{
		 public short start_index; /// Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
		 public short end_index; /// End index, equal or greater than start index.
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN = 6;
		public const byte MAVLINK_MSG_ID_NAMED_VALUE_FLOAT = 251;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_named_value_float_t
		{
		 public uint time_boot_ms; /// Timestamp (milliseconds since system boot)
		public float value; /// Floating point value
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=10)] 
		 public byte[] name; /// Name of the debug variable
		};

		public const byte MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN = 18;
		public const byte MAVLINK_MSG_ID_NAMED_VALUE_INT = 252;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_named_value_int_t
		{
		 public uint time_boot_ms; /// Timestamp (milliseconds since system boot)
		 public int value; /// Signed integer value
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=10)] 
		 public byte[] name; /// Name of the debug variable
		};

		public const byte MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN = 18;
		public const byte MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT = 62;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_nav_controller_output_t
		{
		public float nav_roll; /// Current desired roll in degrees
		public float nav_pitch; /// Current desired pitch in degrees
		public float alt_error; /// Current altitude error in meters
		public float aspd_error; /// Current airspeed error in meters/second
		public float xtrack_error; /// Current crosstrack error on x-y plane in meters
		 public short nav_bearing; /// Current desired heading in degrees
		 public short target_bearing; /// Bearing to current MISSION/target in degrees
		 public ushort wp_dist; /// Distance to active MISSION in meters
		};

		public const byte MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN = 26;
		public const byte MAVLINK_MSG_ID_OPTICAL_FLOW = 100;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_optical_flow_t
		{
		 public ulong time_usec; /// Timestamp (UNIX)
		public float ground_distance; /// Ground distance in meters
		 public short flow_x; /// Flow in pixels in x-sensor direction
		 public short flow_y; /// Flow in pixels in y-sensor direction
		 public byte sensor_id; /// Sensor ID
		 public byte quality; /// Optical flow quality / confidence. 0: bad, 255: maximum quality
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
		 public short param_index; /// Parameter index. Send -1 to use the param ID field as identifier
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=16)] 
		 public byte[] param_id; /// Onboard parameter id
		};

		public const byte MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN = 20;
		public const byte MAVLINK_MSG_ID_PARAM_SET = 23;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_param_set_t
		{
		public float param_value; /// Onboard parameter value
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=16)] 
		 public byte[] param_id; /// Onboard parameter id
		 public byte param_type; /// Onboard parameter type: see MAV_VAR enum
		};

		public const byte MAVLINK_MSG_ID_PARAM_SET_LEN = 23;
		public const byte MAVLINK_MSG_ID_PARAM_VALUE = 22;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_param_value_t
		{
		public float param_value; /// Onboard parameter value
		 public ushort param_count; /// Total number of onboard parameters
		 public ushort param_index; /// Index of this onboard parameter
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=16)] 
		 public byte[] param_id; /// Onboard parameter id
		 public byte param_type; /// Onboard parameter type: see MAV_VAR enum
		};

		public const byte MAVLINK_MSG_ID_PARAM_VALUE_LEN = 25;
		public const byte MAVLINK_MSG_ID_PING = 4;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_ping_t
		{
		 public ulong time_usec; /// Unix timestamp in microseconds
		 public uint seq; /// PING sequence
		 public byte target_system; /// 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
		 public byte target_component; /// 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
		};

		public const byte MAVLINK_MSG_ID_PING_LEN = 14;
		public const byte MAVLINK_MSG_ID_RAW_IMU = 27;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_raw_imu_t
		{
		 public ulong time_usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
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
		public const byte MAVLINK_MSG_ID_RAW_PRESSURE = 28;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_raw_pressure_t
		{
		 public ulong time_usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
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
		 public ushort chan1_raw; /// RC channel 1 value, in microseconds
		 public ushort chan2_raw; /// RC channel 2 value, in microseconds
		 public ushort chan3_raw; /// RC channel 3 value, in microseconds
		 public ushort chan4_raw; /// RC channel 4 value, in microseconds
		 public ushort chan5_raw; /// RC channel 5 value, in microseconds
		 public ushort chan6_raw; /// RC channel 6 value, in microseconds
		 public ushort chan7_raw; /// RC channel 7 value, in microseconds
		 public ushort chan8_raw; /// RC channel 8 value, in microseconds
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN = 18;
		public const byte MAVLINK_MSG_ID_RC_CHANNELS_RAW = 35;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_rc_channels_raw_t
		{
		 public uint time_boot_ms; /// Timestamp (milliseconds since system boot)
		 public ushort chan1_raw; /// RC channel 1 value, in microseconds
		 public ushort chan2_raw; /// RC channel 2 value, in microseconds
		 public ushort chan3_raw; /// RC channel 3 value, in microseconds
		 public ushort chan4_raw; /// RC channel 4 value, in microseconds
		 public ushort chan5_raw; /// RC channel 5 value, in microseconds
		 public ushort chan6_raw; /// RC channel 6 value, in microseconds
		 public ushort chan7_raw; /// RC channel 7 value, in microseconds
		 public ushort chan8_raw; /// RC channel 8 value, in microseconds
		 public byte port; /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
		 public byte rssi; /// Receive signal strength indicator, 0: 0%, 255: 100%
		};

		public const byte MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN = 22;
		public const byte MAVLINK_MSG_ID_RC_CHANNELS_SCALED = 34;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_rc_channels_scaled_t
		{
		 public uint time_boot_ms; /// Timestamp (milliseconds since system boot)
		 public short chan1_scaled; /// RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan2_scaled; /// RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan3_scaled; /// RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan4_scaled; /// RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan5_scaled; /// RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan6_scaled; /// RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan7_scaled; /// RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public short chan8_scaled; /// RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
		 public byte port; /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
		 public byte rssi; /// Receive signal strength indicator, 0: 0%, 255: 100%
		};

		public const byte MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN = 22;
		public const byte MAVLINK_MSG_ID_REQUEST_DATA_STREAM = 66;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_request_data_stream_t
		{
		 public ushort req_message_rate; /// The requested interval between two messages of this type
		 public byte target_system; /// The target requested to send the message stream.
		 public byte target_component; /// The target requested to send the message stream.
		 public byte req_stream_id; /// The ID of the requested data stream
		 public byte start_stop; /// 1 to start sending, 0 to stop sending.
		};

		public const byte MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN = 6;
		public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT = 59;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_roll_pitch_yaw_speed_thrust_setpoint_t
		{
		 public uint time_boot_ms; /// Timestamp in milliseconds since system boot
		public float roll_speed; /// Desired roll angular speed in rad/s
		public float pitch_speed; /// Desired pitch angular speed in rad/s
		public float yaw_speed; /// Desired yaw angular speed in rad/s
		public float thrust; /// Collective thrust, normalized to 0 .. 1
		};

		public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT_LEN = 20;
		public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT = 58;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_roll_pitch_yaw_thrust_setpoint_t
		{
		 public uint time_boot_ms; /// Timestamp in milliseconds since system boot
		public float roll; /// Desired roll angle in radians
		public float pitch; /// Desired pitch angle in radians
		public float yaw; /// Desired yaw angle in radians
		public float thrust; /// Collective thrust, normalized to 0 .. 1
		};

		public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT_LEN = 20;
		public const byte MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA = 55;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_safety_allowed_area_t
		{
		public float p1x; /// x position 1 / Latitude 1
		public float p1y; /// y position 1 / Longitude 1
		public float p1z; /// z position 1 / Altitude 1
		public float p2x; /// x position 2 / Latitude 2
		public float p2y; /// y position 2 / Longitude 2
		public float p2z; /// z position 2 / Altitude 2
		 public byte frame; /// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
		};

		public const byte MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA_LEN = 25;
		public const byte MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA = 54;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_safety_set_allowed_area_t
		{
		public float p1x; /// x position 1 / Latitude 1
		public float p1y; /// y position 1 / Longitude 1
		public float p1z; /// z position 1 / Altitude 1
		public float p2x; /// x position 2 / Latitude 2
		public float p2y; /// y position 2 / Longitude 2
		public float p2z; /// z position 2 / Altitude 2
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte frame; /// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
		};

		public const byte MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN = 27;
		public const byte MAVLINK_MSG_ID_SCALED_IMU = 26;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_scaled_imu_t
		{
		 public uint time_boot_ms; /// Timestamp (milliseconds since system boot)
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

		public const byte MAVLINK_MSG_ID_SCALED_IMU_LEN = 22;
		public const byte MAVLINK_MSG_ID_SCALED_PRESSURE = 29;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_scaled_pressure_t
		{
		 public uint time_boot_ms; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float press_abs; /// Absolute pressure (hectopascal)
		public float press_diff; /// Differential pressure 1 (hectopascal)
		 public short temperature; /// Temperature measurement (0.01 degrees celsius)
		};

		public const byte MAVLINK_MSG_ID_SCALED_PRESSURE_LEN = 14;
		public const byte MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 36;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_servo_output_raw_t
		{
		 public uint time_usec; /// Timestamp (since UNIX epoch or microseconds since system boot)
		 public ushort servo1_raw; /// Servo output 1 value, in microseconds
		 public ushort servo2_raw; /// Servo output 2 value, in microseconds
		 public ushort servo3_raw; /// Servo output 3 value, in microseconds
		 public ushort servo4_raw; /// Servo output 4 value, in microseconds
		 public ushort servo5_raw; /// Servo output 5 value, in microseconds
		 public ushort servo6_raw; /// Servo output 6 value, in microseconds
		 public ushort servo7_raw; /// Servo output 7 value, in microseconds
		 public ushort servo8_raw; /// Servo output 8 value, in microseconds
		 public byte port; /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
		};

		public const byte MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN = 21;
		public const byte MAVLINK_MSG_ID_SET_GLOBAL_POSITION_SETPOINT_INT = 53;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_global_position_setpoint_int_t
		{
		 public int latitude; /// WGS84 Latitude position in degrees * 1E7
		 public int longitude; /// WGS84 Longitude position in degrees * 1E7
		 public int altitude; /// WGS84 Altitude in meters * 1000 (positive for up)
		 public short yaw; /// Desired yaw angle in degrees * 100
		 public byte coordinate_frame; /// Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
		};

		public const byte MAVLINK_MSG_ID_SET_GLOBAL_POSITION_SETPOINT_INT_LEN = 15;
		public const byte MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN = 48;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_gps_global_origin_t
		{
		 public int latitude; /// global position * 1E7
		 public int longitude; /// global position * 1E7
		 public int altitude; /// global position * 1000
		 public byte target_system; /// System ID
		};

		public const byte MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN = 13;
		public const byte MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT = 50;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_local_position_setpoint_t
		{
		public float x; /// x position
		public float y; /// y position
		public float z; /// z position
		public float yaw; /// Desired yaw angle
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		 public byte coordinate_frame; /// Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
		};

		public const byte MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN = 19;
		public const byte MAVLINK_MSG_ID_SET_MODE = 11;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_mode_t
		{
		 public uint custom_mode; /// The new autopilot-specific mode. This field can be ignored by an autopilot.
		 public byte target_system; /// The system setting the mode
		 public byte base_mode; /// The new base mode
		};

		public const byte MAVLINK_MSG_ID_SET_MODE_LEN = 6;
		public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST = 57;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_roll_pitch_yaw_speed_thrust_t
		{
		public float roll_speed; /// Desired roll angular speed in rad/s
		public float pitch_speed; /// Desired pitch angular speed in rad/s
		public float yaw_speed; /// Desired yaw angular speed in rad/s
		public float thrust; /// Collective thrust, normalized to 0 .. 1
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
		};

		public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST_LEN = 18;
		public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST = 56;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_set_roll_pitch_yaw_thrust_t
		{
		public float roll; /// Desired roll angle in radians
		public float pitch; /// Desired pitch angle in radians
		public float yaw; /// Desired yaw angle in radians
		public float thrust; /// Collective thrust, normalized to 0 .. 1
		 public byte target_system; /// System ID
		 public byte target_component; /// Component ID
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
		public const byte MAVLINK_MSG_ID_STATUSTEXT = 253;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_statustext_t
		{
		 public byte severity; /// Severity of status, 0 = info message, 255 = critical fault
		[MarshalAs(
				    	UnmanagedType.ByValArray,
				    	SizeConst=50)] 
		 public byte[] text; /// Status text message, without null termination character
		};

		public const byte MAVLINK_MSG_ID_STATUSTEXT_LEN = 51;
		public const byte MAVLINK_MSG_ID_SYSTEM_TIME = 2;
		public const byte MAVLINK_MSG_ID_SYS_STATUS = 1;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_sys_status_t
		{
		 public uint onboard_control_sensors_present; /// Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
		 public uint onboard_control_sensors_enabled; /// Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
		 public uint onboard_control_sensors_health; /// Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
		 public ushort load; /// Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
		 public ushort voltage_battery; /// Battery voltage, in millivolts (1 = 1 millivolt)
		 public short current_battery; /// Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
		 public ushort drop_rate_comm; /// Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
		 public ushort errors_comm; /// Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
		 public ushort errors_count1; /// Autopilot-specific errors
		 public ushort errors_count2; /// Autopilot-specific errors
		 public ushort errors_count3; /// Autopilot-specific errors
		 public ushort errors_count4; /// Autopilot-specific errors
		 public byte battery_remaining; /// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
		};

		public const byte MAVLINK_MSG_ID_SYS_STATUS_LEN = 31;
		public const byte MAVLINK_MSG_ID_VFR_HUD = 74;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_vfr_hud_t
		{
		public float airspeed; /// Current airspeed in m/s
		public float groundspeed; /// Current ground speed in m/s
		public float alt; /// Current altitude (MSL), in meters
		public float climb; /// Current climb rate in meters/second
		 public short heading; /// Current heading in degrees, in compass units (0..360, 0=north)
		 public ushort throttle; /// Current throttle setting in integer percent, 0 to 100
		};

		public const byte MAVLINK_MSG_ID_VFR_HUD_LEN = 20;
		public const byte MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE = 104;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_vicon_position_estimate_t
		{
		 public ulong usec; /// Timestamp (milliseconds)
		public float x; /// Global X position
		public float y; /// Global Y position
		public float z; /// Global Z position
		public float roll; /// Roll angle in rad
		public float pitch; /// Pitch angle in rad
		public float yaw; /// Yaw angle in rad
		};

		public const byte MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN = 32;
		public const byte MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE = 102;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_vision_position_estimate_t
		{
		 public ulong usec; /// Timestamp (milliseconds)
		public float x; /// Global X position
		public float y; /// Global Y position
		public float z; /// Global Z position
		public float roll; /// Roll angle in rad
		public float pitch; /// Pitch angle in rad
		public float yaw; /// Yaw angle in rad
		};

		public const byte MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN = 32;
		public const byte MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE = 103;
		[StructLayout(LayoutKind.Sequential,Pack=1)]
		public struct __mavlink_vision_speed_estimate_t
		{
		 public ulong usec; /// Timestamp (milliseconds)
		public float x; /// Global X speed
		public float y; /// Global Y speed
		public float z; /// Global Z speed
		};

		public const byte MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN = 20;
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

Type[] mavstructs = new Type[] {typeof( __mavlink_heartbeat_t) ,typeof( __mavlink_sys_status_t) ,null ,null ,typeof( __mavlink_ping_t) ,typeof( __mavlink_change_operator_control_t) ,typeof( __mavlink_change_operator_control_ack_t) ,typeof( __mavlink_auth_key_t) ,null ,null ,null ,typeof( __mavlink_set_mode_t) ,null ,null ,null ,null ,null ,null ,null ,null ,typeof( __mavlink_param_request_read_t) ,typeof( __mavlink_param_request_list_t) ,typeof( __mavlink_param_value_t) ,typeof( __mavlink_param_set_t) ,typeof( __mavlink_gps_raw_int_t) ,typeof( __mavlink_gps_status_t) ,typeof( __mavlink_scaled_imu_t) ,typeof( __mavlink_raw_imu_t) ,typeof( __mavlink_raw_pressure_t) ,typeof( __mavlink_scaled_pressure_t) ,typeof( __mavlink_attitude_t) ,typeof( __mavlink_attitude_quaternion_t) ,typeof( __mavlink_local_position_ned_t) ,typeof( __mavlink_global_position_int_t) ,typeof( __mavlink_rc_channels_scaled_t) ,typeof( __mavlink_rc_channels_raw_t) ,typeof( __mavlink_servo_output_raw_t) ,typeof( __mavlink_mission_request_partial_list_t) ,typeof( __mavlink_mission_write_partial_list_t) ,typeof( __mavlink_mission_item_t) ,typeof( __mavlink_mission_request_t) ,typeof( __mavlink_mission_set_current_t) ,typeof( __mavlink_mission_current_t) ,typeof( __mavlink_mission_request_list_t) ,typeof( __mavlink_mission_count_t) ,typeof( __mavlink_mission_clear_all_t) ,typeof( __mavlink_mission_item_reached_t) ,typeof( __mavlink_mission_ack_t) ,typeof( __mavlink_set_gps_global_origin_t) ,typeof( __mavlink_gps_global_origin_t) ,typeof( __mavlink_set_local_position_setpoint_t) ,typeof( __mavlink_local_position_setpoint_t) ,typeof( __mavlink_global_position_setpoint_int_t) ,typeof( __mavlink_set_global_position_setpoint_int_t) ,typeof( __mavlink_safety_set_allowed_area_t) ,typeof( __mavlink_safety_allowed_area_t) ,typeof( __mavlink_set_roll_pitch_yaw_thrust_t) ,typeof( __mavlink_set_roll_pitch_yaw_speed_thrust_t) ,typeof( __mavlink_roll_pitch_yaw_thrust_setpoint_t) ,typeof( __mavlink_roll_pitch_yaw_speed_thrust_setpoint_t) ,null ,null ,typeof( __mavlink_nav_controller_output_t) ,null ,typeof( __mavlink_state_correction_t) ,null ,typeof( __mavlink_request_data_stream_t) ,typeof( __mavlink_data_stream_t) ,null ,typeof( __mavlink_manual_control_t) ,typeof( __mavlink_rc_channels_override_t) ,null ,null ,null ,typeof( __mavlink_vfr_hud_t) ,null ,typeof( __mavlink_command_long_t) ,typeof( __mavlink_command_ack_t) ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,typeof( __mavlink_hil_state_t) ,typeof( __mavlink_hil_controls_t) ,typeof( __mavlink_hil_rc_inputs_raw_t) ,null ,null ,null ,null ,null ,null ,null ,typeof( __mavlink_optical_flow_t) ,typeof( __mavlink_global_vision_position_estimate_t) ,typeof( __mavlink_vision_position_estimate_t) ,typeof( __mavlink_vision_speed_estimate_t) ,typeof( __mavlink_vicon_position_estimate_t) ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,typeof( __mavlink_sensor_offsets_t) ,typeof( __mavlink_set_mag_offsets_t) ,typeof( __mavlink_meminfo_t) ,typeof( __mavlink_ap_adc_t) ,typeof( __mavlink_digicam_configure_t) ,typeof( __mavlink_digicam_control_t) ,typeof( __mavlink_mount_configure_t) ,typeof( __mavlink_mount_control_t) ,typeof( __mavlink_mount_status_t) ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,null ,typeof( __mavlink_memory_vect_t) ,typeof( __mavlink_debug_vect_t) ,typeof( __mavlink_named_value_float_t) ,typeof( __mavlink_named_value_int_t) ,typeof( __mavlink_statustext_t) ,typeof( __mavlink_debug_t) ,typeof( __mavlink_extended_message_t) ,null ,};

	}
	#endif
}


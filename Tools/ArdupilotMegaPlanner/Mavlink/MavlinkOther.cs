using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega
{
    public partial class MAVLink
    {

#if !MAVLINK10
        enum MAV_CLASS
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

        public enum MAV_ACTION
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

        public enum MAV_MODE
        {
            MAV_MODE_UNINIT = 0,     /// System is in undefined state
            MAV_MODE_LOCKED = 1,     /// Motors are blocked, system is safe
            MAV_MODE_MANUAL = 2,     /// System is allowed to be active, under manual (RC) control
            MAV_MODE_GUIDED = 3,     /// System is allowed to be active, under autonomous control, manual setpoint
            MAV_MODE_AUTO = 4,     /// System is allowed to be active, under autonomous control and navigation
            MAV_MODE_TEST1 = 5,     /// Generic test mode, for custom use
            MAV_MODE_TEST2 = 6,     /// Generic test mode, for custom use
            MAV_MODE_TEST3 = 7,     /// Generic test mode, for custom use
            MAV_MODE_READY = 8,     /// System is ready, motors are unblocked, but controllers are inactive
            MAV_MODE_RC_TRAINING = 9 /// System is blocked, only RC valued are read and reported back
        };

        public enum MAV_STATE
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

        public enum MAV_NAV
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

        public enum MAV_TYPE
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

        public enum MAV_AUTOPILOT_TYPE
        {
            MAV_AUTOPILOT_GENERIC = 0,
            MAV_AUTOPILOT_PIXHAWK = 1,
            MAV_AUTOPILOT_SLUGS = 2,
            MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
            MAV_AUTOPILOT_NONE = 4
        };

        public enum MAV_COMPONENT
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

        public enum MAV_FRAME
        {
            GLOBAL = 0,
            LOCAL = 1,
            MISSION = 2,
            GLOBAL_RELATIVE_ALT = 3,
            LOCAL_ENU = 4
        };
#endif
    }
}

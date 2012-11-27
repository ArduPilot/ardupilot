//  CONFIG FILE FOR APM_Rover project by Jean-Louis Naudin
//
#define GCS_PORT	    3
#define GCS_PROTOCOL	    GCS_PROTOCOL_MAVLINK   // QGroundControl protocol
//#define GCS_PROTOCOL	    GCS_PROTOCOL_NONE      // No GCS protocol to save memory
#define HIL_MODE            HIL_MODE_DISABLED

#define MAV_SYSTEM_ID	    1

// Add a ground start delay in seconds
//#define GROUND_START_DELAY  1

#define AIRSPEED_SENSOR	    DISABLED

#define SONAR_ENABLED       DISABLED
#define SONAR_TRIGGER       200        // trigger distance in cm 

#if LITE == DISABLED
  #define LOGGING_ENABLED   ENABLED
#endif

// for an accurate navigation a magnetometer must be used with the APM1 
#define MAGNETOMETER	    ENABLED
//#define MAG_ORIENTATION		AP_COMPASS_COMPONENTS_UP_PINS_FORWARD
//#define PARAM_DECLINATION       0.18  // Paris

//////////////////////////////////////////////////////////////////////////////
// Serial port speeds.
//
#define SERIAL0_BAUD        115200
#define SERIAL3_BAUD        57600

//////////////////////////////////////////////////////////////////////////////
// GPS_PROTOCOL 
#define GPS_PROTOCOL          GPS_PROTOCOL_AUTO

#define CH7_OPTION	      CH7_SAVE_WP

#define FLIGHT_MODE_1         AUTO         // pos 0 ---
#define FLIGHT_MODE_2         AUTO         // pos 1
#define FLIGHT_MODE_3         LEARNING    // pos 2
#define FLIGHT_MODE_4         LEARNING    // pos 3 --- 
#define FLIGHT_MODE_5         MANUAL       // pos 4
#define FLIGHT_MODE_6         MANUAL       // pos 5 ---

#define MANUAL_LEVEL	      DISABLED

#define TURN_GAIN		5

#define CLOSED_LOOP_NAV       ENABLED     // set to ENABLED if closed loop navigation else set to DISABLED (Return To Lauch)

#define MAX_DIST             50  //300       // max distance (in m) for the HEADALT mode
#define SARSEC_BRANCH        50              // Long branch of the SARSEC pattern
/*
During straight lines if the speed booster is enabled, after passing the Wp, 
the speed is multplied by a speed factor ROV_BOOSTER   =  2     
(i.e. this is my tested value... so the required speed will be 2 x 4 = 8 m/s in straight lines), 
the when the rover approach the wp, it slow down to 4 m/s (TRIM_ARSPD_CM)...
This feature works only if the ROV_AWPR_NAV is set to 0
*/

#define BOOSTER              2    // booster factor x1 = 1 or x2 = 2
#define AUTO_WP_RADIUS       DISABLED
#define AIRSPEED_CRUISE      4    // 4m/s
#define THROTTLE_SLEW_LIMIT  2    // set to 2 for a smooth acceleration by 0.2 step

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// FLIGHT AND NAVIGATION CONTROL
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// AIRSPEED_CRUISE                          OPTIONAL
//
// The speed in metres per second to maintain during cruise.  The default
// is 10m/s, which is a conservative value suitable for relatively small,
// light aircraft.
//
#define GSBOOST             0  
#define NUDGE_OFFSET	    0  
#define MIN_GNDSPEED       3

//////////////////////////////////////////////////////////////////////////////
// FLY_BY_WIRE_B airspeed control (also used for throttle "nudging" in AUTO)
//
// AIRSPEED_FBW_MIN                         OPTIONAL
// AIRSPEED_FBW_MAX                         OPTIONAL
//
// Airspeed corresponding to minimum and maximum throttle in Fly By Wire B mode.
// The defaults are 6 and 30 metres per second.
//
// AIRSPEED_FBW_MAX also sets the maximum airspeed that the cruise airspeed can be "nudged" to in AUTO mode when ENABLE_STICK_MIXING is set.
// In AUTO the cruise airspeed can be increased between AIRSPEED_CRUISE and AIRSPEED_FBW_MAX by positioning the throttle
// stick in the top 1/2 of its range.  Throttle stick in the bottom 1/2 provide regular AUTO control.
//
#define AIRSPEED_FBW_MIN    6
#define AIRSPEED_FBW_MAX    35
//

//////////////////////////////////////////////////////////////////////////////
// Servo mapping
//
// THROTTLE_MIN                             OPTIONAL
//
// The minimum throttle setting to which the autopilot will reduce the
// throttle while descending.  The default is zero, which is
// suitable for aircraft with a steady power-off glide.  Increase this
// value if your aircraft needs throttle to maintain a stable descent in
// level flight.
//
// THROTTLE_CRUISE                          OPTIONAL
//
// The approximate throttle setting to achieve AIRSPEED_CRUISE in level flight.
// The default is 45%, which is reasonable for a modestly powered aircraft.
//
// THROTTLE_MAX                             OPTIONAL
//
// The maximum throttle setting the autopilot will apply.  The default is 75%.
// Reduce this value if your aicraft is overpowered, or has complex flight
// characteristics at high throttle settings.
//
#define THROTTLE_MIN		0   // percent
#define THROTTLE_CRUISE		3   // 40
#define THROTTLE_MAX		100

//////////////////////////////////////////////////////////////////////////////
// AUTO_TRIM                                OPTIONAL
//
// ArduPilot Mega can update its trim settings by looking at the
// radio inputs when switching out of MANUAL mode.  This allows you to
// manually trim your aircraft before switching to an assisted mode, but it
// also means that you should avoid switching out of MANUAL while you have
// any control stick deflection.
//
// The default is to enable AUTO_TRIM.
//
#define AUTO_TRIM           ENABLED
#define THROTTLE_FAILSAFE   DISABLED

//////////////////////////////////////////////////////////////////////////////
// Autopilot control limits
//
// HEAD_MAX                                 OPTIONAL
//
// The maximum commanded bank angle in either direction.
// The default is 45 degrees.  Decrease this value if your aircraft is not
// stable or has difficulty maintaining altitude in a steep bank.
//
// PITCH_MAX                                OPTIONAL
//
// The maximum commanded pitch up angle.
// The default is 15 degrees.  Care should be taken not to set this value too
// large, as the aircraft may stall.
//
// PITCH_MIN
//
// The maximum commanded pitch down angle.  Note that this value must be
// negative.  The default is -25 degrees.  Care should be taken not to set
// this value too large as it may result in overspeeding the aircraft.
//
// PITCH_TARGET
//
// The target pitch for cruise flight.  When the APM measures this pitch
// value, the pitch error will be calculated to be 0 for the pitch PID
// control loop.
//
#define HEAD_MAX            80
#define PITCH_MAX           15
#define PITCH_MIN           -20  //-25
#define PITCH_TARGET        0

//////////////////////////////////////////////////////////////////////////////
// Attitude control gains
//
// Tuning values for the attitude control PID loops.
//
// The P term is the primary tuning value.  This determines how the control
// deflection varies in proportion to the required correction.
//
// The I term is used to help control surfaces settle.  This value should
// normally be kept low.
//
// The D term is used to control overshoot.  Avoid using or adjusting this
// term if you are not familiar with tuning PID loops.  It should normally
// be zero for most aircraft.
//
// Note: When tuning these values, start with changes of no more than 25% at
// a time.
//
// SERVO_ROLL_P                             OPTIONAL
// SERVO_ROLL_I                             OPTIONAL
// SERVO_ROLL_D                             OPTIONAL
//
// P, I and D terms for roll control.  Defaults are 0.4, 0, 0.
//
// SERVO_ROLL_INT_MAX                       OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. crosstracking).
// Default is 5 degrees.
//
// ROLL_SLEW_LIMIT                          EXPERIMENTAL
//
// Limits the slew rate of the roll control in degrees per second.  If zero,
// slew rate is not limited.  Default is to not limit the roll control slew rate.
// (This feature is currently not implemented.)
//
// SERVO_PITCH_P                            OPTIONAL
// SERVO_PITCH_I                            OPTIONAL
// SERVO_PITCH_D                            OPTIONAL
//
// P, I and D terms for the pitch control.  Defaults are 0.6, 0, 0.
//
// SERVO_PITCH_INT_MAX                      OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. native flight
// AoA).  If you find this value is insufficient, consider adjusting the AOA
// parameter.
// Default is 5 degrees.
//
// PITCH_COMP                               OPTIONAL
//
// Adds pitch input to compensate for the loss of lift due to roll control.
// Default is 0.20 (20% of roll control also applied to pitch control).
//
// SERVO_YAW_P                              OPTIONAL
// SERVO_YAW_I                              OPTIONAL
// SERVO_YAW_D                              OPTIONAL
//
// P, I and D terms for the YAW control.  Defaults are 0., 0., 0.
// Note units of this control loop are unusual.  PID input is in m/s**2.
//
// SERVO_YAW_INT_MAX                        OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. crosstracking).
// Default is 0.
//
// RUDDER_MIX                               OPTIONAL
//
// Roll to yaw mixing.  This allows for co-ordinated turns.
// Default is 0.50 (50% of roll control also applied to yaw control.)
//
#define SERVO_ROLL_P        0.0
#define SERVO_ROLL_I        0.0
#define SERVO_ROLL_D        0.0
#define SERVO_ROLL_INT_MAX  5
#define ROLL_SLEW_LIMIT     0
#define SERVO_PITCH_P       0.0   
#define SERVO_PITCH_I       0.0
#define SERVO_PITCH_D       0.0
#define SERVO_PITCH_INT_MAX 5
#define PITCH_COMP          0.0
#define SERVO_YAW_P         0.0		// Default is zero.  A suggested value if you want to use this parameter is 0.5
#define SERVO_YAW_I         0.0
#define SERVO_YAW_D         0.0
#define SERVO_YAW_INT_MAX   5
#define RUDDER_MIX          0.0
//
//////////////////////////////////////////////////////////////////////////////
// Navigation control gains
//
// Tuning values for the navigation control PID loops.
//
// The P term is the primary tuning value.  This determines how the control
// deflection varies in proportion to the required correction.
//
// The I term is used to control drift.
//
// The D term is used to control overshoot.  Avoid adjusting this term if
// you are not familiar with tuning PID loops.
//
// Note: When tuning these values, start with changes of no more than 25% at
// a time.
//
// NAV_ROLL_P                               OPTIONAL
// NAV_ROLL_I                               OPTIONAL
// NAV_ROLL_D                               OPTIONAL
//
// P, I and D terms for navigation control over roll, normally used for
// controlling the aircraft's course.  The P term controls how aggressively
// the aircraft will bank to change or hold course.
// Defaults are 0.7, 0.0, 0.02.
//
// NAV_ROLL_INT_MAX                         OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. crosstracking).
// Default is 5 degrees.
//
// NAV_PITCH_ASP_P                          OPTIONAL
// NAV_PITCH_ASP_I                          OPTIONAL
// NAV_PITCH_ASP_D                          OPTIONAL
//
// P, I and D terms for pitch adjustments made to maintain airspeed.
// Defaults are 0.65, 0, 0.
//
// NAV_PITCH_ASP_INT_MAX                    OPTIONAL
//
// Maximum pitch offset due to the integral.  This limits the control
// output from being overdriven due to a persistent offset (eg. inability
// to maintain the programmed airspeed).
// Default is 5 degrees.
//
// NAV_PITCH_ALT_P                          OPTIONAL
// NAV_PITCH_ALT_I                          OPTIONAL
// NAV_PITCH_ALT_D                          OPTIONAL
//
// P, I and D terms for pitch adjustments made to maintain altitude.
// Defaults are 0.65, 0, 0.
//
// NAV_PITCH_ALT_INT_MAX                    OPTIONAL
//
// Maximum pitch offset due to the integral.  This limits the control
// output from being overdriven due to a persistent offset (eg. inability
// to maintain the programmed altitude).
// Default is 5 meters.
//
#define NAV_ROLL_P          0.7
#define NAV_ROLL_I          0.001
#define NAV_ROLL_D          0.06
#define NAV_ROLL_INT_MAX    5

#define NAV_PITCH_ASP_P     0.0 
#define NAV_PITCH_ASP_I     0.0
#define NAV_PITCH_ASP_D     0.0
#define NAV_PITCH_ASP_INT_MAX 5

#define NAV_PITCH_ALT_P     0.0
#define NAV_PITCH_ALT_I     0.0
#define NAV_PITCH_ALT_D     0.0
#define NAV_PITCH_ALT_INT_MAX 5

//////////////////////////////////////////////////////////////////////////////
// Energy/Altitude control gains
//
// The Energy/altitude control system uses throttle input to control aircraft
// altitude.
//
// The P term is the primary tuning value.  This determines how the throttle
// setting varies in proportion to the required correction.
//
// The I term is used to compensate for small offsets.
//
// The D term is used to control overshoot.  Avoid adjusting this term if
// you are not familiar with tuning PID loops.
//
// Note units of this control loop are unusual.  PID input is in m**2/s**2.
//
// THROTTLE_TE_P                            OPTIONAL
// THROTTLE_TE_I                            OPTIONAL
// THROTTLE_TE_D                            OPTIONAL
//
// P, I and D terms for throttle adjustments made to control altitude.
// Defaults are 0.5, 0, 0.
//
// THROTTLE_TE_INT_MAX                      OPTIONAL
//
// Maximum throttle input due to the integral term.  This limits the
// throttle from being overdriven due to a persistent offset (e.g.
// inability to maintain the programmed altitude).
// Default is 20%.
//
// THROTTLE_SLEW_LIMIT                      OPTIONAL
//
// Limits the slew rate of the throttle, in percent per second.  Helps
// avoid sudden throttle changes, which can destabilise the aircraft.
// A setting of zero disables the feature.
// Default is zero (disabled).
//
// P_TO_T                                   OPTIONAL
//
// Pitch to throttle feed-forward gain.  Default is 0.
//
// T_TO_P                                   OPTIONAL
//
// Throttle to pitch feed-forward gain.  Default is 0.
//
#define THROTTLE_TE_P       0.1
#define THROTTLE_TE_I       0.0
#define THROTTLE_TE_D       0.0
#define THROTTLE_TE_INT_MAX 20
#define P_TO_T              0.0
#define T_TO_P              0

//////////////////////////////////////////////////////////////////////////////
// Crosstrack compensation
//
// XTRACK_GAIN                              OPTIONAL
//
// Crosstrack compensation in degrees per metre off track.
// Default value is 1.0 degrees per metre.  Values lower than 0.001 will
// disable crosstrack compensation.
//
// XTRACK_ENTRY_ANGLE                       OPTIONAL
//
// Maximum angle used to correct for track following.
// Default value is 30 degrees.
//
#define XTRACK_GAIN         1  // deg/m
#define XTRACK_ENTRY_ANGLE  20 // deg

/////////////////////////////////////////////////////////////////////////////
// Navigation defaults
//
// WP_RADIUS_DEFAULT                        OPTIONAL
//
// When the user performs a factory reset on the APM, set the waypoint radius
// (the radius from a target waypoint within which the APM will consider
// itself to have arrived at the waypoint) to this value in meters.  This is
// mainly intended to allow users to start using the APM without running the
// WaypointWriter first.
//
#define WP_RADIUS_DEFAULT       1   // meters

//////////////////////////////////////////////////////////////////////////////
// INPUT_VOLTAGE                            OPTIONAL
//
// In order to have accurate pressure and battery voltage readings, this
// value should be set to the voltage measured on the 5V rail on the oilpan.
//
// See the manual for more details.  The default value should be close.
//
#define INPUT_VOLTAGE 5.2
//

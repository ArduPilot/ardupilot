// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//---------------------------------------------------------------------------
//  Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------
#include <stdlib.h>
#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>              // ArduPilot Mega Vector/Matrix math Library
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_AHRS.h>             //  Attitude and Heading Reference System
#include <AP_Compass.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_GPS.h>
#include <AP_Notify.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Vehicle.h>
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <Filter.h>             // library to Filter Sensor Data
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <PID.h>                // ArduPilot Mega RC Library

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
//---------------------------------------------------------------------------
static AP_GPS_UBLOX g_gps_driver;
static GPS         *g_gps = &g_gps_driver;

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
static SITL sitl;
#endif

//---------------------------------------------------------------------------
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_InertialSensor_MPU6000 ins;
AP_Compass_HMC5843 compass;
#else
AP_InertialSensor_HIL ins;
AP_Compass_HIL compass;
#endif

AP_AHRS_DCM  ahrs(&ins, g_gps);
//---------------------------------------------------------------------------
#include <DataFlash.h>       // Flash card access

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
DataFlash_SITL DataFlash;
#endif

//---------------------------------------------------------------------------
AP_HAL::AnalogSource* Ch0;
AP_HAL::AnalogSource* Ch1;
AP_HAL::AnalogSource* Ch2;
AP_HAL::AnalogSource* Ch3;
AP_HAL::AnalogSource* Ch4;
AP_HAL::AnalogSource* Vcc;
float adc0 =  0.0; 
float adc1 =  0.0; 
float adc2 =  0.0;
float adc3 =  0.0;
float adc4 =  0.0;
float vcc  =  0.0; 
//---------------------------------------------------------------------------
static int      battery_volt_pin;
static int      battery_curr_pin;
static float 	battery_voltage;
static float	current_amps;
static float	current_total;
//---------------------------------------------------------------------------
// Some constants
#define pi      3.141592653589793f
//---------------------------------------------------------------------------
struct wp_class{
    float lon;        // [rad]
    float lat;        // [rad]
    float alt;        // [m]
};
//---------------------------------------------------------------------------
struct CCD{
  float course;       // [rad]
  float dist;         // [nm]
};
//---------------------------------------------------------------------------
struct gps_fix {
    float     lat;     // [rad]
    float     lon;     // [rad]
    float     alt;     // [m]
    float     cog;     // [rad]
    float     sog;     // [knots]
    uint32_t  time;    // []
    int       nsats;   // [-] number of satellites 
    char      status;  //uint8_t
};
//---------------------------------------------------------------------------

static float heading       = 0.0;      // [rad] Ship course (=cc or cog depending on speed)
static float pitch         = 0.0;      // [rad] Ship pitch angle
static float roll          = 0.0;      // [rad] Ship heel angle
static float cc            = 0.0;      // [rad] Magnetic compass course
static float depth         = 0.0;      // [m]   
static float rpm           = 0.0;      // [rpm] 
static float k_xtrack      = 0.0;      // 0:steer directly to wp, 1:steer to closest point on rhumb-line

static float target_ctt    = 0.0;      // [rad] Compass course
static float target_dtt    = 0.0;      // [rad] Compass course
static float target_depth  = 0.0;      // [m]   
static float target_rpm    = 0.0;      // [m]   
static float ctrl_cc       = 0;        // [rad]  
static float ctrl_depth    = 0;        // [m]

static float err_cc       = 0.0;       // For PID-conrol
static float err_depth    = 0.0;       // For PID-conrol
static float err_alt      = 0.0;       // For PID-conrol
static float err_rpm      = 0.0;       // For PID-conrol
static float err_roll     = 0.0;       // For PID-conrol
static float err_pitch    = 0.0;       // For PID-conrol

static int   pwm_cc       = 1500;
static int   pwm_depth    = 1500;
static int   pwm_port     = 1500;
static int   pwm_stbd     = 1500;
static int   pwm_rpm      = 1000;          
static int   pwm_roll     = 1500;          
static int   pwm_pitch    = 1500;          
static int   pwm_rudder   = 1500;          
static int   pwm_motor    = 1000; 

static int   packets_written = 0;     // To mem-card     
      
//---------------------------------------------------------------------------

static uint32_t ms_start; //
static Vector3f accel;    // 
static float acc_norm;    // if static should be 9.81
static Vector3f gyro;     //
static PID pid_1;         // pid is the instance of class PID
static PID pid_2;         // pid is the instance of class PID
static PID pid_3;         // pid is the instance of class PID
static PID pid_4;         // pid is the instance of class PID

static wp_class  current_pos={18,59,0};  // {lat, lon, alt}
static wp_class  mission_start_pos;      // {lat, lon, alt}

static uint32_t birth_ms;
static uint32_t time_since_birth_ms;
static uint32_t mission_start_ms;
static uint32_t mission_ms;
static uint32_t time_ms;
static uint32_t loop_timer_ms;
static uint32_t mission_update_timer_ms;
static uint32_t last_ctrl_ms;
static uint32_t last_log_ms;
static uint32_t last_rpm_PID_ms;
static uint32_t last_cc_PID_ms;
static uint32_t last_data_sent_ms;
static uint32_t last_GPS_fix;
static uint32_t last_adc_update;
static uint32_t last_telemetry_check;

static char      ctrl_mode         = 'i';    // idle mode
static char      craft_type        = 'A';    // Default craft type
static bool      RC_feedthrough    = false;  //
static bool      continously_send  = false;  // Telemetry
static float     sog_threshold     = 999;    // [m/s] speed when to switch from Heading=Compass to Heading=SOG 
static gps_fix   gps;                        // Contains nice data (in proper units) from latest fix

struct CC_leg{
    float duration;   // [sec]
    float course;     // [rad]
    float depth;      // [m]
    float rpm;        // [-]
};
static CC_leg   CC_mission[50];

struct GPS_leg{
    float lon;        // [rad]
    float lat;        // [rad]
    float depth;      // [m]
    float rpm;        // [-]
    float wp_radius;  // [m] 
};
static GPS_leg  GPS_mission[50];

static int        Nlegs_cc;
static int        Nlegs_GPS;
static int        current_leg_nr;
//-------------------------------------------------------------------------------
static uint32_t ms() {return hal.scheduler->millis();}
static void wait_ms(uint32_t ms_to_wait){hal.scheduler->delay(ms_to_wait);}
//-------------------------------------------------------------------------------
void setup(void)
{   
    hal.console->println("\n\n---------------------------------------------------------------------------------");
    ms_start = hal.scheduler->millis();
    hal.console->println("\n << MSY goes Autonomous. Bitte Anschnallen!  >>\n");
    hal.console->println("Starting setup:");
    init_GPS();  //wait_ms(5000);update_GPS();print_GPS();
    setup_Flash();
    init_AHRS();

    // lower the rate at which the accelerometers and GPS corrects the
    // attitude
    // this will smooth out the roll/pitch when still
    ahrs._kp.set(0.1);

    init_analog();
    setup_default_CC_mission();
    setup_default_GPS_mission();
    AUV_craft_setup();
    print_settings();
    print_main_menu();
    //;
    hal.console->print("Leaving setup.\n");
    hal.console->print("Now it is up to you :-) \n");
}

//---------------------------------------------------------------------------
void loop(void)
{    
    // This loop is running for eternity regardless of Craft type etc
    // ---------------------------------------------------------------
    time_ms               = hal.scheduler->millis();       // The official time keeper
    time_since_birth_ms   = time_ms-birth_ms;              // Since I woke up :-)
    update_GPS();
    read_analogue_channels();                              // 
    update_AHRS();                                         // 
    update_heading();  // Chooses if CC or GPS sets heading
    if (ctrl_mode =='c')  { CC_mission_manager();       }  // 
    if (ctrl_mode =='g')  { GPS_mission_manager();      }  // 
    if (ctrl_mode =='d')  { Deviation_mission_manager();}  // 
    if (craft_type=='A')  { AUV();                      }  // We control an AUV
    if (craft_type=='S')  { Solar();                    }  // We control a Solar powered boat
    if (craft_type=='P')  { Plane();                    }  // We control a Plane
    if (craft_type=='K')  { Kite();                     }  // We control a Kite 
    if (hal.console->available()) { if (hal.console->read()=='#') {parse_incoming_telemetry(); }}
}
//-------------------------------------------------------------------------------
AP_HAL_MAIN();


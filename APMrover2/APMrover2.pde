/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduRover v2.49"
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* 
   This is the APMrover2 firmware. It was originally derived from
   ArduPlane by Jean-Louis Naudin (JLN), and then rewritten after the
   AP_HAL merge by Andrew Tridgell

   Maintainer: Andrew Tridgell

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Jean-Louis Naudin

   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier 

   APMrover alpha version tester: Franco Borasio, Daniel Chapelat... 

   Please contribute your ideas! See http://dev.ardupilot.com for details
*/

// Radio setup:
// APM INPUT (Rec = receiver)
// Rec ch1: Steering
// Rec ch2: not used
// Rec ch3: Throttle
// Rec ch4: not used
// Rec ch5: not used
// Rec ch6: not used
// Rec ch7: Option channel to 2 position switch
// Rec ch8: Mode channel to 6 position switch
// APM OUTPUT
// Ch1: Wheel servo (direction)
// Ch2: not used
// Ch3: to the motor ESC
// Ch4: not used

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor.h> // Inertial Sensor (uncalibated IMU) Library
#include <AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <AP_NavEKF.h>
#include <AP_Mission.h>     // Mission command library
#include <AP_Rally.h>
#include <AP_Terrain.h>
#include <PID.h>            // PID library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>	// Range finder library
#include <Filter.h>			// Filter library
#include <Butter.h>			// Filter library - butterworth filter
#include <AP_Buffer.h>      // FIFO buffer library
#include <ModeFilter.h>		// Mode Filter from Filter library
#include <AverageFilter.h>	// Mode Filter from Filter library
#include <AP_Relay.h>       // APM relay
#include <AP_ServoRelayEvents.h>
#include <AP_Mount.h>		// Camera/Antenna mount
#include <AP_Camera.h>		// Camera triggering
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_SerialManager.h>   // Serial manager library
#include <AP_Airspeed.h>    // needed for AHRS build
#include <AP_Vehicle.h>     // needed for AHRS build
#include <DataFlash.h>
#include <AP_RCMapper.h>        // RC input mapping library
#include <SITL.h>
#include <AP_Scheduler.h>       // main loop scheduler
#include <stdarg.h>
#include <AP_Navigation.h>
#include <APM_Control.h>
#include <AP_L1_Control.h>
#include <AP_BoardConfig.h>
#include <AP_Frsky_Telem.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include "compat.h"

#include <AP_Notify.h>      // Notify library
#include <AP_BattMonitor.h> // Battery monitor library
#include <AP_OpticalFlow.h>     // Optical Flow library

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
#include "Parameters.h"
#include "GCS.h"

#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library

#include "Rover.h"

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static Rover rover;

/*
  setup is called when the sketch starts
 */
void Rover::setup() 
{
    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    notify.init(false);

    // rover does not use arming nor pre-arm checks
    AP_Notify::flags.armed = true;
    AP_Notify::flags.pre_arm_check = true;
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.failsafe_battery = false;

    rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

/*
  loop() is called rapidly while the sketch is running
 */
void Rover::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = hal.scheduler->micros();

    delta_us_fast_loop	= timer - fast_loopTimer_us;
    G_Dt                = delta_us_fast_loop * 1.0e-6f;
    fast_loopTimer_us   = timer;

	if (delta_us_fast_loop > G_Dt_max)
		G_Dt_max = delta_us_fast_loop;

    mainLoop_count++;

    // tell the scheduler one tick has passed
    scheduler.tick();

    scheduler.run(19500U);
}

// update AHRS system
void Rover::ahrs_update()
{
    hal.util->set_soft_armed(hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);

#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs_update();
#endif

    // when in reverse we need to tell AHRS not to assume we are a
    // 'fly forward' vehicle, otherwise it will see a large
    // discrepancy between the mag and the GPS heading and will try to
    // correct for it, leading to a large yaw error
    ahrs.set_fly_forward(!in_reverse);

    ahrs.update();

    // if using the EKF get a speed update now (from accelerometers)
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = pythagorous2(velocity.x, velocity.y);
    }

    if (should_log(MASK_LOG_ATTITUDE_FAST))
        Log_Write_Attitude();

    if (should_log(MASK_LOG_IMU))
        DataFlash.Log_Write_IMU(ins);
}

/*
  update camera mount - 50Hz
 */
void Rover::mount_update(void)
{
#if MOUNT == ENABLED
	camera_mount.update();
#endif
#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif
}

void Rover::update_alt()
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
}

/*
  check for GCS failsafe - 10Hz
 */
void Rover::gcs_failsafe_check(void)
{
	if (g.fs_gcs_enabled) {
        failsafe_trigger(FAILSAFE_EVENT_GCS, last_heartbeat_ms != 0 && (millis() - last_heartbeat_ms) > 2000);
    }
}

/*
  if the compass is enabled then try to accumulate a reading
 */
void Rover::compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }    
}

/*
  check for new compass data - 10Hz
 */
void Rover::update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        // update offsets
        compass.learn_offsets();
        if (should_log(MASK_LOG_COMPASS)) {
            DataFlash.Log_Write_Compass(compass);
        }
    } else {
        ahrs.set_compass(NULL);
    }
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST))
        Log_Write_Attitude();
    
    if (should_log(MASK_LOG_CTUN))
        Log_Write_Control_Tuning();

    if (should_log(MASK_LOG_NTUN))
        Log_Write_Nav_Tuning();
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging2(void)
{
    if (should_log(MASK_LOG_STEERING)) {
        if (control_mode == STEERING || control_mode == AUTO || control_mode == RTL || control_mode == GUIDED) {
            Log_Write_Steering();
        }
    }

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();
}


/*
  update aux servo mappings
 */
void Rover::update_aux(void)
{
    RC_Channel_aux::enable_aux_servos();
}

/*
  once a second events
 */
void Rover::one_second_loop(void)
{
	if (should_log(MASK_LOG_CURRENT))
		Log_Write_Current();
	// send a heartbeat
	gcs_send_message(MSG_HEARTBEAT);

    // allow orientation change at runtime to aid config
    ahrs.set_orientation();

    set_control_channels();

    // cope with changes to aux functions
    update_aux();

    // cope with changes to mavlink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    uint8_t Rover::counter;

    counter++;

    // write perf data every 20s
    if (counter % 10 == 0) {
        if (scheduler.debug() != 0) {
            hal.console->printf_P(PSTR("G_Dt_max=%lu\n"), (unsigned long)G_Dt_max);
        }
        if (should_log(MASK_LOG_PM))
            Log_Write_Performance();
        G_Dt_max = 0;
        resetPerfData();
    }

    // save compass offsets once a minute
    if (counter >= 60) {				
        if (g.compass_enabled) {
            compass.save_offsets();
        }
        counter = 0;
    }

    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
}

void Rover::update_GPS_50Hz(void)
{        
    uint32_t Rover::last_gps_reading[GPS_MAX_INSTANCES];
	gps.update();

    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            if (should_log(MASK_LOG_GPS)) {
                DataFlash.Log_Write_GPS(gps, i, current_loc.alt);
            }
        }
    }
}


void Rover::update_GPS_10Hz(void)
{        
    have_position = ahrs.get_position(current_loc);

	if (have_position && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

		if (ground_start_count > 1){
			ground_start_count--;

		} else if (ground_start_count == 1) {
			// We countdown N number of good GPS fixes
			// so that the altitude is more accurate
			// -------------------------------------
			if (current_loc.lat == 0) {
				ground_start_count = 20;
			} else {
                init_home();

                // set system clock for log timestamps
                hal.util->set_system_clock(gps.time_epoch_usec());

				if (g.compass_enabled) {
					// Set compass declination automatically
					compass.set_initial_location(gps.location().lat, gps.location().lng);
				}
				ground_start_count = 0;
			}
		}
        Vector3f velocity;
        if (ahrs.get_velocity_NED(velocity)) {
            ground_speed = pythagorous2(velocity.x, velocity.y);
        } else {
            ground_speed   = gps.ground_speed();
        }

#if CAMERA == ENABLED
        if (camera.update_location(current_loc) == true) {
            do_take_picture();
        }
#endif        
	}
}

void Rover::update_current_mode(void)
{ 
    switch (control_mode){
    case AUTO:
    case RTL:
        set_reverse(false);
        calc_lateral_acceleration();
        calc_nav_steer();
        calc_throttle(g.speed_cruise);
        break;

    case GUIDED:
        set_reverse(false);
        if (!rtl_complete) {
            if (verify_RTL()) {
                // we have reached destination so stop where we are
                channel_throttle->servo_out = g.throttle_min.get();
                channel_steer->servo_out = 0;
                lateral_acceleration = 0;
            } else {
                calc_lateral_acceleration();
                calc_nav_steer();
                calc_throttle(g.speed_cruise);
            }
        }
        break;

    case STEERING: {
        /*
          in steering mode we control lateral acceleration
          directly. We first calculate the maximum lateral
          acceleration at full steering lock for this speed. That is
          V^2/R where R is the radius of turn. We get the radius of
          turn from half the STEER2SRV_P.
         */
        float max_g_force = ground_speed * ground_speed / steerController.get_turn_radius();

        // constrain to user set TURN_MAX_G
        max_g_force = constrain_float(max_g_force, 0.1f, g.turn_max_g * GRAVITY_MSS);

        lateral_acceleration = max_g_force * (channel_steer->pwm_to_angle()/4500.0f);
        calc_nav_steer();

        // and throttle gives speed in proportion to cruise speed, up
        // to 50% throttle, then uses nudging above that.
        float target_speed = channel_throttle->pwm_to_angle() * 0.01f * 2 * g.speed_cruise;
        set_reverse(target_speed < 0);
        if (in_reverse) {
            target_speed = constrain_float(target_speed, -g.speed_cruise, 0);
        } else {
            target_speed = constrain_float(target_speed, 0, g.speed_cruise);
        }
        calc_throttle(target_speed);
        break;
    }

    case LEARNING:
    case MANUAL:
        /*
          in both MANUAL and LEARNING we pass through the
          controls. Setting servo_out here actually doesn't matter, as
          we set the exact value in set_servos(), but it helps for
          logging
         */
        channel_throttle->servo_out = channel_throttle->control_in;
        channel_steer->servo_out = channel_steer->pwm_to_angle();

        // mark us as in_reverse when using a negative throttle to
        // stop AHRS getting off
        set_reverse(channel_throttle->servo_out < 0);
        break;

    case HOLD:
        // hold position - stop motors and center steering
        channel_throttle->servo_out = 0;
        channel_steer->servo_out = 0;
        set_reverse(false);
        break;

    case INITIALISING:
        break;
	}
}

void Rover::update_navigation()
{
    switch (control_mode) {
    case MANUAL:
    case HOLD:
    case LEARNING:
    case STEERING:
    case INITIALISING:
        break;

    case AUTO:
		mission.update();
        break;

    case RTL:
        // no loitering around the wp with the rover, goes direct to the wp position
        calc_lateral_acceleration();
        calc_nav_steer();
        if (verify_RTL()) {  
            channel_throttle->servo_out = g.throttle_min.get();
            set_mode(HOLD);
        }
        break;

    case GUIDED:
        // no loitering around the wp with the rover, goes direct to the wp position
        calc_lateral_acceleration();
        calc_nav_steer();
        if (!rtl_complete) {
            if (verify_RTL()) {
                // we have reached destination so stop where we are
                channel_throttle->servo_out = g.throttle_min.get();
                channel_steer->servo_out = 0;
                lateral_acceleration = 0;
            }
        }
        break;
	}
}

AP_HAL_MAIN();

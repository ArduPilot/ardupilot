// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the GCS_MAVLink routing 
//

#include <stdarg.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <GCS.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_NavEKF.h>
#include <AP_HAL_Linux.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>
#include <SITL.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

extern mavlink_system_t mavlink_system;

const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
    AP_GROUPEND
};

static MAVLink_routing routing;

void setup(void)
{
    hal.console->println("routing test startup...");
    gcs[0].init(hal.uartA);
}

void loop(void)
{
    uint16_t err_count = 0;

    // incoming heartbeat
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat = {0};

    mavlink_msg_heartbeat_encode(3, 1, &msg, &heartbeat);

    if (!routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("heartbeat should be processed locally\n");
        err_count++;
    }

    // incoming non-targetted message
    mavlink_attitude_t attitude = {0};
    mavlink_msg_attitude_encode(3, 1, &msg, &attitude);
    if (!routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("attitude should be processed locally\n");
        err_count++;
    }

    // incoming targetted message for someone else
    mavlink_param_set_t param_set = {0};
    param_set.target_system = mavlink_system.sysid+1;
    param_set.target_component = mavlink_system.compid;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("param set 1 should not be processed locally\n");
        err_count++;
    }

    // incoming targetted message for us
    param_set.target_system = mavlink_system.sysid;
    param_set.target_component = mavlink_system.compid;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (!routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("param set 2 should be processed locally\n");
        err_count++;
    }

    // incoming targetted message for our system, but other compid
    // should be processed locally
    param_set.target_system = mavlink_system.sysid;
    param_set.target_component = mavlink_system.compid+1;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (!routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("param set 3 should be processed locally\n");
        err_count++;
    }

    // incoming broadcast message should be processed locally
    param_set.target_system = 0;
    param_set.target_component = mavlink_system.compid+1;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (!routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("param set 4 should be processed locally\n");
        err_count++;
    }

    if (err_count == 0) {
        hal.console->printf("All OK\n");
    }
    hal.scheduler->delay(1000);
}


AP_HAL_MAIN();

/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_InertialSensor_Stub.h"

void AP_InertialSensor_Stub::init( AP_PeriodicProcess * scheduler ) {}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_Stub::update( void ) { return true; }


float AP_InertialSensor_Stub::gx() { return 0.0f; }
float AP_InertialSensor_Stub::gy() { return 0.0f; }
float AP_InertialSensor_Stub::gz() { return 0.0f; }

void AP_InertialSensor_Stub::get_gyros( float * g ) { }

float AP_InertialSensor_Stub::ax() { return 0.0f; }
float AP_InertialSensor_Stub::ay() { return 0.0f; }
float AP_InertialSensor_Stub::az() { return 0.0f; }

void AP_InertialSensor_Stub::get_accels( float * a ) {}
void AP_InertialSensor_Stub::get_sensors( float * sensors ) {}

float AP_InertialSensor_Stub::temperature() { return 0.0; }
uint32_t AP_InertialSensor_Stub::sample_time() { return 0; }
void AP_InertialSensor_Stub::reset_sample_time() {}

// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
 *       AP_RangeFinder_MaxsonarXL.cpp - Arduino Library for Sharpe GP2Y0A02YK0F
 *       infrared proximity sensor
 *       Code by Jose Julio and Randy Mackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       Sparkfun URL: http://www.sparkfun.com/products/9491
 *       datasheet: http://www.sparkfun.com/datasheets/Sensors/Proximity/XL-EZ0-Datasheet.pdf
 *
 *       Sensor should be connected to one of the analog ports
 *
 *       Variables:
 *               int raw_value : raw value from the sensor
 *               int distance : distance in cm
 *               int max_distance : maximum measurable distance (in cm)
 *               int min_distance : minimum measurable distance (in cm)
 *
 *       Methods:
 *               read() : read value from analog port and returns the distance (in cm)
 *
 */

#include "AP_RangeFinder_MaxsonarXL.h"

// Constructor //////////////////////////////////////////////////////////////

AP_RangeFinder_MaxsonarXL::AP_RangeFinder_MaxsonarXL(AP_HAL::AnalogSource *source, FilterInt16 *filter) :
    RangeFinder(source, filter),
    _scaler(AP_RANGEFINDER_MAXSONARXL_SCALER)
{
    max_distance = AP_RANGEFINDER_MAXSONARXL_MAX_DISTANCE;
    min_distance = AP_RANGEFINDER_MAXSONARXL_MIN_DISTANCE;
}

// Public Methods //////////////////////////////////////////////////////////////
float AP_RangeFinder_MaxsonarXL::calculate_scaler(int sonar_type, float adc_refence_voltage)
{
    float type_scaler = 1.0f;
    switch(sonar_type) {
    case AP_RANGEFINDER_MAXSONARXL:
        type_scaler = AP_RANGEFINDER_MAXSONARXL_SCALER;
        min_distance = AP_RANGEFINDER_MAXSONARXL_MIN_DISTANCE;
        max_distance = AP_RANGEFINDER_MAXSONARXL_MAX_DISTANCE;
        break;
    case AP_RANGEFINDER_MAXSONARLV:
        type_scaler = AP_RANGEFINDER_MAXSONARLV_SCALER;
        min_distance = AP_RANGEFINDER_MAXSONARLV_MIN_DISTANCE;
        max_distance = AP_RANGEFINDER_MAXSONARLV_MAX_DISTANCE;
        break;
    case AP_RANGEFINDER_MAXSONARXLL:
        type_scaler = AP_RANGEFINDER_MAXSONARXLL_SCALER;
        min_distance = AP_RANGEFINDER_MAXSONARXLL_MIN_DISTANCE;
        max_distance = AP_RANGEFINDER_MAXSONARXLL_MAX_DISTANCE;
        break;
    case AP_RANGEFINDER_MAXSONARHRLV:
        type_scaler = AP_RANGEFINDER_MAXSONARHRLV_SCALER;
        min_distance = AP_RANGEFINDER_MAXSONARHRLV_MIN_DISTANCE;
        max_distance = AP_RANGEFINDER_MAXSONARHRLV_MAX_DISTANCE;
        break;
    }
    _scaler = type_scaler * adc_refence_voltage / 5.0f;
    return _scaler;
}

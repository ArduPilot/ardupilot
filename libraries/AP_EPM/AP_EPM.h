// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_EPM.h
 *
 *  Created on: DEC 06, 2013
 *      Author: Andreas Jochum
 */

/// @file	AP_EPM.h
/// @brief	AP_EPM control class

#ifndef __AP_EPM_h__
#define __AP_EPM_h__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    #define EPM_PIN_1       61    // On pin - AN7
    #define EPM_PIN_2       62    // Off pin - AN8
    #define EPM_SUPPORTED   true
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
    #define EPM_PIN_1       -1    // to be determine 
    #define EPM_PIN_2       -1    // to be determine
    #define EPM_SUPPORTED   false
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
    #define EPM_PIN_1       -1    // to be determine 
    #define EPM_PIN_2       -1    // to be determine
    #define EPM_SUPPORTED   false
#else
    #define EPM_PIN_1       -1    // not supported
    #define EPM_PIN_2       -1    // not supported
    #define EPM_SUPPORTED   false
#endif

/// @class	AP_EPM
/// @brief	Class to manage the EPM_CargoGripper 
class AP_EPM {
public:
    AP_EPM();

    // setup the epm pins
    void        init();

    // enabled - true if the epm is enabled
    bool        enabled();

    // activate the EPM
    void        on();

    // de-activate the EPM
    void        off();

    // do nothing
    void        neutral();

    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Int8     _enabled;

};

#endif /* _AP_EPM_H_ */

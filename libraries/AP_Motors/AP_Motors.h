#pragma once

#include "AP_Motors_config.h"

#include "AP_Motors_Class.h"
#include "AP_MotorsMulticopter.h"
#include "AP_MotorsMatrix.h"
#if AP_MOTORS_TRI_ENABLED
#include "AP_MotorsTri.h"
#endif  // AP_MOTORS_TRI_ENABLED
#include "AP_MotorsHeli_Single.h"
#include "AP_MotorsHeli_Dual.h"
#include "AP_MotorsHeli_Quad.h"
#include "AP_MotorsSingle.h"
#include "AP_MotorsCoax.h"
#include "AP_MotorsTailsitter.h"
#include "AP_Motors6DOF.h"
#include "AP_MotorsMatrix_6DoF_Scripting.h"
#include "AP_MotorsMatrix_Scripting_Dynamic.h"

// doesn't make sense to have more motors than servo channels, so clamp:
#if NUM_SERVO_CHANNELS < AP_MOTORS_MAX_NUM_MOTORS
#undef AP_MOTORS_MAX_NUM_MOTORS
#define AP_MOTORS_MAX_NUM_MOTORS NUM_SERVO_CHANNELS
#endif

// various Motors backends will not compile if we don't have 16 motors
// available (eg. AP_Motors6DOF).  Until we stop compiling those
// backends in when there aren't enough motors to support those
// backends we will support a minimum of 12 motors, the limit before
// we moved to 32 motor support:
#if AP_MOTORS_MAX_NUM_MOTORS < 12
#undef AP_MOTORS_MAX_NUM_MOTORS
#define AP_MOTORS_MAX_NUM_MOTORS 12
#endif

// scale factor for top layer to prevent beat frequency between top and bottom
// layers of co-rotating motors. Must be less than 1.0
#ifndef AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR
#endif

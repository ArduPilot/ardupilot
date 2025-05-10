#pragma once
// @file    SampleController.h
// @brief   This is an example controller code that will utilize the different classes in AP_Simulink library.

#include <AP_Common/AP_Common.h>
#include "AC_Simulink_AHRSHandle.h"
#include "AC_Simulink_AttControlHandle.h"
#include "AC_Simulink_MotorsHandle.h"
#include "AC_Simulink_PosControlHandle.h"

// functions
void SampleController_init(void);
void SampleController_loop(void);
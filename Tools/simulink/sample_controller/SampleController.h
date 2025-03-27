#pragma once

#include <AP_Common/AP_Common.h>
#include "AC_Simulink_AHRSHandle.h"
#include "AC_Simulink_AttControlHandle.h"
#include "AC_Simulink_MotorsHandle.h"
#include "AC_Simulink_PosControlHandle.h"

// functions
void SampleController_init(void);
void SampleController_loop(void);
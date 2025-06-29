#pragma once

/// @file    AP_Simulink_Factory.h
/// @brief   Factory class for choosing the right Simulink instance class based on the simulation mode

#include "AP_Simulink_Base.h"
#ifdef MW_EXTERNAL_MODE
#include "AP_Simulink_ExtMode.h"
#elif defined(MW_NORMAL_MODE)
#include "AP_Simulink_Normal.h"
#elif defined(MW_CONNECTEDIO_MODE)
#include "AP_Simulink_ConnectedIO.h"
#else
#include "AP_Simulink_Empty.h"
#endif
class AP_Simulink_Factory
{
public:
    static AP_Simulink_Base* createSimulinkInstance();
};
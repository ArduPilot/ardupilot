#pragma once

/// @file    AC_Simulink_Factory.h
/// @brief   Factory class for choosing the right class for Simulink integration based on the simulation mode

#include "AC_Simulink_Base.h"
#ifdef MW_EXTERNAL_MODE
#include "AC_Simulink_ExtMode.h"
#elif defined(MW_NORMAL_MODE)
#include "AC_Simulink_Normal.h"
#elif defined(MW_CONNECTEDIO_MODE)
#include "AC_Simulink_ConnectedIO.h"
#else
#include "AC_Simulink_Empty.h"
#endif
class AC_Simulink_Factory {
public:
    static AC_Simulink_Base* createSimulinkInstance();
};
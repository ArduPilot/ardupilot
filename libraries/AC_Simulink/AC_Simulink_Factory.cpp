
/// @file    AC_Simulink_Factory.cpp
/// @brief   Factory class for choosing the right class for Simulink integration based on the simulation mode

#include "AC_Simulink_Factory.h"

AC_Simulink_Base* AC_Simulink_Factory::createSimulinkInstance() {
#ifdef MW_EXTERNAL_MODE
    return new AC_Simulink_ExtMode();
#elif defined(MW_NORMAL_MODE)
    return new AC_Simulink_Normal();
#elif defined(MW_CONNECTEDIO_MODE)
    return new AC_Simulink_ConnectedIO();
#else
    return new AC_Simulink_Empty();
#endif
}
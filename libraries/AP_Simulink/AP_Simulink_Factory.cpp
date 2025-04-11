
/// @file    AP_Simulink_Factory.cpp
/// @brief   Factory class for choosing the right Simulink instance class based on the simulation mode

#include "AP_Simulink_Factory.h"

AP_Simulink_Base* AP_Simulink_Factory::createSimulinkInstance()
{
#ifdef MW_EXTERNAL_MODE
    return new AP_Simulink_ExtMode();
#elif defined(MW_NORMAL_MODE)
    return new AP_Simulink_Normal();
#elif defined(MW_CONNECTEDIO_MODE)
    return new AP_Simulink_ConnectedIO();
#else
    return new AP_Simulink_Empty();
#endif
}
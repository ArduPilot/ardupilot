/// @file    AP_Simulink_Empty.cpp
/// @brief   Empty class implementation for Simulink instance class

#include "AP_Simulink_Empty.h"
AP_Simulink_Empty::AP_Simulink_Empty() {}

void AP_Simulink_Empty::init()
{
    SampleController_init();
}

void AP_Simulink_Empty::update()
{
    SampleController_loop();
}

void AP_Simulink_Empty::reset()
{

}
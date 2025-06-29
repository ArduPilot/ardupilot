#pragma once

/// @file    AP_Simulink_Empty.h
/// @brief   Empty class implementation for Simulink instance class

#include "AP_Simulink_Base.h"
#include "SampleController.h"


class AP_Simulink_Empty : public AP_Simulink_Base
{
public:
    AP_Simulink_Empty();
    ~AP_Simulink_Empty() override {}

    void init() override;
    void update() override;
    void reset() override;

    CLASS_NO_COPY(AP_Simulink_Empty);
};
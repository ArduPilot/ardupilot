#pragma once

/// @file    AC_Simulink_Empty.h
/// @brief   External mode implementation of Simulink integration

#include "AC_Simulink_Base.h"
#include "SampleController.h"


class AC_Simulink_Empty : public AC_Simulink_Base {
public:
    AC_Simulink_Empty();
    ~AC_Simulink_Empty() override {}

    void init() override;
    void update() override;
    void reset() override;

    CLASS_NO_COPY(AC_Simulink_Empty);
};
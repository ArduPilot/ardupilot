#pragma once

/// @file    AC_Simulink_Empty.h
/// @brief   Empty implementation for Simulink integration

#include "AC_Simulink_Base.h"

class AC_Simulink_Empty : public AC_Simulink_Base {
public:
    AC_Simulink_Empty();
    ~AC_Simulink_Empty() override {}

    void init() override;
    void update() override;
    void reset() override;

    CLASS_NO_COPY(AC_Simulink_Empty);
};
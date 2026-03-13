#pragma once

/// @file    AC_Custom_FastTask_Empty.h
/// @brief   Empty class implementation for custom fast task instance

#include "AC_Custom_FastTask_Base.h"

class AC_Custom_FastTask_Empty : public AC_Custom_FastTask_Base
{
public:
    AC_Custom_FastTask_Empty();
    ~AC_Custom_FastTask_Empty() override {}

    void init() override;
    void update() override;
    CLASS_NO_COPY(AC_Custom_FastTask_Empty);
};
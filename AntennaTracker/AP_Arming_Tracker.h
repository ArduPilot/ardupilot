#pragma once

#include <AP_Arming/AP_Arming.h>

// this class isn't actually used by Tracker; it's really just here so
// the singleton doesn't come back as nullptr
class AP_Arming_Tracker : public AP_Arming
{
public:
    friend class Tracker;

private:

};

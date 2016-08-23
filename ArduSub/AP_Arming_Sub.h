#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_Sub : public AP_Arming {
public:

    AP_Arming_Sub() : AP_Arming() { }

    /* Do not allow copies */
    AP_Arming_Sub(const AP_Arming_Sub &other) = delete;
    AP_Arming_Sub &operator=(const AP_Arming_Sub&) = delete;

    void rc_calibration_checks() override;
    void pre_arm_checks(bool report) override;

protected:
    void ins_checks() override;
};

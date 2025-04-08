#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_Sub : public AP_Arming {
public:

    AP_Arming_Sub() : AP_Arming() { }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Arming_Sub);

    bool rc_calibration_checks(bool display_failure) override;
    bool pre_arm_checks(bool display_failure) override;
    bool has_disarm_function() const;

    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

protected:
    bool ins_checks(bool display_failure) override;
};

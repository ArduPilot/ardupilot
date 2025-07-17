/*
  ESC Telemetry for HobbyWing Platinum Pro v3
 */

#pragma once

#include "AP_ESC_Telem_config.h"

#if AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED

#include "AP_HobbyWing_ESC.h"

class AP_HobbyWing_Platinum_PRO_v3 : public AP_HobbyWing_ESC {
public:

    using AP_HobbyWing_ESC::AP_HobbyWing_ESC;

    void update() override;

private:

    float rpm_from_commutation_time(uint32_t commutation_time_us) const;

};

#endif  // AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED

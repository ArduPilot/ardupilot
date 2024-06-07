#include "AP_EFI_config.h"

#if AP_EFI_THROTTLE_LINEARISATION_ENABLED

/*
  throttle linearisation support
 */
class AP_EFI_ThrLin {
public:

    AP_EFI_ThrLin();

    static const struct AP_Param::GroupInfo var_info[];

    /*
      apply throttle linearisation, returning value to pass to the
      engine
    */
    float linearise_throttle(float throttle_pct);

private:
    AP_Int8 enable;
    AP_Float coefficient[3];
    AP_Float offset;
};

#endif // AP_EFI_THROTTLE_LINEARISATION_ENABLED


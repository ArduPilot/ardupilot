
#ifndef __AP_HAL_SMACCM_CLASS_H__
#define __AP_HAL_SMACCM_CLASS_H__

#include <AP_HAL.h>

#include "AP_HAL_SMACCM_Namespace.h"
#include "PrivateMember.h"

class HAL_SMACCM : public AP_HAL::HAL {
public:
    HAL_SMACCM();
    void init(int argc, char * const * argv) const;
};

extern const HAL_SMACCM AP_HAL_SMACCM;

#endif // __AP_HAL_SMACCM_CLASS_H__


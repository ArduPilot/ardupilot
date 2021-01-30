#include "SIM_Invensense_v3.h"

namespace SITL {

class ICM40609DevReg : public InvensenseV3DevReg {
public:
};

class ICM40609 : public InvensenseV3
{
public:
    void init() override {
        InvensenseV3::init();

        set_register(ICM40609DevReg::WHOAMI, (uint8_t)0x3b);
    }

    float accel_scale() const override { return (GRAVITY_MSS / 1024); }

private:
};

} // namespace SITL

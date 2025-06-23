#include "AC_PrecLand_config.h"

#if AC_PRECLAND_IRLOCK_ENABLED

#include "AC_PrecLand_IRLock.h"
#include <AP_HAL/AP_HAL.h>

// Constructor
AC_PrecLand_IRLock::AC_PrecLand_IRLock(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state),
      irlock()
{
}

// init - perform initialisation of this backend
void AC_PrecLand_IRLock::init()
{
    irlock.init(get_bus());
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand_IRLock::update()
{
    // update health
    _state.healthy = irlock.healthy();
    
    // get new sensor data
    irlock.update();
    
    if (irlock.num_targets() > 0 && irlock.last_update_ms() != _los_meas.time_ms) {
        irlock.get_unit_vector_body(_los_meas.vec_unit);
        _los_meas.frame = AC_PrecLand::VectorFrame::BODY_FRD;
        _los_meas.valid = true;
        _los_meas.time_ms = irlock.last_update_ms();
    }
    _los_meas.valid = _los_meas.valid && AP_HAL::millis() - _los_meas.time_ms <= 1000;
}

#endif // AC_PRECLAND_IRLOCK_ENABLED

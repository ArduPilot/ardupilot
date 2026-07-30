#include "AP_Compass_SITL.h"

#if AP_COMPASS_SITL_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_Compass_SITL::AP_Compass_SITL(uint8_t _sitl_instance) :
    _sitl(AP::sitl()),
    sitl_instance(_sitl_instance)
{
    if (sitl_instance > ARRAY_SIZE(_sitl->mag_devid)) {
        return;
    }

            const uint32_t dev_id = _sitl->mag_devid[sitl_instance];
            if (dev_id == 0) {
                return;
            }
            if (!register_compass(dev_id)) {
                return;
            }
                if (_sitl->mag_save_ids) {
                    // save so the compass always comes up configured in SITL
                    save_dev_id();
                }
                set_rotation(ROTATION_NONE);

        // Scroll through the registered compasses, and set the offsets
            if (_compass.get_offsets(instance).is_zero()) {
                _compass.set_offsets(instance, _sitl->mag_ofs[sitl_instance]);
            }

        // we want to simulate a calibrated compass by default, so set
        // scale to 1
        AP_Param::set_default_by_name("COMPASS_SCALE", 1);
        AP_Param::set_default_by_name("COMPASS_SCALE2", 1);
        AP_Param::set_default_by_name("COMPASS_SCALE3", 1);

        // make first compass external
        if (sitl_instance == 0) {
            set_external(true);
        }

        hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &AP_Compass_SITL::_timer, void));
}


void AP_Compass_SITL::_timer()
{
    Vector3f f;
    if (!_compass_sim.update(f)) {
        return;
    }

    accumulate_sample(f, 10);
}

#endif  // AP_COMPASS_SITL_ENABLED

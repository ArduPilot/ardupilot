#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if AP_SIM_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>
#include <AP_Common/AP_Common.h>
#include <AP_Compass/AP_Compass_config.h>

namespace SITL {

/*
  simulation of what one compass sensor reports.

  This lives here rather than in AP_Compass_SITL because several
  simulated devices want the same numbers: the directly-attached
  compasses, the DroneCAN compass, and (for the offsets a
  perfectly-calibrated compass would end up with) AP_AHRS_SIM.

  Each consumer owns its own instance of this object.  Sampling is
  deliberately left per-consumer: sharing one set of samples would mean
  whichever device asked first consumed them, and the sensors are only
  simulated at 100Hz, so nothing is gained by sharing the work.
 */
class CompassSim {
public:
    CompassSim(uint8_t _instance) : instance{_instance} {}

    /*
      fill "field" with the field this compass would report, in
      milligauss.  Returns false if no new sample is due -- the sensor
      is simulated at 100Hz -- or if the sensor is failed
      (SIM_MAGn_FAIL=1); the caller should not accumulate a sample in
      either case.
     */
    bool update(Vector3f &field);

private:
    const uint8_t instance;

    // create the correction matrix for diagonals and off-diagonals
    void setup_eliptical_correction();

    // delay buffer, so SIM_MAG_DELAY can be simulated
    struct readings_compass {
        uint32_t time;
        Vector3f data;
    };
    static const uint8_t buffer_length = 50;
    VectorN<readings_compass, buffer_length> buffer;
    uint8_t store_index;
    uint32_t last_store_time;

    uint32_t last_sample_time;

    Matrix3f eliptical_corr;
    Vector3f last_dia;
    Vector3f last_odi;

    // last field reported, so SIM_MAGn_FAIL=2 can freeze it
    Vector3f last_data;
};

}  // namespace SITL

#endif  // AP_SIM_ENABLED

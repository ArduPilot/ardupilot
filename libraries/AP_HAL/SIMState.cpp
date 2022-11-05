#include "SIMState.h"

#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL

/*
 *  This is a very-much-cut-down AP_HAL_SITL object.  We should make
 *  PA_HAL_SITL use this object - by moving a lot more code from over
 *  there into here.
 */

#include <SITL/SIM_Multicopter.h>
#include <SITL/SIM_Plane.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include <AP_Baro/AP_Baro.h>

extern const AP_HAL::HAL& hal;

using namespace AP_HAL;

#include <AP_Terrain/AP_Terrain.h>

void SIMState::update()
{
    static bool init_done;
    if (!init_done) {
        init_done = true;
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
        sitl_model = SITL::MultiCopter::create("+");
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        sitl_model = SITL::Plane::create("plane");
#endif
    }

    _fdm_input_step();
}

/*
  setup for SITL handling
 */
void SIMState::_sitl_setup(const char *home_str)
{
    _home_str = home_str;

    printf("Starting SITL input\n");

    // find the barometer object if it exists
    _barometer = AP_Baro::get_singleton();
}


/*
  step the FDM by one time step
 */
void SIMState::_fdm_input_step(void)
{
    fdm_input_local();
}

/*
  get FDM input from a local model
 */
void SIMState::fdm_input_local(void)
{
    struct sitl_input input;

    // construct servos structure for FDM
    _simulator_servos(input);

    // read servo inputs from ride along flight controllers
    // ride_along.receive(input);

    // update the model
    sitl_model->update_model(input);

    // get FDM output from the model
    if (_sitl == nullptr) {
        _sitl = AP::sitl();
    }
    if (_sitl) {
        sitl_model->fill_fdm(_sitl->state);

        if (_sitl->rc_fail == SITL::SIM::SITL_RCFail_None) {
            for (uint8_t i=0; i< _sitl->state.rcin_chan_count; i++) {
                pwm_input[i] = 1000 + _sitl->state.rcin[i]*1000;
            }
        }
    }

    // output JSON state to ride along flight controllers
    // ride_along.send(_sitl->state,sitl_model->get_position_relhome());

#if HAL_SIM_GIMBAL_ENABLED
    if (gimbal != nullptr) {
        gimbal->update();
    }
#endif
#if HAL_SIM_ADSB_ENABLED
    if (adsb != nullptr) {
        adsb->update();
    }
#endif
    if (vicon != nullptr) {
        Quaternion attitude;
        sitl_model->get_attitude(attitude);
        vicon->update(sitl_model->get_location(),
                      sitl_model->get_position_relhome(),
                      sitl_model->get_velocity_ef(),
                      attitude);
    }
    if (benewake_tf02 != nullptr) {
        benewake_tf02->update(sitl_model->rangefinder_range());
    }
    if (benewake_tf03 != nullptr) {
        benewake_tf03->update(sitl_model->rangefinder_range());
    }
    if (benewake_tfmini != nullptr) {
        benewake_tfmini->update(sitl_model->rangefinder_range());
    }
    if (teraranger_serial != nullptr) {
        teraranger_serial->update(sitl_model->rangefinder_range());
    }
    if (lightwareserial != nullptr) {
        lightwareserial->update(sitl_model->rangefinder_range());
    }
    if (lightwareserial_binary != nullptr) {
        lightwareserial_binary->update(sitl_model->rangefinder_range());
    }
    if (lanbao != nullptr) {
        lanbao->update(sitl_model->rangefinder_range());
    }
    if (blping != nullptr) {
        blping->update(sitl_model->rangefinder_range());
    }
    if (leddarone != nullptr) {
        leddarone->update(sitl_model->rangefinder_range());
    }
    if (USD1_v0 != nullptr) {
        USD1_v0->update(sitl_model->rangefinder_range());
    }
    if (USD1_v1 != nullptr) {
        USD1_v1->update(sitl_model->rangefinder_range());
    }
    if (maxsonarseriallv != nullptr) {
        maxsonarseriallv->update(sitl_model->rangefinder_range());
    }
    if (wasp != nullptr) {
        wasp->update(sitl_model->rangefinder_range());
    }
    if (nmea != nullptr) {
        nmea->update(sitl_model->rangefinder_range());
    }
    if (rf_mavlink != nullptr) {
        rf_mavlink->update(sitl_model->rangefinder_range());
    }
    if (gyus42v2 != nullptr) {
        gyus42v2->update(sitl_model->rangefinder_range());
    }
    if (efi_ms != nullptr) {
        efi_ms->update();
    }

    if (frsky_d != nullptr) {
        frsky_d->update();
    }
    // if (frsky_sport != nullptr) {
    //     frsky_sport->update();
    // }
    // if (frsky_sportpassthrough != nullptr) {
    //     frsky_sportpassthrough->update();
    // }

#if AP_SIM_CRSF_ENABLED
    if (crsf != nullptr) {
        crsf->update();
    }
#endif

#if HAL_SIM_PS_RPLIDARA2_ENABLED
    if (rplidara2 != nullptr) {
        rplidara2->update(sitl_model->get_location());
    }
#endif

#if HAL_SIM_PS_TERARANGERTOWER_ENABLED
    if (terarangertower != nullptr) {
        terarangertower->update(sitl_model->get_location());
    }
#endif

#if HAL_SIM_PS_LIGHTWARE_SF45B_ENABLED
    if (sf45b != nullptr) {
        sf45b->update(sitl_model->get_location());
    }
#endif

    if (vectornav != nullptr) {
        vectornav->update();
    }

    if (lord != nullptr) {
        lord->update();
    }

#if HAL_SIM_AIS_ENABLED
    if (ais != nullptr) {
        ais->update();
    }
#endif
    for (uint8_t i=0; i<ARRAY_SIZE(gps); i++) {
        if (gps[i] != nullptr) {
            gps[i]->update();
        }
    }

    // update simulation time
    if (_sitl) {
        hal.scheduler->stop_clock(_sitl->state.timestamp_us);
    } else {
        hal.scheduler->stop_clock(AP_HAL::micros64()+100);
    }

    set_height_agl();

    _synthetic_clock_mode = true;
    _update_count++;
}

/*
  create sitl_input structure for sending to FDM
 */
void SIMState::_simulator_servos(struct sitl_input &input)
{
    // output at chosen framerate
    uint32_t now = AP_HAL::micros();
    // last_update_usec = now;

    float altitude = _barometer?_barometer->get_altitude():0;
    float wind_speed = 0;
    float wind_direction = 0;
    float wind_dir_z = 0;

    // give 5 seconds to calibrate airspeed sensor at 0 wind speed
    if (wind_start_delay_micros == 0) {
        wind_start_delay_micros = now;
    } else if (_sitl && (now - wind_start_delay_micros) > 5000000 ) {
        // The EKF does not like step inputs so this LPF keeps it happy.
        wind_speed =     _sitl->wind_speed_active     = (0.95f*_sitl->wind_speed_active)     + (0.05f*_sitl->wind_speed);
        wind_direction = _sitl->wind_direction_active = (0.95f*_sitl->wind_direction_active) + (0.05f*_sitl->wind_direction);
        wind_dir_z =     _sitl->wind_dir_z_active     = (0.95f*_sitl->wind_dir_z_active)     + (0.05f*_sitl->wind_dir_z);
        
        // pass wind into simulators using different wind types via param SIM_WIND_T*.
        switch (_sitl->wind_type) {
        case SITL::SIM::WIND_TYPE_SQRT:
            if (altitude < _sitl->wind_type_alt) {
                wind_speed *= sqrtf(MAX(altitude / _sitl->wind_type_alt, 0));
            }
            break;

        case SITL::SIM::WIND_TYPE_COEF:
            wind_speed += (altitude - _sitl->wind_type_alt) * _sitl->wind_type_coef;
            break;

        case SITL::SIM::WIND_TYPE_NO_LIMIT:
        default:
            break;
        }

        // never allow negative wind velocity
        wind_speed = MAX(wind_speed, 0);
    }

    input.wind.speed = wind_speed;
    input.wind.direction = wind_direction;
    input.wind.turbulence = _sitl?_sitl->wind_turbulance:0;
    input.wind.dir_z = wind_dir_z;

    for (uint8_t i=0; i<SITL_NUM_CHANNELS; i++) {
        if (pwm_output[i] == 0xFFFF) {
            input.servos[i] = 0;
        } else {
            input.servos[i] = pwm_output[i];
        }
    }

    if (_sitl != nullptr) {
        // FETtec ESC simulation support.  Input signals of 1000-2000
        // are positive thrust, 0 to 1000 are negative thrust.  Deeper
        // changes required to support negative thrust - potentially
        // adding a field to input.
        if (_sitl != nullptr) {
            if (_sitl->fetteconewireesc_sim.enabled()) {
                _sitl->fetteconewireesc_sim.update_sitl_input_pwm(input);
                for (uint8_t i=0; i<ARRAY_SIZE(input.servos); i++) {
                    if (input.servos[i] != 0 && input.servos[i] < 1000) {
                        AP_HAL::panic("Bad input servo value (%u)", input.servos[i]);
                    }
                }
            }
        }
    }

    float voltage = 0;
    _current = 0;
    
    if (_sitl != nullptr) {
        if (_sitl->state.battery_voltage <= 0) {
        } else {
            // FDM provides voltage and current
            voltage = _sitl->state.battery_voltage;
            _current = _sitl->state.battery_current;
        }
    }

    // assume 3DR power brick
    voltage_pin_value = ((voltage / 10.1f) / 5.0f) * 1024;
    current_pin_value = ((_current / 17.0f) / 5.0f) * 1024;
    // fake battery2 as just a 25% gain on the first one
    voltage2_pin_value = ((voltage * 0.25f / 10.1f) / 5.0f) * 1024;
    current2_pin_value = ((_current * 0.25f / 17.0f) / 5.0f) * 1024;
}

/*
  set height above the ground in meters
 */
void SIMState::set_height_agl(void)
{
    static float home_alt = -1;

    if (!_sitl) {
        // in example program
        return;
    }

    if (is_equal(home_alt, -1.0f) && _sitl->state.altitude > 0) {
        // remember home altitude as first non-zero altitude
        home_alt = _sitl->state.altitude;
    }

#if AP_TERRAIN_AVAILABLE
    if (_sitl != nullptr &&
        _sitl->terrain_enable) {
        // get height above terrain from AP_Terrain. This assumes
        // AP_Terrain is working
        float terrain_height_amsl;
        struct Location location;
        location.lat = _sitl->state.latitude*1.0e7;
        location.lng = _sitl->state.longitude*1.0e7;

        AP_Terrain *_terrain = AP_Terrain::get_singleton();
        if (_terrain != nullptr &&
            _terrain->height_amsl(location, terrain_height_amsl)) {
            _sitl->height_agl = _sitl->state.altitude - terrain_height_amsl;
            return;
        }
    }
#endif

    if (_sitl != nullptr) {
        // fall back to flat earth model
        _sitl->height_agl = _sitl->state.altitude - home_alt;
    }
}

#endif  // AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL

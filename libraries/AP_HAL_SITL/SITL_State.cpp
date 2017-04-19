#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "UARTDriver.h"
#include "Scheduler.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/select.h>

#include <AP_Param/AP_Param.h>
#include <SITL/SIM_JSBSim.h>
#include <AP_HAL/utility/Socket.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

void SITL_State::_set_param_default(const char *parm)
{
    char *pdup = strdup(parm);
    char *p = strchr(pdup, '=');
    if (p == nullptr) {
        printf("Please specify parameter as NAME=VALUE");
        exit(1);
    }
    float value = strtof(p+1, nullptr);
    *p = 0;
    enum ap_var_type var_type;
    AP_Param *vp = AP_Param::find(pdup, &var_type);
    if (vp == nullptr) {
        printf("Unknown parameter %s\n", pdup);
        exit(1);
    }
    if (var_type == AP_PARAM_FLOAT) {
        ((AP_Float *)vp)->set_and_save(value);
    } else if (var_type == AP_PARAM_INT32) {
        ((AP_Int32 *)vp)->set_and_save(value);
    } else if (var_type == AP_PARAM_INT16) {
        ((AP_Int16 *)vp)->set_and_save(value);
    } else if (var_type == AP_PARAM_INT8) {
        ((AP_Int8 *)vp)->set_and_save(value);
    } else {
        printf("Unable to set parameter %s\n", pdup);
        exit(1);
    }
    printf("Set parameter %s to %f\n", pdup, value);
    free(pdup);
}


/*
  setup for SITL handling
 */
void SITL_State::_sitl_setup(const char *home_str)
{
    _home_str = home_str;

#ifndef __CYGWIN__
    _parent_pid = getppid();
#endif
    _rcout_addr.sin_family = AF_INET;
    _rcout_addr.sin_port = htons(_rcout_port);
    inet_pton(AF_INET, _fdm_address, &_rcout_addr.sin_addr);

#ifndef HIL_MODE
    _setup_fdm();
#endif
    fprintf(stdout, "Starting SITL input\n");

    // find the barometer object if it exists
    _sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    _barometer = (AP_Baro *)AP_Param::find_object("GND_");
    _ins = (AP_InertialSensor *)AP_Param::find_object("INS_");
    _compass = (Compass *)AP_Param::find_object("COMPASS_");
#if AP_TERRAIN_AVAILABLE
    _terrain = (AP_Terrain *)AP_Param::find_object("TERRAIN_");
#endif

    if (_sitl != nullptr) {
        // setup some initial values
#ifndef HIL_MODE
        _update_barometer(100);
        _update_ins(0);
        _update_compass();
        _update_gps(0, 0, 0, 0, 0, 0, false);
#endif
        if (enable_gimbal) {
            gimbal = new SITL::Gimbal(_sitl->state);
        }

        if (_use_fg_view) {
            fg_socket.connect("127.0.0.1", _fg_view_port);
        }

        fprintf(stdout, "Using Irlock at port : %d\n", _irlock_port);
        _sitl->irlock_port = _irlock_port;
    }

    if (_synthetic_clock_mode) {
        // start with non-zero clock
        hal.scheduler->stop_clock(1);
    }
}


#ifndef HIL_MODE
/*
  setup a SITL FDM listening UDP port
 */
void SITL_State::_setup_fdm(void)
{
    if (!_sitl_rc_in.bind("0.0.0.0", _rcin_port)) {
        fprintf(stderr, "SITL: socket bind failed on RC in port : %d - %s\n", _rcin_port, strerror(errno));
        fprintf(stderr, "Abording launch...\n");
        exit(1);
    }
    _sitl_rc_in.reuseaddress();
    _sitl_rc_in.set_blocking(false);
}
#endif


/*
  step the FDM by one time step
 */
void SITL_State::_fdm_input_step(void)
{
    static uint32_t last_pwm_input = 0;

    _fdm_input_local();

    /* make sure we die if our parent dies */
    if (kill(_parent_pid, 0) != 0) {
        exit(1);
    }

    if (_scheduler->interrupts_are_blocked() || _sitl == nullptr) {
        return;
    }

    // simulate RC input at 50Hz
    if (AP_HAL::millis() - last_pwm_input >= 20 && _sitl->rc_fail == 0) {
        last_pwm_input = AP_HAL::millis();
        new_rc_input = true;
    }

    _scheduler->sitl_begin_atomic();

    if (_update_count == 0 && _sitl != nullptr) {
        _update_gps(0, 0, 0, 0, 0, 0, false);
        _update_barometer(0);
        _scheduler->timer_event();
        _scheduler->sitl_end_atomic();
        return;
    }

    if (_sitl != nullptr) {
        _update_gps(_sitl->state.latitude, _sitl->state.longitude,
                    _sitl->state.altitude,
                    _sitl->state.speedN, _sitl->state.speedE, _sitl->state.speedD,
                    !_sitl->gps_disable);
        _update_ins(_sitl->state.airspeed);
        _update_barometer(_sitl->state.altitude);
        _update_compass();

        if (_sitl->adsb_plane_count >= 0 &&
            adsb == nullptr) {
            adsb = new SITL::ADSB(_sitl->state, _home_str);
        } else if (_sitl->adsb_plane_count == -1 &&
                   adsb != nullptr) {
            delete adsb;
            adsb = nullptr;
        }
    }

    // trigger all APM timers.
    _scheduler->timer_event();
    _scheduler->sitl_end_atomic();
}


void SITL_State::wait_clock(uint64_t wait_time_usec)
{
    while (AP_HAL::micros64() < wait_time_usec) {
        _fdm_input_step();
    }
}

#ifndef HIL_MODE
/*
  check for a SITL RC input packet
 */
void SITL_State::_check_rc_input(void)
{
    ssize_t size;
    struct pwm_packet {
        uint16_t pwm[16];
    } pwm_pkt;

    size = _sitl_rc_in.recv(&pwm_pkt, sizeof(pwm_pkt), 0);
    switch (size) {
    case 8*2:
    case 16*2: {
        // a packet giving the receiver PWM inputs
        uint8_t i;
        for (i=0; i<size/2; i++) {
            // setup the pwm input for the RC channel inputs
            if (i < _sitl->state.rcin_chan_count) {
                // we're using rc from simulator
                continue;
            }
            if (pwm_pkt.pwm[i] != 0) {
                pwm_input[i] = pwm_pkt.pwm[i];
            }
        }
        break;
    }
    }
}

/*
  output current state to flightgear
 */
void SITL_State::_output_to_flightgear(void)
{
    SITL::FGNetFDM fdm {};
    const SITL::sitl_fdm &sfdm = _sitl->state;

    fdm.version = 0x18;
    fdm.padding = 0;
    fdm.longitude = radians(sfdm.longitude);
    fdm.latitude = radians(sfdm.latitude);
    fdm.altitude = sfdm.altitude;
    fdm.agl = sfdm.altitude;
    fdm.phi   = radians(sfdm.rollDeg);
    fdm.theta = radians(sfdm.pitchDeg);
    fdm.psi   = radians(sfdm.yawDeg);
    if (_vehicle == ArduCopter) {
        fdm.num_engines = 4;
        for (uint8_t i=0; i<4; i++) {
            fdm.rpm[i] = constrain_float((pwm_output[i]-1000), 0, 1000);
        }
    } else {
        fdm.num_engines = 4;
        fdm.rpm[0] = constrain_float((pwm_output[2]-1000)*3, 0, 3000);
        // for quadplane
        fdm.rpm[1] = constrain_float((pwm_output[5]-1000)*12, 0, 12000);
        fdm.rpm[2] = constrain_float((pwm_output[6]-1000)*12, 0, 12000);
        fdm.rpm[3] = constrain_float((pwm_output[7]-1000)*12, 0, 12000);
    }
    fdm.ByteSwap();

    fg_socket.send(&fdm, sizeof(fdm));
}

/*
  get FDM input from a local model
 */
void SITL_State::_fdm_input_local(void)
{
    SITL::Aircraft::sitl_input input;

    // check for direct RC input
    _check_rc_input();

    // construct servos structure for FDM
    _simulator_servos(input);

    // update the model
    sitl_model->update(input);

    // get FDM output from the model
    if (_sitl) {
        sitl_model->fill_fdm(_sitl->state);
        _sitl->update_rate_hz = sitl_model->get_rate_hz();

        if (_sitl->rc_fail == 0) {
            for (uint8_t i=0; i< _sitl->state.rcin_chan_count; i++) {
                pwm_input[i] = 1000 + _sitl->state.rcin[i]*1000;
            }
        }
    }

    if (gimbal != nullptr) {
        gimbal->update();
    }
    if (adsb != nullptr) {
        adsb->update();
    }

    if (_sitl && _use_fg_view) {
        _output_to_flightgear();
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
#endif

/*
  create sitl_input structure for sending to FDM
 */
void SITL_State::_simulator_servos(SITL::Aircraft::sitl_input &input)
{
    static uint32_t last_update_usec;

    /* this maps the registers used for PWM outputs. The RC
     * driver updates these whenever it wants the channel output
     * to change */
    uint8_t i;

    if (last_update_usec == 0) {
        for (i=0; i<SITL_NUM_CHANNELS; i++) {
            pwm_output[i] = 1000;
        }
        if (_vehicle == ArduPlane) {
            pwm_output[0] = pwm_output[1] = pwm_output[3] = 1500;
        }
        if (_vehicle == APMrover2) {
            pwm_output[0] = pwm_output[1] = pwm_output[2] = pwm_output[3] = 1500;
        }
    }

    // output at chosen framerate
    uint32_t now = AP_HAL::micros();
    last_update_usec = now;

    // pass wind into simulators, using a wind gradient below 60m
    float altitude = _barometer?_barometer->get_altitude():0;
    float wind_speed = 0;
    float wind_direction = 0;
    if (_sitl) {
        // The EKF does not like step inputs so this LPF keeps it happy.
        wind_speed = _sitl->wind_speed_active = (0.95f*_sitl->wind_speed_active) + (0.05f*_sitl->wind_speed);
        wind_direction = _sitl->wind_direction_active = (0.95f*_sitl->wind_direction_active) + (0.05f*_sitl->wind_direction);
    }

    if (altitude < 0) {
        altitude = 0;
    }
    if (altitude < 60) {
        wind_speed *= sqrtf(MAX(altitude / 60, 0));
    }

    input.wind.speed = wind_speed;
    input.wind.direction = wind_direction;
    input.wind.turbulence = _sitl?_sitl->wind_turbulance:0;

    for (i=0; i<SITL_NUM_CHANNELS; i++) {
        if (pwm_output[i] == 0xFFFF) {
            input.servos[i] = 0;
        } else {
            input.servos[i] = pwm_output[i];
        }
    }

    float engine_mul = _sitl?_sitl->engine_mul.get():1;
    uint8_t engine_fail = _sitl?_sitl->engine_fail.get():0;
    bool motors_on = false;
    
    if (engine_fail >= ARRAY_SIZE(input.servos)) {
        engine_fail = 0;
    }
    // apply engine multiplier to motor defined by the SIM_ENGINE_FAIL parameter
    input.servos[engine_fail] = ((input.servos[engine_fail]-1000) * engine_mul) + 1000;
    
    if (_vehicle == ArduPlane) {
        motors_on = ((input.servos[2]-1000)/1000.0f) > 0;
    } else if (_vehicle == APMrover2) {
        motors_on = ((input.servos[2]-1500)/500.0f) != 0;
    } else {
        motors_on = false;
        // run checks on each motor
        for (i=0; i<4; i++) {
            // check motors do not exceed their limits
            if (input.servos[i] > 2000) input.servos[i] = 2000;
            if (input.servos[i] < 1000) input.servos[i] = 1000;
            // update motor_on flag
            if ((input.servos[i]-1000)/1000.0f > 0) {
                motors_on = true;
            }
        }
    }
    if (_sitl) {
        _sitl->motors_on = motors_on;
    }

    float voltage = 0;
    _current = 0;
    
    if (_sitl != nullptr) {
        if (_sitl->state.battery_voltage <= 0) {
            // simulate simple battery setup
            float throttle = motors_on?(input.servos[2]-1000) / 1000.0f:0;
            // lose 0.7V at full throttle
            voltage = _sitl->batt_voltage - 0.7f*fabsf(throttle);
            
            // assume 50A at full throttle
            _current = 50.0f * fabsf(throttle);
        } else {
            // FDM provides voltage and current
            voltage = _sitl->state.battery_voltage;
            _current = _sitl->state.battery_current;
        }
    }

    // assume 3DR power brick
    voltage_pin_value = ((voltage / 10.1f) / 5.0f) * 1024;
    current_pin_value = ((_current / 17.0f) / 5.0f) * 1024;
}


// generate a random float between -1 and 1
float SITL_State::_rand_float(void)
{
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
}

// generate a random Vector3f of size 1
Vector3f SITL_State::_rand_vec3f(void)
{
    Vector3f v = Vector3f(_rand_float(),
                          _rand_float(),
                          _rand_float());
    if (v.length() != 0.0f) {
        v.normalize();
    }
    return v;
}


void SITL_State::init(int argc, char * const argv[])
{
    pwm_input[0] = pwm_input[1] = pwm_input[3] = 1500;
    pwm_input[4] = pwm_input[7] = 1800;
    pwm_input[2] = pwm_input[5] = pwm_input[6] = 1000;

    _scheduler = Scheduler::from(hal.scheduler);
    _parse_command_line(argc, argv);
}

/*
  set height above the ground in meters
 */
void SITL_State::set_height_agl(void)
{
    static float home_alt = -1;

    if (home_alt == -1 && _sitl->state.altitude > 0) {
        // remember home altitude as first non-zero altitude
        home_alt = _sitl->state.altitude;
    }

#if AP_TERRAIN_AVAILABLE
    if (_terrain &&
            _sitl->terrain_enable) {
        // get height above terrain from AP_Terrain. This assumes
        // AP_Terrain is working
        float terrain_height_amsl;
        struct Location location;
        location.lat = _sitl->state.latitude*1.0e7;
        location.lng = _sitl->state.longitude*1.0e7;

        if (_terrain->height_amsl(location, terrain_height_amsl, false)) {
            _sitl->height_agl = _sitl->state.altitude - terrain_height_amsl;
            return;
        }
    }
#endif

    // fall back to flat earth model
    _sitl->height_agl = _sitl->state.altitude - home_alt;
}

#endif

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)

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
void SITL_State::_sitl_setup()
{
#if !defined(__CYGWIN__) && !defined(__CYGWIN64__)
    _parent_pid = getppid();
#endif

    _setup_fdm();
    fprintf(stdout, "Starting SITL input\n");

    // find the barometer object if it exists
    _sitl = AP::sitl();

    if (_sitl != nullptr) {
        // setup some initial values
        _update_airspeed(0);
        if (enable_gimbal) {
            gimbal = new SITL::Gimbal(_sitl->state);
        }

        sitl_model->set_buzzer(&_sitl->buzzer_sim);
        sitl_model->set_sprayer(&_sitl->sprayer_sim);
        sitl_model->set_gripper_servo(&_sitl->gripper_sim);
        sitl_model->set_gripper_epm(&_sitl->gripper_epm_sim);
        sitl_model->set_parachute(&_sitl->parachute_sim);
        sitl_model->set_precland(&_sitl->precland_sim);
        _sitl->i2c_sim.init();
        sitl_model->set_i2c(&_sitl->i2c_sim);

        if (_use_fg_view) {
            fg_socket.connect(_fg_address, _fg_view_port);
        }

        fprintf(stdout, "Using Irlock at port : %d\n", _irlock_port);
        _sitl->irlock_port = _irlock_port;
    }

    if (_synthetic_clock_mode) {
        // start with non-zero clock
        hal.scheduler->stop_clock(1);
    }
}


/*
  setup a SITL FDM listening UDP port
 */
void SITL_State::_setup_fdm(void)
{
    if (!_sitl_rc_in.reuseaddress()) {
        fprintf(stderr, "SITL: socket reuseaddress failed on RC in port: %d - %s\n", _rcin_port, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    if (!_sitl_rc_in.bind("0.0.0.0", _rcin_port)) {
        fprintf(stderr, "SITL: socket bind failed on RC in port : %d - %s\n", _rcin_port, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    if (!_sitl_rc_in.set_blocking(false)) {
        fprintf(stderr, "SITL: socket set_blocking(false) failed on RC in port: %d - %s\n", _rcin_port, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    if (!_sitl_rc_in.set_cloexec()) {
        fprintf(stderr, "SITL: socket set_cloexec() failed on RC in port: %d - %s\n", _rcin_port, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
}


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
    if (AP_HAL::millis() - last_pwm_input >= 20 && _sitl != nullptr && _sitl->rc_fail != SITL::SIM::SITL_RCFail_NoPulses) {
        last_pwm_input = AP_HAL::millis();
        new_rc_input = true;
    }

    _scheduler->sitl_begin_atomic();

    if (_update_count == 0 && _sitl != nullptr) {
        HALSITL::Scheduler::timer_event();
        _scheduler->sitl_end_atomic();
        return;
    }

    if (_sitl != nullptr) {
        _update_airspeed(_sitl->state.airspeed);
        _update_rangefinder();
    }

    // trigger all APM timers.
    HALSITL::Scheduler::timer_event();
    _scheduler->sitl_end_atomic();
}


void SITL_State::wait_clock(uint64_t wait_time_usec)
{
    while (AP_HAL::micros64() < wait_time_usec) {
        if (hal.scheduler->in_main_thread() ||
            Scheduler::from(hal.scheduler)->semaphore_wait_hack_required()) {
            _fdm_input_step();
        } else {
            usleep(1000);
        }
    }
    // check the outbound TCP queue size.  If it is too long then
    // MAVProxy/pymavlink take too long to process packets and it ends
    // up seeing traffic well into our past and hits time-out
    // conditions.
    if (sitl_model->get_speedup() > 1) {
        while (true) {
            const int queue_length = ((HALSITL::UARTDriver*)hal.serial(0))->get_system_outqueue_length();
            // ::fprintf(stderr, "queue_length=%d\n", (signed)queue_length);
            if (queue_length < 1024) {
                break;
            }
            usleep(1000);
        }
    }
}

#define streq(a, b) (!strcmp(a, b))
SITL::SerialDevice *SITL_State::create_serial_sim(const char *name, const char *arg)
{
    if (streq(name, "vicon")) {
        if (vicon != nullptr) {
            AP_HAL::panic("Only one vicon system at a time");
        }
        vicon = new SITL::Vicon();
        return vicon;
#if HAL_SIM_ADSB_ENABLED
    } else if (streq(name, "adsb")) {
        // ADSB is a stand-out as it is the only serial device which
        // will cope with begin() being called multiple times on a
        // serial port
        if (adsb == nullptr) {
            adsb = new SITL::ADSB();
        }
        return adsb;
#endif
    } else if (streq(name, "benewake_tf02")) {
        if (benewake_tf02 != nullptr) {
            AP_HAL::panic("Only one benewake_tf02 at a time");
        }
        benewake_tf02 = new SITL::RF_Benewake_TF02();
        return benewake_tf02;
    } else if (streq(name, "benewake_tf03")) {
        if (benewake_tf03 != nullptr) {
            AP_HAL::panic("Only one benewake_tf03 at a time");
        }
        benewake_tf03 = new SITL::RF_Benewake_TF03();
        return benewake_tf03;
    } else if (streq(name, "benewake_tfmini")) {
        if (benewake_tfmini != nullptr) {
            AP_HAL::panic("Only one benewake_tfmini at a time");
        }
        benewake_tfmini = new SITL::RF_Benewake_TFmini();
        return benewake_tfmini;
    } else if (streq(name, "lightwareserial")) {
        if (lightwareserial != nullptr) {
            AP_HAL::panic("Only one lightwareserial at a time");
        }
        lightwareserial = new SITL::RF_LightWareSerial();
        return lightwareserial;
    } else if (streq(name, "lightwareserial-binary")) {
        if (lightwareserial_binary != nullptr) {
            AP_HAL::panic("Only one lightwareserial-binary at a time");
        }
        lightwareserial_binary = new SITL::RF_LightWareSerialBinary();
        return lightwareserial_binary;
    } else if (streq(name, "lanbao")) {
        if (lanbao != nullptr) {
            AP_HAL::panic("Only one lanbao at a time");
        }
        lanbao = new SITL::RF_Lanbao();
        return lanbao;
    } else if (streq(name, "blping")) {
        if (blping != nullptr) {
            AP_HAL::panic("Only one blping at a time");
        }
        blping = new SITL::RF_BLping();
        return blping;
    } else if (streq(name, "leddarone")) {
        if (leddarone != nullptr) {
            AP_HAL::panic("Only one leddarone at a time");
        }
        leddarone = new SITL::RF_LeddarOne();
        return leddarone;
    } else if (streq(name, "USD1_v0")) {
        if (USD1_v0 != nullptr) {
            AP_HAL::panic("Only one USD1_v0 at a time");
        }
        USD1_v0 = new SITL::RF_USD1_v0();
        return USD1_v0;
    } else if (streq(name, "USD1_v1")) {
        if (USD1_v1 != nullptr) {
            AP_HAL::panic("Only one USD1_v1 at a time");
        }
        USD1_v1 = new SITL::RF_USD1_v1();
        return USD1_v1;
    } else if (streq(name, "maxsonarseriallv")) {
        if (maxsonarseriallv != nullptr) {
            AP_HAL::panic("Only one maxsonarseriallv at a time");
        }
        maxsonarseriallv = new SITL::RF_MaxsonarSerialLV();
        return maxsonarseriallv;
    } else if (streq(name, "wasp")) {
        if (wasp != nullptr) {
            AP_HAL::panic("Only one wasp at a time");
        }
        wasp = new SITL::RF_Wasp();
        return wasp;
    } else if (streq(name, "nmea")) {
        if (nmea != nullptr) {
            AP_HAL::panic("Only one nmea at a time");
        }
        nmea = new SITL::RF_NMEA();
        return nmea;

    } else if (streq(name, "rf_mavlink")) {
        if (rf_mavlink != nullptr) {
            AP_HAL::panic("Only one rf_mavlink at a time");
        }
        rf_mavlink = new SITL::RF_MAVLink();
        return rf_mavlink;

    } else if (streq(name, "frsky-d")) {
        if (frsky_d != nullptr) {
            AP_HAL::panic("Only one frsky_d at a time");
        }
        frsky_d = new SITL::Frsky_D();
        return frsky_d;
    // } else if (streq(name, "frsky-SPort")) {
    //     if (frsky_sport != nullptr) {
    //         AP_HAL::panic("Only one frsky_sport at a time");
    //     }
    //     frsky_sport = new SITL::Frsky_SPort();
    //     return frsky_sport;

    // } else if (streq(name, "frsky-SPortPassthrough")) {
    //     if (frsky_sport_passthrough != nullptr) {
    //         AP_HAL::panic("Only one frsky_sport passthrough at a time");
    //     }
    //     frsky_sport = new SITL::Frsky_SPortPassthrough();
    //     return frsky_sportpassthrough;
#if AP_SIM_CRSF_ENABLED
    } else if (streq(name, "crsf")) {
        if (crsf != nullptr) {
            AP_HAL::panic("Only one crsf at a time");
        }
        crsf = new SITL::CRSF();
        return crsf;
#endif
#if HAL_SIM_PS_RPLIDARA2_ENABLED
    } else if (streq(name, "rplidara2")) {
        if (rplidara2 != nullptr) {
            AP_HAL::panic("Only one rplidara2 at a time");
        }
        rplidara2 = new SITL::PS_RPLidarA2();
        return rplidara2;
#endif
#if HAL_SIM_PS_TERARANGERTOWER_ENABLED
    } else if (streq(name, "terarangertower")) {
        if (terarangertower != nullptr) {
            AP_HAL::panic("Only one terarangertower at a time");
        }
        terarangertower = new SITL::PS_TeraRangerTower();
        return terarangertower;
#endif
#if HAL_SIM_PS_LIGHTWARE_SF45B_ENABLED
    } else if (streq(name, "sf45b")) {
        if (sf45b != nullptr) {
            AP_HAL::panic("Only one sf45b at a time");
        }
        sf45b = new SITL::PS_LightWare_SF45B();
        return sf45b;
#endif
    } else if (streq(name, "richenpower")) {
        sitl_model->set_richenpower(&_sitl->richenpower_sim);
        return &_sitl->richenpower_sim;
    } else if (streq(name, "fetteconewireesc")) {
        sitl_model->set_fetteconewireesc(&_sitl->fetteconewireesc_sim);
        return &_sitl->fetteconewireesc_sim;
    } else if (streq(name, "ie24")) {
        sitl_model->set_ie24(&_sitl->ie24_sim);
        return &_sitl->ie24_sim;
    } else if (streq(name, "gyus42v2")) {
        if (gyus42v2 != nullptr) {
            AP_HAL::panic("Only one gyus42v2 at a time");
        }
        gyus42v2 = new SITL::RF_GYUS42v2();
        return gyus42v2;
    } else if (streq(name, "megasquirt")) {
        if (efi_ms != nullptr) {
            AP_HAL::panic("Only one megasquirt at a time");
        }
        efi_ms = new SITL::EFI_MegaSquirt();
        return efi_ms;
    } else if (streq(name, "VectorNav")) {
        if (vectornav != nullptr) {
            AP_HAL::panic("Only one VectorNav at a time");
        }
        vectornav = new SITL::VectorNav();
        return vectornav;
    } else if (streq(name, "LORD")) {
        if (lord != nullptr) {
            AP_HAL::panic("Only one LORD at a time");
        }
        lord = new SITL::LORD();
        return lord;
#if HAL_SIM_AIS_ENABLED
    } else if (streq(name, "AIS")) {
        if (ais != nullptr) {
            AP_HAL::panic("Only one AIS at a time");
        }
        ais = new SITL::AIS();
        return ais;
#endif
    } else if (strncmp(name, "gps", 3) == 0) {
        const char *p = strchr(name, ':');
        if (p == nullptr) {
            AP_HAL::panic("Need a GPS number (e.g. sim:gps:1)");
        }
        uint8_t x = atoi(p+1);
        if (x <= 0 || x > ARRAY_SIZE(gps)) {
            AP_HAL::panic("Bad GPS number %u", x);
        }
        gps[x-1] = new SITL::GPS(x-1);
        return gps[x-1];
    }

    AP_HAL::panic("unknown simulated device: %s", name);
}

/*
  check for a SITL RC input packet
 */
void SITL_State::_check_rc_input(void)
{
    uint32_t count = 0;
    while (_read_rc_sitl_input()) {
        count++;
    }

    if (count > 100) {
        ::fprintf(stderr, "Read %u rc inputs\n", count);
    }
}

bool SITL_State::_read_rc_sitl_input()
{
    struct pwm_packet {
        uint16_t pwm[16];
    } pwm_pkt;

    const ssize_t size = _sitl_rc_in.recv(&pwm_pkt, sizeof(pwm_pkt), 0);

    if (_sitl->rc_fail == SITL::SIM::SITL_RCFail_Throttle950) {
        // discard anything we just read from the "receiver" and set
        // values to bind values:
        for (uint8_t i=0; i<ARRAY_SIZE(pwm_input); i++) {
            pwm_input[0] = 1500;  // centre all inputs
        }
        pwm_input[2] = 950;  // reset throttle (assumed to be on channel 3...)
        return size != -1;  // we must continue to drain _sitl_rc
    }

    switch (size) {
    case -1:
        return false;
    case 8*2:
    case 16*2: {
        // a packet giving the receiver PWM inputs
        for (uint8_t i=0; i<size/2; i++) {
            // setup the pwm input for the RC channel inputs
            if (i < _sitl->state.rcin_chan_count) {
                // we're using rc from simulator
                continue;
            }
            uint16_t pwm = pwm_pkt.pwm[i];
            if (pwm != 0) {
                pwm_input[i] = pwm;
            }
        }
        return true;
    }
    default:
        fprintf(stderr, "Malformed SITL RC input (%ld)", (long)size);
    }
    return false;
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
    fdm.longitude = DEG_TO_RAD_DOUBLE*sfdm.longitude;
    fdm.latitude = DEG_TO_RAD_DOUBLE*sfdm.latitude;
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
    struct sitl_input input;

    // check for direct RC input
    if (_sitl != nullptr) {
        _check_rc_input();
    }

    // construct servos structure for FDM
    _simulator_servos(input);

#if HAL_SIM_JSON_MASTER_ENABLED
    // read servo inputs from ride along flight controllers
    ride_along.receive(input);
#endif

    // update the model
    sitl_model->update_model(input);

    // get FDM output from the model
    if (_sitl) {
        sitl_model->fill_fdm(_sitl->state);

        if (_sitl->rc_fail == SITL::SIM::SITL_RCFail_None) {
            for (uint8_t i=0; i< _sitl->state.rcin_chan_count; i++) {
                pwm_input[i] = 1000 + _sitl->state.rcin[i]*1000;
            }
        }
    }

#if HAL_SIM_JSON_MASTER_ENABLED
    // output JSON state to ride along flight controllers
    ride_along.send(_sitl->state,sitl_model->get_position_relhome());
#endif

    if (gimbal != nullptr) {
        gimbal->update();
    }
#if HAL_SIM_ADSB_ENABLED
    if (adsb != nullptr) {
        adsb->update(*sitl_model);
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

/*
  create sitl_input structure for sending to FDM
 */
void SITL_State::_simulator_servos(struct sitl_input &input)
{
    static uint32_t last_update_usec;

    /* this maps the registers used for PWM outputs. The RC
     * driver updates these whenever it wants the channel output
     * to change */

    if (last_update_usec == 0 || !output_ready) {
        for (uint8_t i=0; i<SITL_NUM_CHANNELS; i++) {
            pwm_output[i] = 1000;
        }
        if (_vehicle == ArduPlane) {
            pwm_output[0] = pwm_output[1] = pwm_output[3] = 1500;
        }
        if (_vehicle == Rover) {
            pwm_output[0] = pwm_output[1] = pwm_output[2] = pwm_output[3] = 1500;
        }
        if (_vehicle == ArduSub) {
            pwm_output[0] = pwm_output[1] = pwm_output[2] = pwm_output[3] =
                    pwm_output[4] = pwm_output[5] = pwm_output[6] = pwm_output[7] = 1500;
        }
    }

    // output at chosen framerate
    uint32_t now = AP_HAL::micros();
    last_update_usec = now;

    float altitude = AP::baro().get_altitude();
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

    float engine_mul = _sitl?_sitl->engine_mul.get():1;
    uint8_t engine_fail = _sitl?_sitl->engine_fail.get():0;
    float throttle = 0.0f;
    
    if (engine_fail >= ARRAY_SIZE(input.servos)) {
        engine_fail = 0;
    }
    // apply engine multiplier to motor defined by the SIM_ENGINE_FAIL parameter
    if (_vehicle != Rover) {
        input.servos[engine_fail] = ((input.servos[engine_fail]-1000) * engine_mul) + 1000;
    } else {
        input.servos[engine_fail] = static_cast<uint16_t>(((input.servos[engine_fail] - 1500) * engine_mul) + 1500);
    }

    if (_vehicle == ArduPlane) {
        float forward_throttle = constrain_float((input.servos[2] - 1000) / 1000.0f, 0.0f, 1.0f);
        // do a little quadplane dance
        float hover_throttle = 0.0f;
        uint8_t running_motors = 0;
        for (uint8_t i=0; i < sitl_model->get_num_motors() - 1; i++) {
            float motor_throttle = constrain_float((input.servos[sitl_model->get_motors_offset() + i] - 1000) / 1000.0f, 0.0f, 1.0f);
            // update motor_on flag
            if (!is_zero(motor_throttle)) {
                hover_throttle += motor_throttle;
                running_motors++;
            }
        }
        if (running_motors > 0) {
            hover_throttle /= running_motors;
        }
        if (!is_zero(forward_throttle)) {
            throttle = forward_throttle;
        } else {
            throttle = hover_throttle;
        }
    } else if (_vehicle == Rover) {
        input.servos[2] = static_cast<uint16_t>(constrain_int16(input.servos[2], 1000, 2000));
        input.servos[0] = static_cast<uint16_t>(constrain_int16(input.servos[0], 1000, 2000));
        throttle = fabsf((input.servos[2] - 1500) / 500.0f);
    } else {
        // run checks on each motor
        uint8_t running_motors = 0;
        for (uint8_t i=0; i < sitl_model->get_num_motors(); i++) {
            float motor_throttle = constrain_float((input.servos[i] - 1000) / 1000.0f, 0.0f, 1.0f);
            // update motor_on flag
            if (!is_zero(motor_throttle)) {
                throttle += motor_throttle;
                running_motors++;
            }
        }
        if (running_motors > 0) {
            throttle /= running_motors;
        }
    }
    if (_sitl) {
        _sitl->throttle = throttle;
    }

    float voltage = 0;
    _current = 0;
    
    if (_sitl != nullptr) {
        if (_sitl->state.battery_voltage <= 0) {
            if (_vehicle == ArduSub) {
                voltage = _sitl->batt_voltage;
                for (uint8_t i=0; i<6; i++) {
                    float pwm = input.servos[i];
                    //printf("i: %d, pwm: %.2f\n", i, pwm);
                    float fraction = fabsf((pwm - 1500) / 500.0f);

                    voltage -= fraction * 0.5f;

                    float draw = fraction * 15;
                    _current += draw;
                }
            } else {
                // simulate simple battery setup
                // lose 0.7V at full throttle
                voltage = _sitl->batt_voltage - 0.7f * throttle;

                // assume 50A at full throttle
                _current = 50.0f * throttle;
            }
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
            _terrain->height_amsl(location, terrain_height_amsl, false)) {
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

#endif

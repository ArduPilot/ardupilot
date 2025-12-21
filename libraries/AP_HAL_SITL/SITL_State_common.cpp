#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "UARTDriver.h"
#include "Scheduler.h"
#include "CANSocketIface.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/select.h>

#include <AP_Param/AP_Param.h>
#include <SITL/SIM_JSBSim.h>
#include <AP_HAL/utility/Socket_native.h>
#include "SITL_State_common.h"

#include <SITL/SIM_RF_Ainstein_LR_D1.h>
#include <SITL/SIM_RF_Ainstein_LR_D1_v19.h>
#include <SITL/SIM_RF_Benewake_TF02.h>
#include <SITL/SIM_RF_Benewake_TF03.h>
#include <SITL/SIM_RF_Benewake_TFmini.h>
#include <SITL/SIM_RF_BLping.h>
#include <SITL/SIM_RF_GYUS42v2.h>
#include <SITL/SIM_RF_JRE.h>
#include <SITL/SIM_RF_Lanbao.h>
#include <SITL/SIM_RF_LeddarOne.h>
#include <SITL/SIM_RF_LightWareSerialBinary.h>
#include <SITL/SIM_RF_LightWareSerial.h>
#include <SITL/SIM_RF_MAVLink.h>
#include <SITL/SIM_RF_MaxsonarSerialLV.h>
#include <SITL/SIM_RF_NMEA.h>
#include <SITL/SIM_RF_NoopLoop.h>
#include <SITL/SIM_RF_RDS02UF.h>
#include <SITL/SIM_RF_TeraRanger_Serial.h>
#include <SITL/SIM_RF_USD1_v0.h>
#include <SITL/SIM_RF_USD1_v1.h>
#include <SITL/SIM_RF_Wasp.h>
#include <SITL/SIM_RF_LightWare_GRF.h>

using namespace HALSITL;

static const struct {
    const char *name;
    SITL::SerialRangeFinder *(*createfn)();
}  serial_rangefinder_definitions[] {
    { "ainsteinlrd1", SITL::RF_Ainstein_LR_D1::create },
    { "ainsteinlrd1_v19", SITL::RF_Ainstein_LR_D1_v19::create },
    { "benewake_tf02", SITL::RF_Benewake_TF02::create },
    { "benewake_tf03", SITL::RF_Benewake_TF03::create },
    { "benewake_tfmini", SITL::RF_Benewake_TFmini::create },
    { "blping", SITL::RF_BLping::create },
    { "gyus42v2", SITL::RF_GYUS42v2::create },
    { "jre", SITL::RF_JRE::create },
    { "lanbao", SITL::RF_Lanbao::create },
    { "leddarone", SITL::RF_LeddarOne::create },
    { "leddarone", SITL::RF_LeddarOne::create },
    { "lightwareserial-binary", SITL::RF_LightWareSerialBinary::create },
    { "lightwareserial", SITL::RF_LightWareSerial::create },
    { "lightware_grf", SITL::RF_LightWareGRF::create },
    { "maxsonarseriallv", SITL::RF_MaxsonarSerialLV::create },
    { "nmea", SITL::RF_NMEA::create },
    { "nmea", SITL::RF_NMEA::create },
    { "nooploop_tofsense", SITL::RF_Nooploop::create },
    { "rds02uf", SITL::RF_RDS02UF::create },
#if !defined(HAL_BUILD_AP_PERIPH)
    { "rf_mavlink", SITL::RF_MAVLink::create },
#endif
    { "teraranger_serial", SITL::RF_TeraRanger_Serial::create },
    { "USD1_v0", SITL::RF_USD1_v0::create },
    { "USD1_v1", SITL::RF_USD1_v1::create },
    { "wasp", SITL::RF_Wasp::create },
};

#define streq(a, b) (!strcmp(a, b))
SITL::SerialDevice *SITL_State_Common::create_serial_sim(const char *name, const char *arg, const uint8_t portNumber)
{
    for (const auto &definition : serial_rangefinder_definitions) {
        if (!streq(definition.name, name)) {
            continue;
        }
        if (num_serial_rangefinders >= ARRAY_SIZE(serial_rangefinders)) {
            AP_HAL::panic("Too many simulated serial rangefinders");
        }
        serial_rangefinders[num_serial_rangefinders] = definition.createfn();
        return serial_rangefinders[num_serial_rangefinders++];
    }

    if (false) {
        // this is an empty clause to ease else-if syntax
#if !defined(HAL_BUILD_AP_PERIPH)
    } else if (streq(name, "vicon")) {
        if (vicon != nullptr) {
            AP_HAL::panic("Only one vicon system at a time");
        }
        vicon = NEW_NOTHROW SITL::Vicon();
        return vicon;
#endif
#if AP_SIM_ADSB_ENABLED
    } else if (streq(name, "adsb")) {
        // ADSB is a stand-out as it is the only serial device which
        // will cope with begin() being called multiple times on a
        // serial port
        if (adsb == nullptr) {
            adsb = NEW_NOTHROW SITL::ADSB();
        }
        sitl_model->set_adsb(adsb);
        return adsb;
#endif  // AP_SIM_ADSB_ENABLED
    } else if (streq(name, "frsky-d")) {
        if (frsky_d != nullptr) {
            AP_HAL::panic("Only one frsky_d at a time");
        }
        frsky_d = NEW_NOTHROW SITL::Frsky_D();
        return frsky_d;
    // } else if (streq(name, "frsky-SPort")) {
    //     if (frsky_sport != nullptr) {
    //         AP_HAL::panic("Only one frsky_sport at a time");
    //     }
    //     frsky_sport = NEW_NOTHROW SITL::Frsky_SPort();
    //     return frsky_sport;

    // } else if (streq(name, "frsky-SPortPassthrough")) {
    //     if (frsky_sport_passthrough != nullptr) {
    //         AP_HAL::panic("Only one frsky_sport passthrough at a time");
    //     }
    //     frsky_sport = NEW_NOTHROW SITL::Frsky_SPortPassthrough();
    //     return frsky_sportpassthrough;
#if AP_SIM_CRSF_ENABLED
    } else if (streq(name, "crsf")) {
        if (crsf != nullptr) {
            AP_HAL::panic("Only one crsf at a time");
        }
        crsf = NEW_NOTHROW SITL::CRSF();
        return crsf;
#endif
#if AP_SIM_PS_LD06_ENABLED
    } else if (streq(name, "ld06")) {
        if (ld06 != nullptr) {
            AP_HAL::panic("Only one ld06 at a time");
        }
        ld06 = NEW_NOTHROW SITL::PS_LD06();
        return ld06;
#endif  // AP_SIM_PS_LD06_ENABLED
#if AP_SIM_PS_RPLIDARA2_ENABLED
    } else if (streq(name, "rplidara2")) {
        if (rplidara2 != nullptr) {
            AP_HAL::panic("Only one rplidara2 at a time");
        }
        rplidara2 = NEW_NOTHROW SITL::PS_RPLidarA2();
        return rplidara2;
#endif
#if AP_SIM_PS_RPLIDARA1_ENABLED
    } else if (streq(name, "rplidara1")) {
        if (rplidara1 != nullptr) {
            AP_HAL::panic("Only one rplidara1 at a time");
        }
        rplidara1 = NEW_NOTHROW SITL::PS_RPLidarA1();
        return rplidara1;
#endif
#if AP_SIM_PS_RPLIDARS2_ENABLED
    } else if (streq(name, "rplidars2")) {
        if (rplidars2 != nullptr) {
            AP_HAL::panic("Only one rplidars2 at a time");
        }
        rplidars2 = NEW_NOTHROW SITL::PS_RPLidarS2();
        return rplidars2;
#endif
#if AP_SIM_PS_TERARANGERTOWER_ENABLED
    } else if (streq(name, "terarangertower")) {
        if (terarangertower != nullptr) {
            AP_HAL::panic("Only one terarangertower at a time");
        }
        terarangertower = NEW_NOTHROW SITL::PS_TeraRangerTower();
        return terarangertower;
#endif
#if AP_SIM_PS_LIGHTWARE_SF45B_ENABLED
    } else if (streq(name, "sf45b")) {
        if (sf45b != nullptr) {
            AP_HAL::panic("Only one sf45b at a time");
        }
        sf45b = NEW_NOTHROW SITL::PS_LightWare_SF45B();
        return sf45b;
#endif
#if AP_SIM_ADSB_SAGETECH_MXS_ENABLED
    } else if (streq(name, "sagetech_mxs")) {
        if (sagetech_mxs != nullptr) {
            AP_HAL::panic("Only one sagetech_mxs at a time");
        }
        sagetech_mxs = NEW_NOTHROW SITL::ADSB_Sagetech_MXS();
        if (adsb == nullptr) {
            adsb = NEW_NOTHROW SITL::ADSB();
        }
        sitl_model->set_adsb(adsb);
        return sagetech_mxs;
#endif
#if AP_SIM_LOWEHEISER_ENABLED
    } else if (streq(name, "loweheiser")) {
        sitl_model->set_loweheiser(&_sitl->loweheiser_sim);
        return &_sitl->loweheiser_sim;
#endif
#if !defined(HAL_BUILD_AP_PERIPH)
    } else if (streq(name, "richenpower")) {
        sitl_model->set_richenpower(&_sitl->richenpower_sim);
        return &_sitl->richenpower_sim;
    } else if (streq(name, "fetteconewireesc")) {
        sitl_model->set_fetteconewireesc(&_sitl->fetteconewireesc_sim);
        return &_sitl->fetteconewireesc_sim;
    } else if (streq(name, "ie24")) {
        sitl_model->set_ie24(&_sitl->ie24_sim);
        return &_sitl->ie24_sim;
#endif // HAL_BUILD_AP_PERIPH
#if AP_SIM_VOLZ_ENABLED
    } else if (streq(name, "volz")) {
        sitl_model->set_volz(&_sitl->volz_sim);
        return &_sitl->volz_sim;
#endif  // AP_SIM_VOLZ_ENABLED
    } else if (streq(name, "megasquirt")) {
        if (efi_ms != nullptr) {
            AP_HAL::panic("Only one megasquirt at a time");
        }
        efi_ms = NEW_NOTHROW SITL::EFI_MegaSquirt();
        return efi_ms;
    } else if (streq(name, "hirth")) {
        if (efi_hirth != nullptr) {
            AP_HAL::panic("Only one hirth at a time");
        }
        efi_hirth = NEW_NOTHROW SITL::EFI_Hirth();
        return efi_hirth;
    } else if (streq(name, "VectorNav")) {
        if (vectornav != nullptr) {
            AP_HAL::panic("Only one VectorNav at a time");
        }
        vectornav = NEW_NOTHROW SITL::VectorNav(SITL::VectorNav::VNModel::VN300);
        return vectornav;
    } else if (streq(name, "VectorNav_VN100")) {
        if (vectornav != nullptr) {
            AP_HAL::panic("Only one VectorNav at a time");
        }
        vectornav = NEW_NOTHROW SITL::VectorNav(SITL::VectorNav::VNModel::VN100);
        return vectornav;
    } else if (streq(name, "MicroStrain5")) {
        if (microstrain5 != nullptr) {
            AP_HAL::panic("Only one MicroStrain5 at a time");
        }
        microstrain5 = NEW_NOTHROW SITL::MicroStrain5();
        return microstrain5;

    } else if (streq(name, "MicroStrain7")) {
        if (microstrain7 != nullptr) {
            AP_HAL::panic("Only one MicroStrain7 at a time");
        }
        microstrain7 = NEW_NOTHROW SITL::MicroStrain7();
        return microstrain7;

    } else if (streq(name, "ILabs")) {
        if (inertiallabs != nullptr) {
            AP_HAL::panic("Only one InertialLabs INS at a time");
        }
        inertiallabs = NEW_NOTHROW SITL::InertialLabs();
        return inertiallabs;

#if AP_SIM_AIS_ENABLED
    } else if (streq(name, "AIS")) {
        if ((ais != nullptr) || (ais_replay != nullptr)) {
            AP_HAL::panic("Only one AIS at a time");
        }
        ais = NEW_NOTHROW SITL::AIS();
        return ais;
    } else if (streq(name, "AISReplay")) {
        if ((ais != nullptr) || (ais_replay != nullptr)) {
            AP_HAL::panic("Only one AIS at a time");
        }
        ais_replay = NEW_NOTHROW SITL::AIS_Replay();
        return ais_replay;
#endif
    } else if (strncmp(name, "gps", 3) == 0) {
        uint8_t x = atoi(arg);
        if (x <= 0 || x > ARRAY_SIZE(gps)) {
            AP_HAL::panic("Bad GPS number %u (%s)", x, arg);
        }
        gps[x-1] = NEW_NOTHROW SITL::GPS(x-1);
        return gps[x-1];
    } else if (streq(name, "ELRS")) {
        // Only allocate if not done already
        // MAVLink serial ports have begin called several times
        if (elrs == nullptr) {
            elrs = NEW_NOTHROW SITL::ELRS(portNumber, this);
            _sitl->set_stop_MAVLink_sim_state();
        }
        return elrs;
    }

    AP_HAL::panic("unknown simulated device: %s", name);
}

/*
  update simulators
 */
void SITL_State_Common::sim_update(void)
{
#if AP_SIM_SOLOGIMBAL_ENABLED
    if (gimbal != nullptr) {
        gimbal->update(*sitl_model);
    }
#endif
#if AP_SIM_ADSB_ENABLED
    if (adsb != nullptr) {
        adsb->update(*sitl_model);
    }
#endif  // AP_SIM_ADSB_ENABLED
#if AP_SIM_VICON_ENABLED
    if (vicon != nullptr) {
        Quaternion attitude;
        sitl_model->get_attitude(attitude);
        vicon->update(sitl_model->get_location(),
                      sitl_model->get_position_relhome(),
                      sitl_model->get_velocity_ef(),
                      attitude);
    }
#endif  // AP_SIM_VICON_ENABLED
    for (uint8_t i=0; i<num_serial_rangefinders; i++) {
        serial_rangefinders[i]->update(sitl_model->rangefinder_range());
    }
    if (efi_ms != nullptr) {
        efi_ms->update();
    }
    if (efi_hirth != nullptr) {
        efi_hirth->update();
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

#if AP_SIM_PS_LD06_ENABLED
    if (ld06 != nullptr) {
        ld06->update(sitl_model->get_location());
    }
#endif  // AP_SIM_PS_LD06_ENABLED

#if AP_SIM_PS_RPLIDARA2_ENABLED
    if (rplidara2 != nullptr) {
        rplidara2->update(sitl_model->get_location());
    }
#endif

#if AP_SIM_PS_RPLIDARA1_ENABLED
    if (rplidara1 != nullptr) {
        rplidara1->update(sitl_model->get_location());
    }
#endif

#if AP_SIM_PS_RPLIDARS2_ENABLED
    if (rplidars2 != nullptr) {
        rplidars2->update(sitl_model->get_location());
    }
#endif

#if AP_SIM_PS_TERARANGERTOWER_ENABLED
    if (terarangertower != nullptr) {
        terarangertower->update(sitl_model->get_location());
    }
#endif

#if AP_SIM_PS_LIGHTWARE_SF45B_ENABLED
    if (sf45b != nullptr) {
        sf45b->update(sitl_model->get_location());
    }
#endif

#if AP_SIM_ADSB_SAGETECH_MXS_ENABLED
    if (sagetech_mxs != nullptr) {
        sagetech_mxs->update(sitl_model);
    }
#endif

    if (vectornav != nullptr) {
        vectornav->update();
    }

    if (microstrain5 != nullptr) {
        microstrain5->update();
    }

    if (microstrain7 != nullptr) {
        microstrain7->update();
    }
    if (inertiallabs != nullptr) {
        inertiallabs->update();
    }

#if AP_SIM_AIS_ENABLED
    if (ais != nullptr) {
        ais->update(*sitl_model);
    }
    if (ais_replay != nullptr) {
        ais_replay->update();
    }
#endif
    for (uint8_t i=0; i<ARRAY_SIZE(gps); i++) {
        if (gps[i] != nullptr) {
            gps[i]->update();
        }
    }

    if (elrs != nullptr) {
        elrs->update();
    }
}

/*
  update voltage and current pins
 */
void SITL_State_Common::update_voltage_current(struct sitl_input &input, float throttle)
{
    float voltage = 0;
    float current = 0;
    
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
                    current += draw;
                }
            } else {
                // simulate simple battery setup
                // lose 0.7V at full throttle
                voltage = _sitl->batt_voltage - 0.7f * throttle;

                // assume 50A at full throttle
                current = 50.0f * throttle;
            }
        } else {
            // FDM provides voltage and current
            voltage = _sitl->state.battery_voltage;
            current = _sitl->state.battery_current;
        }
    }

    // assume 3DR power brick
    voltage_pin_voltage = (voltage / 10.1f);
    current_pin_voltage = current/17.0f;
    // fake battery2 as just a 25% gain on the first one
    voltage2_pin_voltage = voltage_pin_voltage * 0.25f;
    current2_pin_voltage = current_pin_voltage * 0.25f;
}

#endif // HAL_BOARD_SITL


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

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

#define streq(a, b) (!strcmp(a, b))
SITL::SerialDevice *SITL_State_Common::create_serial_sim(const char *name, const char *arg)
{
    if (streq(name, "benewake_tf02")) {
        if (benewake_tf02 != nullptr) {
            AP_HAL::panic("Only one benewake_tf02 at a time");
        }
        benewake_tf02 = new SITL::RF_Benewake_TF02();
        return benewake_tf02;
#if !defined(HAL_BUILD_AP_PERIPH)
    } else if (streq(name, "vicon")) {
        if (vicon != nullptr) {
            AP_HAL::panic("Only one vicon system at a time");
        }
        vicon = new SITL::Vicon();
        return vicon;
#endif
#if HAL_SIM_ADSB_ENABLED
    } else if (streq(name, "adsb")) {
        // ADSB is a stand-out as it is the only serial device which
        // will cope with begin() being called multiple times on a
        // serial port
        if (adsb == nullptr) {
            adsb = new SITL::ADSB();
        }
        sitl_model->set_adsb(adsb);
        return adsb;
#endif
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
    } else if (streq(name, "nooploop_tofsense")) {
        if (nooploop != nullptr) {
            AP_HAL::panic("Only one nooploop_tofsense at a time");
        }
        nooploop = new SITL::RF_Nooploop();
        return nooploop;
    } else if (streq(name, "teraranger_serial")) {
        if (teraranger_serial != nullptr) {
            AP_HAL::panic("Only one teraranger_serial at a time");
        }
        teraranger_serial = new SITL::RF_TeraRanger_Serial();
        return teraranger_serial;
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
    } else if (streq(name, "rds02uf")) {
        if (rds02uf != nullptr) {
            AP_HAL::panic("Only one rds02uf at a time");
        }
        rds02uf = new SITL::RF_RDS02UF();
        return rds02uf;
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

#if !defined(HAL_BUILD_AP_PERIPH)
    } else if (streq(name, "rf_mavlink")) {
        if (rf_mavlink != nullptr) {
            AP_HAL::panic("Only one rf_mavlink at a time");
        }
        rf_mavlink = new SITL::RF_MAVLink();
        return rf_mavlink;
#endif
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
#if HAL_SIM_PS_RPLIDARA1_ENABLED
    } else if (streq(name, "rplidara1")) {
        if (rplidara1 != nullptr) {
            AP_HAL::panic("Only one rplidara1 at a time");
        }
        rplidara1 = new SITL::PS_RPLidarA1();
        return rplidara1;
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
#if AP_SIM_ADSB_SAGETECH_MXS_ENABLED
    } else if (streq(name, "sagetech_mxs")) {
        if (sagetech_mxs != nullptr) {
            AP_HAL::panic("Only one sagetech_mxs at a time");
        }
        sagetech_mxs = new SITL::ADSB_Sagetech_MXS();
        if (adsb == nullptr) {
            adsb = new SITL::ADSB();
        }
        sitl_model->set_adsb(adsb);
        return sagetech_mxs;
#endif
#if !defined(HAL_BUILD_AP_PERIPH)
    } else if (streq(name, "loweheiser")) {
        sitl_model->set_loweheiser(&_sitl->loweheiser_sim);
        return &_sitl->loweheiser_sim;
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
    } else if (streq(name, "jre")) {
        if (jre != nullptr) {
            AP_HAL::panic("Only one jre at a time");
        }
        jre = new SITL::RF_JRE();
        return jre;
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
    } else if (streq(name, "hirth")) {
        if (efi_hirth != nullptr) {
            AP_HAL::panic("Only one hirth at a time");
        }
        efi_hirth = new SITL::EFI_Hirth();
        return efi_hirth;
    } else if (streq(name, "VectorNav")) {
        if (vectornav != nullptr) {
            AP_HAL::panic("Only one VectorNav at a time");
        }
        vectornav = new SITL::VectorNav();
        return vectornav;
    } else if (streq(name, "MicroStrain5")) {
        if (microstrain5 != nullptr) {
            AP_HAL::panic("Only one MicroStrain5 at a time");
        }
        microstrain5 = new SITL::MicroStrain5();
        return microstrain5;

    } else if (streq(name, "MicroStrain7")) {
        if (microstrain7 != nullptr) {
            AP_HAL::panic("Only one MicroStrain7 at a time");
        }
        microstrain7 = new SITL::MicroStrain7();
        return microstrain7;

    } else if (streq(name, "ILabs")) {
        if (inertiallabs != nullptr) {
            AP_HAL::panic("Only one InertialLabs INS at a time");
        }
        inertiallabs = new SITL::InertialLabs();
        return inertiallabs;

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
  update simulators
 */
void SITL_State_Common::sim_update(void)
{
#if HAL_SIM_GIMBAL_ENABLED
    if (gimbal != nullptr) {
        gimbal->update();
    }
#endif
#if HAL_SIM_ADSB_ENABLED
    if (adsb != nullptr) {
        adsb->update(*sitl_model);
    }
#endif
#if !defined(HAL_BUILD_AP_PERIPH)
    if (vicon != nullptr) {
        Quaternion attitude;
        sitl_model->get_attitude(attitude);
        vicon->update(sitl_model->get_location(),
                      sitl_model->get_position_relhome(),
                      sitl_model->get_velocity_ef(),
                      attitude);
    }
#endif
    if (benewake_tf02 != nullptr) {
        benewake_tf02->update(sitl_model->rangefinder_range());
    }
    if (benewake_tf03 != nullptr) {
        benewake_tf03->update(sitl_model->rangefinder_range());
    }
    if (benewake_tfmini != nullptr) {
        benewake_tfmini->update(sitl_model->rangefinder_range());
    }
    if (jre != nullptr) {
        jre->update(sitl_model->rangefinder_range());
    }
    if (nooploop != nullptr) {
        nooploop->update(sitl_model->rangefinder_range());
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
    if (rds02uf != nullptr) {
        rds02uf->update(sitl_model->rangefinder_range());
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

#if HAL_SIM_PS_RPLIDARA2_ENABLED
    if (rplidara2 != nullptr) {
        rplidara2->update(sitl_model->get_location());
    }
#endif

#if HAL_SIM_PS_RPLIDARA1_ENABLED
    if (rplidara1 != nullptr) {
        rplidara1->update(sitl_model->get_location());
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
    voltage2_pin_voltage = voltage_pin_voltage * .25f;
    current2_pin_voltage = current_pin_voltage * .25f;
}

#endif // HAL_BOARD_SITL


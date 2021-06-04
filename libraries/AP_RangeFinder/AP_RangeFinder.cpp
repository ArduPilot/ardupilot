/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_analog.h"
#include "AP_RangeFinder_PulsedLightLRF.h"
#include "AP_RangeFinder_MaxsonarI2CXL.h"
#include "AP_RangeFinder_MaxsonarSerialLV.h"
#include "AP_RangeFinder_BBB_PRU.h"
#include "AP_RangeFinder_LightWareI2C.h"
#include "AP_RangeFinder_LightWareSerial.h"
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) &&      \
    defined(HAVE_LIBIIO)
#include "AP_RangeFinder_Bebop.h"
#endif
#include "AP_RangeFinder_MAVLink.h"
#include "AP_RangeFinder_LeddarOne.h"
#include "AP_RangeFinder_uLanding.h"
#include "AP_RangeFinder_TeraRangerI2C.h"
#include "AP_RangeFinder_VL53L0X.h"
#include "AP_RangeFinder_VL53L1X.h"
#include "AP_RangeFinder_NMEA.h"
#include "AP_RangeFinder_Wasp.h"
#include "AP_RangeFinder_Benewake_TF02.h"
#include "AP_RangeFinder_Benewake_TF03.h"
#include "AP_RangeFinder_Benewake_TFMini.h"
#include "AP_RangeFinder_Benewake_TFMiniPlus.h"
#include "AP_RangeFinder_PWM.h"
#include "AP_RangeFinder_GYUS42v2.h"
#include "AP_RangeFinder_HC_SR04.h"
#include "AP_RangeFinder_BLPing.h"
#include "AP_RangeFinder_UAVCAN.h"
#include "AP_RangeFinder_Lanbao.h"
#include "AP_RangeFinder_LeddarVu8.h"
#include "AP_RangeFinder_SITL.h"
#include "AP_RangeFinder_MSP.h"
#include "AP_RangeFinder_USD1_CAN.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo RangeFinder::var_info[] = {

	// @Group: 1_
	// @Path: AP_RangeFinder_Params.cpp
	AP_SUBGROUPINFO(params[0], "1_", 25, RangeFinder, AP_RangeFinder_Params),

    // @Group: 1_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  57, RangeFinder, backend_var_info[0]),

#if RANGEFINDER_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, RangeFinder, AP_RangeFinder_Params),

    // @Group: 2_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, RangeFinder, backend_var_info[1]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 29, RangeFinder, AP_RangeFinder_Params),

    // @Group: 3_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_",  59, RangeFinder, backend_var_info[2]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 3
    // @Group: 4_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[3], "4_", 31, RangeFinder, AP_RangeFinder_Params),

    // @Group: 4_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[3], "4_",  60, RangeFinder, backend_var_info[3]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 4
    // @Group: 5_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[4], "5_", 33, RangeFinder, AP_RangeFinder_Params),

    // @Group: 5_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[4], "5_",  34, RangeFinder, backend_var_info[4]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 5
    // @Group: 6_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[5], "6_", 35, RangeFinder, AP_RangeFinder_Params),

    // @Group: 6_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[5], "6_",  36, RangeFinder, backend_var_info[5]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 6
    // @Group: 7_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[6], "7_", 37, RangeFinder, AP_RangeFinder_Params),

    // @Group: 7_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[6], "7_",  38, RangeFinder, backend_var_info[6]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 7
    // @Group: 8_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[7], "8_", 39, RangeFinder, AP_RangeFinder_Params),

    // @Group: 8_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[7], "8_",  40, RangeFinder, backend_var_info[7]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 8
    // @Group: 9_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[8], "9_", 41, RangeFinder, AP_RangeFinder_Params),

    // @Group: 9_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[8], "9_",  42, RangeFinder, backend_var_info[8]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 9
    // @Group: A_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[9], "A_", 43, RangeFinder, AP_RangeFinder_Params),

    // @Group: A_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[9], "A_",  44, RangeFinder, backend_var_info[9]),
#endif
    
    AP_GROUPEND
};

const AP_Param::GroupInfo *RangeFinder::backend_var_info[RANGEFINDER_MAX_INSTANCES];

RangeFinder::RangeFinder()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Rangefinder must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

void RangeFinder::convert_params(void) {
    if (params[0].type.configured_in_storage()) {
        // _params[0]._type will always be configured in storage after conversion is done the first time
        return;
    }

    struct ConversionTable {
        uint8_t old_element;
        uint8_t new_index;
        uint8_t instance;
    };

    const struct ConversionTable conversionTable[] = {
            // rangefinder 1
            {0, 0, 0}, //0, TYPE 1
            {1, 1, 0}, //1, PIN 1
            {2, 2, 0}, //2, SCALING 1
            {3, 3, 0}, //3, OFFSET 1
            {4, 4, 0}, //4, FUNCTION 1
            {5, 5, 0}, //5, MIN_CM 1
            {6, 6, 0}, //6, MAX_CM 1
            {7, 7, 0}, //7, STOP_PIN 1
            {9, 8, 0}, //9, RMETRIC 1
            {10, 9, 0}, //10, PWRRNG 1 (previously existed only once for all sensors)
            {11, 10, 0}, //11, GNDCLEAR 1
            {23, 11, 0}, //23, ADDR 1
            {49, 12, 0}, //49, POS 1
            {53, 13, 0}, //53, ORIENT 1

            // rangefinder 2
            {12, 0, 1}, //12, TYPE 2
            {13, 1, 1}, //13, PIN 2
            {14, 2, 1}, //14, SCALING 2
            {15, 3, 1}, //15, OFFSET 2
            {16, 4, 1}, //16, FUNCTION 2
            {17, 5, 1}, //17, MIN_CM 2
            {18, 6, 1}, //18, MAX_CM 2
            {19, 7, 1}, //19, STOP_PIN 2
            {21, 8, 1}, //21, RMETRIC 2
            {10, 9, 1}, //10, PWRRNG 1 (previously existed only once for all sensors)
            {22, 10, 1}, //22, GNDCLEAR 2
            {24, 11, 1}, //24, ADDR 2
            {50, 12, 1}, //50, POS 2
            {54, 13, 1}, //54, ORIENT 2
    };

    char param_name[17] = {0};
    AP_Param::ConversionInfo info;
    info.new_name = param_name;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    info.old_key = 71;
#elif APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    info.old_key = 53;
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
    info.old_key = 35;
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
    info.old_key = 197;
#else
    params[0].type.save(true);
    return; // no conversion is supported on this platform
#endif

    for (uint8_t i = 0; i < ARRAY_SIZE(conversionTable); i++) {
        uint8_t param_instance = conversionTable[i].instance + 1;
        uint8_t destination_index = conversionTable[i].new_index;

        info.old_group_element = conversionTable[i].old_element;
        info.type = (ap_var_type)AP_RangeFinder_Params::var_info[destination_index].type;

        hal.util->snprintf(param_name, sizeof(param_name), "RNGFND%X_%s", param_instance, AP_RangeFinder_Params::var_info[destination_index].name);
        param_name[sizeof(param_name)-1] = '\0';

        AP_Param::convert_old_parameter(&info, 1.0f, 0);
    }

    // force _params[0]._type into storage to flag that conversion has been done
    params[0].type.save(true);
}

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
void RangeFinder::init(enum Rotation orientation_default)
{
    if (init_done) {
        // init called a 2nd time?
        return;
    }
    init_done = true;

    convert_params();

    // set orientation defaults
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        params[i].orientation.set_default(orientation_default);
    }

    for (uint8_t i=0, serial_instance = 0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        WITH_SEMAPHORE(detect_sem);
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy). We use MAX()
            // here as a UAVCAN rangefinder may already have been
            // found
            num_instances = MAX(num_instances, i+1);
        }

        // initialise status
        state[i].status = Status::NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void RangeFinder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if ((Type)params[i].type.get() == Type::NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = Status::NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
        }
    }
#ifndef HAL_BUILD_AP_PERIPH
    Log_RFND();
#endif
}

bool RangeFinder::_add_backend(AP_RangeFinder_Backend *backend, uint8_t instance, uint8_t serial_instance)
{
    if (!backend) {
        return false;
    }
    if (instance >= RANGEFINDER_MAX_INSTANCES) {
        AP_HAL::panic("Too many RANGERS backends");
    }
    if (drivers[instance] != nullptr) {
        // we've allocated the same instance twice
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    backend->init_serial(serial_instance);
    drivers[instance] = backend;
    num_instances = MAX(num_instances, instance+1);

    return true;
}

/*
  detect if an instance of a rangefinder is connected. 
 */
void RangeFinder::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    const Type _type = (Type)params[instance].type.get();
    switch (_type) {
    case Type::PLI2C:
    case Type::PLI2CV3:
    case Type::PLI2CV3HP:
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_PulsedLightLRF::detect(i, state[instance], params[instance], _type),
                             instance)) {
                break;
            }
        }
        break;
    case Type::MBI2C: {
        uint8_t addr = AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR;
        if (params[instance].address != 0) {
            addr = params[instance].address;
        }
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_MaxsonarI2CXL::detect(state[instance], params[instance],
                                                                  hal.i2c_mgr->get_device(i, addr)),
                             instance)) {
                break;
            }
        }
        break;
    }
    case Type::LWI2C:
        if (params[instance].address) {
            // the LW20 needs a long time to boot up, so we delay 1.5s here
            if (!hal.util->was_watchdog_armed()) {
                hal.scheduler->delay(1500);
            }
#ifdef HAL_RANGEFINDER_LIGHTWARE_I2C_BUS
            _add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                                                             hal.i2c_mgr->get_device(HAL_RANGEFINDER_LIGHTWARE_I2C_BUS, params[instance].address)),
                                                             instance);
#else
            FOREACH_I2C(i) {
                if (_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                                                                     hal.i2c_mgr->get_device(i, params[instance].address)),
                                 instance)) {
                    break;
                }
            }
#endif
        }
        break;
    case Type::TRI2C:
        if (params[instance].address) {
            FOREACH_I2C(i) {
                if (_add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance], params[instance],
                                                                      hal.i2c_mgr->get_device(i, params[instance].address)),
                                 instance)) {
                    break;
                }
            }
        }
        break;
    case Type::VL53L0X:
    case Type::VL53L1X_Short:
            FOREACH_I2C(i) {
                if (_add_backend(AP_RangeFinder_VL53L0X::detect(state[instance], params[instance],
                                                                hal.i2c_mgr->get_device(i, params[instance].address)),
                        instance)) {
                    break;
                }
                if (_add_backend(AP_RangeFinder_VL53L1X::detect(state[instance], params[instance],
                                                                hal.i2c_mgr->get_device(i, params[instance].address),
                                                                _type == Type::VL53L1X_Short ?  AP_RangeFinder_VL53L1X::DistanceMode::Short :
                                                                AP_RangeFinder_VL53L1X::DistanceMode::Long),
                                 instance)) {
                    break;
                }
            }
        break;
    case Type::BenewakeTFminiPlus:
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_Benewake_TFMiniPlus::detect(state[instance], params[instance],
                                                                        hal.i2c_mgr->get_device(i, params[instance].address)),
                    instance)) {
                break;
            }
        }
        break;
    case Type::PX4_PWM:
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#ifndef HAL_BUILD_AP_PERIPH
        // to ease moving from PX4 to ChibiOS we'll lie a little about
        // the backend driver...
        if (AP_RangeFinder_PWM::detect()) {
            _add_backend(new AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height), instance);
        }
#endif
#endif
        break;
    case Type::BBB_PRU:
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
        if (AP_RangeFinder_BBB_PRU::detect()) {
            _add_backend(new AP_RangeFinder_BBB_PRU(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::LWSER:
        if (AP_RangeFinder_LightWareSerial::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_LightWareSerial(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::LEDDARONE:
        if (AP_RangeFinder_LeddarOne::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_LeddarOne(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::ULANDING:
        if (AP_RangeFinder_uLanding::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_uLanding(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::BEBOP:
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) && defined(HAVE_LIBIIO)
        if (AP_RangeFinder_Bebop::detect()) {
            _add_backend(new AP_RangeFinder_Bebop(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::MAVLink:
#ifndef HAL_BUILD_AP_PERIPH
        if (AP_RangeFinder_MAVLink::detect()) {
            _add_backend(new AP_RangeFinder_MAVLink(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::MBSER:
        if (AP_RangeFinder_MaxsonarSerialLV::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_MaxsonarSerialLV(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::ANALOG:
#ifndef HAL_BUILD_AP_PERIPH
        // note that analog will always come back as present if the pin is valid
        if (AP_RangeFinder_analog::detect(params[instance])) {
            _add_backend(new AP_RangeFinder_analog(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::HC_SR04:
#ifndef HAL_BUILD_AP_PERIPH
        // note that this will always come back as present if the pin is valid
        if (AP_RangeFinder_HC_SR04::detect(params[instance])) {
            _add_backend(new AP_RangeFinder_HC_SR04(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::NMEA:
        if (AP_RangeFinder_NMEA::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_NMEA(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::WASP:
        if (AP_RangeFinder_Wasp::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_Wasp(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::BenewakeTF02:
        if (AP_RangeFinder_Benewake_TF02::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_Benewake_TF02(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::BenewakeTFmini:
        if (AP_RangeFinder_Benewake_TFMini::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_Benewake_TFMini(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::BenewakeTF03:
        if (AP_RangeFinder_Benewake_TF03::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_Benewake_TF03(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::PWM:
#ifndef HAL_BUILD_AP_PERIPH
        if (AP_RangeFinder_PWM::detect()) {
            _add_backend(new AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height), instance);
        }
#endif
        break;
    case Type::BLPing:
        if (AP_RangeFinder_BLPing::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_BLPing(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::Lanbao:
        if (AP_RangeFinder_Lanbao::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_Lanbao(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::LeddarVu8_Serial:
        if (AP_RangeFinder_LeddarVu8::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_LeddarVu8(state[instance], params[instance]), instance, serial_instance++);
        }
        break;

    case Type::UAVCAN:
#if HAL_CANMANAGER_ENABLED
        /*
          the UAVCAN driver gets created when we first receive a
          measurement. We take the instance slot now, even if we don't
          yet have the driver
         */
        num_instances = MAX(num_instances, instance+1);
#endif
        break;

    case Type::GYUS42v2:
        if (AP_RangeFinder_GYUS42v2::detect(serial_instance)) {
            _add_backend(new AP_RangeFinder_GYUS42v2(state[instance], params[instance]), instance, serial_instance++);
        }
        break;

    case Type::SITL:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        _add_backend(new AP_RangeFinder_SITL(state[instance], params[instance], instance), instance);
#endif
        break;

    case Type::MSP:
#if HAL_MSP_RANGEFINDER_ENABLED
        if (AP_RangeFinder_MSP::detect()) {
            _add_backend(new AP_RangeFinder_MSP(state[instance], params[instance]), instance);
        }
#endif // HAL_MSP_RANGEFINDER_ENABLED
        break;

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    case Type::USD1_CAN:
        _add_backend(new AP_RangeFinder_USD1_CAN(state[instance], params[instance]), instance);
        break;
#endif
    case Type::NONE:
    default:
        break;
    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);

        // param count could have changed
        AP_Param::invalidate_count();
    }
}

AP_RangeFinder_Backend *RangeFinder::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == Type::NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

RangeFinder::Status RangeFinder::status_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return Status::NotConnected;
    }
    return backend->status();
}

void RangeFinder::handle_msg(const mavlink_message_t &msg)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && ((Type)params[i].type.get() != Type::NONE)) {
          drivers[i]->handle_msg(msg);
        }
    }
}

#if HAL_MSP_RANGEFINDER_ENABLED
void RangeFinder::handle_msp(const MSP::msp_rangefinder_data_message_t &pkt)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && ((Type)params[i].type.get() == Type::MSP)) {
          drivers[i]->handle_msp(pkt);
        }
    }
}
#endif // HAL_MSP_RANGEFINDER_ENABLED

// return true if we have a range finder with the specified orientation
bool RangeFinder::has_orientation(enum Rotation orientation) const
{
    return (find_instance(orientation) != nullptr);
}

// find first range finder instance with the specified orientation
AP_RangeFinder_Backend *RangeFinder::find_instance(enum Rotation orientation) const
{
    // first try for a rangefinder that is in range
    for (uint8_t i=0; i<num_instances; i++) {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->orientation() == orientation &&
            backend->status() == Status::Good) {
            return backend;
        }
    }
    // if none in range then return first with correct orientation
    for (uint8_t i=0; i<num_instances; i++) {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->orientation() == orientation) {
            return backend;
        }
    }
    return nullptr;
}

uint16_t RangeFinder::distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->distance_cm();
}

int16_t RangeFinder::max_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->max_distance_cm();
}

int16_t RangeFinder::min_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->min_distance_cm();
}

int16_t RangeFinder::ground_clearance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->ground_clearance_cm();
}

bool RangeFinder::has_data_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}

uint8_t RangeFinder::range_valid_count_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->range_valid_count();
}

const Vector3f &RangeFinder::get_pos_offset_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return pos_offset_zero;
    }
    return backend->get_pos_offset();
}

uint32_t RangeFinder::last_reading_ms(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->last_reading_ms();
}

MAV_DISTANCE_SENSOR RangeFinder::get_mav_distance_sensor_type_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    return backend->get_mav_distance_sensor_type();
}

// get temperature reading in C.  returns true on success and populates temp argument
bool RangeFinder::get_temp(enum Rotation orientation, float &temp) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->get_temp(temp);
}

// Write an RFND (rangefinder) packet
void RangeFinder::Log_RFND() const
{
    if (_log_rfnd_bit == uint32_t(-1)) {
        return;
    }

    AP_Logger &logger = AP::logger();
    if (!logger.should_log(_log_rfnd_bit)) {
        return;
    }

    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        const AP_RangeFinder_Backend *s = get_backend(i);
        if (s == nullptr) {
            continue;
        }

        const struct log_RFND pkt = {
                LOG_PACKET_HEADER_INIT(LOG_RFND_MSG),
                time_us      : AP_HAL::micros64(),
                instance     : i,
                dist         : s->distance_cm(),
                status       : (uint8_t)s->status(),
                orient       : s->orientation(),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}

bool RangeFinder::prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        if (((Type)params[i].type.get() != Type::NONE) && (drivers[i] == nullptr)) {
          hal.util->snprintf(failure_msg, failure_msg_len, "Rangefinder %X was not detected", i + 1);
          return false;
        }
    }

    return true;
}

RangeFinder *RangeFinder::_singleton;

namespace AP {

RangeFinder *rangefinder()
{
    return RangeFinder::get_singleton();
}

}

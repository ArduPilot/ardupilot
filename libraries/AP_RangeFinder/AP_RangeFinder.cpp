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
#include "AP_RangeFinder_USD1_Serial.h"
#include "AP_RangeFinder_TeraRangerI2C.h"
#include "AP_RangeFinder_TeraRanger_Serial.h"
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
#include "AP_RangeFinder_Bebop.h"
#include "AP_RangeFinder_BLPing.h"
#include "AP_RangeFinder_UAVCAN.h"
#include "AP_RangeFinder_Lanbao.h"
#include "AP_RangeFinder_LeddarVu8.h"
#include "AP_RangeFinder_SITL.h"
#include "AP_RangeFinder_MSP.h"
#include "AP_RangeFinder_USD1_CAN.h"
#include "AP_RangeFinder_Benewake_CAN.h"

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
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  57, RangeFinder, backend_var_info[0]),

#if RANGEFINDER_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, RangeFinder, AP_RangeFinder_Params),

    // @Group: 2_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, RangeFinder, backend_var_info[1]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 29, RangeFinder, AP_RangeFinder_Params),

    // @Group: 3_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_",  59, RangeFinder, backend_var_info[2]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 3
    // @Group: 4_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[3], "4_", 31, RangeFinder, AP_RangeFinder_Params),

    // @Group: 4_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[3], "4_",  60, RangeFinder, backend_var_info[3]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 4
    // @Group: 5_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[4], "5_", 33, RangeFinder, AP_RangeFinder_Params),

    // @Group: 5_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[4], "5_",  34, RangeFinder, backend_var_info[4]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 5
    // @Group: 6_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[5], "6_", 35, RangeFinder, AP_RangeFinder_Params),

    // @Group: 6_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[5], "6_",  36, RangeFinder, backend_var_info[5]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 6
    // @Group: 7_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[6], "7_", 37, RangeFinder, AP_RangeFinder_Params),

    // @Group: 7_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[6], "7_",  38, RangeFinder, backend_var_info[6]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 7
    // @Group: 8_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[7], "8_", 39, RangeFinder, AP_RangeFinder_Params),

    // @Group: 8_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[7], "8_",  40, RangeFinder, backend_var_info[7]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 8
    // @Group: 9_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[8], "9_", 41, RangeFinder, AP_RangeFinder_Params),

    // @Group: 9_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[8], "9_",  42, RangeFinder, backend_var_info[8]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 9
    // @Group: A_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[9], "A_", 43, RangeFinder, AP_RangeFinder_Params),

    // @Group: A_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
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
#if HAL_LOGGING_ENABLED
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
#if AP_RANGEFINDER_ENABLED
    AP_RangeFinder_Backend_Serial *(*serial_create_fn)(RangeFinder::RangeFinder_State&, AP_RangeFinder_Params&) = nullptr;

    const Type _type = (Type)params[instance].type.get();
    switch (_type) {
    case Type::PLI2C:
    case Type::PLI2CV3:
    case Type::PLI2CV3HP:
#if AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_PulsedLightLRF::detect(i, state[instance], params[instance], _type),
                             instance)) {
                break;
            }
        }
#endif
        break;
    case Type::MBI2C: {
#if AP_RANGEFINDER_MAXSONARI2CXL_ENABLED
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
#endif
    }
    case Type::LWI2C:
#if AP_RANGEFINDER_LWI2C_ENABLED
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
#endif  // AP_RANGEFINDER_LWI2C_ENABLED
        break;
    case Type::TRI2C:
#if AP_RANGEFINDER_TRI2C_ENABLED
        if (params[instance].address) {
            FOREACH_I2C(i) {
                if (_add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance], params[instance],
                                                                      hal.i2c_mgr->get_device(i, params[instance].address)),
                                 instance)) {
                    break;
                }
            }
        }
#endif
        break;
    case Type::VL53L0X:
    case Type::VL53L1X_Short:
            FOREACH_I2C(i) {
#if AP_RANGEFINDER_VL53L0X_ENABLED
                if (_add_backend(AP_RangeFinder_VL53L0X::detect(state[instance], params[instance],
                                                                hal.i2c_mgr->get_device(i, params[instance].address)),
                        instance)) {
                    break;
                }
#endif
#if AP_RANGEFINDER_VL53L1X_ENABLED
                if (_add_backend(AP_RangeFinder_VL53L1X::detect(state[instance], params[instance],
                                                                hal.i2c_mgr->get_device(i, params[instance].address),
                                                                _type == Type::VL53L1X_Short ?  AP_RangeFinder_VL53L1X::DistanceMode::Short :
                                                                AP_RangeFinder_VL53L1X::DistanceMode::Long),
                                 instance)) {
                    break;
                }
#endif
            }
        break;
    case Type::BenewakeTFminiPlus: {
#if AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED
        uint8_t addr = TFMINIPLUS_ADDR_DEFAULT;
        if (params[instance].address != 0) {
            addr = params[instance].address;
        }
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_Benewake_TFMiniPlus::detect(state[instance], params[instance],
                                                                        hal.i2c_mgr->get_device(i, addr)),
                    instance)) {
                break;
            }
        }
        break;
#endif
    }
    case Type::PX4_PWM:
#if AP_RANGEFINDER_PWM_ENABLED
        // to ease moving from PX4 to ChibiOS we'll lie a little about
        // the backend driver...
        if (AP_RangeFinder_PWM::detect()) {
            _add_backend(new AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height), instance);
        }
#endif
        break;
    case Type::BBB_PRU:
#if AP_RANGEFINDER_BBB_PRU_ENABLED
        if (AP_RangeFinder_BBB_PRU::detect()) {
            _add_backend(new AP_RangeFinder_BBB_PRU(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::LWSER:
#if AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED
        serial_create_fn = AP_RangeFinder_LightWareSerial::create;
#endif
        break;
    case Type::LEDDARONE:
#if AP_RANGEFINDER_LEDDARONE_ENABLED
        serial_create_fn = AP_RangeFinder_LeddarOne::create;
#endif
        break;
    case Type::USD1_Serial:
#if AP_RANGEFINDER_USD1_SERIAL_ENABLED
        serial_create_fn = AP_RangeFinder_USD1_Serial::create;
#endif
        break;
    case Type::BEBOP:
#if AP_RANGEFINDER_BEBOP_ENABLED
        if (AP_RangeFinder_Bebop::detect()) {
            _add_backend(new AP_RangeFinder_Bebop(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::MAVLink:
#if AP_RANGEFINDER_MAVLINK_ENABLED
        if (AP_RangeFinder_MAVLink::detect()) {
            _add_backend(new AP_RangeFinder_MAVLink(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::MBSER:
#if AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED
        serial_create_fn = AP_RangeFinder_MaxsonarSerialLV::create;
#endif
        break;
    case Type::ANALOG:
#if AP_RANGEFINDER_ANALOG_ENABLED
        // note that analog will always come back as present if the pin is valid
        if (AP_RangeFinder_analog::detect(params[instance])) {
            _add_backend(new AP_RangeFinder_analog(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::HC_SR04:
#if AP_RANGEFINDER_HC_SR04_ENABLED
        // note that this will always come back as present if the pin is valid
        if (AP_RangeFinder_HC_SR04::detect(params[instance])) {
            _add_backend(new AP_RangeFinder_HC_SR04(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::NMEA:
#if AP_RANGEFINDER_NMEA_ENABLED
        serial_create_fn = AP_RangeFinder_NMEA::create;
#endif
        break;
    case Type::WASP:
#if AP_RANGEFINDER_WASP_ENABLED
        serial_create_fn = AP_RangeFinder_Wasp::create;
#endif
        break;
    case Type::BenewakeTF02:
#if AP_RANGEFINDER_BENEWAKE_TF02_ENABLED
        serial_create_fn = AP_RangeFinder_Benewake_TF02::create;
#endif
        break;
    case Type::BenewakeTFmini:
#if AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED
        serial_create_fn = AP_RangeFinder_Benewake_TFMini::create;
#endif
        break;
    case Type::BenewakeTF03:
#if AP_RANGEFINDER_BENEWAKE_TF03_ENABLED
        serial_create_fn = AP_RangeFinder_Benewake_TF03::create;
#endif
        break;
    case Type::TeraRanger_Serial:
#if AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED
        serial_create_fn = AP_RangeFinder_TeraRanger_Serial::create;
#endif
        break;
    case Type::PWM:
#if AP_RANGEFINDER_PWM_ENABLED
        if (AP_RangeFinder_PWM::detect()) {
            _add_backend(new AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height), instance);
        }
#endif
        break;
    case Type::BLPing:
#if AP_RANGEFINDER_BLPING_ENABLED
        serial_create_fn = AP_RangeFinder_BLPing::create;
#endif
        break;
    case Type::Lanbao:
#if AP_RANGEFINDER_LANBAO_ENABLED
        serial_create_fn = AP_RangeFinder_Lanbao::create;
#endif
        break;
    case Type::LeddarVu8_Serial:
#if AP_RANGEFINDER_LEDDARVU8_ENABLED
        serial_create_fn = AP_RangeFinder_LeddarVu8::create;
#endif
        break;

    case Type::UAVCAN:
#if AP_RANGEFINDER_UAVCAN_ENABLED
        /*
          the UAVCAN driver gets created when we first receive a
          measurement. We take the instance slot now, even if we don't
          yet have the driver
         */
        num_instances = MAX(num_instances, instance+1);
#endif
        break;

    case Type::GYUS42v2:
#if AP_RANGEFINDER_GYUS42V2_ENABLED
        serial_create_fn = AP_RangeFinder_GYUS42v2::create;
#endif
        break;

    case Type::SIM:
#if AP_RANGEFINDER_SIM_ENABLED
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

    case Type::USD1_CAN:
#if AP_RANGEFINDER_USD1_CAN_ENABLED
        _add_backend(new AP_RangeFinder_USD1_CAN(state[instance], params[instance]), instance);
#endif
        break;
    case Type::Benewake_CAN:
#if AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
        _add_backend(new AP_RangeFinder_Benewake_CAN(state[instance], params[instance]), instance);
        break;
#endif
    case Type::NONE:
        break;
    }

    if (serial_create_fn != nullptr) {
        if (AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance)) {
            auto *b = serial_create_fn(state[instance], params[instance]);
            if (b != nullptr) {
                _add_backend(b, instance, serial_instance++);
            }
        }
    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);

        // param count could have changed
        AP_Param::invalidate_count();
    }
#endif //AP_RANGEFINDER_ENABLED
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

float RangeFinder::distance_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->distance();
}

uint16_t RangeFinder::distance_cm_orient(enum Rotation orientation) const
{
    return distance_orient(orientation) * 100.0;
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
        if ((Type)params[i].type.get() == Type::NONE) {
            continue;
        }

        if (drivers[i] == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "Rangefinder %X: Not Detected", i + 1);
            return false;
        }

        // backend-specific checks.  This might end up drivers[i]->arming_checks(...).
        switch (drivers[i]->allocated_type()) {
        case Type::ANALOG:
        case Type::PX4_PWM:
        case Type::PWM: {
            // ensure pin is configured
            if (params[i].pin == -1) {
                hal.util->snprintf(failure_msg, failure_msg_len, "RNGFND%u_PIN not set", unsigned(i + 1));
                return false;
            }
            // ensure that the pin we're configured to use is available
            if (!hal.gpio->valid_pin(params[i].pin)) {
                uint8_t servo_ch;
                if (hal.gpio->pin_to_servo_channel(params[i].pin, servo_ch)) {
                    hal.util->snprintf(failure_msg, failure_msg_len, "RNGFND%u_PIN=%d, set SERVO%u_FUNCTION=-1", unsigned(i + 1), int(params[i].pin.get()), unsigned(servo_ch+1));
                } else {
                    hal.util->snprintf(failure_msg, failure_msg_len, "RNGFND%u_PIN=%d invalid", unsigned(i + 1), int(params[i].pin.get()));
                }
                return false;
            }
            break;
        }
        default:
            break;
        }

        switch (drivers[i]->status()) {
        case Status::NoData:
            hal.util->snprintf(failure_msg, failure_msg_len, "Rangefinder %X: No Data", i + 1);
            return false;
        case Status::NotConnected:
            hal.util->snprintf(failure_msg, failure_msg_len, "Rangefinder %X: Not Connected", i + 1);
            return false;
        case Status::OutOfRangeLow:
        case Status::OutOfRangeHigh:
        case Status::Good:  
            break;
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


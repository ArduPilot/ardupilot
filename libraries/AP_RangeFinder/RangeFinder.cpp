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

#include "RangeFinder.h"
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
#include "AP_RangeFinder_Benewake.h"
#include "AP_RangeFinder_PWM.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo RangeFinder::var_info[] = {

	// @Group: 1_
	// @Path: AP_RangeFinder_Params.cpp
	AP_SUBGROUPINFO_FLAGS(params[0], "1_", 25, RangeFinder, AP_RangeFinder_Params, AP_PARAM_FLAG_IGNORE_ENABLE),

    // @Group: 1_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  26, RangeFinder, backend_var_info[0]),

#if RANGEFINDER_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, RangeFinder, AP_RangeFinder_Params),

    // @Group: 2_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  28, RangeFinder, backend_var_info[1]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 29, RangeFinder, AP_RangeFinder_Params),

    // @Group: 3_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_",  30, RangeFinder, backend_var_info[2]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 3
    // @Group: 4_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[3], "4_", 31, RangeFinder, AP_RangeFinder_Params),

    // @Group: 4_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "4_",  32, RangeFinder, backend_var_info[3]),
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

RangeFinder::RangeFinder(AP_SerialManager &_serial_manager, enum Rotation orientation_default) :
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);

    // set orientation defaults
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        params[i].orientation.set_default(orientation_default);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Rangefinder must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

void RangeFinder::convert_params(void) {
//    if (params[0].type.configured_in_storage()) {
//        // _params[0]._type will always be configured in storage after conversion is done the first time
//        return;
//    }

    struct ConversionTable {
        uint8_t old_element;
        uint8_t new_index;
        uint8_t instance;
    };

//    const struct ConversionTable conversionTable[] = {
//            {0, 0, 0},
//            {1, 1, 0},
//            {2, 2, 0},
//            {3, 3, 0},
//            {4, 4, 0},
//            {5, 5, 0},
//            {6, 6, 0},
//            {7, 7, 0},
//            {8, 8, 0},
//            {9, 9, 0},
//            {11, 11, 0},
//            {23, 23, 0},
//            {49, 49, 0},
//            {53, 53, 0},
//            //{57, , 0}, // backend
//
//            {12, 0, 1},
//            {13, 1, 1},
//            {14, 2, 1},
//            {15, 3, 1},
//            {16, 4, 1},
//            {17, 5, 1},
//            {18, 6, 1},
//            {19, 7, 1},
//            {20, 8, 1},
//            {21, 9, 1},
//            {22, 11, 1},
//            {24, 23, 1},
//            {50, 49, 1},
//            {54, 53, 1},
//            //{58, , 1}, // backend
//
//            {25, 0, 2},
//            {26, 1, 2},
//            {27, 2, 2},
//            {28, 3, 2},
//            {29, 4, 2},
//            {30, 5, 2},
//            {31, 6, 2},
//            {32, 7, 2},
//            {33, 8, 2},
//            {34, 9, 2},
//            {35, 11, 2},
//            {36, 23, 2},
//            {51, 49, 2},
//            {55, 53, 2},
//            //{59, , 2}, // backend
//
//            {37, 0, 3},
//            {38, 1, 3},
//            {39, 2, 3},
//            {40, 3, 3},
//            {41, 4, 3},
//            {42, 5, 3},
//            {43, 6, 3},
//            {44, 7, 3},
//            {45, 8, 3},
//            {46, 9, 3},
//            {47, 11, 3},
//            {48, 23, 3},
//            {52, 49, 3},
//            {56, 53, 3},
//            //{60, , 3}, // backend
//    };

    const struct ConversionTable conversionTable[] = {
            {0, 0, 0}, //0, TYPE 1
            {1, 1, 0}, //1, PIN 1
            {2, 2, 0}, //2, SCALING 1
            {3, 3, 0}, //3, OFFSET 1
            {4, 4, 0}, //4, FUNCTION 1
            {5, 5, 0}, //5, MIN_CM 1
            {6, 6, 0}, //6, MAX_CM 1
            {7, 7, 0}, //7, STOP_PIN 1
            {8, 8, 0}, //8, SETTLE 1
            {9, 9, 0}, //9, RMETRIC 1
            {10, 10, 0}, //10, PWRRNG 1 (previously existed only once for all sensors)
            {11, 11, 0}, //11, GNDCLEAR 1
            {12, 12, 0}, //23, ADDR 1
            {13, 13, 0}, //49, POS 1
            {14, 14, 0}, //53, ORIENT 1

            //{15, 1, 0}, //57, backend 1

            {16, 0, 1}, //12, TYPE 2
            {17, 1, 1}, //13, PIN 2
            {18, 2, 1}, //14, SCALING 2
            {19, 3, 1}, //15, OFFSET 2
            {20, 4, 1}, //16, FUNCTION 2
            {21, 5, 1}, //17, MIN_CM 2
            {22, 6, 1}, //18, MAX_CM 2
            {23, 7, 2}, //19, STOP_PIN 2
            {24, 8, 3}, //20, SETTLE 2
            {25, 9, 3}, //21, RMETRIC 2
            //{26, 10, 1}, //PWRRNG 2 (previously existed only once for all sensors)
            {26, 11, 1}, //22, GNDCLEAR 2
            {27, 12, 1}, //24, ADDR 2
            {28, 13, 1}, //50, POS 2
            {29, 14, 1}, //54, ORIENT 2

            //{30, 3, 1}, //58, backend 2

            {31, 0, 2}, //25, TYPE 3
            {32, 1, 2}, //26, PIN 3
            {33, 2, 2}, //27, SCALING 3
            {34, 3, 2}, //28, OFFSET 3
            {35, 4, 2}, //29, FUNCTION 3
            {36, 5, 2}, //30, MIN_CM 3
            {37, 6, 2}, //31, MAX_CM 3
            {38, 7, 2}, //32, STOP_PIN 3
            {39, 8, 2}, //33, SETTLE 3
            {40, 9, 2}, //34, RMETRIC 3
            //{41, 10, 2}, //PWRRNG 3 (previously existed only once for all sensors)
            {41, 11, 2}, //35, GNDCLEAR 3
            {42, 12, 2}, //36, ADDR 3
            {43, 13, 2}, //51, POS 3
            {44, 14, 2}, //55, ORIENT 3

            //{45, 5, 2}, //59, backend 3

            {46, 0, 3}, //37, TYPE 4
            {47, 1, 3}, //38, PIN 4
            {48, 2, 3}, //39, SCALING 4
            {49, 3, 3}, //40, OFFSET 4
            {50, 4, 3}, //41, FUNCTION 4
            {51, 5, 3}, //42, MIN_CM 4
            {52, 6, 3}, //43, MAX_CM 4
            {53, 7, 3}, //44, STOP_PIN 4
            {54, 8, 3}, //45, SETTLE 4
            {55, 9, 3}, //46, RMETRIC 4
            //{56, 10, 3}, //PWRRNG 4 (previously existed only once for all sensors)
            {56, 11, 3}, //47, GNDCLEAR 4
            {57, 12, 3}, //48, ADDR 4
            {58, 13, 3}, //52, POS 4
            {59, 14, 3}, //56, ORIENT 4

            //{60, 7, 3}, //60, backend 4
    };

    char param_name[17];
    AP_Param::ConversionInfo info;
    info.new_name = param_name;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    info.old_key = 71;
#elif APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    info.old_key = 53;
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
    info.old_key = 35;
#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
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

        hal.util->snprintf(param_name, 17, "RNGFND%X_%s", param_instance, AP_RangeFinder_Params::var_info[destination_index].name);

        printf("%s[%d/%d]: %d %d\n", info.new_name, i, ARRAY_SIZE(conversionTable), info.old_key, info.old_group_element);

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
void RangeFinder::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }

    convert_params();

    for (uint8_t i=0, serial_instance = 0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
        // initialise pre-arm check variables
        state[i].pre_arm_check = false;
        state[i].pre_arm_distance_min = 9999;  // initialise to an arbitrary large value
        state[i].pre_arm_distance_max = 0;

        // initialise status
        state[i].status = RangeFinder_NotConnected;
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
            if (params[i].type == RangeFinder_TYPE_NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = RangeFinder_NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
            drivers[i]->update_pre_arm_check();
        }
    }
}

bool RangeFinder::_add_backend(AP_RangeFinder_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_instances == RANGEFINDER_MAX_INSTANCES) {
        AP_HAL::panic("Too many RANGERS backends");
    }

    drivers[num_instances++] = backend;
    return true;
}

/*
  detect if an instance of a rangefinder is connected. 
 */
void RangeFinder::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    enum RangeFinder_Type _type = (enum RangeFinder_Type)params[instance].type.get();
    switch (_type) {
    case RangeFinder_TYPE_PLI2C:
    case RangeFinder_TYPE_PLI2CV3:
    case RangeFinder_TYPE_PLI2CV3HP:
        if (!_add_backend(AP_RangeFinder_PulsedLightLRF::detect(1, state[instance], params[instance], _type))) {
            _add_backend(AP_RangeFinder_PulsedLightLRF::detect(0, state[instance], params[instance], _type));
        }
        break;
    case RangeFinder_TYPE_MBI2C:
        if (!_add_backend(AP_RangeFinder_MaxsonarI2CXL::detect(state[instance], params[instance],
                                                hal.i2c_mgr->get_device(1, AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR)))) {
            _add_backend(AP_RangeFinder_MaxsonarI2CXL::detect(state[instance], params[instance],
                                               hal.i2c_mgr->get_device(0, AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR)));
        }
        break;
    case RangeFinder_TYPE_LWI2C:
        if (params[instance].address) {
#ifdef HAL_RANGEFINDER_LIGHTWARE_I2C_BUS
            _add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                hal.i2c_mgr->get_device(HAL_RANGEFINDER_LIGHTWARE_I2C_BUS, params[instance].address)));
#else
            if (!_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                                                                  hal.i2c_mgr->get_device(1, params[instance].address)))) {
                _add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                                                                 hal.i2c_mgr->get_device(0, params[instance].address)));
            }
#endif
        }
        break;
    case RangeFinder_TYPE_TRI2C:
        if (params[instance].address) {
            if (!_add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance], params[instance],
                                                                   hal.i2c_mgr->get_device(1, params[instance].address)))) {
                _add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance], params[instance],
                                                                  hal.i2c_mgr->get_device(0, params[instance].address)));
            }
        }
        break;
    case RangeFinder_TYPE_VL53L0X:
        if (!_add_backend(AP_RangeFinder_VL53L0X::detect(state[instance], params[instance],
                                                         hal.i2c_mgr->get_device(1, params[instance].address)))) {
            if (!_add_backend(AP_RangeFinder_VL53L0X::detect(state[instance], params[instance],
                                                             hal.i2c_mgr->get_device(0, params[instance].address)))) {
                if (!_add_backend(AP_RangeFinder_VL53L1X::detect(state[instance], params[instance],
                                                                 hal.i2c_mgr->get_device(1, params[instance].address)))) {
                    _add_backend(AP_RangeFinder_VL53L1X::detect(state[instance], params[instance],
                                                                hal.i2c_mgr->get_device(0, params[instance].address)));
                }
            }
        }
        break;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    case RangeFinder_TYPE_PX4_PWM:
        // to ease moving from PX4 to ChibiOS we'll lie a little about
        // the backend driver...
        if (AP_RangeFinder_PWM::detect()) {
            drivers[instance] = new AP_RangeFinder_PWM(state[instance], params[instance], _powersave_range, estimated_terrain_height);
        }
        break;
#endif
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    case RangeFinder_TYPE_BBB_PRU:
        if (AP_RangeFinder_BBB_PRU::detect()) {
            drivers[instance] = new AP_RangeFinder_BBB_PRU(state[instance], params[instance]);
        }
        break;
#endif
    case RangeFinder_TYPE_LWSER:
        if (AP_RangeFinder_LightWareSerial::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_LightWareSerial(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_LEDDARONE:
        if (AP_RangeFinder_LeddarOne::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_LeddarOne(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_ULANDING:
        if (AP_RangeFinder_uLanding::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_uLanding(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) && defined(HAVE_LIBIIO)
    case RangeFinder_TYPE_BEBOP:
        if (AP_RangeFinder_Bebop::detect()) {
            drivers[instance] = new AP_RangeFinder_Bebop(state[instance]);
        }
        break;
#endif
    case RangeFinder_TYPE_MAVLink:
        if (AP_RangeFinder_MAVLink::detect()) {
            drivers[instance] = new AP_RangeFinder_MAVLink(state[instance], params[instance]);
        }
        break;
    case RangeFinder_TYPE_MBSER:
        if (AP_RangeFinder_MaxsonarSerialLV::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_MaxsonarSerialLV(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_ANALOG:
        // note that analog will always come back as present if the pin is valid
        if (AP_RangeFinder_analog::detect(params[instance])) {
            drivers[instance] = new AP_RangeFinder_analog(state[instance], params[instance]);
        }
        break;
    case RangeFinder_TYPE_NMEA:
        if (AP_RangeFinder_NMEA::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_NMEA(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_WASP:
        if (AP_RangeFinder_Wasp::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Wasp(state[instance], params[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_BenewakeTF02:
        if (AP_RangeFinder_Benewake::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Benewake(state[instance], params[instance], serial_manager, serial_instance++, AP_RangeFinder_Benewake::BENEWAKE_TF02);
        }
        break;
    case RangeFinder_TYPE_BenewakeTFmini:
        if (AP_RangeFinder_Benewake::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Benewake(state[instance], params[instance], serial_manager, serial_instance++, AP_RangeFinder_Benewake::BENEWAKE_TFmini);
        }
        break;
    case RangeFinder_TYPE_PWM:
        if (AP_RangeFinder_PWM::detect()) {
            drivers[instance] = new AP_RangeFinder_PWM(state[instance], _powersave_range, estimated_terrain_height);
        }
        break;
    default:
        break;
    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);
    }
}

AP_RangeFinder_Backend *RangeFinder::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == RangeFinder_TYPE_NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

RangeFinder::RangeFinder_Status RangeFinder::status_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return RangeFinder_NotConnected;
    }
    return backend->status();
}

void RangeFinder::handle_msg(mavlink_message_t *msg)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && (params[i].type != RangeFinder_TYPE_NONE)) {
          drivers[i]->handle_msg(msg);
        }
    }
}

// return true if we have a range finder with the specified orientation
bool RangeFinder::has_orientation(enum Rotation orientation) const
{
    return (find_instance(orientation) != nullptr);
}

// find first range finder instance with the specified orientation
AP_RangeFinder_Backend *RangeFinder::find_instance(enum Rotation orientation) const
{
    for (uint8_t i=0; i<num_instances; i++) {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend == nullptr) {
            continue;
        }
        if (backend->orientation() != orientation) {
            continue;
        }
        return backend;
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

uint16_t RangeFinder::voltage_mv_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->voltage_mv();
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

/*
  returns true if pre-arm checks have passed for all range finders
  these checks involve the user lifting or rotating the vehicle so that sensor readings between
  the min and 2m can be captured
 */
bool RangeFinder::pre_arm_check() const
{
    for (uint8_t i=0; i<num_instances; i++) {
        // if driver is valid but pre_arm_check is false, return false
        if ((drivers[i] != nullptr) && (params[i].type != RangeFinder_TYPE_NONE) && !state[i].pre_arm_check) {
            return false;
        }
    }
    return true;
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

RangeFinder *RangeFinder::_singleton;

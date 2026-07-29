/*
  compile-time verification that each serial port's default protocol is
  actually compiled into this firmware.

  A board's hwdef may pin a port's default protocol, e.g.

      define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_SLCAN

  The value must be a SerialProtocol_* name (bare protocol numbers are rejected
  at hwdef-processing time -- see hwdef.py).  Some protocols are only
  meaningful when an optional feature is compiled in (SLCAN, for example, is
  the CAN<->serial bridge and only exists when the board has CAN interfaces).
  Pinning such a protocol as a port default while its feature is compiled out
  ships a default that silently does nothing.

  serial_protocol_compiled_in() maps each gated protocol to the feature macro
  that controls it, and the static_asserts below fire if a default selects a
  protocol whose feature is off in this build.  The feature macros are used as
  *values* (not via #if), so an undefined or renamed macro is a hard compile
  error here rather than a silently-wrong "off" -- include the owning
  subsystem's config header below when you add a protocol.

  Unlike a configure-time hwdef lint this runs with the fully-resolved,
  per-vehicle feature set, so it also covers protocols whose enablement is a
  vehicle/feature decision (Scripting, EFI, ...), not just board-level ones.

  This translation unit deliberately contains only static_asserts and a
  constexpr helper: it emits no runtime code.  The checks are off by default
  and are turned on (e.g. by Tools/scripts/test_new_boards.py, when building
  newly-added boards) by setting AP_SERIALMANAGER_DEFAULTS_CHECKS_ENABLED to 1.
 */

#include "AP_SerialManager_config.h"

#if AP_SERIALMANAGER_DEFAULTS_CHECKS_ENABLED

#include "AP_SerialManager.h"

// config headers providing the feature macros referenced below.  Keep these
// to lightweight *_config.h headers where the subsystem provides one.
#include <AP_HAL/AP_HAL_Boards.h>            // AP_CAN_SLCAN_ENABLED
#include <AP_RCProtocol/AP_RCProtocol_config.h>   // AP_RCPROTOCOL_ENABLED
#include <AP_MSP/AP_MSP_config.h>            // HAL_MSP_ENABLED
#include <AP_Scripting/AP_Scripting_config.h>     // AP_SCRIPTING_ENABLED
#include <AP_EFI/AP_EFI_config.h>            // HAL_EFI_ENABLED
#include <AP_Torqeedo/AP_Torqeedo_config.h>  // HAL_TORQEEDO_ENABLED
#include <AP_Generator/AP_Generator_config.h>     // HAL_GENERATOR_ENABLED
#include <AP_Networking/AP_Networking_Config.h>   // AP_NETWORKING_BACKEND_PPP
#include <AP_Volz_Protocol/AP_Volz_Protocol.h>    // AP_VOLZ_ENABLED
#include <AP_FETtecOneWire/AP_FETtecOneWire.h>    // AP_FETTEC_ONEWIRE_ENABLED
#include <AP_RobotisServo/AP_RobotisServo.h>      // AP_ROBOTISSERVO_ENABLED
#include <AP_Camera/AP_Camera_config.h>      // HAL_RUNCAM_ENABLED
#include <AP_VideoTX/AP_VideoTX_config.h>    // AP_SMARTAUDIO_ENABLED, AP_TRAMP_ENABLED
#include <AP_RCTelemetry/AP_RCTelemetry_config.h> // HAL_CRSF_TELEM_ENABLED
#include <AP_AIS/AP_AIS_config.h>            // AP_AIS_ENABLED
#include <AP_ADSB/AP_ADSB_config.h>          // HAL_ADSB_ENABLED
#include <AP_OpticalFlow/AP_OpticalFlow_config.h> // AP_OPTICALFLOW_ENABLED
#include <AP_Beacon/AP_Beacon_config.h>      // AP_BEACON_ENABLED
#include <AP_WindVane/AP_WindVane_config.h>  // AP_WINDVANE_ENABLED
#include <AP_Winch/AP_Winch_config.h>        // AP_WINCH_ENABLED
#include <AP_DDS/AP_DDS_config.h>            // AP_DDS_ENABLED
#include <AP_NMEA_Output/AP_NMEA_Output_config.h> // HAL_NMEA_OUTPUT_ENABLED
#include <AP_Mount/AP_Mount_config.h>        // HAL_MOUNT_ENABLED, HAL_MOUNT_ALEXMOS_ENABLED
#include <AP_Frsky_Telem/AP_Frsky_config.h>  // AP_FRSKY_TELEM_ENABLED
#include <AP_Devo_Telem/AP_Devo_Telem.h>     // AP_DEVO_TELEM_ENABLED
#include <AP_LTM_Telem/AP_LTM_Telem.h>       // AP_LTM_TELEM_ENABLED
#include <AP_Hott_Telem/AP_Hott_Telem.h>     // HAL_HOTT_TELEM_ENABLED
#include <AP_IBus2/AP_IBus2_config.h>        // AP_IBUS2_MASTER_ENABLED, AP_IBUS2_SLAVE_ENABLED

/*
  returns true if handling for the given serial protocol is compiled into this
  firmware.  Each gated protocol contributes "(p != X || (FEATURE))": if a
  port defaults to X while X's feature is off, that term -- and the whole
  expression -- is false.  Protocols that are always present (MAVLink, GPS,
  None, ...) have no term and fall through to true.  The feature macro is
  parenthesised because several expand to compound expressions.
 */
constexpr bool serial_protocol_compiled_in(AP_SerialManager::SerialProtocol p)
{
    return
        (p != AP_SerialManager::SerialProtocol_ADSB             || (HAL_ADSB_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_AIS              || (AP_AIS_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_AlexMos          || (HAL_MOUNT_ALEXMOS_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_Beacon           || (AP_BEACON_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_CRSF             || (HAL_CRSF_TELEM_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_DDS_XRCE         || (AP_DDS_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_Devo_Telem       || (AP_DEVO_TELEM_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_DJI_FPV          || (HAL_MSP_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_EFI              || (HAL_EFI_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_FETtecOneWire    || (AP_FETTEC_ONEWIRE_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_FrSky_D          || (AP_FRSKY_TELEM_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_FrSky_SPort      || (AP_FRSKY_TELEM_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough || (AP_FRSKY_TELEM_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_Generator        || (HAL_GENERATOR_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_Gimbal           || (HAL_MOUNT_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_Hott             || (HAL_HOTT_TELEM_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_IBUS2_Master     || (AP_IBUS2_MASTER_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_IBUS2_Slave      || (AP_IBUS2_SLAVE_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_IMUOUT           || (AP_SERIALMANAGER_IMUOUT_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_LTM_Telem        || (AP_LTM_TELEM_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_MSP              || (HAL_MSP_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_MSP_DisplayPort  || (HAL_MSP_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_NMEAOutput       || (HAL_NMEA_OUTPUT_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_OpticalFlow      || (AP_OPTICALFLOW_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_PPP              || (AP_NETWORKING_BACKEND_PPP)) &&
        (p != AP_SerialManager::SerialProtocol_RCIN             || (AP_RCPROTOCOL_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_Robotis          || (AP_ROBOTISSERVO_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_RunCam           || (HAL_RUNCAM_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_Scripting        || (AP_SCRIPTING_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_SLCAN            || (AP_CAN_SLCAN_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_SmartAudio       || (AP_SMARTAUDIO_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_Torqeedo         || (HAL_TORQEEDO_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_Tramp            || (AP_TRAMP_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_Volz             || (AP_VOLZ_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_Winch            || (AP_WINCH_ENABLED)) &&
        (p != AP_SerialManager::SerialProtocol_WindVane         || (AP_WINDVANE_ENABLED)) &&
        true;
}

// anchor that always references the helper (keeps it "used" even on a board
// that pins no port defaults) and sanity-checks a never-gated protocol
static_assert(serial_protocol_compiled_in(AP_SerialManager::SerialProtocol_Console),
              "Console must always be available");

// One check per serial port.  Only ports whose default is pinned by the hwdef
// define DEFAULT_SERIALn_PROTOCOL; the "AP_SerialManager::" prefix qualifies
// the expanded SerialProtocol_* name (numeric defaults are rejected by
// hwdef.py, so the value is always a name).
#ifdef DEFAULT_SERIAL0_PROTOCOL
static_assert(serial_protocol_compiled_in(AP_SerialManager::DEFAULT_SERIAL0_PROTOCOL), "SERIAL0 default protocol is not compiled into this firmware");
#endif
#ifdef DEFAULT_SERIAL1_PROTOCOL
static_assert(serial_protocol_compiled_in(AP_SerialManager::DEFAULT_SERIAL1_PROTOCOL), "SERIAL1 default protocol is not compiled into this firmware");
#endif
#ifdef DEFAULT_SERIAL2_PROTOCOL
static_assert(serial_protocol_compiled_in(AP_SerialManager::DEFAULT_SERIAL2_PROTOCOL), "SERIAL2 default protocol is not compiled into this firmware");
#endif
#ifdef DEFAULT_SERIAL3_PROTOCOL
static_assert(serial_protocol_compiled_in(AP_SerialManager::DEFAULT_SERIAL3_PROTOCOL), "SERIAL3 default protocol is not compiled into this firmware");
#endif
#ifdef DEFAULT_SERIAL4_PROTOCOL
static_assert(serial_protocol_compiled_in(AP_SerialManager::DEFAULT_SERIAL4_PROTOCOL), "SERIAL4 default protocol is not compiled into this firmware");
#endif
#ifdef DEFAULT_SERIAL5_PROTOCOL
static_assert(serial_protocol_compiled_in(AP_SerialManager::DEFAULT_SERIAL5_PROTOCOL), "SERIAL5 default protocol is not compiled into this firmware");
#endif
#ifdef DEFAULT_SERIAL6_PROTOCOL
static_assert(serial_protocol_compiled_in(AP_SerialManager::DEFAULT_SERIAL6_PROTOCOL), "SERIAL6 default protocol is not compiled into this firmware");
#endif
#ifdef DEFAULT_SERIAL7_PROTOCOL
static_assert(serial_protocol_compiled_in(AP_SerialManager::DEFAULT_SERIAL7_PROTOCOL), "SERIAL7 default protocol is not compiled into this firmware");
#endif
#ifdef DEFAULT_SERIAL8_PROTOCOL
static_assert(serial_protocol_compiled_in(AP_SerialManager::DEFAULT_SERIAL8_PROTOCOL), "SERIAL8 default protocol is not compiled into this firmware");
#endif
#ifdef DEFAULT_SERIAL9_PROTOCOL
static_assert(serial_protocol_compiled_in(AP_SerialManager::DEFAULT_SERIAL9_PROTOCOL), "SERIAL9 default protocol is not compiled into this firmware");
#endif

#endif  // AP_SERIALMANAGER_DEFAULTS_CHECKS_ENABLED

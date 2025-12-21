#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#define SITL_MCAST_IP "239.255.145.51"
#define SITL_MCAST_PORT 20721
#define SITL_SERVO_PORT 20722

#include <AP_HAL/utility/Socket_native.h>
#include <SITL/SIM_SoloGimbal.h>
#include <SITL/SIM_ADSB.h>
#include <SITL/SIM_ADSB_Sagetech_MXS.h>
#include <SITL/SIM_EFI_Hirth.h>
#include <SITL/SIM_Vicon.h>
#include <SITL/SIM_VectorNav.h>
#include <SITL/SIM_MicroStrain.h>
#include <SITL/SIM_InertialLabs.h>
#include <SITL/SIM_AIS.h>
#include <SITL/SIM_GPS.h>

#include <SITL/SIM_SerialRangeFinder.h>

#include <SITL/SIM_Frsky_D.h>
#include <SITL/SIM_CRSF.h>
// #include <SITL/SIM_Frsky_SPort.h>
// #include <SITL/SIM_Frsky_SPortPassthrough.h>
#include <SITL/SIM_PS_LD06.h>
#include <SITL/SIM_PS_RPLidarA2.h>
#include <SITL/SIM_PS_RPLidarA1.h>
#include <SITL/SIM_PS_RPLidarS2.h>
#include <SITL/SIM_PS_TeraRangerTower.h>
#include <SITL/SIM_PS_LightWare_SF45B.h>

#include <SITL/SIM_RichenPower.h>
#include <SITL/SIM_Loweheiser.h>
#include <SITL/SIM_FETtecOneWireESC.h>

#include <SITL/SIM_ELRS.h>

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "RCInput.h"

#include <sys/types.h>
#include <vector>

#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Terrain/AP_Terrain.h>
#include <SITL/SITL.h>
#include <SITL/SITL_Input.h>

class HAL_SITL;

class HALSITL::SITL_State_Common {
    friend class HALSITL::Scheduler;
    friend class HALSITL::Util;
    friend class HALSITL::GPIO;
public:
    virtual void init(int argc, char * const argv[]) = 0;

    enum vehicle_type {
        NONE,
        ArduCopter,
        Rover,
        ArduPlane,
        ArduSub,
        Blimp
    };

    // create a simulated serial device; type of device is given by
    // name parameter
    SITL::SerialDevice *create_serial_sim(const char *name, const char *arg, const uint8_t portNumber);

    // simulated airspeed, sonar and battery monitor
    float sonar_pin_voltage;    // pin 0
    float airspeed_pin_voltage[AIRSPEED_MAX_SENSORS]; // pin 1
    float voltage_pin_voltage;  // pin 13
    float current_pin_voltage;  // pin 12
    float voltage2_pin_voltage;  // pin 15
    float current2_pin_voltage;  // pin 14

    uint16_t pwm_input[SITL_RC_INPUT_CHANNELS];
    bool new_rc_input;
    uint16_t pwm_output[SITL_NUM_CHANNELS];
    bool output_ready = false;

#if AP_SIM_SOLOGIMBAL_ENABLED
    // simulated gimbal
    bool enable_gimbal;
    SITL::SoloGimbal *gimbal;
#endif

#if AP_SIM_ADSB_ENABLED
    // simulated ADSb
    SITL::ADSB *adsb;
#endif  // AP_SIM_ADSB_ENABLED

#if AP_SIM_ADSB_SAGETECH_MXS_ENABLED
    SITL::ADSB_Sagetech_MXS *sagetech_mxs;
#endif

#if AP_SIM_VICON_ENABLED
    // simulated vicon system:
    SITL::Vicon *vicon;
#endif  // AP_SIM_VICON_ENABLED

    SITL::SerialRangeFinder *serial_rangefinders[16];
    uint8_t num_serial_rangefinders;

    // simulated Frsky devices
    SITL::Frsky_D *frsky_d;
    // SITL::Frsky_SPort *frsky_sport;
    // SITL::Frsky_SPortPassthrough *frsky_sportpassthrough;

#if AP_SIM_PS_LD06_ENABLED
    // simulated LD06:
    SITL::PS_LD06 *ld06;
#endif  // AP_SIM_PS_LD06_ENABLED

#if AP_SIM_PS_RPLIDARA2_ENABLED
    // simulated RPLidarA2:
    SITL::PS_RPLidarA2 *rplidara2;
#endif

#if AP_SIM_PS_RPLIDARA1_ENABLED
    // simulated RPLidarA1:
    SITL::PS_RPLidarA1 *rplidara1;
#endif

#if AP_SIM_PS_RPLIDARS2_ENABLED
    // simulated RPLidarS2:
    SITL::PS_RPLidarS2 *rplidars2;
#endif

    // simulated FETtec OneWire ESCs:
    SITL::FETtecOneWireESC *fetteconewireesc;

#if AP_SIM_PS_LIGHTWARE_SF45B_ENABLED
    // simulated SF45B proximity sensor:
    SITL::PS_LightWare_SF45B *sf45b;
#endif

#if AP_SIM_PS_TERARANGERTOWER_ENABLED
    SITL::PS_TeraRangerTower *terarangertower;
#endif

#if AP_SIM_CRSF_ENABLED
    // simulated CRSF devices
    SITL::CRSF *crsf;
#endif

    // simulated VectorNav system:
    SITL::VectorNav *vectornav;

    // simulated MicroStrain system
    SITL::MicroStrain5 *microstrain5;

    // simulated MicroStrain system
    SITL::MicroStrain7 *microstrain7;

    // simulated InertialLabs INS
    SITL::InertialLabs *inertiallabs;

#if AP_SIM_JSON_MASTER_ENABLED
    // Ride along instances via JSON SITL backend
    SITL::JSON_Master ride_along;
#endif

#if AP_SIM_AIS_ENABLED
    // simulated AIS stream
    SITL::AIS *ais;
    SITL::AIS_Replay *ais_replay;
#endif

    // simulated EFI MegaSquirt device:
    SITL::EFI_MegaSquirt *efi_ms;

    // simulated EFI MegaSquirt device:
    SITL::EFI_Hirth *efi_hirth;

    // output socket for flightgear viewing
    SocketAPM_native fg_socket{true};
    
    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;

    // simulated GPS devices
    SITL::GPS *gps[AP_SIM_MAX_GPS_SENSORS];  // constrained by # of parameter sets

    // Simulated ELRS radio
    SITL::ELRS *elrs;

    // returns a voltage between 0V to 5V which should appear as the
    // voltage from the sensor
    float _sonar_pin_voltage() const;

    // multicast state
    int mc_out_fd = -1;
    
    // send out SITL state as UDP multicast
    void multicast_state_open(void);
    void multicast_state_send(void);

    // number of times we have paused the simulation for 1ms because
    // the TCP queue is full:
    uint32_t _serial_0_outqueue_full_count;

protected:
    enum vehicle_type _vehicle;

    void sim_update(void);

    // internal SITL model
    SITL::Aircraft *sitl_model;

    SITL::SIM *_sitl;

    void update_voltage_current(struct sitl_input &input, float throttle);
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

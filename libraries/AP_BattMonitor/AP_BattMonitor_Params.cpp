#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AP_BattMonitor_Params.h"
#include "AP_BattMonitor.h"

#if APM_BUILD_COPTER_OR_HELI
  #define DEFAULT_LOW_BATTERY_VOLTAGE 10.5f
#else
  #define DEFAULT_LOW_BATTERY_VOLTAGE 0.0f
#endif // APM_BUILD_COPTER_OR_HELI

const AP_Param::GroupInfo AP_BattMonitor_Params::var_info[] = {
    // @Param: MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Analog Voltage Only,4:Analog Voltage and Current,5:Solo,6:Bebop,7:SMBus-Generic,8:DroneCAN-BatteryInfo,9:ESC,10:Sum Of Selected Monitors,11:FuelFlow,12:FuelLevelPWM,13:SMBUS-SUI3,14:SMBUS-SUI6,15:NeoDesign,16:SMBus-Maxell,17:Generator-Elec,18:Generator-Fuel,19:Rotoye,20:MPPT,21:INA2XX,22:LTC2946,23:Torqeedo,24:FuelLevelAnalog,25:Synthetic Current and Analog Voltage
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("MONITOR", 1, AP_BattMonitor_Params, _type, int8_t(AP_BattMonitor::Type::NONE), AP_PARAM_FLAG_ENABLE),
    
    // 2 was VOLT_PIN

    // 3 was CURR_PIN

    // 4 was VOLT_MULT

    // 5 was AMP_PERVLT

    // 6 was AMP_OFFSET

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: CAPACITY
    // @DisplayName: Battery capacity
    // @Description: Capacity of the battery in mAh when full
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("CAPACITY", 7, AP_BattMonitor_Params, _pack_capacity, 3300),

    // @Param{Plane}: WATT_MAX
    // @DisplayName: Maximum allowed power (Watts)
    // @Description: If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.
    // @Units: W
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO_FRAME("WATT_MAX", 8, AP_BattMonitor_Params, _watt_max, 0, AP_PARAM_FRAME_PLANE),
#endif // HAL_BUILD_AP_PERIPH

    // @Param: SERIAL_NUM
    // @DisplayName: Battery serial number
    // @Description: Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With DroneCan it is the battery_id.
    // @User: Advanced
    AP_GROUPINFO("SERIAL_NUM", 9, AP_BattMonitor_Params, _serial_number, AP_BATT_SERIAL_NUMBER_DEFAULT),

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: LOW_TIMER
    // @DisplayName: Low voltage timeout
    // @Description: This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.
    // @Units: s
    // @Increment: 1
    // @Range: 0 120
    // @User: Advanced
    AP_GROUPINFO("LOW_TIMER", 10, AP_BattMonitor_Params, _low_voltage_timeout, 10),

    // @Param: FS_VOLTSRC
    // @DisplayName: Failsafe voltage source
    // @Description: Voltage type used for detection of low voltage event
    // @Values: 0:Raw Voltage, 1:Sag Compensated Voltage
    // @User: Advanced
    AP_GROUPINFO("FS_VOLTSRC", 11, AP_BattMonitor_Params, _failsafe_voltage_source, BattMonitor_LowVoltageSource_Raw),

    // @Param: LOW_VOLT
    // @DisplayName: Low battery voltage
    // @Description: Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the @PREFIX@LOW_TIMER parameter then the vehicle will perform the failsafe specified by the @PREFIX@FS_LOW_ACT parameter.
    // @Units: V
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LOW_VOLT", 12, AP_BattMonitor_Params, _low_voltage, DEFAULT_LOW_BATTERY_VOLTAGE),

    // @Param: LOW_MAH
    // @DisplayName: Low battery capacity
    // @Description: Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the @PREFIX@FS_LOW_ACT parameter.
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("LOW_MAH", 13, AP_BattMonitor_Params, _low_capacity, 0),

    // @Param: CRT_VOLT
    // @DisplayName: Critical battery voltage
    // @Description: Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the @PREFIX@LOW_TIMER parameter then the vehicle will perform the failsafe specified by the @PREFIX@FS_CRT_ACT parameter.
    // @Units: V
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("CRT_VOLT", 14, AP_BattMonitor_Params, _critical_voltage, 0),

    // @Param: CRT_MAH
    // @DisplayName: Battery critical capacity
    // @Description: Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the @PREFIX@_FS_CRT_ACT parameter.
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("CRT_MAH", 15, AP_BattMonitor_Params, _critical_capacity, 0),

    // @Param: FS_LOW_ACT
    // @DisplayName: Low battery failsafe action
    // @Description: What action the vehicle should perform if it hits a low battery failsafe
    // @Values{Plane}: 0:None,1:RTL,2:Land,3:Terminate,4:QLand,6:Loiter to QLand
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL or RTL,4:SmartRTL or Land,5:Terminate,6:Auto DO_LAND_START or RTL
    // @Values{Sub}: 0:None,2:Disarm,3:Enter surface mode
    // @Values{Rover}: 0:None,1:RTL,2:Hold,3:SmartRTL,4:SmartRTL or Hold,5:Terminate
    // @Values{Tracker}: 0:None
    // @Values{Blimp}: 0:None,1:Land
    // @User: Standard
    AP_GROUPINFO("FS_LOW_ACT", 16, AP_BattMonitor_Params, _failsafe_low_action, 0),

    // @Param: FS_CRT_ACT
    // @DisplayName: Critical battery failsafe action
    // @Description: What action the vehicle should perform if it hits a critical battery failsafe
    // @Values{Plane}: 0:None,1:RTL,2:Land,3:Terminate,4:QLand,5:Parachute,6:Loiter to QLand
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL or RTL,4:SmartRTL or Land,5:Terminate,6:Auto DO_LAND_START or RTL
    // @Values{Sub}: 0:None,2:Disarm,3:Enter surface mode
    // @Values{Rover}: 0:None,1:RTL,2:Hold,3:SmartRTL,4:SmartRTL or Hold,5:Terminate
    // @Values{Tracker}: 0:None
    // @Values{Blimp}: 0:None,1:Land
    // @User: Standard
    AP_GROUPINFO("FS_CRT_ACT", 17, AP_BattMonitor_Params, _failsafe_critical_action, 0),

    // @Param: ARM_VOLT
    // @DisplayName: Required arming voltage
    // @Description: Battery voltage level which is required to arm the aircraft. Set to 0 to allow arming at any voltage.
    // @Units: V
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ARM_VOLT", 18, AP_BattMonitor_Params, _arming_minimum_voltage, 0),

    // @Param: ARM_MAH
    // @DisplayName: Required arming remaining capacity
    // @Description: Battery capacity remaining which is required to arm the aircraft. Set to 0 to allow arming at any capacity. Note that execept for smart batteries rebooting the vehicle will always reset the remaining capacity estimate, which can lead to this check not providing sufficent protection, it is recommended to always use this in conjunction with the @PREFIX@_ARM_VOLT parameter.
    // @Units: mAh
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("ARM_MAH", 19, AP_BattMonitor_Params, _arming_minimum_capacity, 0),

    // 20 was BUS

    // @Param: OPTIONS
    // @DisplayName: Battery monitor options
    // @Description: This sets options to change the behaviour of the battery monitor
    // @Bitmask: 0:Ignore DroneCAN SoC, 1:MPPT reports input voltage and current, 2:MPPT Powered off when disarmed, 3:MPPT Powered on when armed, 4:MPPT Powered off at boot, 5:MPPT Powered on at boot, 6:Send resistance compensated voltage to GCS
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 21, AP_BattMonitor_Params, _options, 0),
#endif // HAL_BUILD_AP_PERIPH

    AP_GROUPEND

};

AP_BattMonitor_Params::AP_BattMonitor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

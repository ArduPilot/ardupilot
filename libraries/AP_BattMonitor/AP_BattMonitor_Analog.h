/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_BATTMONITOR_ANALOG_H
#define AP_BATTMONITOR_ANALOG_H

#include <AP_ADC.h>                 // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

// default pins and dividers
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
 # define AP_BATT_VOLT_PIN                  0       // Battery voltage on A0
 # define AP_BATT_CURR_PIN                  1       // Battery current on A1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       3.56    // on-board APM1 voltage divider with a 3.9kOhm resistor
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  0
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
 # define AP_BATT_VOLT_PIN                  13      // APM2.5/2.6 with 3dr power module
 # define AP_BATT_CURR_PIN                  12
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#elif CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
// Flymaple board pin 20 is connected to the external battery supply
// via a 24k/5.1k voltage divider. The schematic claims the divider is 25k/5k,
// but the actual installed resistors are not so.
// So the divider ratio is 5.70588 = (24000+5100)/5100
 # define AP_BATT_VOLT_PIN                  20
 # define AP_BATT_CURR_PIN                  19
 # define AP_BATT_VOLTDIVIDER_DEFAULT       5.70588
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4 && defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
 // px4
 # define AP_BATT_VOLT_PIN                  100
 # define AP_BATT_CURR_PIN                  101
 # define AP_BATT_VOLTDIVIDER_DEFAULT       1.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4 && defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
 // pixhawk
 # define AP_BATT_VOLT_PIN                  2
 # define AP_BATT_CURR_PIN                  3
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 # define AP_BATT_VOLT_PIN                  13
 # define AP_BATT_CURR_PIN                  12
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN && defined(CONFIG_ARCH_BOARD_VRBRAIN_V40)
 # define AP_BATT_VOLT_PIN                  10
 # define AP_BATT_CURR_PIN                   -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       1.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN && defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)
 # define AP_BATT_VOLT_PIN                  10
 # define AP_BATT_CURR_PIN                  11
 # define AP_BATT_VOLTDIVIDER_DEFAULT       1.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN && defined(CONFIG_ARCH_BOARD_VRBRAIN_V50)
 # define AP_BATT_VOLT_PIN                  10
 # define AP_BATT_CURR_PIN                  11
 # define AP_BATT_VOLTDIVIDER_DEFAULT       1.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN && defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
 # define AP_BATT_VOLT_PIN                  10
 # define AP_BATT_CURR_PIN                  11
 # define AP_BATT_VOLTDIVIDER_DEFAULT       1.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN && defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
 # define AP_BATT_VOLT_PIN                  10
 # define AP_BATT_CURR_PIN                  -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       1.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN && defined(CONFIG_ARCH_BOARD_VRHERO_V10)
 # define AP_BATT_VOLT_PIN                  100
 # define AP_BATT_CURR_PIN                  101
 # define AP_BATT_VOLTDIVIDER_DEFAULT       1.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#else
 # define AP_BATT_VOLT_PIN                  -1
 # define AP_BATT_CURR_PIN                  -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0
#endif

// Other values normally set directly by mission planner
// # define AP_BATT_VOLTDIVIDER_DEFAULT 15.70   // Volt divider for AttoPilot 50V/90A sensor
// # define AP_BATT_VOLTDIVIDER_DEFAULT 4.127   // Volt divider for AttoPilot 13.6V/45A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 27.32  // Amp/Volt for AttoPilot 50V/90A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 13.66  // Amp/Volt for AttoPilot 13.6V/45A sensor

class AP_BattMonitor_Analog : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_Analog(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read();

protected:

    AP_HAL::AnalogSource *_volt_pin_analog_source;
    AP_HAL::AnalogSource *_curr_pin_analog_source;
};
#endif  // AP_BATTMONITOR_ANALOG_H

#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

// default pins and dividers
#if defined(HAL_BATT_VOLT_PIN)
 // pins defined in board config (hwdef.dat on ChibiOS)
 # define AP_BATT_VOLT_PIN                  HAL_BATT_VOLT_PIN
 # define AP_BATT_CURR_PIN                  HAL_BATT_CURR_PIN
 # define AP_BATT_VOLTDIVIDER_DEFAULT       HAL_BATT_VOLT_SCALE
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  HAL_BATT_CURR_SCALE
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4 && defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
 // px4
 # define AP_BATT_VOLT_PIN                  100
 # define AP_BATT_CURR_PIN                  101
 # define AP_BATT_VOLTDIVIDER_DEFAULT       1.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4 && (defined(CONFIG_ARCH_BOARD_PX4FMU_V2) || defined(CONFIG_ARCH_BOARD_PX4FMU_V4) || defined(CONFIG_ARCH_BOARD_PX4FMU_V4PRO))
 // pixhawk
 # define AP_BATT_VOLT_PIN                  2
 # define AP_BATT_CURR_PIN                  3
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
 # define AP_BATT_VOLT_PIN                  4
 # define AP_BATT_CURR_PIN                  3
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
 # define AP_BATT_VOLT_PIN                  13
 # define AP_BATT_CURR_PIN                  12
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN && (defined(CONFIG_ARCH_BOARD_VRBRAIN_V45) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V52) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V52E) || defined(CONFIG_ARCH_BOARD_VRCORE_V10) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V54))
 # define AP_BATT_VOLT_PIN                  10
 # define AP_BATT_CURR_PIN                  11
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN && defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
 # define AP_BATT_VOLT_PIN                  10
 # define AP_BATT_CURR_PIN                  -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN && defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)
 # define AP_BATT_VOLT_PIN                  10
 # define AP_BATT_CURR_PIN                  1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX &&  (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF)
 # define AP_BATT_VOLT_PIN                  5
 # define AP_BATT_CURR_PIN                  6
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
 # define AP_BATT_VOLT_PIN                  0
 # define AP_BATT_CURR_PIN                  1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
 # define AP_BATT_VOLT_PIN                  0
 # define AP_BATT_CURR_PIN                  1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
 # define AP_BATT_VOLT_PIN                  1
 # define AP_BATT_CURR_PIN                  0
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2)
 # define AP_BATT_VOLT_PIN                  2
 # define AP_BATT_CURR_PIN                  3
 # define AP_BATT_VOLTDIVIDER_DEFAULT       11.3f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE)
 # define AP_BATT_VOLT_PIN                  3
 # define AP_BATT_CURR_PIN                  2
 # define AP_BATT_VOLTDIVIDER_DEFAULT       18.62
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  62.98f

 # define AP_BATT2_VOLT_PIN                  5
 # define AP_BATT2_CURR_PIN                  4
 # define AP_BATT2_VOLTDIVIDER_DEFAULT       18.62
 # define AP_BATT2_CURR_AMP_PERVOLT_DEFAULT  62.98f

#else
 # define AP_BATT_VOLT_PIN                  -1
 # define AP_BATT_CURR_PIN                  -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
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
    AP_BattMonitor_Analog(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read();

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return has_current(); }

    /// returns true if battery monitor provides current info
    bool has_current() const override;

    void init(void) override {}

protected:

    AP_HAL::AnalogSource *_volt_pin_analog_source;
    AP_HAL::AnalogSource *_curr_pin_analog_source;
};

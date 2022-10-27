#pragma once
#include "AP_BattMonitor_Analog.h"
#include "AP_HAL/I2CDevice.h"

#define AP_BATTMONITOR_ANALOG_GPIO_BUS_INTERNAL           0
#define AP_BATTMONITOR_ANALOG_GPIO_BUS_EXTERNAL           1
#define AP_BATTMONITOR_ANALOG_GPIO_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

// Battery switch board rev 2
#define AP_BATTMONITOR_ANALOG_GPIO_I2C_ADDR_REV2          0x49 //PCA9537
#define AP_BATTMONITOR_FET_EN_TETHER_REV2                 0x04 //Bit 3 of register 0

// Battery switch board rev 3
#define AP_BATTMONITOR_ANALOG_GPIO_I2C_ADDR_REV3          0x70 //PCAL9538APWJ
#define AP_BATTMONITOR_FET_EN_TETHER_REV3                 0x02 //Bit 1 of register 0

class AP_BattMonitor_Analog_GPIO : public AP_BattMonitor_Analog {
public:
    AP_BattMonitor_Analog_GPIO(AP_BattMonitor &mon,
        AP_BattMonitor::BattMonitor_State &mon_state,
        AP_BattMonitor_Params &params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void read() override;

    void init(void) override;

    bool get_is_using_battery() { return _is_using_battery; };

    /// returns true if battery monitor provides current info
    bool has_current() const override;

    /// return the I2C device address
    static uint8_t get_I2C_addr(const AP_BattMonitor_Params::BattMonitor_Type type);

private:
  AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
  bool _is_using_battery = false;
  uint8_t _use_tether_mask = 0x00;

  void timer();
};

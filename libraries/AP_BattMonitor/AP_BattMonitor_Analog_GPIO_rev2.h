#pragma once
#include "AP_BattMonitor_Analog.h"
#include "AP_HAL/I2CDevice.h"

#define AP_BATTMONITOR_ANALOG_GPIO_BUS_INTERNAL           0 // 0:mRoControlZeroH7 / 1:mRoControlZeroH7
#define AP_BATTMONITOR_ANALOG_GPIO_BUS_EXTERNAL           1
#define AP_BATTMONITOR_ANALOG_GPIO_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

// Battery switch board rev 2
#define AP_BATTMONITOR_ANALOG_GPIO_I2C_ADDR_REV2          0x49 //PCA9537 (7bit address since it does not include the bit for read/write)
#define AP_BATTMONITOR_FET_EN_TETHER_REV2                 0x04 //Bit 3 of register 0

class AP_BattMonitor_Analog_GPIO_rev2 : public AP_BattMonitor_Analog {
public:
    AP_BattMonitor_Analog_GPIO_rev2(AP_BattMonitor &mon,
        AP_BattMonitor::BattMonitor_State &mon_state,
        AP_BattMonitor_Params &params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void read() override;

    void init(void) override;

    /// returns true if battery monitor provides current info
    bool has_current() const override;

private:
  AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
  bool _is_using_battery = false;

  void timer();
};

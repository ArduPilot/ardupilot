#pragma once
#include "AP_BattMonitor_Analog.h"
#include "AP_HAL/I2CDevice.h"

// Battery switch board rev 3
#define AP_BATTMONITOR_ANALOG_GPIO_I2C_ADDR_REV3          0x70 //PCAL9538APWJ (7bit address since it does not include the bit for read/write)

#define AP_BATTMONITOR_INPUT_REGISTER_REV3                0x00
#define AP_BATTMONITOR_OUTPUT_REGISTER_REV3               0x01

#define AP_BATTMONITOR_CFG_REGISTER_REV3                  0x03
#define AP_BATTMONITOR_CFG_OUTPUT_REV3                    0xCF // Outputs: P4, P5: 11001111

#define AP_BATTMONITOR_FET_EN_TETHER_REV3                 0x02 //Bit 1
#define AP_BATTMONITOR_MCU_ALIVE_REV3                     0x80 //Bit 7

#define AP_BATTMONITOR_BATT_DISCO_EN_REV3                 0x10 //Bit 4
#define AP_BATTMONITOR_BATT_KILL_REV3                     0x20 //Bit 5


class AP_BattMonitor_Analog_GPIO_rev3 : public AP_BattMonitor_Analog {
public:
    AP_BattMonitor_Analog_GPIO_rev3(AP_BattMonitor &mon,
        AP_BattMonitor::BattMonitor_State &mon_state,
        AP_BattMonitor_Params &params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void read() override;

    void init(void) override;

    /// returns true if battery monitor provides current info
    bool has_current() const override;

    void set_batt_disco_en(bool enable) override;
    void set_batt_kill(bool enable) override;

private:
  AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
  bool _is_using_battery = false;
  bool _mcu_alive = false;

  struct send_state {
    bool batt_disco_en = false;
    bool batt_kill = false;
  } _send_state;
  bool _send_required = false;

  void timer();
};

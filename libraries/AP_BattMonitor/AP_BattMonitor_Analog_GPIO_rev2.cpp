#include "AP_BattMonitor_Analog_GPIO_rev2.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_BattMonitor_Analog_GPIO_rev2::AP_BattMonitor_Analog_GPIO_rev2(AP_BattMonitor &mon,
                                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                                       AP_BattMonitor_Params &params,
                                                       AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_Analog(mon, mon_state, params), _dev(std::move(dev)) {
  _state.on_tether_power = false;
}

void AP_BattMonitor_Analog_GPIO_rev2::init(void) {
    if (_dev) {
        _dev->register_periodic_callback(500000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_Analog_GPIO_rev2::timer, void));
    }
    AP_BattMonitor_Analog::init();
}

void AP_BattMonitor_Analog_GPIO_rev2::read(void) {
  //Read the analog status
  AP_BattMonitor_Analog::read();
}

void AP_BattMonitor_Analog_GPIO_rev2::timer() {
  //Read the state of the i2c device by reading one byte from register 0
  uint8_t buf;
  if(!_dev->read_registers(0, &buf, 1)) {
      return;
  }

  bool new_is_using_battery = (bool)((buf & AP_BATTMONITOR_FET_EN_TETHER_REV2) == 0); // Bit 1 of register 0 is FET_EN_TETHER.
  if(_is_using_battery ^ new_is_using_battery) {
      if (new_is_using_battery) {
          gcs().send_text(MAV_SEVERITY_CRITICAL, "Using battery power");
      } else {
          gcs().send_text(MAV_SEVERITY_CRITICAL, "Using tether power");
      }
  }

  //The state of the battery discharging is on GPIO 0
  _is_using_battery = new_is_using_battery;
  _state.on_tether_power = !_is_using_battery;
}

/// return true if battery provides current info
bool AP_BattMonitor_Analog_GPIO_rev2::has_current() const
{
    return (_params.type() == AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT_AND_GPIO_REV2);
}

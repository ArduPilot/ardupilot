#include "AP_BattMonitor_Analog_GPIO.h"

extern const AP_HAL::HAL& hal;

AP_BattMonitor_Analog_GPIO::AP_BattMonitor_Analog_GPIO(AP_BattMonitor &mon,
                                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                                       AP_BattMonitor_Params &params,
                                                       AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_Analog(mon, mon_state, params), _dev(std::move(dev)) {

}

void AP_BattMonitor_Analog_GPIO::init(void) {
    if (_dev) {
        _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_Analog_GPIO::timer, void));
    }
    AP_BattMonitor_Analog::init();
}

void AP_BattMonitor_Analog_GPIO::read(void) {
  //Read the analog status
  AP_BattMonitor_Analog::read();
}

void AP_BattMonitor_Analog_GPIO::timer() {
  //Read the state of the i2c device by reading one byte from register 0
  uint8_t buf;
  if(!_dev->read_registers(0, &buf, 1)) {
    return;
  }
  //The state of the battery discharging is on GPIO 0
  _is_using_battery = (bool)(buf & 0x01);
}

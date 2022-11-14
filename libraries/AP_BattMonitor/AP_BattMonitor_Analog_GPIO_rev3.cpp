#include "AP_BattMonitor_Analog_GPIO_rev3.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_BattMonitor_Analog_GPIO_rev3::AP_BattMonitor_Analog_GPIO_rev3(AP_BattMonitor &mon,
                                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                                       AP_BattMonitor_Params &params,
                                                       AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_Analog(mon, mon_state, params), _dev(std::move(dev)) {
  _state.on_tether_power = false;
  _state.mcu_alive = false;
}

void AP_BattMonitor_Analog_GPIO_rev3::init(void) {
    if (_dev) {
        _dev->register_periodic_callback(500000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_Analog_GPIO_rev3::timer, void));

        // Configuration inputs / outputs register
        _dev->write_register(AP_BATTMONITOR_CFG_REGISTER_REV3, AP_BATTMONITOR_CFG_OUTPUT_REV3);
    }
    AP_BattMonitor_Analog::init();
}

void AP_BattMonitor_Analog_GPIO_rev3::read(void) {
  //Read the analog status
  AP_BattMonitor_Analog::read();
}

void AP_BattMonitor_Analog_GPIO_rev3::timer() {
  //Read the state of the i2c device by reading one byte from register 0
  uint8_t buf;
  if(!_dev->read_registers(AP_BATTMONITOR_INPUT_REGISTER_REV3, &buf, 1)) {
    return;
  }

  // AP_BATTMONITOR_FET_EN_TETHER_REV3: Active high
  bool new_is_using_battery = (bool)((buf & AP_BATTMONITOR_FET_EN_TETHER_REV3) == 0);
  if(_is_using_battery ^ new_is_using_battery) {
    if (new_is_using_battery) {
      gcs().send_text(MAV_SEVERITY_CRITICAL, "Using battery power");
    } else {
      gcs().send_text(MAV_SEVERITY_CRITICAL, "Using tether power");
    }
  }
  _is_using_battery = new_is_using_battery;

  // AP_BATTMONITOR_MCU_ALIVE_REV3: Active low
  bool new_mcu_alive = (bool)((buf & AP_BATTMONITOR_MCU_ALIVE_REV3) == 0);
  if(_mcu_alive ^ new_mcu_alive) {
    if (new_mcu_alive) {
      gcs().send_text(MAV_SEVERITY_CRITICAL, "Battery MCU alive");
    } else {
      gcs().send_text(MAV_SEVERITY_CRITICAL, "Battery MCU not alive");
    }
  }
  _mcu_alive = new_mcu_alive;

  // Push state:
  _state.on_tether_power = !_is_using_battery;
  _state.mcu_alive   = _mcu_alive;

  if (_send_required) {
    // Configuration inputs / outputs register
    _dev->write_register(AP_BATTMONITOR_CFG_REGISTER_REV3, AP_BATTMONITOR_CFG_OUTPUT_REV3);

    // BATT_DISCO_EN: active low
    uint8_t register_value = 0;
    if(!_send_state.batt_disco_en)
      register_value |= AP_BATTMONITOR_BATT_DISCO_EN_REV3;

    // BATT_KILL: active high
    if(_send_state.batt_kill)
      register_value |= AP_BATTMONITOR_BATT_KILL_REV3;

    _dev->write_register(AP_BATTMONITOR_OUTPUT_REGISTER_REV3, register_value);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "BATT_DISCO_EN:(%d) BATT_KILL:(%d)", (int)_send_state.batt_disco_en, (int)_send_state.batt_kill);

    // flag updates sent
    _send_required = false;
  }
}

void AP_BattMonitor_Analog_GPIO_rev3::set_batt_disco_en(bool enable)
{
  if(_send_state.batt_disco_en !=  enable) {
    _send_state.batt_disco_en = enable;
    _send_required = true;
  }
}

void AP_BattMonitor_Analog_GPIO_rev3::set_batt_kill(bool enable)
{
  if(_send_state.batt_kill !=  enable) {
    _send_state.batt_kill = enable;
    _send_required = true;
  }
}

/// return true if battery provides current info
bool AP_BattMonitor_Analog_GPIO_rev3::has_current() const
{
    return (_params.type() == AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT_AND_GPIO_REV3);
}

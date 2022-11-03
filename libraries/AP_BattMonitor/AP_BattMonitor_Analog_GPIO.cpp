#include "AP_BattMonitor_Analog_GPIO.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_BattMonitor_Analog_GPIO::AP_BattMonitor_Analog_GPIO(AP_BattMonitor &mon,
                                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                                       AP_BattMonitor_Params &params,
                                                       AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_Analog(mon, mon_state, params), _dev(std::move(dev)) {
  _state.on_tether_power = false;

  if(params._type == AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT_AND_GPIO_REV2)
    _use_tether_mask = AP_BATTMONITOR_FET_EN_TETHER_REV2;

  else if(params._type == AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT_AND_GPIO_REV3)
    _use_tether_mask = AP_BATTMONITOR_FET_EN_TETHER_REV3;
}

void AP_BattMonitor_Analog_GPIO::init(void) {
    if (_dev) {
        _dev->register_periodic_callback(500000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_Analog_GPIO::timer, void));

        // May also need to set IO Expander configuration here with the correct
        // input pins and output pins

        // Configuration register: 0x03
        // Inputs P0, P1, P2, P3, P6, P7
        // Outputs: P4, P5
        // 11110011 = 0xF3
        _dev->write_register(0x03, 0xF3);
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

  bool new_is_using_battery = (bool)((buf & _use_tether_mask) == 0); // Bit 1 of register 0 is FET_EN_TETHER.
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


  // exit immediately if send not required, or state is being updated
  if (_params._type != AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT_AND_GPIO_REV3 || !_send_required) {
      return;
  }

  uint8_t register_value = 0;
  if(_send_state.batt_disco_en)
    register_value |= AP_BATTMONITOR_BATT_DISCO_EN_REV3;

  if(_send_state.batt_kill)
    register_value |= AP_BATTMONITOR_BATT_KILL_REV3;

  _dev->write_register(0x01, register_value);
  gcs().send_text(MAV_SEVERITY_CRITICAL, "BATT_DISCO_EN:(%d) BATT_KILL:(%d)", (int)_send_state.batt_disco_en, (int)_send_state.batt_kill);

  // flag updates sent
  _send_required = false;
}

void AP_BattMonitor_Analog_GPIO::set_batt_disco_en(bool enable)
{
  _send_state.batt_disco_en = enable;
  _send_required = true;
}

void AP_BattMonitor_Analog_GPIO::set_batt_kill(bool enable)
{
  _send_state.batt_kill = enable;
  _send_required = true;
}

uint8_t AP_BattMonitor_Analog_GPIO::get_I2C_addr(const AP_BattMonitor_Params::BattMonitor_Type type)
{
  if(type == AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT_AND_GPIO_REV2)
    return AP_BATTMONITOR_ANALOG_GPIO_I2C_ADDR_REV2;
  else if(type == AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT_AND_GPIO_REV3)
    return AP_BATTMONITOR_ANALOG_GPIO_I2C_ADDR_REV3;
  else
    return 0x00;
}

/// return true if battery provides current info
bool AP_BattMonitor_Analog_GPIO::has_current() const
{
    return (_params.type() == AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT_AND_GPIO_REV2 || AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT_AND_GPIO_REV3);
}

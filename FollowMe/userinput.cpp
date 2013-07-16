
#include <AP_HAL.h>
#include "userinput.h"

extern const AP_HAL::HAL& hal;

AP_HAL::AnalogSource* UserInput::_joy_x = NULL;
AP_HAL::AnalogSource* UserInput::_joy_y = NULL;
DigitalDebounce* UserInput::_side_btn = NULL;
DigitalDebounce* UserInput::_joy_btn = NULL;

uint32_t UserInput::_last_periodic = 0;

class DigitalInvert : public AP_HAL::DigitalSource {
public:
  DigitalInvert(AP_HAL::DigitalSource* p) : _p(p) {}
  uint8_t read() { return (_p->read()) ? 0 : 1; }
  void mode(uint8_t m) { _p->mode(m); }
  void write(uint8_t v) { _p->write( v == 0 ? 1 : 0 ); }
private:
  AP_HAL::DigitalSource *_p;
};

void UserInput::init( int side_btn_ch, int joy_x_ch,
                      int joy_y_ch, int joy_btn_ch) {

  _joy_x = hal.analogin->channel(joy_x_ch);
  _joy_y = hal.analogin->channel(joy_y_ch);
  _side_btn = new DigitalDebounce(
                new DigitalInvert(hal.gpio->channel(side_btn_ch)), 100);
  _joy_btn = new DigitalDebounce(hal.gpio->channel(joy_btn_ch), 100);
  hal.scheduler->register_timer_process(_periodic);
}

void UserInput::print(AP_HAL::BetterStream* s) {
  s->printf_P(PSTR("side: %d joy: %f, %f, %d\r\n"),
      (int) _side_btn->read(), 
      _joy_x->read_average(),
      _joy_y->read_average(),
      (int) _joy_btn->read());
}

void UserInput::_periodic(uint32_t us) {
  uint32_t millis = us / 1000;
  _side_btn->periodic(millis);
  _joy_btn->periodic(millis);
}

bool DigitalDebounce::read() {
  switch (_state) {
    case STATE_DOWN:
    case STATE_RISING:
      return false;
      break;
    case STATE_UP:
    case STATE_FALLING:
      return true;
      break;
  }
}

void DigitalDebounce::periodic(uint32_t millis) {
  bool latest = _in->read();
  uint32_t dt = millis - _last_periodic;
  _last_periodic = millis;
  switch (_state) {
    case STATE_DOWN:
      if (latest == true) {
        _state = STATE_RISING;
        _transition = 0;
      }
      break;
    case STATE_RISING:
      if (latest == true) {
        _transition += dt;
        if (_transition > _thresh_ms) {
          _state = STATE_UP;
          if (_evt_cb) {
            _evt_cb(BUTTON_UP);
          }
        }
      } else {
        _state = STATE_DOWN;
      }
      break;
    case STATE_UP:
      if (latest == false) {
        _state = STATE_FALLING;
        _transition = 0;
      }
      break;
    case STATE_FALLING:
      if (latest == false) {
        _transition += dt;
        if (_transition > _thresh_ms) {
          _state = STATE_DOWN;
          if (_evt_cb) {
            _evt_cb(BUTTON_DOWN);
          }
        }
      } else {
        _state = STATE_UP;
      }
      break;
  }
}


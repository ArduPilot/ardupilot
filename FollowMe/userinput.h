
#ifndef __FOLLOWME_USERINPUT_H__
#define __FOLLOWME_USERINPUT_H__

#include <AP_HAL.h>

class DigitalDebounce {
public:
    DigitalDebounce(AP_HAL::DigitalSource* in, int thresh_ms) :
        _in(in), _thresh_ms(thresh_ms), _state(STATE_UP)
    {}

    enum Event {
        BUTTON_DOWN,
        BUTTON_UP
    };

    enum State {
      STATE_DOWN,
      STATE_RISING,
      STATE_UP,
      STATE_FALLING
    };

    void periodic(uint32_t ms);
    void set_callback(void(*evt_cb)(int evt)) {
      _evt_cb = evt_cb;
    }
    bool get_raw() { return _in->read(); }
    int  get_state() { return _state; }
    bool read();
private:
    AP_HAL::DigitalSource* _in;
    int _thresh_ms;
    int _state;
    int _transition;
    uint32_t _last_periodic;
    void(*_evt_cb)(int evt);
};

class UserInput {
public:
    static void init(int side_btn_ch, int joy_x_ch, int joy_y_ch, int joy_btn_ch);
    static void print(AP_HAL::BetterStream* s);

    static float get_joy_x() { 
      return _joy_x->read_average();
    }

    static float get_joy_y() { 
      return _joy_y->read_average();
    }

    static void side_btn_event_callback(void(*cb)(int)) {
      _side_btn->set_callback(cb);
    }
    static void joy_btn_event_callback(void(*cb)(int)) {
      _joy_btn->set_callback(cb);
    }
private:
    static AP_HAL::AnalogSource* _joy_x;
    static AP_HAL::AnalogSource* _joy_y;
    static DigitalDebounce* _side_btn;
    static DigitalDebounce* _joy_btn;

    static void _periodic(uint32_t millis);
    static uint32_t _last_periodic;
};

#endif // __FOLLOWME_USERINPUT_H__


/*
  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.
 */
#include "dropping_module.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_SBusOut/AP_SBusOut.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_Periph.h"

#include "include/mavlink/v2.0/protocol.h"
#include "include/mavlink/v2.0/mavlink_types.h"
#include "include/mavlink/v2.0/ardupilotmega/mavlink.h"
#include "include/mavlink/v2.0/mavlink_helpers.h"


//#ifdef HAL_PERIPH_ENABLE_SBUS
#define DROPING_MODULE_MODE_DEF 0
extern const AP_HAL::HAL& hal;

// constructor
DROP_Module::DROP_Module(void)
{
}

void DROP_Module::init(AP_HAL::UARTDriver *_uart)
{
    this->state = DROPING_MODULE_MODE_DEF;
    uart = _uart;
    uart->begin(19200);
    uart->set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);
}

/*
  change_state DROP_Module
 */
void DROP_Module::change_state(uint8_t new_st)
{
    switch (new_st)
        {
        case NONE :
            this->st = NONE;
            break;
        case DROPING :
            this->st = DROPING;
            break;
        case ERROR :
            this->st = ERROR;
            break;
        
        default:
            this->st = NONE;
            error_count += 1;
            break;
        }
        //this->st = static_cast<DROP_Module_st>(new_st);
}

/*
read switch state ever 10ms
*/
void DROP_Module::sw_update()
{   

}

/*
send mavlink report 10ms
*/
void DROP_Module::mav_rep_send()
{   

}

/*
send hardbeat ever 1s
*/
void DROP_Module::mav_hardbeat_send()
{   

}

void DROP_Module::adc_update()
{

}
/*
  update DROP_Module
 */
bool DROP_Module::update()
{
    bool ret = false;
    static uint32_t last_sw_ms;
    static uint32_t last_adc_ms;
    uint32_t now = AP_HAL::millis();
    if (now - last_sw_ms > 10) {
        last_sw_ms = now;
        sw_update();

    }

    if (now - last_adc_ms > 10) {
        last_adc_ms = now;
        adc_update();
    }

    if(periph.g.debug_log_msg > 0){
        can_printf("Drop module status ");
    }

    switch (this->st)
    {
    case NONE :
        /* code */
        break;
    case DROPING :
        /* code */
        break;
    case ERROR :
        /* code */
        break;
    
    default:
        this->st = NONE;
        error_count += 1;
        ret = false;
        break;
    }
    
    return ret;
}

//#endif // HAL_PERIPH_ENABLE_DROP_Module


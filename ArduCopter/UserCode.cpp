#include "Copter.h"

// setup UART at 57600 kkouer add
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL)
    {
        // that UART doesn't exist on this platform
        return;
    }
    /// begin(baudrate,Rx,Tx)
    uart->begin(115200);
}

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    //防止参数错误导致初始化失败
    if(copter.g.data16_port_num<0)
        return;
    // put your initialisation code here
    // this will be called once at start-up
    gcs().send_text(MAV_SEVERITY_INFO, "user hook init!");
    setup_uart(hal.serial((uint8_t)(copter.g.data16_port_num)), "uartD");
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{

    uint8_t portNum = (uint8_t)(copter.g.data16_port_num);

    char incoming[16];
    uint8_t data;
    uint8_t index = 0;
    int16_t numc;

    numc = hal.serial(portNum)->available();

    // gcs().send_text(MAV_SEVERITY_INFO, "data16 port num %d",portNum);
    //gcs().send_text(MAV_SEVERITY_INFO, "data16 length %u ", numc);

    if(numc > 0)
        gcs().send_text(MAV_SEVERITY_INFO, "data 16 length %u ", numc);
    // hal.serial(portNum)->printf("serial number: %d\n",portNum);
    if (numc > 0)
    {
        mavlink_data16_t p1;
        memset(&p1, 0, sizeof(p1));

        hal.serial(portNum)->printf("in loop! length: %u \n", numc);

        //防止溢出错误
        if (numc > 16)
            numc = 16;
        for (int16_t i = 0; i < numc; i++)
        {
            data = hal.serial(portNum)->read();
            if (i < numc)
            {
                if (i == 0)
                {
                    p1.type = (uint8_t)data;
                }
                else
                {
                    p1.data[i - 1] = (uint8_t)data;
                }
                incoming[index++] = data;
            }
            if (i == numc - 1)
            {
                incoming[index] = '\0';
                index = 0;
                hal.serial(portNum)->printf("value: %s\n", incoming);
                p1.len = numc;

                mavlink_msg_data16_send(MAVLINK_COMM_0, p1.type, p1.len, p1.data);
            }
        }
    }

    // put your 1Hz code here
    // gcs().send_text(MAV_SEVERITY_INFO, "user hook 1 hz!");
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

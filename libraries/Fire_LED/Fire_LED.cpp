#include "Fire_LED.h"
/*#定义照明灯端口，暂留
PC6 POWER_LED OUTPUT LOW GPIO(2)
PC7 ALERT_LED OUTPUT LOW GPIO(3)
*/
void Fire_LED::Fire_LED_Init()
{
    hal.gpio->pinMode(2,HAL_GPIO_OUTPUT);  //设置成输出模式 
    hal.gpio->pinMode(3,HAL_GPIO_OUTPUT);  //设置成输出模式 
}

Fire_LED::Fire_LED(/* args */)
{
    ;
}


void Fire_LED::Fire_Alert_LED()
{
    uint16_t under_offset = 1550;   //死区设置
    uint16_t low_offset = 1450;
    // uint16_t mid_offset = 1500;
    uint16_t rcin_7 = hal.rcin->read(7); 
    if (rcin_7 > under_offset/* condition */)
    {
        Alert_led_on;
        /* code */
    }
    else if (rcin_7 < low_offset)
    {
        Alert_led_off;
    }
}

void Fire_LED::Fire_Power_LED(float V,uint8_t DT_ms)
{
    static uint16_t time_samp = 0;
    Fire_Alert_LED();
    if (V > 50.7f)   //电量大于百分之50%时
    {
        power_led_on;
    }
    else if(V > 48.75f)   //电量大于百分之25小于百分之和50
    {
        if(time_samp > 500)
            power_led_on;
        else if(time_samp < 500)
            power_led_off;
        if (time_samp > 1000/* condition */)
        {
            time_samp = 0;
            /* code */
        }
        time_samp += DT_ms;
    }
    else if(V > 46.8)   //电量在0～25%之间，发出快闪警告
    {
        if(time_samp > 300)  
            power_led_on;
        else if(time_samp < 300)
            power_led_off;
        if (time_samp > 400/* condition */)
        {
            time_samp = 0;
            /* code */
        }     
        time_samp += DT_ms;

    }
    else    //电池单节低于3.6伏，急需返航充电
    {
        if(time_samp > 100)   
            power_led_on;
        else if(time_samp < 100)
            power_led_off;
        if (time_samp > 200/* condition */)
        {
            time_samp = 0;
            /* code */
        }     
        time_samp += DT_ms;

    }

}


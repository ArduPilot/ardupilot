
#include <AP_HAL.h>
#include "GPIO.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO

#include "RCOutput_Navio.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

using namespace Linux;

#define PWM_CHAN_COUNT 13
#define PCA9685_OUTPUT_ENABLE RPI_GPIO_27

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void LinuxRCOutput_Navio::init(void* machtnicht)
{
    _i2c_sem = hal.i2c->get_semaphore();
    if (_i2c_sem == NULL) {
        hal.scheduler->panic(PSTR("PANIC: RCOutput_Navio did not get "
                                  "valid I2C semaphore!"));
        return; // never reached
    }

    // Set the initial frequency
    set_freq(0, 50);
    
    /* Enable PCA9685 PWM */
    enable_pin = hal.gpio->channel(PCA9685_OUTPUT_ENABLE);
    enable_pin->mode(HAL_GPIO_OUTPUT);
    enable_pin->write(0);
}

void LinuxRCOutput_Navio::set_freq(uint32_t chmask, uint16_t freq_hz)
{    
    if (!_i2c_sem->take(10)) {
        return;
    }
    
    // Put PCA9685 to sleep (required to write prescaler)
    hal.i2c->writeRegister(PCA9685_ADDRESS, PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT);
    
    // Calculate and write prescale value to match frequency
    uint8_t prescale = round(24576000.f / 4096.f / freq_hz)  - 1;
    hal.i2c->writeRegister(PCA9685_ADDRESS, PCA9685_RA_PRE_SCALE, prescale);
    
    // Reset all channels
    uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
    hal.i2c->writeRegisters(PCA9685_ADDRESS, PCA9685_RA_ALL_LED_ON_L, 4, data);
    
    // Enable external clocking
    hal.i2c->writeRegister(PCA9685_ADDRESS, PCA9685_RA_MODE1, 
                            PCA9685_MODE1_SLEEP_BIT | PCA9685_MODE1_EXTCLK_BIT); 
                            
    // Restart the device to apply new settings and enable auto-incremented write
    hal.i2c->writeRegister(PCA9685_ADDRESS, PCA9685_RA_MODE1, 
                            PCA9685_MODE1_RESTART_BIT | PCA9685_MODE1_AI_BIT);    
    _frequency = freq_hz;
    
    _i2c_sem->give();
}

uint16_t LinuxRCOutput_Navio::get_freq(uint8_t ch)
{
    return _frequency;
}

void LinuxRCOutput_Navio::enable_ch(uint8_t ch)
{
    
}

void LinuxRCOutput_Navio::disable_ch(uint8_t ch)
{
    write(ch, 0);
}

void LinuxRCOutput_Navio::write(uint8_t ch, uint16_t period_us)
{   
    if(ch >= PWM_CHAN_COUNT){
        return;
    }
    
    if (!_i2c_sem->take_nonblocking()) {
        return;
    }
    
    uint16_t length;
    
    if (period_us == 0)
        length = 0;
    else
        length = round((period_us * 4096) / (1000000.f / _frequency)) - 1;
        
    uint8_t data[2] = {length & 0xFF, length >> 8};
    uint8_t status = hal.i2c->writeRegisters(PCA9685_ADDRESS, 
                                             PCA9685_RA_LED0_OFF_L + 4 * (ch + 3), 
                                             2, 
                                             data);
                                             
    _i2c_sem->give();                                         
}

void LinuxRCOutput_Navio::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++)
        write(ch + i, period_us[i]);
}

uint16_t LinuxRCOutput_Navio::read(uint8_t ch)
{
    if (!_i2c_sem->take_nonblocking()) {
        return 0;
    }
    
    uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
    hal.i2c->readRegisters(PCA9685_ADDRESS, 
                           PCA9685_RA_LED0_ON_L + 4 * (ch + 3), 
                           4, 
                           data);
                           
    uint16_t length = data[2] + ((data[3] & 0x0F) << 8);    
    uint16_t period_us = (length + 1) * (1000000.f / _frequency) / 4096.f;
    
    _i2c_sem->give(); 
    
    return length == 0 ? 0 : period_us;   
}

void LinuxRCOutput_Navio::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) 
        period_us[i] = read(0 + i);
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO

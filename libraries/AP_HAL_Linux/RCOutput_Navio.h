
#ifndef __AP_HAL_LINUX_RCOUTPUT_NAVIO_H__
#define __AP_HAL_LINUX_RCOUTPUT_NAVIO_H__

#include <AP_HAL_Linux.h>

#define PCA9685_ADDRESS             0x40 // All address pins low, Navio default

#define PCA9685_RA_MODE1            0x00
#define PCA9685_RA_MODE2            0x01
#define PCA9685_RA_LED0_ON_L        0x06
#define PCA9685_RA_LED0_ON_H        0x07
#define PCA9685_RA_LED0_OFF_L       0x08
#define PCA9685_RA_LED0_OFF_H       0x09
#define PCA9685_RA_ALL_LED_ON_L     0xFA
#define PCA9685_RA_ALL_LED_ON_H     0xFB
#define PCA9685_RA_ALL_LED_OFF_L    0xFC
#define PCA9685_RA_ALL_LED_OFF_H    0xFD
#define PCA9685_RA_PRE_SCALE        0xFE

#define PCA9685_MODE1_RESTART_BIT   1<<7
#define PCA9685_MODE1_EXTCLK_BIT    1<<6
#define PCA9685_MODE1_AI_BIT        1<<5
#define PCA9685_MODE1_SLEEP_BIT     1<<4
#define PCA9685_MODE1_SUB1_BIT      1<<3
#define PCA9685_MODE1_SUB2_BIT      1<<2
#define PCA9685_MODE1_SUB3_BIT      1<<1
#define PCA9685_MODE1_ALLCALL_BIT   1<<0

#define PCA9685_MODE2_INVRT_BIT     1<<4
#define PCA9685_MODE2_OCH_BIT       1<<3
#define PCA9685_MODE2_OUTDRV_BIT    1<<2
#define PCA9685_MODE2_OUTNE1_BIT    1<<1
#define PCA9685_MODE2_OUTNE0_BIT    1<<0

class Linux::LinuxRCOutput_Navio : public AP_HAL::RCOutput {
    void     init(void* machtnichts);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

private:
    void reset();
    AP_HAL::Semaphore *_i2c_sem;
    AP_HAL::DigitalSource *enable_pin;
    uint16_t _frequency;    
};

#endif // __AP_HAL_LINUX_RCOUTPUT_NAVIO_H__

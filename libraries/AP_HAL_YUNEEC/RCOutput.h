
#ifndef __AP_HAL_YUNEEC_RCOUTPUT_H__
#define __AP_HAL_YUNEEC_RCOUTPUT_H__

#include <AP_HAL_YUNEEC.h>

#define CH_1 	0
#define CH_2 	1
#define CH_3 	2
#define CH_4 	3
#define CH_5 	4
#define CH_6 	5
#define CH_7 	6
#define CH_8 	7

#define	_BV(_N)		(1<<_N)

class YUNEEC::YUNEECRCOutput : public AP_HAL::RCOutput {
    void     init(void* machtnichts);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
};

#endif // __AP_HAL_YUNEEC_RCOUTPUT_H__

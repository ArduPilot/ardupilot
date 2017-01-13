
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
#include "RCOutput_Raspilot.h"

#include <cmath>
#include <dirent.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "px4io_protocol.h"
#include "GPIO.h"

using namespace Linux;

#define PWM_CHAN_COUNT 8

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void RCOutput_Raspilot::init()
{
    _dev = hal.spi->get_device("raspio");

    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&RCOutput_Raspilot::_update, void));
}

void RCOutput_Raspilot::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    _new_frequency = freq_hz;
}

uint16_t RCOutput_Raspilot::get_freq(uint8_t ch)
{
    return _frequency;
}

void RCOutput_Raspilot::enable_ch(uint8_t ch)
{

}

void RCOutput_Raspilot::disable_ch(uint8_t ch)
{
    write(ch, 0);
}

void RCOutput_Raspilot::write(uint8_t ch, uint16_t period_us)
{
    if(ch >= PWM_CHAN_COUNT){
        return;
    }

    _period_us[ch] = period_us;
}

uint16_t RCOutput_Raspilot::read(uint8_t ch)
{
    if(ch >= PWM_CHAN_COUNT){
        return 0;
    }

    return _period_us[ch];
}

void RCOutput_Raspilot::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++)
        period_us[i] = read(0 + i);
}

void RCOutput_Raspilot::_update(void)
{
    int i;

    if (_corked) {
        return;
    }

    if (_new_frequency) {
        _frequency = _new_frequency;
        _new_frequency = 0;
        struct IOPacket _dma_packet_tx, _dma_packet_rx;
        uint16_t count = 1;
        _dma_packet_tx.count_code = count | PKT_CODE_WRITE;
        _dma_packet_tx.page = 50;
        _dma_packet_tx.offset = 3;
        _dma_packet_tx.regs[0] = _frequency;
        _dma_packet_tx.crc = 0;
        _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
        _dev->transfer((uint8_t *)&_dma_packet_tx, sizeof(_dma_packet_tx),
                       (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_rx));
    }

    struct IOPacket _dma_packet_tx, _dma_packet_rx;
    uint16_t count = 1;
    _dma_packet_tx.count_code = count | PKT_CODE_WRITE;
    _dma_packet_tx.page = 50;
    _dma_packet_tx.offset = 1;
    _dma_packet_tx.regs[0] = 75;
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    _dev->transfer((uint8_t *)&_dma_packet_tx, sizeof(_dma_packet_tx),
                   (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_rx));

    count = 1;
    _dma_packet_tx.count_code = count | PKT_CODE_WRITE;
    _dma_packet_tx.page = 50;
    _dma_packet_tx.offset = 12;
    _dma_packet_tx.regs[0] = 0x560B;
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    _dev->transfer((uint8_t *)&_dma_packet_tx, sizeof(_dma_packet_tx),
                   (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_rx));

    count = PWM_CHAN_COUNT;
    _dma_packet_tx.count_code = count | PKT_CODE_WRITE;
    _dma_packet_tx.page = 54;
    _dma_packet_tx.offset = 0;
    for (i=0; i<PWM_CHAN_COUNT; i++) {
        _dma_packet_tx.regs[i] = _period_us[i];
    }
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    _dev->transfer((uint8_t *)&_dma_packet_tx, sizeof(_dma_packet_tx),
                   (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_rx));
}

void RCOutput_Raspilot::cork(void)
{
    _corked = true;
}

void RCOutput_Raspilot::push(void)
{
    _corked = false;
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

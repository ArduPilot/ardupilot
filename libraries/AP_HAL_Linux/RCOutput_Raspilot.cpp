
#include <AP_HAL/AP_HAL.h>
#include "GPIO.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include "RCOutput_Raspilot.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "px4io_protocol.h"

using namespace Linux;

#define PWM_CHAN_COUNT 8

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void RCOutput_Raspilot::init(void* machtnicht)
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_RASPIO);
    _spi_sem = _spi->get_semaphore();
    
    if (_spi_sem == NULL) {
        hal.scheduler->panic(PSTR("PANIC: RCOutput_Raspilot did not get "
                                  "valid SPI semaphore!"));
        return; // never reached
    }
    
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&RCOutput_Raspilot::_update, void));
}

void RCOutput_Raspilot::set_freq(uint32_t chmask, uint16_t freq_hz)
{    
    if (!_spi_sem->take(10)) {
        return;
    }
    
    struct IOPacket _dma_packet_tx, _dma_packet_rx;
    uint16_t count = 1;
    _dma_packet_tx.count_code = count | PKT_CODE_WRITE;
    _dma_packet_tx.page = 50;
    _dma_packet_tx.offset = 3;
    _dma_packet_tx.regs[0] = freq_hz;
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));
    
    _frequency = freq_hz;
    
    _spi_sem->give();
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
    
    if (hal.scheduler->micros() - _last_update_timestamp < 10000) {
        return;
    }
    
    _last_update_timestamp = hal.scheduler->micros();
    
    if (!_spi_sem->take_nonblocking()) {
        return;
    }

    struct IOPacket _dma_packet_tx, _dma_packet_rx;
    uint16_t count = 1;
    _dma_packet_tx.count_code = count | PKT_CODE_WRITE;
    _dma_packet_tx.page = 50;
    _dma_packet_tx.offset = 1;
    _dma_packet_tx.regs[0] = 75;
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));
    
    count = 1;
    _dma_packet_tx.count_code = count | PKT_CODE_WRITE;
    _dma_packet_tx.page = 50;
    _dma_packet_tx.offset = 12;
    _dma_packet_tx.regs[0] = 0x560B;
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));
    
    count = PWM_CHAN_COUNT;
    _dma_packet_tx.count_code = count | PKT_CODE_WRITE;
    _dma_packet_tx.page = 54;
    _dma_packet_tx.offset = 0;
    for (i=0; i<PWM_CHAN_COUNT; i++) {
        _dma_packet_tx.regs[i] = _period_us[i];
    }
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));
    
    _spi_sem->give();
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

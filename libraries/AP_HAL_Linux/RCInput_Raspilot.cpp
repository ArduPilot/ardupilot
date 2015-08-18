#include <AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>

#include "RCInput_Raspilot.h"

#include "protocol.h"

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

using namespace Linux;

void LinuxRCInput_Raspilot::init(void*)
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_RASPIO);
    _spi_sem = _spi->get_semaphore();
    
    if (_spi_sem == NULL) {
        hal.scheduler->panic(PSTR("PANIC: RCIutput_Raspilot did not get "
                                  "valid SPI semaphore!"));
        return; // never reached
    }
    
    // start the timer process to read samples
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&LinuxRCInput_Raspilot::_poll_data, void));
}

void LinuxRCInput_Raspilot::_poll_data(void)
{
    // Throttle read rate to 100hz maximum.
    if (hal.scheduler->micros() - _last_timer < 10000) {
        return;
    }
    
    _last_timer = hal.scheduler->micros();
    
    if (!_spi_sem->take_nonblocking()) {
        return;
    }

    struct IOPacket _dma_packet_tx, _dma_packet_rx;
    uint16_t count = LINUX_RC_INPUT_NUM_CHANNELS;
    _dma_packet_tx.count_code = count | PKT_CODE_READ;
    _dma_packet_tx.page = 4;
    _dma_packet_tx.offset = 0;
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    /* set raspilotio to read reg4 */
    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));
    /* get reg4 data from raspilotio */
    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));
    
    _process_rpio_data(_dma_packet_rx.regs);
    
    _spi_sem->give();
}

#endif // CONFIG_HAL_BOARD_SUBTYPE

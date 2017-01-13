#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "RCInput_Raspilot.h"

#include "px4io_protocol.h"

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace Linux;

void RCInput_Raspilot::init()
{
    _dev = hal.spi->get_device("raspio");

    // start the timer process to read samples
    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&RCInput_Raspilot::_poll_data, void));
}

void RCInput_Raspilot::_poll_data(void)
{
    struct IOPacket _dma_packet_tx, _dma_packet_rx;
    uint16_t count = LINUX_RC_INPUT_NUM_CHANNELS;
    _dma_packet_tx.count_code = count | PKT_CODE_READ;
    _dma_packet_tx.page = 4;
    _dma_packet_tx.offset = 0;
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    /* set raspilotio to read reg4 */
    _dev->transfer((uint8_t *)&_dma_packet_tx, sizeof(_dma_packet_tx),
                   (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_rx));
    /* get reg4 data from raspilotio */
    _dev->transfer((uint8_t *)&_dma_packet_tx, sizeof(_dma_packet_tx),
		   (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_rx));

    uint16_t num_values = _dma_packet_rx.regs[0];
    uint16_t rc_ok = _dma_packet_rx.regs[1] & (1 << 4);
    uint8_t rx_crc = _dma_packet_rx.crc;

    _dma_packet_rx.crc = 0;

    if ( rc_ok && (rx_crc == crc_packet(&_dma_packet_rx)) ) {
      _update_periods(&_dma_packet_rx.regs[6], (uint8_t)num_values);
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE

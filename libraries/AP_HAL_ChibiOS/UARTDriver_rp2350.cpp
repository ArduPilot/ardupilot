/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * UARTDriver support for the RP2350 PL011 UARTs, driven through the
 * ChibiOS SIO driver.
 */

#include <AP_HAL/AP_HAL.h>
#include <hal.h>
#include "UARTDriver.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && AP_HAL_UARTDRIVER_ENABLED && HAL_USE_SIO == TRUE

#include "shared_dma.h"
#include "hwdef/common/stm32_util.h"

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

// start or restart the PL011 at the current settings
void UARTDriver::sio_begin(bool clear_buffers)
{
    if (_baudrate != 0) {
#ifndef HAL_UART_NODMA
        // sioStart() leaves the FIFOs alone when the peripheral is already
        // running, so bytes framed at the old baud would survive into the
        // new window. Clearing FEN flushes RX and TX; setting it re-enables
        // them empty.
        if (clear_buffers && _device_initialised) {
            auto *pl011 = ((SIODriver*)sdef.serial)->uart;
            pl011->UARTLCR_H &= ~UART_UARTLCR_H_FEN;
            pl011->UARTLCR_H |=  UART_UARTLCR_H_FEN;
        }
        if (clear_buffers || !rx_dma_enabled) {
            // an armed channel does not match what this begin() is asking
            // for: it is either framing at the old baud, or it is about to
            // be left out of UARTDMACR. Either way it needs re-arming when
            // the flag next comes back on.
            rx_dma_running = false;
        }
        // Rx DMA for RP2350. Deliberately not gated on _device_initialised:
        // rx_dma_enabled is recomputed from the bounce buffers and
        // OPTION_NODMA_RX on every _begin(), so a port that first came up
        // without RX DMA - the option set, or a bounce buffer that could not
        // be allocated - would otherwise turn the flag back on later with no
        // channel behind it. _rx_timer_tick() then runs neither the DMA path,
        // which needs rxdma, nor read_bytes_NODMA(), which needs the flag
        // clear, and receive stops with nothing reading at all.
        if (rx_dma_enabled && rxdma == nullptr) {
            chSysLock();
            rxdma = dmaChannelAllocI(sdef.dma_rx_stream_id,
                                     3,   // IRQ priority
                                     (rp_dmaisr_t)rxbuff_full_irq,
                                     (void *)this);
            if (rxdma == nullptr) {
                // hwdef assigns fixed channel numbers, but the RP SPI and ADC
                // drivers take RP_DMA_CHANNEL_ID_ANY and start earlier, so the
                // assigned channel is usually gone by now. TREQ selects the
                // peripheral, so any free channel serves just as well.
                rxdma = dmaChannelAllocI(RP_DMA_CHANNEL_ID_ANY,
                                         3,
                                         (rp_dmaisr_t)rxbuff_full_irq,
                                         (void *)this);
            }
            if (rxdma != nullptr) {
                // Set source to UART RX data register (fixed/not-incremented)
                dmaChannelSetSourceX(rxdma,
                                     (uint32_t)&((SIODriver*)sdef.serial)->uart->UARTDR);
            } else {
                // leaving RXDMAE set with no channel behind it just overruns
                // the FIFO in silence
                rx_dma_enabled = false;
                rx_dma_running = false;
            }
            chSysUnlock();
        }
        _device_initialised = true;
        if (tx_dma_enabled && dma_handle == nullptr) {
            // TX DMA channel is managed via Shared_DMA wrapper
            dma_handle = NEW_NOTHROW Shared_DMA(sdef.dma_tx_stream_id,
                                                SHARED_DMA_NONE,
                                                FUNCTOR_BIND_MEMBER(&UARTDriver::dma_tx_allocate, void, Shared_DMA *),
                                                FUNCTOR_BIND_MEMBER(&UARTDriver::dma_tx_deallocate, void, Shared_DMA *));
            if (dma_handle == nullptr) {
                tx_dma_enabled = false;
            }
        }
#endif // HAL_UART_NODMA

        SIOConfig siocfg = {
            .baud      = _baudrate,
            .UARTLCR_H = sio_lcr_h(),
            .UARTCR    = UART_UARTCR_RXE | UART_UARTCR_TXE | UART_UARTCR_UARTEN,
            .UARTIFLS  = UART_UARTIFLS_RXIFLSEL_1_4F,
            .UARTDMACR = 0,
        };
#ifndef HAL_UART_NODMA
        if (rx_dma_enabled) {
            siocfg.UARTDMACR |= UART_UARTDMACR_RXDMAE;
        }
        if (tx_dma_enabled) {
            siocfg.UARTDMACR |= UART_UARTDMACR_TXDMAE;
        }
#endif // HAL_UART_NODMA
        /*
          the pads reset to FUNCSEL NULL and nothing in the startup path
          routes them to the UART. RX is pulled up before sioStart() so
          the UART does not see a permanent BREAK, whose interrupt storm
          would starve USB
         */
        const uint32_t uart_funcsel = sdef.uart_pin_funcsel ? sdef.uart_pin_funcsel : 2U;
        if (sdef.tx_line != 0) {
            /* TX pin: board-specific FUNCSEL (UART alternate function), IE+SCHMITT */
            palSetLineMode(sdef.tx_line, PAL_MODE_ALTERNATE(uart_funcsel));
        }
        if (sdef.rx_line != 0) {
            /* RX pin: board-specific FUNCSEL, IE+SCHMITT, then add PUE pull-up */
            palSetLineMode(sdef.rx_line, PAL_MODE_ALTERNATE(uart_funcsel));
            palLineSetPushPull(sdef.rx_line, PAL_PUSHPULL_PULLUP);
        }
        // set_options() normally runs before begin(), and the pad setup above wiped what it wrote
        sio_apply_inversion();
        sioStart((SIODriver*)sdef.serial, &siocfg);

        // the SIO path does not set these, and set_flow_control() below needs them
        arts_line = (ioline_t)sdef.rts_line;
        acts_line = (ioline_t)sdef.cts_line;

#ifndef HAL_UART_NODMA
        if (rx_dma_enabled && !rx_dma_running) {
            // the channel can already be live here, so take the lock:
            // rxbuff_full_irq() calls dma_rx_enable() as well, and both call
            // sites in _rx_timer_tick() hold it. dmaChannelDisableX() aborts
            // and clears, which is what discards the stale counter and the
            // bytes the bounce buffer framed at the previous settings.
            chSysLock();
            dmaChannelDisableX(rxdma);
            dma_rx_enable();
            chSysUnlock();
            rx_dma_running = true;
        }
#endif // HAL_UART_NODMA
    }
}

void UARTDriver::sio_apply_inversion()
{
    // an RP2350 line is the absolute GPIO number, which is how IO_BANK0 is indexed
    if (arx_line != 0) {
        uint32_t ctrl = IO_BANK0->GPIO[arx_line].CTRL;
        ctrl &= ~PAL_RP_IOCTRL_INOVER_DRVHIGH;  // both bits of the field
        ctrl |= option_is_set(OPTION_RXINV) ? PAL_RP_IOCTRL_INOVER_INV : PAL_RP_IOCTRL_INOVER_NOTINV;
        IO_BANK0->GPIO[arx_line].CTRL = ctrl;
    }
    if (atx_line != 0) {
        uint32_t ctrl = IO_BANK0->GPIO[atx_line].CTRL;
        ctrl &= ~PAL_RP_IOCTRL_OUTOVER_DRVHIGH;
        ctrl |= option_is_set(OPTION_TXINV) ? PAL_RP_IOCTRL_OUTOVER_DRVINVPERI : PAL_RP_IOCTRL_OUTOVER_DRVPERI;
        IO_BANK0->GPIO[atx_line].CTRL = ctrl;
    }
}

uint32_t UARTDriver::sio_lcr_h() const
{
    // parity: 0=none, 1=odd, 2=even. _cr2_options holds the stop bit count.
    uint32_t lcr_h = UART_UARTLCR_H_WLEN_8BITS | UART_UARTLCR_H_FEN;
    if (parity == 2U) {
        lcr_h |= UART_UARTLCR_H_PEN | UART_UARTLCR_H_EPS;
    } else if (parity == 1U) {
        lcr_h |= UART_UARTLCR_H_PEN;
    }
    if (_cr2_options == 2U) {
        lcr_h |= UART_UARTLCR_H_STP2;
    }
    return lcr_h;
}

/*
  reprogram the framing of a running PL011. Callers such as SBUS output
  change parity and stop bits after begin(), which is the only other place
  LCR_H is written
 */
void UARTDriver::sio_apply_framing()
{
    if (!_device_initialised) {
        // _begin() will program it
        return;
    }
    WITH_SEMAPHORE(_write_mutex);
    auto *pl011 = ((SIODriver*)sdef.serial)->uart;
    // BUSY stays set with bytes in the FIFO once UARTEN is cleared, so let
    // the transmitter drain first, bounded in case it is held off by CTS
    const uint32_t start_us = AP_HAL::micros();
    while ((pl011->UARTFR & UART_UARTFR_BUSY) && AP_HAL::micros() - start_us < 20000U) {
        hal.scheduler->delay_microseconds(100);
    }
    const uint32_t cr = pl011->UARTCR;
    pl011->UARTCR = cr & ~UART_UARTCR_UARTEN;
    pl011->UARTLCR_H = sio_lcr_h();
    pl011->UARTCR = cr;
}
#endif  // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && AP_HAL_UARTDRIVER_ENABLED && HAL_USE_SIO == TRUE

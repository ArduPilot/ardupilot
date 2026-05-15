/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && AP_HAL_UARTDRIVER_ENABLED

#include <hal.h>
#include "UARTDriver.h"
#include "GPIO.h"
#include <usbcfg.h>
#include "shared_dma.h"
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Common/ExpandingString.h>
#include "Scheduler.h"
#include "hwdef/common/stm32_util.h"
#ifndef HAL_BOOTLOADER_BUILD
// MAVLink is included to use the MAV_POWER flags for the USB power
#include <GCS_MAVLink/GCS_MAVLink.h>
#endif

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

#if defined(HAL_USB_VENDOR_ID) && !defined(HAVE_USB_SERIAL)
// USB has been configured in hwdef.dat
#define HAVE_USB_SERIAL
#endif

#if defined (STM32L4PLUS)
#ifndef USART_CR1_RXNEIE
#define USART_CR1_RXNEIE USART_CR1_RXNEIE_RXFNEIE
#endif
#endif

#if HAL_WITH_IO_MCU
extern ChibiOS::UARTDriver uart_io;
#endif

#ifdef HAL_BOOTLOADER_BUILD
// BL uses cout()/cin() via BOOTLOADER_DEV_LIST directly; _serial_tab unused
const UARTDriver::SerialDef UARTDriver::_serial_tab[] = {};
#else
const UARTDriver::SerialDef UARTDriver::_serial_tab[] = { HAL_SERIAL_DEVICE_LIST };
#endif

// handle for UART handling thread
thread_t* volatile UARTDriver::uart_rx_thread_ctx;

// table to find UARTDrivers from serial number, used for event handling
UARTDriver *UARTDriver::serial_drivers[UART_MAX_DRIVERS];

// event used to wake up waiting thread. This event number is for
// caller threads
static const eventmask_t EVT_DATA = EVENT_MASK(10);

// event for parity error
static const eventmask_t EVT_PARITY = EVENT_MASK(11);

// event for transmit end for half-duplex
static const eventmask_t EVT_TRANSMIT_END = EVENT_MASK(12);

// event for framing error
static const eventmask_t EVT_ERROR = EVENT_MASK(13);

// events for dma tx, thread per UART so can be from 0
static const eventmask_t EVT_TRANSMIT_DMA_START = EVENT_MASK(0);
static const eventmask_t EVT_TRANSMIT_DMA_COMPLETE = EVENT_MASK(1);
static const eventmask_t EVT_TRANSMIT_DATA_READY = EVENT_MASK(2);
static const eventmask_t EVT_TRANSMIT_UNBUFFERED = EVENT_MASK(3);

#ifndef HAL_UART_MIN_TX_SIZE
#define HAL_UART_MIN_TX_SIZE 512
#endif

#ifndef HAL_UART_MIN_RX_SIZE
#define HAL_UART_MIN_RX_SIZE 512
#endif

#ifndef HAL_UART_STACK_SIZE
#define HAL_UART_STACK_SIZE 320
#endif

#ifndef HAL_UART_RX_STACK_SIZE
#define HAL_UART_RX_STACK_SIZE 768
#endif

// threshold for disabling TX DMA due to contention
#if defined(USART_CR1_FIFOEN)
#define CONTENTION_BAUD_THRESHOLD 460800
#else
#define CONTENTION_BAUD_THRESHOLD 115200
#endif

UARTDriver::UARTDriver(uint8_t _serial_num) :
sdef(_serial_tab[_serial_num]),
serial_num(_serial_num),
_baudrate(57600)
{
    osalDbgAssert(serial_num < UART_MAX_DRIVERS, "too many SERIALn drivers");
    serial_drivers[serial_num] = this;
}

/*
  thread for handling UART send/receive

  We use events indexed by serial_num to trigger a more rapid send for
  unbuffered_write uarts, and run at 1kHz for general UART handling
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=128"
void UARTDriver::uart_thread()
{
    uint32_t last_thread_run_us = 0; // last time we did a 1kHz run of this uart

    while (uart_thread_ctx == nullptr) {
        hal.scheduler->delay_microseconds(1000);
    }

    while (true) {
        eventmask_t mask = chEvtWaitAnyTimeout(EVT_TRANSMIT_DATA_READY | EVT_TRANSMIT_END | EVT_TRANSMIT_UNBUFFERED, chTimeMS2I(1));
        uint32_t now = AP_HAL::micros();
        bool need_tick = false;
        if (now - last_thread_run_us >= 1000) {
            // run the timer tick if it's been more than 1ms since we last run
            need_tick = true;
            last_thread_run_us = now;
        }

        // change the thread priority if requested - if unbuffered it should only have higher priority than the owner so that
        // handoff occurs immediately
        if (mask & EVT_TRANSMIT_UNBUFFERED) {
            chThdSetPriority(unbuffered_writes ? MIN(_uart_owner_thd->realprio + 1, APM_UART_UNBUFFERED_PRIORITY) : APM_UART_PRIORITY);
        }
#ifndef HAL_UART_NODMA
        osalDbgAssert(!dma_handle || !dma_handle->is_locked(), "DMA handle is already locked");
#endif
        // send more data
        if (_tx_initialised && ((mask & EVT_TRANSMIT_DATA_READY) || need_tick || (hd_tx_active && (mask & EVT_TRANSMIT_END)))) {
            _tx_timer_tick();
        }
    }
}
#pragma GCC diagnostic pop

/*
  thread for handling UART receive

  We use events indexed by serial_num to trigger a more rapid send for
  unbuffered_write uarts, and run at 1kHz for general UART handling
 */
void UARTDriver::uart_rx_thread(void* arg)
{
    while (uart_rx_thread_ctx == nullptr) {
        hal.scheduler->delay_microseconds(1000);
    }

    while (true) {
        hal.scheduler->delay_microseconds(1000);

        for (uint8_t i=0; i<UART_MAX_DRIVERS; i++) {
            if (serial_drivers[i] == nullptr) {
                continue;
            }
            if (serial_drivers[i]->_rx_initialised) {
                serial_drivers[i]->_rx_timer_tick();
            }
        }
    }
}

/*
  initialise UART RX thread
 */
void UARTDriver::thread_rx_init(void)
{
    if (uart_rx_thread_ctx == nullptr) {
        uart_rx_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(HAL_UART_RX_STACK_SIZE),
                                              "UART_RX",
                                              APM_UART_PRIORITY,
                                              uart_rx_thread,
                                              nullptr);
        if (uart_rx_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create UART RX thread");
        }
    }
}

/*
  initialise UART TX_thread
 */
void UARTDriver::thread_init(void)
{
    if (uart_thread_ctx == nullptr) {
        hal.util->snprintf(uart_thread_name, sizeof(uart_thread_name), sdef.is_usb ? "OTG%1u" : "UART%1u", sdef.instance);
/*
 * Briefly boost priority so the newly-created UART thread can't run before uart_thread_ctx is assigned.
 */
        const tprio_t saved_prio = chThdSetPriority(APM_UART_UNBUFFERED_PRIORITY + 1);
        uart_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(HAL_UART_STACK_SIZE),
                                              uart_thread_name,
                                              unbuffered_writes ? APM_UART_UNBUFFERED_PRIORITY : APM_UART_PRIORITY,
                                              uart_thread_trampoline,
                                              this);
        chThdSetPriority(saved_prio);
        if (uart_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create UART TX thread");
        }
    }
}

void UARTDriver::uart_thread_trampoline(void* p)
{
    UARTDriver* uart = static_cast<UARTDriver*>(p);
    uart->uart_thread();
}

#ifndef HAL_STDOUT_SERIAL
/*
  hook to allow printf() to work on hal.console when we don't have a
  dedicated debug console
 */
static int hal_console_vprintf(const char *fmt, va_list arg)
{
    hal.console->vprintf(fmt, arg);
    return 1; // wrong length, but doesn't matter for what this is used for
}
#endif

void UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (b == 0 && txS == 0 && rxS == 0 && _tx_initialised && _rx_initialised) {
        // just changing port owner
        _uart_owner_thd = chThdGetSelfX();
        return;
    }

    thread_rx_init();

    if (sdef.serial == nullptr) {
        return;
    }

#if HAL_USE_SERIAL_USB
    if (sdef.is_usb) {
        _usb_tx_attempts = 0;
        _usb_tx_bytes_requested = 0;
        _usb_tx_bytes_accepted = 0;
        _usb_tx_zero_returns = 0;
        _usb_tx_poll_calls = 0;
        _usb_tx_poll_success = 0;
        _usb_tx_queue_full_events = 0;
        _usb_tx_backlog_drops = 0;
        _usb_tx_timer_ticks = 0;
        _usb_tx_timer_ticks_with_pending = 0;
    }
#endif

    uint16_t min_tx_buffer = HAL_UART_MIN_TX_SIZE;
    uint16_t min_rx_buffer = HAL_UART_MIN_RX_SIZE;

    /*
      increase min RX size to ensure we can receive a fully utilised
      UART if we are running our receive loop at 40Hz. This means 25ms
      of data. Assumes 10 bits per byte, which is normal for most
      protocols
     */
    bool rx_size_by_baudrate = true;
#if HAL_WITH_IO_MCU
    if (this == &uart_io) {
        // iomcu doesn't need extra space, just speed
        rx_size_by_baudrate = false;
        min_tx_buffer = 0;
        min_rx_buffer = 0;
    }
#endif
    if (rx_size_by_baudrate) {
        min_rx_buffer = MAX(min_rx_buffer, b/(40*10));
    }

    if (sdef.is_usb) {
        // give more buffer space for log download on USB
        min_tx_buffer *= 2;
    }

#if HAL_MEM_CLASS >= HAL_MEM_CLASS_500
    // on boards with plenty of memory we can use larger buffers
    min_tx_buffer *= 2;
    min_rx_buffer *= 2;
#endif

    // on PX4 we have enough memory to have a larger transmit and
    // receive buffer for all ports. This means we don't get delays
    // while waiting to write GPS config packets
    if (txS < min_tx_buffer) {
        txS = min_tx_buffer;
    }
    if (rxS < min_rx_buffer) {
        rxS = min_rx_buffer;
    }

    /*
      allocate the read buffer
      we allocate buffers before we successfully open the device as we
      want to allocate in the early stages of boot, and cause minimum
      thrashing of the heap once we are up. The ttyACM0 driver may not
      connect for some time after boot
     */
    WITH_SEMAPHORE(rx_sem);
    if (rxS != _readbuf.get_size()) {
        _rx_initialised = false;
        _readbuf.set_size_best(rxS);
    }
    _rts_threshold = _readbuf.get_size() / 16U;

    bool clear_buffers = false;
    if (b != 0) {
        // clear buffers on baudrate change, but not on the console (which is usually USB)
        if (_baudrate != b && hal.console != this) {
            clear_buffers = true;
        }
        _baudrate = b;
    }

    if (clear_buffers) {
        _rx_stats_dropped_bytes += _readbuf.available();
        _readbuf.clear();
    }

#ifndef HAL_UART_NODMA
    if (!half_duplex && !(_last_options & OPTION_NODMA_RX)) {
        if (rx_bounce_buf[0] == nullptr && sdef.dma_rx) {
            rx_bounce_buf[0] = (uint8_t *)hal.util->malloc_type(RX_BOUNCE_BUFSIZE, AP_HAL::Util::MEM_DMA_SAFE);
        }
        if (rx_bounce_buf[1] == nullptr && sdef.dma_rx) {
            rx_bounce_buf[1] = (uint8_t *)hal.util->malloc_type(RX_BOUNCE_BUFSIZE, AP_HAL::Util::MEM_DMA_SAFE);
        }
    }
    if (tx_bounce_buf == nullptr && sdef.dma_tx && !(_last_options & OPTION_NODMA_TX)) {
        tx_bounce_buf = (uint8_t *)hal.util->malloc_type(TX_BOUNCE_BUFSIZE, AP_HAL::Util::MEM_DMA_SAFE);
    }
    if (half_duplex) {
        rx_dma_enabled = tx_dma_enabled = false;
    } else {
        rx_dma_enabled = rx_bounce_buf[0] != nullptr && rx_bounce_buf[1] != nullptr;
        tx_dma_enabled = tx_bounce_buf != nullptr;
    }
    if (_last_options & OPTION_NODMA_RX) {
        rx_dma_enabled = false;
    }
    if (_last_options & OPTION_NODMA_TX) {
        tx_dma_enabled = false;
    }
    if (contention_counter > 1000 && _baudrate <= CONTENTION_BAUD_THRESHOLD) {
        // we've previously disabled TX DMA due to contention, don't
        // re-enable on a new begin() unless high baudrate
        tx_dma_enabled = false;
    }
    if (_baudrate <= 115200 && sdef.dma_tx && Shared_DMA::is_shared(sdef.dma_tx_stream_id)) {
        // avoid DMA on shared low-baudrate links
        tx_dma_enabled = false;
    }
#endif

#if defined(USART_CR1_FIFOEN)
    // enable the UART FIFO on G4 and H7. This allows for much higher baudrates
    // without data loss when not using DMA
    if (_last_options & OPTION_NOFIFO) {
        _cr1_options &= ~USART_CR1_FIFOEN;
    } else {
        _cr1_options |= USART_CR1_FIFOEN;
    }
#endif

    /*
      allocate the write buffer
     */
    WITH_SEMAPHORE(tx_sem);
    if (txS != _writebuf.get_size()) {
        _tx_initialised = false;
        _writebuf.set_size_best(txS);
    }

    if (clear_buffers) {
        _writebuf.clear();
    }

    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        /*
         * Initializes a serial-over-USB CDC driver.
         */
        if (!_device_initialised) {
            if ((SerialUSBDriver*)sdef.serial == &SDU1
#if HAL_HAVE_DUAL_USB_CDC
                || (SerialUSBDriver*)sdef.serial == &SDU2
#endif
            ) {
                usb_initialise();
            }
            _device_initialised = true;
        }
        // USB has flow control
        _flow_control = FLOW_CONTROL_ENABLE;
#endif
    } else {
#if HAL_USE_SERIAL == TRUE
        if (_baudrate != 0) {
#ifndef HAL_UART_NODMA
            bool was_initialised = _device_initialised;
            // setup Rx DMA
            if (!_device_initialised) {
                if (rx_dma_enabled) {
                    osalDbgAssert(rxdma == nullptr, "double DMA allocation");
                    chSysLock();
                    rxdma = dmaStreamAllocI(sdef.dma_rx_stream_id,
                                            12,  //IRQ Priority
                                            (stm32_dmaisr_t)rxbuff_full_irq,
                                            (void *)this);
                    osalDbgAssert(rxdma, "stream alloc failed");
                    chSysUnlock();
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
                    dmaStreamSetPeripheral(rxdma, &((SerialDriver*)sdef.serial)->usart->RDR);
#elif defined(STM32F1) || defined(STM32F4)
                    dmaStreamSetPeripheral(rxdma, &((SerialDriver*)sdef.serial)->usart->DR);
#else
                    #warning "DMA RX peripheral address not defined for this STM32 variant; add a dmaStreamSetPeripheral call to match the USART data register name"
#endif // STM32F7
#if STM32_DMA_SUPPORTS_DMAMUX
                    dmaSetRequestSource(rxdma, sdef.dma_rx_channel_id);
#endif
                }
                _device_initialised = true;
            }
            if (tx_dma_enabled && dma_handle == nullptr) {
                // we only allow for sharing of the TX DMA channel, not the RX
                // DMA channel, as the RX side is active all the time, so
                // cannot be shared
                dma_handle = NEW_NOTHROW Shared_DMA(sdef.dma_tx_stream_id,
                                            SHARED_DMA_NONE,
                                            FUNCTOR_BIND_MEMBER(&UARTDriver::dma_tx_allocate, void, Shared_DMA *),
                                            FUNCTOR_BIND_MEMBER(&UARTDriver::dma_tx_deallocate, void, Shared_DMA *));
                if (dma_handle == nullptr) {
                    tx_dma_enabled = false;
                }
            }
#endif // HAL_UART_NODMA
            sercfg.speed = _baudrate;

            // start with options from set_options()
            sercfg.cr1 = _cr1_options;
            sercfg.cr2 = _cr2_options;
            sercfg.cr3 = _cr3_options;

#if defined(STM32H7)
            /*
              H7 defaults to 16x oversampling. To get the highest
              possible baudrates we need to drop back to 8x
              oversampling. The H7 UART clock is 100MHz. This allows
              for up to 12.5MBps on H7 UARTs
             */
            if (_baudrate > 100000000UL / 16U) {
                sercfg.cr1 |= USART_CR1_OVER8;
            }
#endif

#ifndef HAL_UART_NODMA
            if (rx_dma_enabled) {
                sercfg.cr1 |= USART_CR1_IDLEIE;
                sercfg.cr3 |= USART_CR3_DMAR;
            }
            if (tx_dma_enabled) {
                sercfg.cr3 |= USART_CR3_DMAT;
            }
            // sercfg.irq_cb = rx_irq_cb; // custom ArduPilot ChibiOS extension, removed from submodule
#if HAL_HAVE_LOW_NOISE_UART
            if (sdef.low_noise_line) {
                // we can mark UART to sample on one bit instead of default 3 bits
                // this allows us to be slightly less sensitive to clock differences
                sercfg.cr3 |= USART_CR3_ONEBIT;
            }
#endif
#endif // HAL_UART_NODMA
            if (!(sercfg.cr2 & USART_CR2_STOP2_BITS)) {
                sercfg.cr2 |= USART_CR2_STOP1_BITS;
            }
            // sercfg.ctx = (void*)this; // custom ArduPilot ChibiOS extension, removed from submodule

            sdStart((SerialDriver*)sdef.serial, &sercfg);

#ifndef HAL_UART_NODMA
            if (rx_dma_enabled) {
                //Configure serial driver to skip handling RX packets
                //because we will handle them via DMA
                ((SerialDriver*)sdef.serial)->usart->CR1 &= ~USART_CR1_RXNEIE;
                // Start DMA
                if (!was_initialised) {
                    dmaStreamDisable(rxdma);
                    dma_rx_enable();
                }
            }
#endif // HAL_UART_NODMA
        }
#elif HAL_USE_SIO == TRUE
        // RP2350 SIO UART path
        if (_baudrate != 0) {
#ifndef HAL_UART_NODMA
            bool was_initialised = _device_initialised;
            // setup Rx DMA for RP2350
            if (!_device_initialised) {
                if (rx_dma_enabled) {
                    osalDbgAssert(rxdma == nullptr, "double DMA allocation");
                    chSysLock();
                    rxdma = dmaChannelAllocI(sdef.dma_rx_stream_id,
                                            3,   // IRQ priority
                                            (rp_dmaisr_t)rxbuff_full_irq,
                                            (void *)this);
                    osalDbgAssert(rxdma, "DMA channel alloc failed");
                    // Set source to UART RX data register (fixed/not-incremented)
                    dmaChannelSetSourceX(rxdma,
                        (uint32_t)&((SIODriver*)sdef.serial)->uart->UARTDR);
                    chSysUnlock();
                }
                _device_initialised = true;
            }
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

            // Build RP2350 SIO config with parity and stop bits applied.
            // parity: 0=none, 1=odd, 2=even (base class). _cr2_options holds stop bits count.
            uint32_t lcr_h = UART_UARTLCR_H_WLEN_8BITS | UART_UARTLCR_H_FEN;
            if (parity == 2U) {
                lcr_h |= UART_UARTLCR_H_PEN | UART_UARTLCR_H_EPS;  // even parity
            } else if (parity == 1U) {
                lcr_h |= UART_UARTLCR_H_PEN;  // odd parity
            }
            if (_cr2_options == 2U) {
                lcr_h |= UART_UARTLCR_H_STP2;  // 2 stop bits
            }
            SIOConfig siocfg = {
                .baud      = _baudrate,
                .UARTLCR_H = lcr_h,
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
 * RP2350: Switch UART TX and RX GPIO pins to FUNCSEL=2 (UART alternate function) so the RP2350 IO mux routes those GPIOs to the UART peripheral.
 * After chip reset the IO_BANK0 GPIO_CTRL register has FUNCSEL=0x1F (NULL/tristate), and nothing else in the startup path (pal_lld_init, sio_lld_start) sets FUNCSEL for UART pins.
 * Without this the UART peripheral output never reaches the pad and received data is not forwarded to the UART, silently breaking all UART comms.
 * Applying PUE before sioStart()/nvicEnableVector() ensures the RX line idles HIGH so the UART peripheral doesn't see a permanent BREAK condition during enumeration (which would fire IRQ33 continuously and starve the USB IRQ14 handler).
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
            sioStart((SIODriver*)sdef.serial, &siocfg);

// Populate arts_line/acts_line from the config so the set_flow_control() call below can route CTS/RTS correctly.
// The SIO set_options() path doesn't set these, so we do it here.
            arts_line = (ioline_t)sdef.rts_line;
            acts_line = (ioline_t)sdef.cts_line;

#ifndef HAL_UART_NODMA
            if (rx_dma_enabled && !was_initialised) {
                dmaChannelDisableX(rxdma);
                dma_rx_enable();
            }
#endif // HAL_UART_NODMA
        }
#endif // HAL_USE_SERIAL / HAL_USE_SIO
    }

// Only mark active when the underlying device can actually transfer.
// For non-USB ports with baud=0 we keep RX/TX inactive so the UART worker threads don't touch an unstarted SIO/Serial backend.
    const bool port_io_enabled = sdef.is_usb || (_baudrate != 0);
    _tx_initialised = (_writebuf.get_size() > 0) && port_io_enabled;
    _rx_initialised = (_readbuf.get_size() > 0) && port_io_enabled;

    _uart_owner_thd = chThdGetSelfX();
    // initialize TX thread only when safe on RP2350, else immediately.
#if defined(RP2350)
    if ((_tx_initialised || _rx_initialised) &&
        (sdef.is_usb || hal.scheduler->is_system_initialized())) {
        thread_init();
    }
#else
    if (_tx_initialised || _rx_initialised) {
        thread_init();
    }
#endif

    // setup flow control
    set_flow_control(_flow_control);

    if (serial_num == 0 && _tx_initialised) {
#ifndef HAL_STDOUT_SERIAL
        // setup hal.console to take printf() output
        vprintf_console_hook = hal_console_vprintf;
#endif
    }

#if HAL_UART_STATS_ENABLED && CH_CFG_USE_EVENTS == TRUE && defined(HAL_USE_SERIAL) && (HAL_USE_SERIAL == TRUE)
    if (!err_listener_initialised) {
        chEvtRegisterMaskWithFlags(chnGetEventSource((SerialDriver*)sdef.serial),
                                &err_listener,
                                EVT_ERROR,
                                SD_FRAMING_ERROR | SD_OVERRUN_ERROR | SD_NOISE_ERROR);
        err_listener_initialised = true;
    }
#endif
}

#ifndef HAL_UART_NODMA
void UARTDriver::dma_tx_allocate(Shared_DMA *ctx)
{
#if HAL_USE_SERIAL == TRUE
    if (txdma != nullptr) {
        return;
    }
    chSysLock();
    txdma = dmaStreamAllocI(sdef.dma_tx_stream_id,
                            12,  //IRQ Priority
                            (stm32_dmaisr_t)tx_complete,
                            (void *)this);
    osalDbgAssert(txdma, "stream alloc failed");
    chSysUnlock();
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
    dmaStreamSetPeripheral(txdma, &((SerialDriver*)sdef.serial)->usart->TDR);
#elif defined(STM32F1) || defined(STM32F4)
    dmaStreamSetPeripheral(txdma, &((SerialDriver*)sdef.serial)->usart->DR);
#else
    #warning "DMA TX peripheral address not defined for this STM32 variant; add a dmaStreamSetPeripheral call to match the USART data register name"
#endif // STM32F7
#if STM32_DMA_SUPPORTS_DMAMUX
    dmaSetRequestSource(txdma, sdef.dma_tx_channel_id);
#endif
#elif HAL_USE_SIO == TRUE
    // RP2350 TX DMA allocation
    if (txdma != nullptr) {
        return;
    }
    chSysLock();
    txdma = dmaChannelAllocI(sdef.dma_tx_stream_id,
                             3,  // IRQ priority
                             (rp_dmaisr_t)tx_complete,
                             (void *)this);
    osalDbgAssert(txdma, "DMA channel alloc failed");
    // Set destination to UART TX data register (fixed/not-incremented)
    dmaChannelSetDestinationX(txdma,
        (uint32_t)&((SIODriver*)sdef.serial)->uart->UARTDR);
    chSysUnlock();
#endif // HAL_USE_SERIAL / HAL_USE_SIO
}

#ifndef HAL_UART_NODMA
void UARTDriver::dma_rx_enable(void)
{
    rx_bounce_idx ^= 1;
#if defined(RP2350)
    // RP2350: configure RX DMA channel (P2M: UART DR -> bounce buffer)
    // Source (READ_ADDR) was set once at alloc time; re-set each start
    dmaChannelSetSourceX(rxdma,
        (uint32_t)&((SIODriver*)sdef.serial)->uart->UARTDR);
    dmaChannelSetDestinationX(rxdma, (uint32_t)rx_bounce_buf[rx_bounce_idx]);
    dmaChannelSetCounterX(rxdma, RX_BOUNCE_BUFSIZE);
    // TREQ = sdef.dma_rx_channel_id (precomputed TREQ_SEL bits)
    // INCR_WRITE = increment WRITE_ADDR (memory side), DATA_SIZE = BYTE
    uint32_t ctrl = sdef.dma_rx_channel_id |
                    DMA_CTRL_TRIG_DATA_SIZE_BYTE |
                    DMA_CTRL_TRIG_INCR_WRITE;
    dmaChannelEnableInterruptX(rxdma);
    dmaChannelSetModeX(rxdma, ctrl);
    dmaChannelEnableX(rxdma);
#else
    uint32_t dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE;
    dmamode |= STM32_DMA_CR_CHSEL(sdef.dma_rx_channel_id);
    dmamode |= STM32_DMA_CR_PL(0);
#if defined(STM32H7)
    dmamode |= DMA_SxCR_TRBUFF;   // TRBUFF See 2.3.1 in the H743 errata
#endif
    stm32_cacheBufferInvalidate(rx_bounce_buf[rx_bounce_idx], RX_BOUNCE_BUFSIZE);
    dmaStreamSetMemory0(rxdma, rx_bounce_buf[rx_bounce_idx]);
    dmaStreamSetTransactionSize(rxdma, RX_BOUNCE_BUFSIZE);
    dmaStreamSetMode(rxdma, dmamode | STM32_DMA_CR_DIR_P2M |
                     STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
    dmaStreamEnable(rxdma);
#endif // RP2350
}
#endif

void UARTDriver::dma_tx_deallocate(Shared_DMA *ctx)
{
    chSysLock();
#if defined(RP2350)
    dmaChannelFreeI(txdma);
#else
    dmaStreamFreeI(txdma);
#endif
    txdma = nullptr;
    chSysUnlock();
}

#ifndef HAL_UART_NODMA
void UARTDriver::rx_irq_cb(void* self)
{
#if HAL_USE_SERIAL == TRUE
    UARTDriver* uart_drv = (UARTDriver*)self;
    if (!uart_drv->rx_dma_enabled) {
        return;
    }
#if defined(STM32F7) || defined(STM32H7)
    //disable dma, triggering DMA transfer complete interrupt
    uart_drv->rxdma->stream->CR &= ~STM32_DMA_CR_EN;
#elif defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
    //disable dma, triggering DMA transfer complete interrupt
    dmaStreamDisable(uart_drv->rxdma);
    uart_drv->rxdma->channel->CCR &= ~STM32_DMA_CR_EN;
#else
    volatile uint16_t sr = ((SerialDriver*)(uart_drv->sdef.serial))->usart->SR;
    if(sr & USART_SR_IDLE) {
        volatile uint16_t dr = ((SerialDriver*)(uart_drv->sdef.serial))->usart->DR;
        (void)dr;
        //disable dma, triggering DMA transfer complete interrupt
        uart_drv->rxdma->stream->CR &= ~STM32_DMA_CR_EN;
    }
#endif // STM32F7
#endif // HAL_USE_SERIAL
}
#endif

/*
  handle a RX DMA full interrupt
 */
__RAMFUNC__ void UARTDriver::rxbuff_full_irq(void* self, uint32_t flags)
{
#if HAL_USE_SERIAL == TRUE
    UARTDriver* uart_drv = (UARTDriver*)self;
    if (!uart_drv->rx_dma_enabled) {
        return;
    }
    uint16_t len = RX_BOUNCE_BUFSIZE - dmaStreamGetTransactionSize(uart_drv->rxdma);
    const uint8_t bounce_idx = uart_drv->rx_bounce_idx;

    // restart the DMA transfers immediately. This switches to the
    // other bounce buffer. We restart the DMA before we copy the data
    // out to minimise the time with DMA disabled, which allows us to
    // handle much higher receiver baudrates
    dmaStreamDisable(uart_drv->rxdma);
    uart_drv->dma_rx_enable();
    
    if (len > 0) {
        /*
          we have data to copy out
         */
        const uint32_t written = uart_drv->_readbuf.write(uart_drv->rx_bounce_buf[bounce_idx], len);
        uart_drv->_rx_stats_bytes += len;
        uart_drv->_rx_stats_dropped_bytes += len - written;
        uart_drv->receive_timestamp_update();
    }

    if (uart_drv->_wait.thread_ctx && uart_drv->_readbuf.available() >= uart_drv->_wait.n) {
        chSysLockFromISR();
        chEvtSignalI(uart_drv->_wait.thread_ctx, EVT_DATA);
        chSysUnlockFromISR();
    }
    if (uart_drv->_rts_is_active) {
        uart_drv->update_rts_line();
    }
#elif HAL_USE_SIO == TRUE
    UARTDriver* uart_drv = (UARTDriver*)self;
    if (!uart_drv->rx_dma_enabled) {
        return;
    }
    // On RP2350, DMA IRQ fires when the full bounce buffer is received (TRANS_COUNT hits 0)
    const uint16_t len = RX_BOUNCE_BUFSIZE;
    const uint8_t bounce_idx = uart_drv->rx_bounce_idx;

    // Restart DMA immediately to minimise dead time, switching to the other bounce buffer
    dmaChannelDisableX(uart_drv->rxdma);
    uart_drv->dma_rx_enable();

    if (len > 0) {
        const uint32_t written = uart_drv->_readbuf.write(uart_drv->rx_bounce_buf[bounce_idx], len);
        uart_drv->_rx_stats_bytes += len;
        uart_drv->_rx_stats_dropped_bytes += len - written;
        uart_drv->receive_timestamp_update();
    }
    if (uart_drv->_wait.thread_ctx && uart_drv->_readbuf.available() >= uart_drv->_wait.n) {
        chSysLockFromISR();
        chEvtSignalI(uart_drv->_wait.thread_ctx, EVT_DATA);
        chSysUnlockFromISR();
    }
    if (uart_drv->_rts_is_active) {
        uart_drv->update_rts_line();
    }
#endif // HAL_USE_SERIAL
}
#endif // HAL_UART_NODMA

void UARTDriver::_end()
{
    WITH_SEMAPHORE(rx_sem);
    WITH_SEMAPHORE(tx_sem);
    _rx_initialised = false;
    _tx_initialised = false;

    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL

        sduStop((SerialUSBDriver*)sdef.serial);
#endif
    } else {
#if HAL_USE_SERIAL == TRUE
        sdStop((SerialDriver*)sdef.serial);
#elif HAL_USE_SIO == TRUE
        sioStop((SIODriver*)sdef.serial);
#endif
    }

    _readbuf.set_size(0);
    _writebuf.set_size(0);
}

#ifdef HAVE_USB_SERIAL
/*
 * Polling TX drain for USB CDC.
 * bypasses obnotify (full-buffer only) and SOF interrupt flush entirely.
 * Worst case spins for ~80 × 125 µs = 10 ms, which covers many 64-byte packets at USB FS 12 Mbit/s.
 */
static bool usb_tx_poll_drain(SerialUSBDriver *sdu)
{
    bool started_transfer = false;

    for (uint8_t i = 0; i < 80; i++) {
        osalSysLock();
        if ((usbGetDriverStateI(sdu->config->usbp) != USB_ACTIVE) ||
            (sdu->state != SDU_READY)) {
            osalSysUnlock();
            return started_transfer;
        }
        if (!usbGetTransmitStatusI(sdu->config->usbp, sdu->config->bulk_in)) {
            /* TX idle — grab next full buffer or force-flush a partial one */
            size_t n;
            uint8_t *buf = obqGetFullBufferI(&sdu->obqueue, &n);
            if (buf == nullptr) {
                if (obqTryFlushI(&sdu->obqueue)) {
                    buf = obqGetFullBufferI(&sdu->obqueue, &n);
                }
            }
            if (buf != nullptr) {
                usbStartTransmitI(sdu->config->usbp, sdu->config->bulk_in, buf, n);
                started_transfer = true;
                osalSysUnlock();
                chThdSleepMicroseconds(125); /* yield — let TX-complete ISR run */
                continue;
            }
            osalSysUnlock();
            return started_transfer; /* queue empty, all data sent */
        }
        osalSysUnlock();
        chThdSleepMicroseconds(125); /* TX in progress — yield and retry */
    }

    return started_transfer;
}

static uint16_t usb_try_write_direct(SerialUSBDriver *sdu, const uint8_t *data, uint16_t len)
{
    int ret = chnWriteTimeout(sdu, data, len, TIME_IMMEDIATE);
    if (ret > 0) {
        (void)usb_tx_poll_drain(sdu);
        return ret;
    }

    return 0;
}

static void usb_try_read_direct(SerialUSBDriver *sdu, ByteBuffer &readbuf, uint32_t &rx_stats_bytes)
{
    ByteBuffer::IoVec vec[2];
    const auto n_vec = readbuf.reserve(vec, readbuf.space());
    for (int i = 0; i < n_vec; i++) {
        const int ret = chnReadTimeout(sdu, vec[i].data, vec[i].len, TIME_IMMEDIATE);
        if (ret <= 0) {
            break;
        }
        readbuf.commit((unsigned)ret);
        rx_stats_bytes += ret;
        if ((unsigned)ret < vec[i].len) {
            break;
        }
    }
}
#endif // HAVE_USB_SERIAL

void UARTDriver::_flush()
{
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        /* Poll-drain bypasses both obnotify (full-buffer only) and SOF flush. */
#if HAL_USE_SERIAL_USB
    _usb_tx_poll_calls++;
    if (usb_tx_poll_drain((SerialUSBDriver*)sdef.serial)) {
        _usb_tx_poll_success++;
    }
#else
    usb_tx_poll_drain((SerialUSBDriver*)sdef.serial);
#endif
#endif
    } else {
#if defined(RP2350)
        if (uart_thread_ctx == nullptr && hal.scheduler->is_system_initialized()) {
            thread_init();
        }
#endif
        if (uart_thread_ctx != nullptr) {
            chEvtSignal(uart_thread_ctx, EVT_TRANSMIT_DATA_READY);
        }
    }
}

bool UARTDriver::is_initialized()
{
    return _tx_initialised && _rx_initialised;
}

bool UARTDriver::tx_pending() { return _writebuf.available() > 0; }


/*
    get the requested usb baudrate - 0 = none
*/
uint32_t UARTDriver::get_usb_baud() const
{
#if HAL_USE_SERIAL_USB
    if (sdef.is_usb) {
        return ::get_usb_baud(sdef.endpoint_id);
    }
#endif
    return 0;
}

/*
    get the requested usb parity.  Valid if get_usb_baud() returned non-zero.
*/
uint8_t UARTDriver::get_usb_parity() const
{
#if HAL_USE_SERIAL_USB
    if (sdef.is_usb) {
        return ::get_usb_parity(sdef.endpoint_id);
    }
#endif
    return 0;
}

#ifdef HAVE_USB_SERIAL
bool UARTDriver::is_usb_active() const
{
    if (!sdef.is_usb) {
        return false;
    }
    SerialUSBDriver *sdu = (SerialUSBDriver*)sdef.serial;
    return (sdu->state == SDU_READY) && (usbGetDriverStateI(sdu->config->usbp) == USB_ACTIVE);
}

bool UARTDriver::is_usb_host_open() const
{
#if HAL_USE_SERIAL_USB
    return is_usb_active() && ::usb_cdc_host_open(sdef.endpoint_id);
#else
    return false;
#endif
}

void UARTDriver::drop_unopened_usb_tx_backlog()
{
#if HAL_USE_SERIAL_USB
    // Preserve early boot output until USB is active; dropping pre-enumeration
    // backlog causes one-shot setup printf() output to disappear on late attach.
    if (!sdef.is_usb || !is_usb_active() || is_usb_host_open()) {
        return;
    }

    // If the host has not opened the CDC port then stale queued output has no value.
    // Drop it so queue-full remains a short transient instead of a persistent wedge.
    _writebuf.clear();
    _usb_tx_backlog_drops++;

    auto *sdu = (SerialUSBDriver *)sdef.serial;
    chSysLock();
    obqResetI(&sdu->obqueue);
    if (sdu->config != nullptr && sdu->config->usbp != nullptr && sdu->config->bulk_in > 0) {
        sdu->config->usbp->transmitting &= ~(1U << (sdu->config->bulk_in - 1U));
    }
    chSysUnlock();
#endif
}
#endif

uint32_t UARTDriver::_available()
{
    if (!_rx_initialised || _uart_owner_thd != chThdGetSelfX()) {
        return 0;
    }
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL

        if (((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE) {
            return 0;
        }

        WITH_SEMAPHORE(rx_sem);
        usb_try_read_direct((SerialUSBDriver*)sdef.serial, _readbuf, _rx_stats_bytes);
#endif
    }
    return _readbuf.available();
}

uint32_t UARTDriver::txspace()
{
    if (!_tx_initialised) {
        return 0;
    }
    return _writebuf.space();
}

bool UARTDriver::_discard_input()
{
    if (_uart_owner_thd != chThdGetSelfX()){
        return false;
    }
    if (!_rx_initialised) {
        return false;
    }

    _rx_stats_dropped_bytes += _readbuf.available();
    _readbuf.clear();

    if (!_rts_is_active) {
        update_rts_line();
    }

    return true;
}

ssize_t UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (_uart_owner_thd != chThdGetSelfX()){
        return -1;
    }
    if (!_rx_initialised) {
        return -1;
    }

    if (sdef.is_usb && _readbuf.available() == 0) {
#ifdef HAVE_USB_SERIAL
        WITH_SEMAPHORE(rx_sem);
        usb_try_read_direct((SerialUSBDriver*)sdef.serial, _readbuf, _rx_stats_bytes);
#endif
    }

    const uint32_t ret = _readbuf.read(buffer, count);
    if (ret == 0) {
        return 0;
    }

    if (!_rts_is_active) {
        update_rts_line();
    }

    return ret;
}

/* write a block of bytes to the port */
size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_tx_initialised) {
		return 0;
	}

#if defined(RP2350)
    if (uart_thread_ctx == nullptr && (sdef.is_usb || hal.scheduler->is_system_initialized())) {
        thread_init();
    }
#endif

    WITH_SEMAPHORE(_write_mutex);

    if (sdef.is_usb && is_usb_active() && !is_usb_host_open() && _writebuf.available() > 0) {
        drop_unopened_usb_tx_backlog();
    }

    if (sdef.is_usb && is_usb_host_open() && _usb_tx_attempts == 0 && _writebuf.available() > 0) {
        // If the host has just opened after a long disconnected period then any residual
        // buffered bytes are stale. Drop them so new traffic can be transmitted immediately.
        _writebuf.clear();
    }

    if (sdef.is_usb && is_usb_active() && size > _writebuf.space()) {
        drop_unopened_usb_tx_backlog();
    }

    size_t direct_written = 0;
    if (sdef.is_usb && is_usb_host_open() && size > 0) {
        auto *sdu = (SerialUSBDriver *)sdef.serial;
#if HAL_USE_SERIAL_USB
        _usb_tx_attempts++;
        _usb_tx_bytes_requested += size;
#endif
        direct_written = usb_try_write_direct(sdu, buffer, size);
#if HAL_USE_SERIAL_USB
        if (direct_written == 0) {
            _usb_tx_zero_returns++;
        } else {
            _usb_tx_bytes_accepted += direct_written;
            _usb_tx_poll_calls++;
            _usb_tx_poll_success++;
        }
#endif
        if (direct_written > 0) {
            _last_write_completed_us = AP_HAL::micros();
            _total_written += direct_written;
            _tx_stats_bytes += direct_written;
            if (direct_written == size) {
                return direct_written;
            }
            buffer += direct_written;
            size -= direct_written;
        }
    }

    size_t ret = _writebuf.write(buffer, size);
    ret += direct_written;
    if (unbuffered_writes && uart_thread_ctx != nullptr) {
        chEvtSignal(uart_thread_ctx, EVT_TRANSMIT_DATA_READY);
    }
    return ret;
}

/*
  wait for data to arrive, or a timeout. Return true if data has
  arrived, false on timeout
 */
bool UARTDriver::wait_timeout(uint16_t n, uint32_t timeout_ms)
{
    uint32_t t0 = AP_HAL::millis();
    while (available() < n) {
        chEvtGetAndClearEvents(EVT_DATA);
        _wait.n = n;
        _wait.thread_ctx = chThdGetSelfX();
        uint32_t now = AP_HAL::millis();
        if (now - t0 >= timeout_ms) {
            break;
        }
        chEvtWaitAnyTimeout(EVT_DATA, chTimeMS2I(timeout_ms - (now - t0)));
    }
    return available() >= n;
}

#ifndef HAL_UART_NODMA
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=128"
/*
  DMA transmit completion interrupt handler
 */
__RAMFUNC__ void UARTDriver::tx_complete(void* self, uint32_t flags)
{
    UARTDriver* uart_drv = (UARTDriver*)self;
    chSysLockFromISR();

#if defined(RP2350)
    dmaChannelDisableX(uart_drv->txdma);
#else
    // check nothing bad happened
    if ((flags & STM32_DMA_ISR_TEIF) != 0) {
        INTERNAL_ERROR(AP_InternalError::error_t::dma_fail);
    }

    dmaStreamDisable(uart_drv->txdma);
#endif

    uart_drv->_last_write_completed_us = AP_HAL::micros();

    chEvtSignalI(uart_drv->uart_thread_ctx, EVT_TRANSMIT_DMA_COMPLETE);
    chSysUnlockFromISR();
}

/*
  write out pending bytes with DMA
 */
void UARTDriver::write_pending_bytes_DMA(uint32_t n)
{
    // sanity check
    if (!dma_handle) {
        return;
    }

    while (n > 0) {
        if (flow_control_enabled(_flow_control) &&
            acts_line != 0 &&
            palReadLine(acts_line)) {
            // we are using hw flow control and the CTS line is high. We
            // will hold off trying to transmit until the CTS line goes
            // low to indicate the receiver has space. We do this before
            // we take the DMA lock to prevent a high CTS line holding a
            // DMA channel that may be needed by another device
            return;
        }

        uint16_t tx_len = 0;

        {
            WITH_SEMAPHORE(_write_mutex);
            // get some more to write
            tx_len = _writebuf.peekbytes(tx_bounce_buf, MIN(n, TX_BOUNCE_BUFSIZE));

            if (tx_len == 0) {
                return; // all done
            }
            // find out how much is still left to write while we still have the lock
            n = MIN(_writebuf.available(), n);
        }

        dma_handle->lock(); // we have our own thread so grab the lock

        chEvtGetAndClearEvents(EVT_TRANSMIT_DMA_COMPLETE);

        if (dma_handle->has_contention()) {
            // on boards with a hw fifo we can use a higher threshold for disabling DMA
            if (_baudrate <= CONTENTION_BAUD_THRESHOLD) {
                contention_counter += 3;
                if (contention_counter > 1000) {
                    // more than 25% of attempts to use this DMA
                    // channel are getting contention and we have a
                    // low baudrate. Switch off DMA for future
                    // transmits on this low baudrate UART
                    tx_dma_enabled = false;
                    dma_handle->unlock(false);
                    break;
                }
            }
            /*
            someone else is using this same DMA channel. To reduce
            latency we will drop the TX size with DMA on this UART to
            keep TX times below 250us. This can still suffer from long
            times due to CTS blockage
            */
            uint32_t max_tx_bytes = 1 + (_baudrate * 250UL / 1000000UL);
            if (tx_len > max_tx_bytes) {
                tx_len = max_tx_bytes;
            }
        } else if (contention_counter > 0) {
            contention_counter--;
        }

        chSysLock();
#if defined(RP2350)
        dmaChannelDisableX(txdma);
        dmaChannelSetSourceX(txdma, (uint32_t)tx_bounce_buf);
        dmaChannelSetCounterX(txdma, tx_len);
        // TREQ in sdef.dma_tx_channel_id, M2P so INCR_READ (memory source increments), DATA_SIZE = BYTE
        uint32_t dmamode = sdef.dma_tx_channel_id |
                           DMA_CTRL_TRIG_DATA_SIZE_BYTE |
                           DMA_CTRL_TRIG_INCR_READ;
        dmaChannelEnableInterruptX(txdma);
        dmaChannelSetModeX(txdma, dmamode);
        dmaChannelEnableX(txdma);
#else
        dmaStreamDisable(txdma);
        stm32_cacheBufferFlush(tx_bounce_buf, tx_len);
        dmaStreamSetMemory0(txdma, tx_bounce_buf);
        dmaStreamSetTransactionSize(txdma, tx_len);
        uint32_t dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE;
        dmamode |= STM32_DMA_CR_CHSEL(sdef.dma_tx_channel_id);
        dmamode |= STM32_DMA_CR_PL(0);
#if defined(STM32H7)
        dmamode |= DMA_SxCR_TRBUFF;   // TRBUFF See 2.3.1 in the H743 errata
#endif
        dmaStreamSetMode(txdma, dmamode | STM32_DMA_CR_DIR_M2P |
                        STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
        dmaStreamEnable(txdma);
#endif // RP2350
        uint32_t timeout_us = ((1000000UL * (tx_len+2) * 10) / _baudrate) + 500;
        // prevent very long timeouts at low baudrates which could cause another thread
        // using begin() to block
        timeout_us = MIN(timeout_us, 100000UL);
        chSysUnlock();
        // wait for the completion or timeout handlers to signal that we are done
        eventmask_t mask = chEvtWaitAnyTimeout(EVT_TRANSMIT_DMA_COMPLETE, chTimeUS2I(timeout_us));
        // handle a TX timeout. This can happen with using hardware flow
        // control if CTS pin blocks transmit or sometimes the DMA completion simply disappears
        if (mask == 0) {
            chSysLock();
            // check whether DMA completion happened in the intervening time
            // first disable the stream to prevent further interrupts
#if defined(RP2350)
            dmaChannelDisableX(txdma);
            const uint32_t tx_size = txdma->channel->TRANS_COUNT;

#else
            dmaStreamDisable(txdma);

            const uint32_t tx_size = dmaStreamGetTransactionSize(txdma);
#endif

            if (tx_size >= tx_len) {
                // we didn't write any of our bytes
                tx_len = 0;
            } else {
                // record how much was sent tx_size is how much was
                // not sent (could be 0)
                tx_len -= tx_size;
            }
            if (tx_len > 0) {
                _last_write_completed_us = AP_HAL::micros();
            }
            chEvtGetAndClearEventsI(EVT_TRANSMIT_DMA_COMPLETE);
            chSysUnlock();
        }
        // clean up pending locks
        dma_handle->unlock(mask & EVT_TRANSMIT_DMA_COMPLETE);

        if (tx_len) {
            WITH_SEMAPHORE(_write_mutex);
            // skip over amount actually written
            _writebuf.advance(tx_len);

            // update stats
            _total_written += tx_len;
            _tx_stats_bytes += tx_len;

            n -= tx_len;
        } else {
            // if we didn't manage to transmit any bytes then stop
            // processing so we can check flow control state in outer
            // loop
            break;
        }
    }
}
#pragma GCC diagnostic pop
#endif // HAL_UART_NODMA

/*
  write any pending bytes to the device, non-DMA method
 */
void UARTDriver::write_pending_bytes_NODMA(uint32_t n)
{
    WITH_SEMAPHORE(_write_mutex);

    ByteBuffer::IoVec vec[2];
    uint16_t nwritten = 0;

    if (half_duplex && n > 1) {
        half_duplex_setup_tx();
    }

    const auto n_vec = _writebuf.peekiovec(vec, n);
    for (int i = 0; i < n_vec; i++) {
        int ret = -1;
        if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
            if (is_usb_active() && !is_usb_host_open()) {
                drop_unopened_usb_tx_backlog();
                break;
            }
#endif
            ret = 0;
#ifdef HAVE_USB_SERIAL
#if HAL_USE_SERIAL_USB
            _usb_tx_attempts++;
            _usb_tx_bytes_requested += vec[i].len;
#endif
            ret = chnWriteTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);

#if HAL_USE_SERIAL_USB
            if (ret == 0) {
                _usb_tx_zero_returns++;
            }
#endif

            /* Immediately poll-drain: don't wait for obnotify or SOF ISR */
#if HAL_USE_SERIAL_USB
            _usb_tx_poll_calls++;
            if (usb_tx_poll_drain((SerialUSBDriver*)sdef.serial)) {
                _usb_tx_poll_success++;
            }
#else
            usb_tx_poll_drain((SerialUSBDriver*)sdef.serial);
#endif

// If immediate-write could not enqueue any bytes, yield once and retry.
// This keeps non-blocking semantics while giving TX completion a chance to release an obqueue buffer under heavy CDC backpressure.
            if (ret == 0 && is_usb_active()) {
#if HAL_USE_SERIAL_USB
                _usb_tx_queue_full_events++;
#endif
                drop_unopened_usb_tx_backlog();
                if (!is_usb_host_open()) {
                    ret = chnWriteTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#if HAL_USE_SERIAL_USB
                    if (ret == 0) {
                        _usb_tx_zero_returns++;
                    }
#endif
                    _usb_tx_poll_calls++;
                    if (usb_tx_poll_drain((SerialUSBDriver*)sdef.serial)) {
                        _usb_tx_poll_success++;
                    }
                }
            }

            if (ret == 0 && is_usb_active()) {
                chThdSleepMicroseconds(125);
                const int retry = chnWriteTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#if HAL_USE_SERIAL_USB
                if (retry == 0) {
                    _usb_tx_zero_returns++;
                }
#endif
                if (retry > 0) {
                    ret = retry;
                }
#if HAL_USE_SERIAL_USB
                _usb_tx_poll_calls++;
                if (usb_tx_poll_drain((SerialUSBDriver*)sdef.serial)) {
                    _usb_tx_poll_success++;
                }
#else
                usb_tx_poll_drain((SerialUSBDriver*)sdef.serial);
#endif
            }
#endif
        } else {
#if HAL_USE_SERIAL == TRUE
            ret = chnWriteTimeout((SerialDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#elif HAL_USE_SIO == TRUE
            ret = chnWriteTimeout((BaseChannel*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#endif
        }
        if (ret < 0) {
            break;
        }
        if (ret > 0) {
            _last_write_completed_us = AP_HAL::micros();
            nwritten += ret;
#if HAL_USE_SERIAL_USB
            if (sdef.is_usb) {
                _usb_tx_bytes_accepted += ret;
            }
#endif
        }
        _writebuf.advance(ret);

        /* We wrote less than we asked for, stop */
        if ((unsigned)ret != vec[i].len) {
            break;
        }
    }

    _total_written += nwritten;
    _tx_stats_bytes += nwritten;
}

/*
  write any pending bytes to the device
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=128"
void UARTDriver::write_pending_bytes(void)
{
    // write any pending bytes
    uint32_t n = _writebuf.available();
    if (n <= 0) {
        return;
    }

#ifndef HAL_UART_NODMA
    if (tx_dma_enabled) {
        write_pending_bytes_DMA(n);
    } else
#endif
    {
        write_pending_bytes_NODMA(n);
    }

    // handle AUTO flow control mode
    if (_flow_control == FLOW_CONTROL_AUTO) {
        if (_first_write_started_us == 0) {
            _first_write_started_us = AP_HAL::micros();
            _total_written = 0;
        }
#ifndef HAL_UART_NODMA
        if (tx_dma_enabled) {
            // when we are using DMA we have a reliable indication that a write
            // has completed from the DMA completion interrupt
            if (_last_write_completed_us != 0) {
                _flow_control = FLOW_CONTROL_ENABLE;
                return;
            }
        } else
#endif
        {
            // without DMA we need to look at the number of bytes written into the queue versus the
            // remaining queue space
#if defined(HAL_USE_SERIAL) && (HAL_USE_SERIAL == TRUE)
            uint32_t space = qSpaceI(&((SerialDriver*)sdef.serial)->oqueue);
            uint32_t used = SERIAL_BUFFERS_SIZE - space;

#if !defined(USART_CR1_FIFOEN)
            // threshold is 8 for the GCS_Common code to unstick SiK radios, which
            // sends 6 bytes with flow control disabled
            const uint8_t threshold = 8;
#else
            // account for TX FIFO buffer
            uint8_t threshold = 12;
            if (_last_options & OPTION_NOFIFO) {
                threshold = 8;
            }
#endif
            if (_total_written > used && _total_written - used > threshold) {
                _flow_control = FLOW_CONTROL_ENABLE;
                return;
            }
#endif // HAL_USE_SERIAL
        }
        if (AP_HAL::micros() - _first_write_started_us > 500*1000UL) {
            // it doesn't look like hw flow control is working
            DEV_PRINTF("disabling flow control on serial %u\n", sdef.get_index());
            set_flow_control(FLOW_CONTROL_DISABLE);
        }
    }
}
#pragma GCC diagnostic pop

/*
  setup for half duplex tramsmit. To cope with uarts that have level
  shifters and pullups we need to play a trick where we temporarily
  disable half-duplex while transmitting. That disables the receive
  part of the uart on the pin which allows the transmit side to
  correctly setup the idle voltage before the transmit starts.
 */
void UARTDriver::half_duplex_setup_tx(void)
{
    if (!hd_tx_active) {
        chEvtGetAndClearFlags(&hd_listener);
        // half-duplex transmission is done when both the output is empty and the transmission is ended
        // if we only wait for empty output the line can be setup for receive too soon losing data bits
        hd_tx_active = CHN_TRANSMISSION_END | CHN_OUTPUT_EMPTY;
#if defined(HAL_USE_SERIAL) && (HAL_USE_SERIAL == TRUE)
        SerialDriver *sd = (SerialDriver*)(sdef.serial);
        sdStop(sd);
        sercfg.cr3 &= ~USART_CR3_HDSEL;
        sdStart(sd, &sercfg);
#endif // HAL_USE_SERIAL
    }
}

/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void UARTDriver::_rx_timer_tick(void)
{
    if (!_rx_initialised || half_duplex) {
        return;
    }

    WITH_SEMAPHORE(rx_sem);

#if HAL_UART_STATS_ENABLED && CH_CFG_USE_EVENTS == TRUE && defined(HAL_USE_SERIAL) && (HAL_USE_SERIAL == TRUE)
    if (!sdef.is_usb) {
        const auto err_flags = chEvtGetAndClearFlags(&err_listener);
        // count the number of errors
        if (err_flags & SD_FRAMING_ERROR) {
            _rx_stats_framing_errors++;
        }
        if (err_flags & SD_OVERRUN_ERROR) {
            _rx_stats_overrun_errors++;
        }
        if (err_flags & SD_NOISE_ERROR) {
            _rx_stats_noise_errors++;
        }
    }
#endif

#ifndef HAL_UART_NODMA
    if (rx_dma_enabled && rxdma) {
        chSysLock();
#if defined(RP2350)
// RP2350 has no IDLE interrupt.
// flush partial receive buffers here (called at 1kHz).
        if (dmaChannelIsBusyX(rxdma)) {
            uint32_t remaining = rxdma->channel->TRANS_COUNT;
            uint8_t len = RX_BOUNCE_BUFSIZE - (uint8_t)remaining;
            if (len > 0) {
                dmaChannelAbortX(rxdma);
                dmaChannelGetAndClearInterrupts(rxdma);
                const uint32_t written = _readbuf.write(rx_bounce_buf[rx_bounce_idx], len);
                _rx_stats_bytes += len;
                _rx_stats_dropped_bytes += len - written;
                receive_timestamp_update();
                if (_rts_is_active) {
                    update_rts_line();
                }
                dma_rx_enable();
            }
        } else {
            // DMA completed but was not restarted (e.g. IRQ was missed) — restart now
            dmaChannelGetAndClearInterrupts(rxdma);
            dma_rx_enable();
        }
#else
        //Check if DMA is enabled
        //if not, it might be because the DMA interrupt was silenced
        //let's handle that here so that we can continue receiving
#if defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
        bool enabled = (rxdma->channel->CCR & STM32_DMA_CR_EN);
#else
        bool enabled = (rxdma->stream->CR & STM32_DMA_CR_EN);
#endif
        if (!enabled) {
            uint8_t len = RX_BOUNCE_BUFSIZE - dmaStreamGetTransactionSize(rxdma);
            if (len != 0) {
                const uint32_t written = _readbuf.write(rx_bounce_buf[rx_bounce_idx], len);
                _rx_stats_bytes += len;
                _rx_stats_dropped_bytes += len - written;

                receive_timestamp_update();
                if (_rts_is_active) {
                    update_rts_line();
                }
            }
            // DMA disabled by idle interrupt never got a chance to be handled
            // we will enable it here
            dmaStreamDisable(rxdma);
            dma_rx_enable();
        }
#endif // RP2350
        chSysUnlock();
    }
#endif

    // don't try IO on a disconnected USB port
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        if (((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE) {
            return;
        }
#endif
    }
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        ((GPIO *)hal.gpio)->set_usb_connected();
#endif
    }
#ifndef HAL_UART_NODMA
    if (!rx_dma_enabled)
#endif
    {
        read_bytes_NODMA();
    }
    if (_wait.thread_ctx && _readbuf.available() >= _wait.n) {
        chEvtSignal(_wait.thread_ctx, EVT_DATA);
    }
}

// regular serial read
void UARTDriver::read_bytes_NODMA()
{
    // try to fill the read buffer
    ByteBuffer::IoVec vec[2];

    const auto n_vec = _readbuf.reserve(vec, _readbuf.space());
    for (int i = 0; i < n_vec; i++) {
        int ret = 0;
        //Do a non-blocking read
        if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
            ret = chnReadTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#endif
        } else {
#if HAL_USE_SERIAL == TRUE
            ret = chnReadTimeout((SerialDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#elif HAL_USE_SIO == TRUE
            ret = chnReadTimeout((BaseChannel*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#endif
        }
        if (ret < 0) {
            break;
        }
#if CH_CFG_USE_EVENTS == TRUE && defined(HAL_USE_SERIAL) && (HAL_USE_SERIAL == TRUE)
        if (parity_enabled && ((chEvtGetAndClearFlags(&ev_listener) & SD_PARITY_ERROR))) {
            // discard bytes with parity error
            ret = -1;
        }
#endif
        if (!hd_tx_active) {
            _readbuf.commit((unsigned)ret);
            _rx_stats_bytes += ret;
            receive_timestamp_update();
        }

        /* stop reading as we read less than we asked for */
        if ((unsigned)ret < vec[i].len) {
            break;
        }
    }
}

/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void UARTDriver::_tx_timer_tick(void)
{
    if (!_tx_initialised) {
        return;
    }

#if HAL_USE_SERIAL_USB
    if (sdef.is_usb) {
        _usb_tx_timer_ticks++;
        if (_writebuf.available() > 0) {
            _usb_tx_timer_ticks_with_pending++;
        }
    }
#endif

    if (hd_tx_active) {
        WITH_SEMAPHORE(tx_sem);
        hd_tx_active &= ~chEvtGetAndClearFlags(&hd_listener);
        if (!hd_tx_active) {
            /*
              half-duplex transmit has finished. We now re-enable the
              HDSEL bit for receive
            */
#if defined(HAL_USE_SERIAL) && (HAL_USE_SERIAL == TRUE)
            SerialDriver *sd = (SerialDriver*)(sdef.serial);
            sdStop(sd);
            sercfg.cr3 |= USART_CR3_HDSEL;
            sdStart(sd, &sercfg);
#endif // HAL_USE_SERIAL
        }
    }

    // don't try IO on a disconnected USB port
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        if (((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE) {
            return;
        }
#endif
    }
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        ((GPIO *)hal.gpio)->set_usb_connected();
#endif
    }

    // half duplex we do reads in the write thread
    if (half_duplex) {
        WITH_SEMAPHORE(rx_sem);
        read_bytes_NODMA();
        if (_wait.thread_ctx && _readbuf.available() >= _wait.n) {
            chEvtSignal(_wait.thread_ctx, EVT_DATA);
        }
    }

    // now do the write
    WITH_SEMAPHORE(tx_sem);
    write_pending_bytes();
}

/*
  change flow control mode for port
 */
void UARTDriver::set_flow_control(enum flow_control flowcontrol)
{
    if (sdef.is_usb) {
        // no hw flow control available
        return;
    }
#if HAL_USE_SERIAL == TRUE
    SerialDriver *sd = (SerialDriver*)(sdef.serial);
    _flow_control = (arts_line == 0) ? FLOW_CONTROL_DISABLE : flowcontrol;
    if (!is_initialized()) {
        // not ready yet, we just set variable for when we call begin
        return;
    }
    switch (_flow_control) {

    case FLOW_CONTROL_DISABLE:
        // force RTS active when flow disabled
        if (arts_line != 0) {
            palSetLineMode(arts_line, 1);
            palClearLine(arts_line);
        }
        _rts_is_active = true;
        // disable hardware CTS support
        chSysLock();
        if ((sd->usart->CR3 & (USART_CR3_CTSE | USART_CR3_RTSE)) != 0) {
            sd->usart->CR1 &= ~USART_CR1_UE;
            sd->usart->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
            sd->usart->CR1 |= USART_CR1_UE;
        }
        chSysUnlock();
        break;

    case FLOW_CONTROL_AUTO:
        // reset flow control auto state machine
        _first_write_started_us = 0;
        _last_write_completed_us = 0;
        FALLTHROUGH;

    case FLOW_CONTROL_ENABLE:
        // we do RTS in software as STM32 hardware RTS support toggles
        // the pin for every byte which loses a lot of bandwidth
        palSetLineMode(arts_line, 1);
        palClearLine(arts_line);
        _rts_is_active = true;
        // enable hardware CTS support, disable RTS support as we do that in software
        chSysLock();
        if ((sd->usart->CR3 & (USART_CR3_CTSE | USART_CR3_RTSE)) != USART_CR3_CTSE) {
            // CTSE and RTSE can only be written when uart is disabled
            sd->usart->CR1 &= ~USART_CR1_UE;
            sd->usart->CR3 |= USART_CR3_CTSE;
            sd->usart->CR3 &= ~USART_CR3_RTSE;
            sd->usart->CR1 |= USART_CR1_UE;
        }
        chSysUnlock();
        break;

    case FLOW_CONTROL_RTS_DE:
        // Driver Enable, RTS pin high during transmit
        // If posible enable in hardware
#if defined(USART_CR3_DEM)
        if (sdef.rts_alternative_function != UINT8_MAX) {
            // Hand over control of RTS pin to the UART driver
            palSetLineMode(arts_line, PAL_MODE_ALTERNATE(sdef.rts_alternative_function));

            // Enable in driver, if not already set
            chSysLock();
            if ((sd->usart->CR3 & USART_CR3_DEM) != USART_CR3_DEM) {
                // Disable UART, set bit and then re-enable
                sd->usart->CR1 &= ~USART_CR1_UE;
                sd->usart->CR3 |= USART_CR3_DEM;
                sd->usart->CR1 |= USART_CR1_UE;
            }
            chSysUnlock();
        } else
#endif
        {
            // No hardware support for DEM mode or
            // No alternative function, RTS GPIO pin is not a conected to the UART peripheral
            // This is typicaly fine becaues we do software flow control.
            set_flow_control(FLOW_CONTROL_DISABLE);
        }
        break;
    }
#elif HAL_USE_SIO == TRUE
// RP2350 PL011 hardware flow control.
// RTS is driven in software by update_rts_line() (same strategy as STM32: hardware RTSEn per-FIFO-level is less flexible).
    _flow_control = (arts_line == 0) ? FLOW_CONTROL_DISABLE : flowcontrol;
    if (!is_initialized()) {
        // Not started yet; variable stored, will be applied by begin() on first use.
        return;
    }
    {
        // uart_pin_funcsel is the same FUNCSEL index for TX/RX/CTS/RTS on the same UART
        // (e.g. FUNCSEL=2 for UART0 on GPIO12/13/15/18).
        const uint32_t uart_funcsel = sdef.uart_pin_funcsel ? sdef.uart_pin_funcsel : 2U;
        SIODriver *sio = (SIODriver*)sdef.serial;
        switch (_flow_control) {

        case FLOW_CONTROL_DISABLE:
            // Return CTS/RTS pins to GPIO mode.  Drive nRTS LOW (asserted) so any
            // connected peer is always allowed to send.  Disable PL011 CTSEn/RTSEn.
            if (arts_line != 0) {
                palSetLineMode(arts_line, PAL_MODE_OUTPUT_PUSHPULL);
                palClearLine(arts_line);   // active-low: LOW = "I am ready to receive"
            }
            _rts_is_active = true;
            sio->uart->UARTCR &= ~(UART_UARTCR_CTSEN | UART_UARTCR_RTSEN);
            break;

        case FLOW_CONTROL_AUTO:
            // Reset the auto-detection timer state then fall through to ENABLE.
            _first_write_started_us = 0;
            _last_write_completed_us = 0;
            FALLTHROUGH;

        case FLOW_CONTROL_ENABLE:
            // RTS pin: keep as GPIO output (software-driven by update_rts_line()).
            // CTS pin: route to UART alternate function so the PL011 reads nCTS.
            if (arts_line != 0) {
                palSetLineMode(arts_line, PAL_MODE_OUTPUT_PUSHPULL);
                palClearLine(arts_line);   // assert nRTS initially
            }
            if (acts_line != 0) {
                // CTS is an input to the UART peripheral; FUNCSEL=uart_funcsel routes it.
                // Pull-up ensures an unconnected pin does not block transmission.
                palSetLineMode(acts_line, PAL_MODE_ALTERNATE(uart_funcsel));
                palLineSetPushPull(acts_line, PAL_PUSHPULL_PULLUP);
            }
            _rts_is_active = true;
            // Enable hardware CTS gating in PL011 UARTCR (bit 15).
            // Leave RTSEn clear so software drives nRTS via arts_line (GPIO).
            sio->uart->UARTCR |= UART_UARTCR_CTSEN;
            break;

        case FLOW_CONTROL_RTS_DE:
            // RS-485 DE mode not supported on RP2350 PL011; fall back to disabled.
            _flow_control = FLOW_CONTROL_DISABLE;
            if (arts_line != 0) {
                palSetLineMode(arts_line, PAL_MODE_OUTPUT_PUSHPULL);
                palClearLine(arts_line);
            }
            _rts_is_active = true;
            sio->uart->UARTCR &= ~(UART_UARTCR_CTSEN | UART_UARTCR_RTSEN);
            break;
        }
    }
#endif // HAL_USE_SERIAL
}

/*
  software update of rts line. We don't use the HW support for RTS as
  it has no hysteresis, so it ends up toggling RTS on every byte
 */
__RAMFUNC__ void UARTDriver::update_rts_line(void)
{
    if (arts_line == 0 || !flow_control_enabled(_flow_control)) {
        return;
    }
    uint16_t space = _readbuf.space();
    if (_rts_is_active && space < _rts_threshold) {
        _rts_is_active = false;
        palSetLine(arts_line);
    } else if (!_rts_is_active && space > _rts_threshold+16) {
        _rts_is_active = true;
        palClearLine(arts_line);
    }
}

/*
   setup unbuffered writes for lower latency
 */
bool UARTDriver::set_unbuffered_writes(bool on)
{
    unbuffered_writes = on;
#if defined(RP2350)
    if (uart_thread_ctx == nullptr && (sdef.is_usb || hal.scheduler->is_system_initialized())) {
        thread_init();
    }
#endif
    if (uart_thread_ctx != nullptr) {
        chEvtSignal(uart_thread_ctx, EVT_TRANSMIT_UNBUFFERED);
    }
    return true;
}

/*
  setup parity
 */
void UARTDriver::configure_parity(uint8_t v)
{
    if (sdef.is_usb) {
        // not possible
        return;
    }
    UARTDriver::parity = v;
#if HAL_USE_SERIAL == TRUE
    // stop and start to take effect
    sdStop((SerialDriver*)sdef.serial);

#ifdef USART_CR1_M0
    // cope with F3 and F7 where there are 2 bits in CR1_M
    const uint32_t cr1_m0 = USART_CR1_M0;
#else
    const uint32_t cr1_m0 = USART_CR1_M;
#endif

    switch (v) {
    case 0:
        // no parity
        sercfg.cr1 &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);
        break;
    case 1:
        // odd parity
        // setting USART_CR1_M ensures extra bit is used as parity
        // not last bit of data
        sercfg.cr1 |= cr1_m0 | USART_CR1_PCE;
        sercfg.cr1 |= USART_CR1_PS;
        break;
    case 2:
        // even parity
        sercfg.cr1 |= cr1_m0 | USART_CR1_PCE;
        sercfg.cr1 &= ~USART_CR1_PS;
        break;
    }

    sdStart((SerialDriver*)sdef.serial, &sercfg);

#if CH_CFG_USE_EVENTS == TRUE
    if (parity_enabled) {
        chEvtUnregister(chnGetEventSource((SerialDriver*)sdef.serial), &ev_listener);
    }
    parity_enabled = (v != 0);
    if (parity_enabled) {
        chEvtRegisterMaskWithFlags(chnGetEventSource((SerialDriver*)sdef.serial),
                                   &ev_listener,
                                   EVT_PARITY,
                                   SD_PARITY_ERROR);
    }
#endif

#ifndef HAL_UART_NODMA
    if (rx_dma_enabled) {
        // Configure serial driver to skip handling RX packets
        // because we will handle them via DMA
        ((SerialDriver*)sdef.serial)->usart->CR1 &= ~USART_CR1_RXNEIE;
    }
#endif
#endif // HAL_USE_SERIAL
}

/*
  set stop bits
 */
void UARTDriver::set_stop_bits(int n)
{
    if (sdef.is_usb) {
        // not possible
        return;
    }
#if HAL_USE_SERIAL
    // stop and start to take effect
    sdStop((SerialDriver*)sdef.serial);

    switch (n) {
    case 1:
        _cr2_options &= ~USART_CR2_STOP2_BITS;
        _cr2_options |= USART_CR2_STOP1_BITS;
        break;
    case 2:
        _cr2_options &= ~USART_CR2_STOP1_BITS;
        _cr2_options |= USART_CR2_STOP2_BITS;
        break;
    }
    sercfg.cr2 = _cr2_options;

    sdStart((SerialDriver*)sdef.serial, &sercfg);
#ifndef HAL_UART_NODMA
    if (rx_dma_enabled) {
        //Configure serial driver to skip handling RX packets
        //because we will handle them via DMA
        ((SerialDriver*)sdef.serial)->usart->CR1 &= ~USART_CR1_RXNEIE;
    }
#endif
#elif HAL_USE_SIO == TRUE
    // RP2350: store stop bits count in _cr2_options (not used as STM32 bitmask).
    _cr2_options = (n == 2) ? 2U : 1U;
#endif // HAL_USE_SERIAL
}


// record timestamp of new incoming data
__RAMFUNC__ void UARTDriver::receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
}

/*
  return timestamp estimate in microseconds for when the start of
  a nbytes packet arrived on the uart. This should be treated as a
  time constraint, not an exact time. It is guaranteed that the
  packet did not start being received after this time, but it
  could have been in a system buffer before the returned time.

  This takes account of the baudrate of the link. For transports
  that have no baudrate (such as USB) the time estimate may be
  less accurate.

  A return value of zero means the HAL does not support this API
*/
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0 && !sdef.is_usb) {
        // assume 10 bits per byte. For USB we assume zero transport delay
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

/*
 set user specified PULLUP/PULLDOWN options from SERIALn_OPTIONS
*/
void UARTDriver::set_pushpull(uint16_t options)
{
#if HAL_USE_SERIAL == TRUE && !defined(STM32F1)
    if ((options & OPTION_PULLDOWN_RX) && arx_line) {
        palLineSetPushPull(arx_line, PAL_PUSHPULL_PULLDOWN);
    }
    if ((options & OPTION_PULLDOWN_TX) && atx_line) {
        palLineSetPushPull(atx_line, PAL_PUSHPULL_PULLDOWN);
    }
    if ((options & OPTION_PULLUP_RX) && arx_line) {
        palLineSetPushPull(arx_line, PAL_PUSHPULL_PULLUP);
    }
    if ((options & OPTION_PULLUP_TX) && atx_line) {
        palLineSetPushPull(atx_line, PAL_PUSHPULL_PULLUP);
    }
#endif
}

// set optional features, return true on success
bool UARTDriver::set_options(uint16_t options)
{
    if (sdef.is_usb) {
        // no options allowed on USB
        return (options == 0);
    }
    bool ret = true;

    _last_options = options;

#if HAL_USE_SERIAL == TRUE
    SerialDriver *sd = (SerialDriver*)(sdef.serial);
    uint32_t cr2 = sd->usart->CR2;
    uint32_t cr3 = sd->usart->CR3;
    bool was_enabled = (sd->usart->CR1 & USART_CR1_UE);

    /*
      allow for RX, TX, RTS and CTS pins to be remapped via BRD_ALT_CONFIG
     */
    arx_line = GPIO::resolve_alt_config(sdef.rx_line, PERIPH_TYPE::UART_RX, sdef.instance);
    atx_line = GPIO::resolve_alt_config(sdef.tx_line, PERIPH_TYPE::UART_TX, sdef.instance);
    arts_line = GPIO::resolve_alt_config(sdef.rts_line, PERIPH_TYPE::OTHER, sdef.instance);
    acts_line = GPIO::resolve_alt_config(sdef.cts_line, PERIPH_TYPE::OTHER, sdef.instance);

    // Check flow control, might have to disable if RTS line is gone
    set_flow_control(_flow_control);

#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
    // F7 has built-in support for inversion in all uarts
    ioline_t rx_line = (options & OPTION_SWAP)?atx_line:arx_line;
    ioline_t tx_line = (options & OPTION_SWAP)?arx_line:atx_line;

    // if we are half-duplex then treat either inversion option as
    // both being enabled. This is easier to understand for users, who
    // can be confused as to which pin is the one that needs inversion
    if ((options & OPTION_HDPLEX) && (options & (OPTION_TXINV|OPTION_RXINV)) != 0) {
        options |= OPTION_TXINV|OPTION_RXINV;
    }

    if (options & OPTION_RXINV) {
        cr2 |= USART_CR2_RXINV;
        _cr2_options |= USART_CR2_RXINV;
        if (rx_line != 0) {
            palLineSetPushPull(rx_line, PAL_PUSHPULL_PULLDOWN);
        }
    } else {
        cr2 &= ~USART_CR2_RXINV;
        _cr2_options &= ~USART_CR2_RXINV;
        if (rx_line != 0) {
            palLineSetPushPull(rx_line, PAL_PUSHPULL_PULLUP);
        }
    }
    if (options & OPTION_TXINV) {
        cr2 |= USART_CR2_TXINV;
        _cr2_options |= USART_CR2_TXINV;
        if (tx_line != 0) {
            palLineSetPushPull(tx_line, PAL_PUSHPULL_PULLDOWN);
        }
    } else {
        cr2 &= ~USART_CR2_TXINV;
        _cr2_options &= ~USART_CR2_TXINV;
        if (tx_line != 0) {
            palLineSetPushPull(tx_line, PAL_PUSHPULL_PULLUP);
        }
    }
    // F7 can also support swapping RX and TX pins
    if (options & OPTION_SWAP) {
        cr2 |= USART_CR2_SWAP;
        _cr2_options |= USART_CR2_SWAP;
    } else {
        cr2 &= ~USART_CR2_SWAP;
        _cr2_options &= ~USART_CR2_SWAP;
    }
#elif defined(STM32F4) // STM32F4
    // F4 can do inversion by GPIO if enabled in hwdef.dat, using
    // TXINV and RXINV options
    if (options & OPTION_RXINV) {
        if (sdef.rxinv_gpio >= 0) {
            hal.gpio->write(sdef.rxinv_gpio, sdef.rxinv_polarity);
            if (arx_line != 0) {
                palLineSetPushPull(arx_line, PAL_PUSHPULL_PULLDOWN);
            }
        } else {
            ret = false;
        }
    } else if (sdef.rxinv_gpio >= 0) {
        hal.gpio->write(sdef.rxinv_gpio, !sdef.rxinv_polarity);
        if (arx_line != 0) {
            palLineSetPushPull(arx_line, PAL_PUSHPULL_PULLUP);
        }
    }
    if (options & OPTION_TXINV) {
        if (sdef.txinv_gpio >= 0) {
            hal.gpio->write(sdef.txinv_gpio, sdef.txinv_polarity);
            if (atx_line != 0) {
                palLineSetPushPull(atx_line, PAL_PUSHPULL_PULLDOWN);
            }
        } else {
            ret = false;
        }
    } else if (sdef.txinv_gpio >= 0) {
        hal.gpio->write(sdef.txinv_gpio, !sdef.txinv_polarity);
        if (atx_line != 0) {
            palLineSetPushPull(atx_line, PAL_PUSHPULL_PULLUP);
        }
    }
    if (options & OPTION_SWAP) {
        ret = false;
    }
#endif // STM32xx

    // both F4 and F7 can do half-duplex
    if (options & OPTION_HDPLEX) {
        cr3 |= USART_CR3_HDSEL;
        _cr3_options |= USART_CR3_HDSEL;
        if (!half_duplex) {
            chEvtRegisterMaskWithFlags(chnGetEventSource((SerialDriver*)sdef.serial),
                                       &hd_listener,
                                       EVT_TRANSMIT_END,
                                       CHN_OUTPUT_EMPTY | CHN_TRANSMISSION_END);
            half_duplex = true;
        }
#ifndef HAL_UART_NODMA
        if (rx_dma_enabled && rxdma) {
            dmaStreamDisable(rxdma);
        }
#endif
        // force DMA off when using half-duplex as the timing may affect other devices
        // sharing the DMA channel
        rx_dma_enabled = tx_dma_enabled = false;
    } else {
        cr3 &= ~USART_CR3_HDSEL;
        _cr3_options &= ~USART_CR3_HDSEL;
    }

    set_pushpull(options);

    if (sd->usart->CR2 == cr2 &&
        sd->usart->CR3 == cr3) {
        // no change
        return ret;
    }

    if (was_enabled) {
        sd->usart->CR1 &= ~USART_CR1_UE;
    }

    sd->usart->CR2 = cr2;
    sd->usart->CR3 = cr3;

    if (was_enabled) {
        sd->usart->CR1 |= USART_CR1_UE;
    }
#elif HAL_USE_SIO == TRUE
    // RP2350: use GPIO IO_BANK0 INOVER/OUTOVER bits to achieve UART line
    // inversion without requiring an external hardware inverter chip.
    arx_line = GPIO::resolve_alt_config(sdef.rx_line, PERIPH_TYPE::UART_RX, sdef.instance);
    atx_line = GPIO::resolve_alt_config(sdef.tx_line, PERIPH_TYPE::UART_TX, sdef.instance);
    // Resolve RTS/CTS lines for software/hardware flow control; parallels the
    // HAL_USE_SERIAL path above so set_flow_control() can route them correctly.
    arts_line = GPIO::resolve_alt_config(sdef.rts_line, PERIPH_TYPE::OTHER, sdef.instance);
    acts_line = GPIO::resolve_alt_config(sdef.cts_line, PERIPH_TYPE::OTHER, sdef.instance);
    if (arx_line != 0) {
        const uint32_t pad = PAL_PAD(arx_line);
        uint32_t ctrl = IO_BANK0->GPIO[pad].CTRL;
        ctrl &= ~(3U << 16U);  // clear INOVER[17:16]
        ctrl |= (options & OPTION_RXINV) ? PAL_RP_IOCTRL_INOVER_INV : PAL_RP_IOCTRL_INOVER_NOTINV;
        IO_BANK0->GPIO[pad].CTRL = ctrl;
    }
    if (atx_line != 0) {
        const uint32_t pad = PAL_PAD(atx_line);
        uint32_t ctrl = IO_BANK0->GPIO[pad].CTRL;
        ctrl &= ~(3U << 12U);  // clear OUTOVER[13:12]
        ctrl |= (options & OPTION_TXINV) ? PAL_RP_IOCTRL_OUTOVER_DRVINVPERI : PAL_RP_IOCTRL_OUTOVER_DRVPERI;
        IO_BANK0->GPIO[pad].CTRL = ctrl;
    }
    if (options & OPTION_SWAP) {
        ret = false;  // pin swap not supported on RP2350 hardware UART
    }
    set_pushpull(options);
#endif // HAL_USE_SERIAL == TRUE
    return ret;
}

// get optional features
uint16_t UARTDriver::get_options(void) const
{
    return _last_options;
}

#if HAL_UART_STATS_ENABLED
// request information on uart I/O for @SYS/uarts.txt for this uart
void UARTDriver::uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms)
{
    const uint32_t tx_bytes = stats.tx.update(_tx_stats_bytes);
    const uint32_t rx_bytes = stats.rx.update(_rx_stats_bytes);
    const uint32_t rx_dropped_bytes = stats.rx_dropped.update(_rx_stats_dropped_bytes);

    if (sdef.is_usb) {
        str.printf("OTG%u  ", unsigned(sdef.instance));
    } else {
        str.printf("UART%u ", unsigned(sdef.instance));
    }
    str.printf("TX%c=%8u RX%c=%8u TXBD=%6u RXBD=%6u RXDRP=%8u"
#if CH_CFG_USE_EVENTS == TRUE
                " FE=%lu OE=%lu NE=%lu"
#endif
                " FlowCtrl=%u\n",
               tx_dma_enabled ? '*' : ' ',
               unsigned(tx_bytes),
               rx_dma_enabled ? '*' : ' ',
               unsigned(rx_bytes),
               unsigned((tx_bytes * 10000) / dt_ms),
               unsigned((rx_bytes * 10000) / dt_ms),
               unsigned(rx_dropped_bytes),
#if CH_CFG_USE_EVENTS == TRUE
               _rx_stats_framing_errors,
               _rx_stats_overrun_errors,
               _rx_stats_noise_errors,
#endif
               _flow_control);
}
#endif

/*
  software control of the CTS pin if available. Return false if
  not available
*/
bool UARTDriver::set_CTS_pin(bool high)
{
    if (_flow_control != FLOW_CONTROL_DISABLE) {
        // CTS pin is being used
        return false;
    }
    if (acts_line == 0) {
        // we don't have a CTS pin on this UART
        return false;
    }
    palSetLineMode(acts_line, 1);
    palWriteLine(acts_line, high?1:0);
    return true;
}

/*
  software control of the RTS pin if available. Return false if
  not available
*/
bool UARTDriver::set_RTS_pin(bool high)
{
    if (_flow_control != FLOW_CONTROL_DISABLE) {
        // RTS pin is being used
        return false;
    }
    if (arts_line == 0) {
        // we don't have a RTS pin on this UART
        return false;
    }
    palSetLineMode(arts_line, 1);
    palWriteLine(arts_line, high?1:0);
    return true;
}

#if HAL_USE_SERIAL_USB == TRUE
/*
  initialise the USB bus, called from both UARTDriver and stdio for startup debug
  This can be called before the hal is initialised so must not call any hal functions
 */
void usb_initialise(void)
{
    static bool initialised;
    if (initialised) {
        return;
    }
    initialised = true;
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg1);
#if HAL_HAVE_DUAL_USB_CDC
    sduObjectInit(&SDU2);
    sduStart(&SDU2, &serusbcfg2);
#endif
    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg1.usbp);
    chThdSleep(chTimeUS2I(1500));
    usbStart(serusbcfg1.usbp, &usbcfg);
    usbConnectBus(serusbcfg1.usbp);
}
#endif

// disable TX/RX pins for unusued uart
void UARTDriver::disable_rxtx(void) const
{
    if (arx_line) {
        palSetLineMode(arx_line, PAL_MODE_INPUT);
    }
    if (atx_line) {
        palSetLineMode(atx_line, PAL_MODE_INPUT);
    }
}

#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

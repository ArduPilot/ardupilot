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

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && !defined(HAL_NO_UARTDRIVER)
#include "UARTDriver.h"
#include "GPIO.h"
#include <usbcfg.h>
#include "shared_dma.h"
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Common/ExpandingString.h>
#include "Scheduler.h"
#include "hwdef/common/stm32_util.h"

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

#ifdef HAL_USB_VENDOR_ID
// USB has been configured in hwdef.dat
#define HAVE_USB_SERIAL
#endif

#if HAL_WITH_IO_MCU
extern ChibiOS::UARTDriver uart_io;
#endif

const UARTDriver::SerialDef UARTDriver::_serial_tab[] = { HAL_UART_DEVICE_LIST };

// handle for UART handling thread
thread_t* volatile UARTDriver::uart_rx_thread_ctx;

// table to find UARTDrivers from serial number, used for event handling
UARTDriver *UARTDriver::uart_drivers[UART_MAX_DRIVERS];

// event used to wake up waiting thread. This event number is for
// caller threads
static const eventmask_t EVT_DATA = EVENT_MASK(10);

// event for parity error
static const eventmask_t EVT_PARITY = EVENT_MASK(11);

// event for transmit end for half-duplex
static const eventmask_t EVT_TRANSMIT_END = EVENT_MASK(12);

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

UARTDriver::UARTDriver(uint8_t _serial_num) :
serial_num(_serial_num),
sdef(_serial_tab[_serial_num]),
_baudrate(57600)
{
    osalDbgAssert(serial_num < UART_MAX_DRIVERS, "too many UART drivers");
    uart_drivers[serial_num] = this;
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
            if (uart_drivers[i] == nullptr) {
                continue;
            }
            if (uart_drivers[i]->_rx_initialised) {
                uart_drivers[i]->_rx_timer_tick();
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
            AP_HAL::panic("Could not create UART RX thread\n");
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
        uart_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(HAL_UART_STACK_SIZE),
                                              uart_thread_name,
                                              unbuffered_writes ? APM_UART_UNBUFFERED_PRIORITY : APM_UART_PRIORITY,
                                              uart_thread_trampoline,
                                              this);
        if (uart_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create UART TX thread\n");
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

void UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    thread_rx_init();

    if (sdef.serial == nullptr) {
        return;
    }
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
    while (_in_rx_timer) {
        hal.scheduler->delay(1);
    }
    if (rxS != _readbuf.get_size()) {
        _rx_initialised = false;
        _readbuf.set_size(rxS);
    }

    bool clear_buffers = false;
    if (b != 0) {
        // clear buffers on baudrate change, but not on the console (which is usually USB)
        if (_baudrate != b && hal.console != this) {
            clear_buffers = true;
        }
        _baudrate = b;
    }

    if (clear_buffers) {
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
    while (_in_tx_timer) {
        hal.scheduler->delay(1);
    }
    if (txS != _writebuf.get_size()) {
        _tx_initialised = false;
        _writebuf.set_size(txS);
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
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4)
                    dmaStreamSetPeripheral(rxdma, &((SerialDriver*)sdef.serial)->usart->RDR);
#else
                    dmaStreamSetPeripheral(rxdma, &((SerialDriver*)sdef.serial)->usart->DR);
#endif // STM32F7
#if STM32_DMA_SUPPORTS_DMAMUX
                    dmaSetRequestSource(rxdma, sdef.dma_rx_channel_id);
#endif
                }
                if (tx_dma_enabled) {
                    // we only allow for sharing of the TX DMA channel, not the RX
                    // DMA channel, as the RX side is active all the time, so
                    // cannot be shared
                    dma_handle = new Shared_DMA(sdef.dma_tx_stream_id,
                                                SHARED_DMA_NONE,
                                                FUNCTOR_BIND_MEMBER(&UARTDriver::dma_tx_allocate, void, Shared_DMA *),
                                                FUNCTOR_BIND_MEMBER(&UARTDriver::dma_tx_deallocate, void, Shared_DMA *));
                }
                _device_initialised = true;
            }
#endif // HAL_UART_NODMA
            sercfg.speed = _baudrate;

            // start with options from set_options()
            sercfg.cr1 = _cr1_options;
            sercfg.cr2 = _cr2_options;
            sercfg.cr3 = _cr3_options;

#ifndef HAL_UART_NODMA
            if (rx_dma_enabled) {
                sercfg.cr1 |= USART_CR1_IDLEIE;
                sercfg.cr3 |= USART_CR3_DMAR;
            }
            if (tx_dma_enabled) {
                sercfg.cr3 |= USART_CR3_DMAT;
            }
            sercfg.irq_cb = rx_irq_cb;
#endif // HAL_UART_NODMA
            if (!(sercfg.cr2 & USART_CR2_STOP2_BITS)) {
                sercfg.cr2 |= USART_CR2_STOP1_BITS;
            }
            sercfg.ctx = (void*)this;

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
#endif // HAL_USE_SERIAL
    }

    if (_writebuf.get_size()) {
        _tx_initialised = true;
    }
    if (_readbuf.get_size()) {
        _rx_initialised = true;
    }
    _uart_owner_thd = chThdGetSelfX();
    // initialize the TX thread if necessary
    thread_init();

    // setup flow control
    set_flow_control(_flow_control);

    if (serial_num == 0 && _tx_initialised) {
#ifndef HAL_STDOUT_SERIAL
        // setup hal.console to take printf() output
        vprintf_console_hook = hal_console_vprintf;
#endif
    }
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
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4)
    dmaStreamSetPeripheral(txdma, &((SerialDriver*)sdef.serial)->usart->TDR);
#else
    dmaStreamSetPeripheral(txdma, &((SerialDriver*)sdef.serial)->usart->DR);
#endif // STM32F7
#if STM32_DMA_SUPPORTS_DMAMUX
    dmaSetRequestSource(txdma, sdef.dma_tx_channel_id);
#endif
#endif // HAL_USE_SERIAL
}

#ifndef HAL_UART_NODMA
void UARTDriver::dma_rx_enable(void)
{
    uint32_t dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE;
    dmamode |= STM32_DMA_CR_CHSEL(sdef.dma_rx_channel_id);
    dmamode |= STM32_DMA_CR_PL(0);
#if defined(STM32H7)
    dmamode |= 1<<20;   // TRBUFF See 2.3.1 in the H743 errata
#endif
    rx_bounce_idx ^= 1;
    stm32_cacheBufferInvalidate(rx_bounce_buf[rx_bounce_idx], RX_BOUNCE_BUFSIZE);
    dmaStreamSetMemory0(rxdma, rx_bounce_buf[rx_bounce_idx]);
    dmaStreamSetTransactionSize(rxdma, RX_BOUNCE_BUFSIZE);
    dmaStreamSetMode(rxdma, dmamode | STM32_DMA_CR_DIR_P2M |
                     STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
    dmaStreamEnable(rxdma);
}
#endif

void UARTDriver::dma_tx_deallocate(Shared_DMA *ctx)
{
    chSysLock();
    dmaStreamFreeI(txdma);
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
#elif defined(STM32F3) || defined(STM32G4)
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
void UARTDriver::rxbuff_full_irq(void* self, uint32_t flags)
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
        uart_drv->_readbuf.write(uart_drv->rx_bounce_buf[bounce_idx], len);
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

void UARTDriver::begin(uint32_t b)
{
    begin(b, 0, 0);
}

void UARTDriver::end()
{
    while (_in_rx_timer) hal.scheduler->delay(1);
    _rx_initialised = false;
    while (_in_tx_timer) hal.scheduler->delay(1);
    _tx_initialised = false;

    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL

        sduStop((SerialUSBDriver*)sdef.serial);
#endif
    } else {
#if HAL_USE_SERIAL == TRUE
        sdStop((SerialDriver*)sdef.serial);
#endif
    }

    _readbuf.set_size(0);
    _writebuf.set_size(0);
}

void UARTDriver::flush()
{
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL

        sduSOFHookI((SerialUSBDriver*)sdef.serial);
#endif
    } else {
        //TODO: Handle this for other serial ports
    }
}

bool UARTDriver::is_initialized()
{
    return _tx_initialised && _rx_initialised;
}

void UARTDriver::set_blocking_writes(bool blocking)
{
    _blocking_writes = blocking;
}

bool UARTDriver::tx_pending() { return false; }

/* Empty implementations of Stream virtual methods */
uint32_t UARTDriver::available() {
    if (!_rx_initialised || lock_read_key) {
        return 0;
    }
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL

        if (((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE) {
            return 0;
        }
#endif
    }
    return _readbuf.available();
}

uint32_t UARTDriver::available_locked(uint32_t key)
{
    if (lock_read_key != 0 && key != lock_read_key) {
        return -1;
    }
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL

        if (((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE) {
            return 0;
        }
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

bool UARTDriver::discard_input()
{
    if (lock_read_key != 0 || _uart_owner_thd != chThdGetSelfX()){
        return false;
    }
    if (!_rx_initialised) {
        return false;
    }

    _readbuf.clear();

    if (!_rts_is_active) {
        update_rts_line();
    }

    return true;
}

ssize_t UARTDriver::read(uint8_t *buffer, uint16_t count)
{
    if (lock_read_key != 0 || _uart_owner_thd != chThdGetSelfX()){
        return -1;
    }
    if (!_rx_initialised) {
        return -1;
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

int16_t UARTDriver::read()
{
    if (lock_read_key != 0 || _uart_owner_thd != chThdGetSelfX()){
        return -1;
    }
    if (!_rx_initialised) {
        return -1;
    }

    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }
    if (!_rts_is_active) {
        update_rts_line();
    }

    return byte;
}

int16_t UARTDriver::read_locked(uint32_t key)
{
    if (lock_read_key != 0 && key != lock_read_key) {
        return -1;
    }
    if (!_rx_initialised) {
        return -1;
    }
    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }
    if (!_rts_is_active) {
        update_rts_line();
    }
    return byte;
}

/* write one byte to the port */
size_t UARTDriver::write(uint8_t c)
{
    if (lock_write_key != 0) {
        return 0;
    }
    _write_mutex.take_blocking();

    if (!_tx_initialised) {
        _write_mutex.give();
        return 0;
    }

    while (_writebuf.space() == 0) {
        if (!_blocking_writes || unbuffered_writes) {
            _write_mutex.give();
            return 0;
        }
        // release the semaphore while sleeping
        _write_mutex.give();
        hal.scheduler->delay(1);
        _write_mutex.take_blocking();
    }
    size_t ret = _writebuf.write(&c, 1);
    if (unbuffered_writes) {
        chEvtSignal(uart_thread_ctx, EVT_TRANSMIT_DATA_READY);
    }
    _write_mutex.give();
    return ret;
}

/* write a block of bytes to the port */
size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_tx_initialised || lock_write_key != 0) {
		return 0;
	}

    if (_blocking_writes && !unbuffered_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    WITH_SEMAPHORE(_write_mutex);

    size_t ret = _writebuf.write(buffer, size);
    if (unbuffered_writes) {
        chEvtSignal(uart_thread_ctx, EVT_TRANSMIT_DATA_READY);
    }
    return ret;
}

/*
  lock the uart for exclusive use by write_locked() and read_locked() with the right key
 */
bool UARTDriver::lock_port(uint32_t write_key, uint32_t read_key)
{
    if (lock_write_key && write_key != lock_write_key && read_key != 0) {
        // someone else is using it
        return false;
    }
    if (lock_read_key && read_key != lock_read_key && read_key != 0) {
        // someone else is using it
        return false;
    }
    lock_write_key = write_key;
    lock_read_key = read_key;
    return true;
}

/*
   write to a locked port. If port is locked and key is not correct then 0 is returned
   and write is discarded. All writes are non-blocking
*/
size_t UARTDriver::write_locked(const uint8_t *buffer, size_t size, uint32_t key)
{
    if (lock_write_key != 0 && key != lock_write_key) {
        return 0;
    }
    WITH_SEMAPHORE(_write_mutex);
    return _writebuf.write(buffer, size);
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
void UARTDriver::tx_complete(void* self, uint32_t flags)
{
    UARTDriver* uart_drv = (UARTDriver*)self;
    chSysLockFromISR();

    // check nothing bad happened
    if ((flags & STM32_DMA_ISR_TEIF) != 0) {
        INTERNAL_ERROR(AP_InternalError::error_t::dma_fail);
    }

    dmaStreamDisable(uart_drv->txdma);
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
        if (_flow_control != FLOW_CONTROL_DISABLE &&
            sdef.cts_line != 0 &&
            palReadLine(sdef.cts_line)) {
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
#if defined(USART_CR1_FIFOEN)
            const uint32_t baud_threshold = 460800;
#else
            const uint32_t baud_threshold = 115200;
#endif
            if (_baudrate <= baud_threshold) {
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
        dmaStreamDisable(txdma);
        stm32_cacheBufferFlush(tx_bounce_buf, tx_len);
        dmaStreamSetMemory0(txdma, tx_bounce_buf);
        dmaStreamSetTransactionSize(txdma, tx_len);
        uint32_t dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE;
        dmamode |= STM32_DMA_CR_CHSEL(sdef.dma_tx_channel_id);
        dmamode |= STM32_DMA_CR_PL(0);
#if defined(STM32H7)
        dmamode |= 1<<20;   // TRBUFF See 2.3.1 in the H743 errata
#endif
        dmaStreamSetMode(txdma, dmamode | STM32_DMA_CR_DIR_M2P |
                        STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
        dmaStreamEnable(txdma);
        uint32_t timeout_us = ((1000000UL * (tx_len+2) * 10) / _baudrate) + 500;
        chSysUnlock();
        // wait for the completion or timeout handlers to signal that we are done
        eventmask_t mask = chEvtWaitAnyTimeout(EVT_TRANSMIT_DMA_COMPLETE, chTimeUS2I(timeout_us));
        // handle a TX timeout. This can happen with using hardware flow
        // control if CTS pin blocks transmit or sometimes the DMA completion simply disappears
        if (mask == 0) {
            chSysLock();
            // check whether DMA completion happened in the intervening time
            // first disable the stream to prevent further interrupts
            dmaStreamDisable(txdma);

            const uint32_t tx_size = dmaStreamGetTransactionSize(txdma);

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
            chEvtGetAndClearEvents(EVT_TRANSMIT_DMA_COMPLETE);
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
            ret = 0;
#ifdef HAVE_USB_SERIAL
            ret = chnWriteTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#endif
        } else {
#if HAL_USE_SERIAL == TRUE
            ret = chnWriteTimeout((SerialDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#endif
        }
        if (ret < 0) {
            break;
        }
        if (ret > 0) {
            _last_write_completed_us = AP_HAL::micros();
            nwritten += ret;
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
            uint32_t space = qSpaceI(&((SerialDriver*)sdef.serial)->oqueue);
            uint32_t used = SERIAL_BUFFERS_SIZE - space;
            // threshold is 8 for the GCS_Common code to unstick SiK radios, which
            // sends 6 bytes with flow control disabled
            const uint8_t threshold = 8;
            if (_total_written > used && _total_written - used > threshold) {
                _flow_control = FLOW_CONTROL_ENABLE;
                return;
            }
        }
        if (AP_HAL::micros() - _first_write_started_us > 500*1000UL) {
            // it doesn't look like hw flow control is working
            hal.console->printf("disabling flow control on serial %u\n", sdef.get_index());
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
#ifdef HAVE_USB_SERIAL
    if (!hd_tx_active) {
        chEvtGetAndClearFlags(&hd_listener);
        // half-duplex transmission is done when both the output is empty and the transmission is ended
        // if we only wait for empty output the line can be setup for receive too soon losing data bits
        hd_tx_active = CHN_TRANSMISSION_END | CHN_OUTPUT_EMPTY;
        SerialDriver *sd = (SerialDriver*)(sdef.serial);
        sdStop(sd);
        sercfg.cr3 &= ~USART_CR3_HDSEL;
        sdStart(sd, &sercfg);
    }
#endif
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

    _in_rx_timer = true;

#ifndef HAL_UART_NODMA
    if (rx_dma_enabled && rxdma) {
        chSysLock();
        //Check if DMA is enabled
        //if not, it might be because the DMA interrupt was silenced
        //let's handle that here so that we can continue receiving
#if defined(STM32F3) || defined(STM32G4)
        bool enabled = (rxdma->channel->CCR & STM32_DMA_CR_EN);
#else
        bool enabled = (rxdma->stream->CR & STM32_DMA_CR_EN);
#endif
        if (!enabled) {
            uint8_t len = RX_BOUNCE_BUFSIZE - dmaStreamGetTransactionSize(rxdma);
            if (len != 0) {
                _readbuf.write(rx_bounce_buf[rx_bounce_idx], len);
                _rx_stats_bytes += len;

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
        chSysUnlock();
    }
#endif

    // don't try IO on a disconnected USB port
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        if (((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE) {
            _in_rx_timer = false;
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
    _in_rx_timer = false;
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
#endif
        }
        if (ret < 0) {
            break;
        }
#if CH_CFG_USE_EVENTS == TRUE
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

    _in_tx_timer = true;

    if (hd_tx_active) {
        hd_tx_active &= ~chEvtGetAndClearFlags(&hd_listener);
        if (!hd_tx_active) {
            /*
                half-duplex transmit has finished. We now re-enable the
                HDSEL bit for receive
            */
            SerialDriver *sd = (SerialDriver*)(sdef.serial);
            sdStop(sd);
            sercfg.cr3 |= USART_CR3_HDSEL;
            sdStart(sd, &sercfg);
        }
    }

    // don't try IO on a disconnected USB port
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        if (((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE) {
            _in_tx_timer = false;
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
        _in_rx_timer = true;
        read_bytes_NODMA();
        if (_wait.thread_ctx && _readbuf.available() >= _wait.n) {
            chEvtSignal(_wait.thread_ctx, EVT_DATA);
        }
        _in_rx_timer = false;
    }

    // now do the write
    write_pending_bytes();

    _in_tx_timer = false;
}

/*
  change flow control mode for port
 */
void UARTDriver::set_flow_control(enum flow_control flowcontrol)
{
    if (sdef.rts_line == 0 || sdef.is_usb) {
        // no hw flow control available
        return;
    }
#if HAL_USE_SERIAL == TRUE
    SerialDriver *sd = (SerialDriver*)(sdef.serial);
    _flow_control = flowcontrol;
    if (!is_initialized()) {
        // not ready yet, we just set variable for when we call begin
        return;
    }
    switch (_flow_control) {

    case FLOW_CONTROL_DISABLE:
        // force RTS active when flow disabled
        palSetLineMode(sdef.rts_line, 1);
        palClearLine(sdef.rts_line);
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
        palSetLineMode(sdef.rts_line, 1);
        palClearLine(sdef.rts_line);
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
    }
#endif // HAL_USE_SERIAL
}

/*
  software update of rts line. We don't use the HW support for RTS as
  it has no hysteresis, so it ends up toggling RTS on every byte
 */
void UARTDriver::update_rts_line(void)
{
    if (sdef.rts_line == 0 || _flow_control == FLOW_CONTROL_DISABLE) {
        return;
    }
    uint16_t space = _readbuf.space();
    if (_rts_is_active && space < 16) {
        _rts_is_active = false;
        palSetLine(sdef.rts_line);
    } else if (!_rts_is_active && space > 32) {
        _rts_is_active = true;
        palClearLine(sdef.rts_line);
    }
}

/*
   setup unbuffered writes for lower latency
 */
bool UARTDriver::set_unbuffered_writes(bool on)
{
    unbuffered_writes = on;
    chEvtSignal(uart_thread_ctx, EVT_TRANSMIT_UNBUFFERED);
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
#endif // HAL_USE_SERIAL
}


// record timestamp of new incoming data
void UARTDriver::receive_timestamp_update(void)
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

#ifdef HAL_PIN_ALT_CONFIG
    /*
      allow for RX and TX pins to be remapped via BRD_ALT_CONFIG
     */
    arx_line = GPIO::resolve_alt_config(sdef.rx_line, PERIPH_TYPE::UART_RX, sdef.instance);
    atx_line = GPIO::resolve_alt_config(sdef.tx_line, PERIPH_TYPE::UART_TX, sdef.instance);
#else
    arx_line = sdef.rx_line;
    atx_line = sdef.tx_line;
#endif

#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4)
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
#else // STM32F4
    // F4 can do inversion by GPIO if enabled in hwdef.dat, using
    // TXINV and RXINV options
    if (options & OPTION_RXINV) {
        if (sdef.rxinv_gpio >= 0) {
            hal.gpio->write(sdef.rxinv_gpio, sdef.rxinv_polarity);
        } else {
            ret = false;
        }
    }
    if (options & OPTION_TXINV) {
        if (sdef.txinv_gpio >= 0) {
            hal.gpio->write(sdef.txinv_gpio, sdef.txinv_polarity);
        } else {
            ret = false;
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
#endif // HAL_USE_SERIAL == TRUE
    return ret;
}

// get optional features
uint8_t UARTDriver::get_options(void) const
{
    return _last_options;
}

// request information on uart I/O for @SYS/uarts.txt for this uart
void UARTDriver::uart_info(ExpandingString &str)
{
    uint32_t now_ms = AP_HAL::millis();
    if (sdef.is_usb) {
        str.printf("OTG%u  ", unsigned(sdef.instance));
    } else {
        str.printf("UART%u ", unsigned(sdef.instance));
    }
    str.printf("TX%c=%8u RX%c=%8u TXBD=%6u RXBD=%6u\n",
               tx_dma_enabled ? '*' : ' ',
               unsigned(_tx_stats_bytes),
               rx_dma_enabled ? '*' : ' ',
               unsigned(_rx_stats_bytes),
               unsigned(_tx_stats_bytes * 10000 / (now_ms - _last_stats_ms)),
               unsigned(_rx_stats_bytes * 10000 / (now_ms - _last_stats_ms)));
    _tx_stats_bytes = 0;
    _rx_stats_bytes = 0;
    _last_stats_ms = now_ms;
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

#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

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

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "UARTDriver.h"
#include "GPIO.h"
#include <usbcfg.h>
#include "shared_dma.h"

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

#ifdef HAL_USB_VENDOR_ID
// USB has been configured in hwdef.dat
#define HAVE_USB_SERIAL
#endif

const UARTDriver::SerialDef UARTDriver::_serial_tab[] = { HAL_UART_DEVICE_LIST };

// event used to wake up waiting thread
#define EVT_DATA EVENT_MASK(0)

UARTDriver::UARTDriver(uint8_t serial_num) :
tx_bounce_buf_ready(true),
sdef(_serial_tab[serial_num]),
_baudrate(57600),
_in_timer(false),
_initialised(false)
{
    chMtxObjectInit(&_write_mutex);
}

void UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    hal.gpio->pinMode(2, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(3, HAL_GPIO_OUTPUT);
    if (sdef.serial == nullptr) {
        return;
    }
    bool was_initialised = _initialised;
    uint16_t min_tx_buffer = 4096;
    uint16_t min_rx_buffer = 1024;
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
    if (rxS != _readbuf.get_size()) {
        _initialised = false;
        while (_in_timer) {
            hal.scheduler->delay(1);
        }

        _readbuf.set_size(rxS);
    }

    if (b != 0) {
        _baudrate = b;
    }

    /*
      allocate the write buffer
     */
    if (txS != _writebuf.get_size()) {
        _initialised = false;
        while (_in_timer) {
            hal.scheduler->delay(1);
        }
        _writebuf.set_size(txS);
    }

    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        /*
         * Initializes a serial-over-USB CDC driver.
         */
        if (!was_initialised) {
            sduObjectInit((SerialUSBDriver*)sdef.serial);
            sduStart((SerialUSBDriver*)sdef.serial, &serusbcfg);
            /*
             * Activates the USB driver and then the USB bus pull-up on D+.
             * Note, a delay is inserted in order to not have to disconnect the cable
             * after a reset.
             */
            usbDisconnectBus(serusbcfg.usbp);
            hal.scheduler->delay_microseconds(1500);
            usbStart(serusbcfg.usbp, &usbcfg);
            usbConnectBus(serusbcfg.usbp);
        }
#endif
    } else {
        if (_baudrate != 0) {
            //setup Rx DMA
            if(!was_initialised) {
                if(sdef.dma_rx) {
                    rxdma = STM32_DMA_STREAM(sdef.dma_rx_stream_id);
                    bool dma_allocated = dmaStreamAllocate(rxdma,
                                               12,  //IRQ Priority
                                               (stm32_dmaisr_t)rxbuff_full_irq,
                                               (void *)this);
                    osalDbgAssert(!dma_allocated, "stream already allocated");
                    dmaStreamSetPeripheral(rxdma, &((SerialDriver*)sdef.serial)->usart->DR);
                }
                if (sdef.dma_tx) {
                    // we only allow for sharing of the TX DMA channel, not the RX
                    // DMA channel, as the RX side is active all the time, so
                    // cannot be shared
                    dma_handle = new Shared_DMA(sdef.dma_tx_stream_id,
                                                SHARED_DMA_NONE,
                                                FUNCTOR_BIND_MEMBER(&UARTDriver::dma_tx_allocate, void),
                                                FUNCTOR_BIND_MEMBER(&UARTDriver::dma_tx_deallocate, void));
                }
            }
            sercfg.speed = _baudrate;
            if (!sdef.dma_tx && !sdef.dma_rx) {
                sercfg.cr1 = 0;
                sercfg.cr3 = 0;
            } else {
                if (sdef.dma_rx) {
                    sercfg.cr1 = USART_CR1_IDLEIE;
                    sercfg.cr3 = USART_CR3_DMAR;
                }
                if (sdef.dma_tx) {
                    sercfg.cr3 |= USART_CR3_DMAT;
                }
            }
            sercfg.cr2 = USART_CR2_STOP1_BITS;
            sercfg.irq_cb = rx_irq_cb;
            sercfg.ctx = (void*)this;
            
            sdStart((SerialDriver*)sdef.serial, &sercfg);
            if(sdef.dma_rx) {
                //Configure serial driver to skip handling RX packets
                //because we will handle them via DMA
                ((SerialDriver*)sdef.serial)->usart->CR1 &= ~USART_CR1_RXNEIE;
                //Start DMA
                if(!was_initialised) {
                    uint32_t dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE;
                    dmamode |= STM32_DMA_CR_CHSEL(STM32_DMA_GETCHANNEL(sdef.dma_rx_stream_id,
                                                                       sdef.dma_rx_channel_id));
                    dmamode |= STM32_DMA_CR_PL(0);
                    dmaStreamSetMemory0(rxdma, rx_bounce_buf);
                    dmaStreamSetTransactionSize(rxdma, RX_BOUNCE_BUFSIZE);
                    dmaStreamSetMode(rxdma, dmamode    | STM32_DMA_CR_DIR_P2M |
                                         STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
                    dmaStreamEnable(rxdma);
                }
            }
        }
    }

    if (_writebuf.get_size() && _readbuf.get_size()) {
        _initialised = true;
    }
    _uart_owner_thd = chThdGetSelfX();

    // setup flow control
    set_flow_control(_flow_control);
}

void UARTDriver::dma_tx_allocate(void)
{
    osalDbgAssert(txdma == nullptr, "double DMA allocation");
    txdma = STM32_DMA_STREAM(sdef.dma_tx_stream_id);
    bool dma_allocated = dmaStreamAllocate(txdma,
                                           12,  //IRQ Priority
                                           (stm32_dmaisr_t)tx_complete,
                                           (void *)this);
    osalDbgAssert(!dma_allocated, "stream already allocated");
    dmaStreamSetPeripheral(txdma, &((SerialDriver*)sdef.serial)->usart->DR);
}

void UARTDriver::dma_tx_deallocate(void)
{
    chSysLock();
    dmaStreamRelease(txdma);
    txdma = nullptr;
    chSysUnlock();
}

void UARTDriver::tx_complete(void* self, uint32_t flags)
{
    UARTDriver* uart_drv = (UARTDriver*)self;
    uart_drv->dma_handle->unlock_from_IRQ();
    uart_drv->_last_write_completed_us = AP_HAL::micros();
    uart_drv->tx_bounce_buf_ready = true;
}


void UARTDriver::rx_irq_cb(void* self)
{
    UARTDriver* uart_drv = (UARTDriver*)self;
    if (!uart_drv->sdef.dma_rx) {
        return;
    }
    volatile uint16_t sr = ((SerialDriver*)(uart_drv->sdef.serial))->usart->SR;
    if(sr & USART_SR_IDLE) {
        volatile uint16_t dr = ((SerialDriver*)(uart_drv->sdef.serial))->usart->DR;
        (void)dr;
        //disable dma, triggering DMA transfer complete interrupt
        uart_drv->rxdma->stream->CR &= ~STM32_DMA_CR_EN;
    }
}

void UARTDriver::rxbuff_full_irq(void* self, uint32_t flags)
{
    UARTDriver* uart_drv = (UARTDriver*)self;
    if (uart_drv->_lock_rx_in_timer_tick) {
        return;
    }
    if (!uart_drv->sdef.dma_rx) {
        return;
    }
    uint8_t len = RX_BOUNCE_BUFSIZE - uart_drv->rxdma->stream->NDTR;
    if (len == 0) {
        return;
    }
    uart_drv->_readbuf.write(uart_drv->rx_bounce_buf, len);
    //restart the DMA transfers
    dmaStreamSetMemory0(uart_drv->rxdma, uart_drv->rx_bounce_buf);
    dmaStreamSetTransactionSize(uart_drv->rxdma, RX_BOUNCE_BUFSIZE);
    dmaStreamEnable(uart_drv->rxdma);
    if (uart_drv->_wait.thread_ctx && uart_drv->_readbuf.available() >= uart_drv->_wait.n) {
        chSysLockFromISR();
        chEvtSignalI(uart_drv->_wait.thread_ctx, EVT_DATA);                    
        chSysUnlockFromISR();
    }
    if (uart_drv->_rts_is_active) {
        uart_drv->update_rts_line();
    }
}

void UARTDriver::begin(uint32_t b)
{
    begin(b, 0, 0);
}

void UARTDriver::end()
{
    _initialised = false;
    while (_in_timer) hal.scheduler->delay(1);

    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL

        sduStop((SerialUSBDriver*)sdef.serial);
#endif
    } else {
        sdStop((SerialDriver*)sdef.serial);
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
    return _initialised;
}

void UARTDriver::set_blocking_writes(bool blocking)
{
    _nonblocking_writes = !blocking;
}

bool UARTDriver::tx_pending() { return false; }

/* Empty implementations of Stream virtual methods */
uint32_t UARTDriver::available() {
    if (!_initialised) {
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

uint32_t UARTDriver::txspace()
{
    if (!_initialised) {
        return 0;
    }
    return _writebuf.space();
}

int16_t UARTDriver::read()
{
    if (_uart_owner_thd != chThdGetSelfX()){
        return -1;
    }
    if (!_initialised) {
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

/* Empty implementations of Print virtual methods */
size_t UARTDriver::write(uint8_t c)
{
    if (!chMtxTryLock(&_write_mutex)) {
        return -1;
    }
    
    if (!_initialised) {
        chMtxUnlock(&_write_mutex);
        return 0;
    }

    while (_writebuf.space() == 0) {
        if (_nonblocking_writes) {
            chMtxUnlock(&_write_mutex);
            return 0;
        }
        hal.scheduler->delay(1);
    }
    size_t ret = _writebuf.write(&c, 1);
    chMtxUnlock(&_write_mutex);
    return ret;
}

size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
		return 0;
	}

    if (!chMtxTryLock(&_write_mutex)) {
        return -1;
    }

    if (!_nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        chMtxUnlock(&_write_mutex);
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    size_t ret = _writebuf.write(buffer, size);
    chMtxUnlock(&_write_mutex);
    return ret;
}

/*
  wait for data to arrive, or a timeout. Return true if data has
  arrived, false on timeout
 */
bool UARTDriver::wait_timeout(uint16_t n, uint32_t timeout_ms)
{
    chEvtGetAndClearEvents(EVT_DATA);
    if (available() >= n) {
        return true;
    }
    _wait.n = n;
    _wait.thread_ctx = chThdGetSelfX();
    eventmask_t mask = chEvtWaitAnyTimeout(EVT_DATA, MS2ST(timeout_ms));
    return (mask & EVT_DATA) != 0;
}

/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void UARTDriver::_timer_tick(void)
{
    int ret;
    uint32_t n;

    if (!_initialised) return;

    if (sdef.dma_rx && rxdma) {
        _lock_rx_in_timer_tick = true;
        //Check if DMA is enabled
        //if not, it might be because the DMA interrupt was silenced
        //let's handle that here so that we can continue receiving
        if (!(rxdma->stream->CR & STM32_DMA_CR_EN)) {
            uint8_t len = RX_BOUNCE_BUFSIZE - rxdma->stream->NDTR;
            if (len != 0) {
                _readbuf.write(rx_bounce_buf, len);
                if (_wait.thread_ctx && _readbuf.available() >= _wait.n) {
                    chEvtSignal(_wait.thread_ctx, EVT_DATA);                    
                }
                if (_rts_is_active) {
                    update_rts_line();
                }
            }
            //DMA disabled by idle interrupt never got a chance to be handled
            //we will enable it here
            dmaStreamSetMemory0(rxdma, rx_bounce_buf);
            dmaStreamSetTransactionSize(rxdma, RX_BOUNCE_BUFSIZE);
            dmaStreamEnable(rxdma);
        }
        _lock_rx_in_timer_tick = false;
    }

    // don't try IO on a disconnected USB port
    if (sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        if (((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE) {
            return;
        }
#endif
    }
    if(sdef.is_usb) {
#ifdef HAVE_USB_SERIAL
        ((GPIO *)hal.gpio)->set_usb_connected();
#endif
    }
    _in_timer = true;

    {
        // try to fill the read buffer
        ByteBuffer::IoVec vec[2];

        const auto n_vec = _readbuf.reserve(vec, _readbuf.space());
        for (int i = 0; i < n_vec; i++) {
            //Do a non-blocking read
            if (sdef.is_usb) {
                ret = 0;
    #ifdef HAVE_USB_SERIAL
                ret = chnReadTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
    #endif
            } else if(!sdef.dma_rx){
                ret = 0;
                ret = chnReadTimeout((SerialDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
            } else {
                ret = 0;
            }
            if (ret < 0) {
                break;
            }
            _readbuf.commit((unsigned)ret);

            /* stop reading as we read less than we asked for */
            if ((unsigned)ret < vec[i].len) {
                break;
            }
        }
    }

    // write any pending bytes
    n = _writebuf.available();
    if (n > 0) {
        if(!sdef.dma_tx) {
            ByteBuffer::IoVec vec[2];
            const auto n_vec = _writebuf.peekiovec(vec, n);
            for (int i = 0; i < n_vec; i++) {
                if (sdef.is_usb) {
                    ret = 0;
    #ifdef HAVE_USB_SERIAL
                    ret = chnWriteTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
    #endif
                } else {
                    ret = chnWriteTimeout((SerialDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
                }
                if (ret < 0) {
                    break;
                }
                if (ret > 0) {
                    _last_write_completed_us = AP_HAL::micros();
                }
                _writebuf.advance(ret);

                /* We wrote less than we asked for, stop */
                if ((unsigned)ret != vec[i].len) {
                    break;
                }
            }
        } else {
            if(tx_bounce_buf_ready) {
                /* TX DMA channel preparation.*/
                _writebuf.advance(tx_len);
                tx_len = _writebuf.peekbytes(tx_bounce_buf, TX_BOUNCE_BUFSIZE);
                if (tx_len == 0) {
                    goto end;
                }
                dma_handle->lock();
                tx_bounce_buf_ready = false;
                osalDbgAssert(txdma != nullptr, "UART TX DMA allocation failed");
                dmaStreamSetMemory0(txdma, tx_bounce_buf);
                dmaStreamSetTransactionSize(txdma, tx_len);
                uint32_t dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE;
                dmamode |= STM32_DMA_CR_CHSEL(STM32_DMA_GETCHANNEL(sdef.dma_tx_stream_id,
                                                                   sdef.dma_tx_channel_id));
                dmamode |= STM32_DMA_CR_PL(0);
                dmaStreamSetMode(txdma, dmamode | STM32_DMA_CR_DIR_M2P |
                                 STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
                dmaStreamEnable(txdma);
            } else if (sdef.dma_tx && txdma) {
                if (!(txdma->stream->CR & STM32_DMA_CR_EN)) {
                    if (txdma->stream->NDTR == 0) {
                        tx_bounce_buf_ready = true;
                        _last_write_completed_us = AP_HAL::micros();
                        dma_handle->unlock();
                    }
                }
            }
        }
        
        // handle AUTO flow control mode
        if (_flow_control == FLOW_CONTROL_AUTO) {
            if (_first_write_started_us == 0) {
                _first_write_started_us = AP_HAL::micros();
            }
            if (_last_write_completed_us != 0) {
                _flow_control = FLOW_CONTROL_ENABLE;
            } else if (AP_HAL::micros() - _first_write_started_us > 500*1000UL) {
                // it doesn't look like hw flow control is working
                hal.console->printf("disabling flow control on serial %u\n", sdef.get_index());
                set_flow_control(FLOW_CONTROL_DISABLE);
            }
        }
    }
end:
    _in_timer = false;
}

/*
  change flow control mode for port
 */
void UARTDriver::set_flow_control(enum flow_control flow_control)
{
    if (sdef.rts_line == 0 || sdef.is_usb) {
        // no hw flow control available
        return;
    }

    _flow_control = flow_control;
    if (!_initialised) {
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
        ((SerialDriver*)(sdef.serial))->usart->CR3 &= ~USART_CR3_CTSE;
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
        // enable hardware CTS support
        ((SerialDriver*)(sdef.serial))->usart->CR3 |= USART_CR3_CTSE;
        break;
    }
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

#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

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
 * Code by @davidbuzz and Claude
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "UARTDriver.h"
#include "hwdef.h"

#include <errno.h>
#include <stdio.h>

#ifdef __ZEPHYR__
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#endif

#include <AP_Common/ExpandingString.h>

using namespace Zephyr;

extern const AP_HAL::HAL& hal;

UARTDriver::UARTDriver(uint8_t serial_num) :
    _serial_num(serial_num)
{
}

#ifdef __ZEPHYR__
/* ArduPilot SERIALn → Zephyr device, generated from hwdef.dat's
   SERIAL_ORDER by zephyr_hwdef.py. Boards without SERIAL_ORDER (or with
   the devices disabled in DTS) fall through to nullptr. */
static const struct device *uart_device_for_serial(uint8_t serial_num)
{
#ifdef HAL_UART_DT_DEVICE_LOOKUP
    HAL_UART_DT_DEVICE_LOOKUP(serial_num)
#else
    (void)serial_num;
#endif
    return nullptr;
}
#endif

#if defined(CONFIG_UART_MCUX_LPUART)
/* Devicetree register base for a bound LPUART device - lets the RXINV/TXINV
   pokes in _begin() work on any port instead of a hardcoded LPUART6 base.
   Returns 0 for non-LPUART devices (e.g. the USB CDC "uart"). */
static uint32_t _ap_lpuart_base(const struct device *dev)
{
#define AP_LPUART_BASE_CASE(label)                                        \
    do {                                                                  \
        if (dev == DEVICE_DT_GET(DT_NODELABEL(label))) {                  \
            return DT_REG_ADDR(DT_NODELABEL(label));                      \
        }                                                                 \
    } while (0)
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart1), okay)
    AP_LPUART_BASE_CASE(lpuart1);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart2), okay)
    AP_LPUART_BASE_CASE(lpuart2);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart3), okay)
    AP_LPUART_BASE_CASE(lpuart3);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart4), okay)
    AP_LPUART_BASE_CASE(lpuart4);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart5), okay)
    AP_LPUART_BASE_CASE(lpuart5);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart6), okay)
    AP_LPUART_BASE_CASE(lpuart6);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart7), okay)
    AP_LPUART_BASE_CASE(lpuart7);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart8), okay)
    AP_LPUART_BASE_CASE(lpuart8);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart9), okay)
    AP_LPUART_BASE_CASE(lpuart9);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart10), okay)
    AP_LPUART_BASE_CASE(lpuart10);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart11), okay)
    AP_LPUART_BASE_CASE(lpuart11);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart12), okay)
    AP_LPUART_BASE_CASE(lpuart12);
#endif
#undef AP_LPUART_BASE_CASE
    return 0;
}
#endif  /* CONFIG_UART_MCUX_LPUART */

void UARTDriver::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    if (baud == 0) {
        baud = 115200;  // fallback if none requested
    }

#ifdef __ZEPHYR__
    /* Only enable USB for serial 0 when the underlying device is USB CDC and
     * the legacy USB device stack is active. With USB_DEVICE_STACK_NEXT the
     * CDC ACM interface is initialised at boot by Zephyr (CDC_ACM_SERIAL_
     * INITIALIZE_AT_BOOT=y), so usb_enable() must not be called. */
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(usb_cdc_acm0), okay) || \
     DT_NODE_HAS_STATUS(DT_NODELABEL(cdc_acm0), okay)) && \
    !defined(CONFIG_USB_DEVICE_STACK_NEXT)
    if (_serial_num == 0) {
        static bool usb_enabled = false;
        if (!usb_enabled) {
            const int ret = usb_enable(nullptr);
            if (ret != 0 && ret != -EALREADY) {
                _initialized = false;
                return;
            }
            usb_enabled = true;
        }
    }
#endif

    _dev = uart_device_for_serial(_serial_num);
    if (_dev == nullptr) {
        /* Warn ONCE per port: AP_RCProtocol's serial_configs[] autodetect reopens the port
         * repeatedly, so an unconditional warning floods the console. */
        if (!_lookup_warned) {
            _lookup_warned = true;
            printk("UART: device lookup returned nullptr for serial %d\n", _serial_num);
        }
        _initialized = false;
        return;
    }
    if (!device_is_ready(_dev)) {
        if (!_lookup_warned) {
            _lookup_warned = true;
            printk("UART: device_is_ready() failed for serial %d\n", _serial_num);
        }
        _initialized = false;
        return;
    }
    _lookup_warned = false;

    struct uart_config cfg = {};
    cfg.baudrate = baud;
    /* _parity/_stop_bits are whatever configure_parity()/set_stop_bits()
       last stored (default 0/1 = 8N1). AP_RCProtocol's serial_configs[]
       cycling (SBUS/CRSF/FastSBUS/etc autodetection) calls both, in that
       order, immediately before every begin() - see the class-level
       comment on those overrides. */
    switch (_parity) {
    case 1:  cfg.parity = UART_CFG_PARITY_ODD;  break;
    case 2:  cfg.parity = UART_CFG_PARITY_EVEN; break;
    default: cfg.parity = UART_CFG_PARITY_NONE; break;
    }
    cfg.stop_bits = (_stop_bits >= 2) ? UART_CFG_STOP_BITS_2 : UART_CFG_STOP_BITS_1;
    cfg.data_bits = UART_CFG_DATA_BITS_8;
    /* ENABLE and AUTO both map to real hardware RTS/CTS - there is no auto mode. */
    switch (_flow_control) {
    case FLOW_CONTROL_ENABLE:
    case FLOW_CONTROL_AUTO:
        cfg.flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS;
        break;
    default:
        cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
        break;
    }

    /* Virtual/USB UARTs legitimately reject runtime configuration, so a failure here
     * is not an error. */
    const int cfg_ret = uart_configure(_dev, &cfg);
    if (cfg_ret != 0 && cfg_ret != -ENOTSUP && cfg_ret != -ENOSYS) {
        _initialized = false;
        return;
    }

    /* quiesce IRQs before (re)sizing buffers — on an autobaud re-begin
       the ISR is still live from the previous begin */
    uart_irq_rx_disable(_dev);
    uart_irq_tx_disable(_dev);

    /* Enforce practical minimum buffer sizes, as AP_HAL_ChibiOS does
       with HAL_UART_MIN_RX/TX_SIZE: AP_SerialManager requests tiny
       per-protocol buffers (GPS TX is 16 B!) and relies on the HAL
       rounding up. Grow-only: never shrink an existing buffer. */
    const uint16_t req_rx = (rxSpace > DEFAULT_RX_BUF_SIZE) ? rxSpace : DEFAULT_RX_BUF_SIZE;
    if (_readbuf.get_size() < req_rx) {
        if (!_readbuf.set_size(req_rx)) {
            _initialized = false;
            return;
        }
    }

    const uint16_t req_tx = (txSpace > DEFAULT_TX_BUF_SIZE) ? txSpace : DEFAULT_TX_BUF_SIZE;
    if (_writebuf.get_size() < req_tx) {
        if (!_writebuf.set_size(req_tx)) {
            _initialized = false;
            return;
        }
    }

    /* Prefer the async (eDMA) path: one interrupt per RX buffer / TX chunk instead of
     * one per byte. */
    _use_async = false;
    if (uart_callback_set(_dev, _async_cb, this) == 0) {
        if (_rx_dma_buf[0] == nullptr) {
            _rx_dma_buf[0] = (uint8_t *)hal.util->malloc_type(RX_DMA_BUF_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
        }
        if (_rx_dma_buf[1] == nullptr) {
            _rx_dma_buf[1] = (uint8_t *)hal.util->malloc_type(RX_DMA_BUF_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
        }
        if (_tx_dma_buf == nullptr) {
            _tx_dma_buf = (uint8_t *)hal.util->malloc_type(TX_DMA_BUF_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
        }
        if (_rx_dma_buf[0] != nullptr && _rx_dma_buf[1] != nullptr && _tx_dma_buf != nullptr) {
            _use_async = true;
            _tx_dma_busy = false;
            _rx_need_restart = false;
            /* uart_rx_disable() before enable, same fix as _rx_timer_tick(). */
            uart_rx_disable(_dev);
            const int rc = uart_rx_enable(_dev, _rx_dma_buf[0], RX_DMA_BUF_SIZE, RX_DMA_TIMEOUT_US);
            if (rc != 0) {
                printk("UART%u: uart_rx_enable failed (%d)\n", _serial_num, rc);
                _rx_need_restart = true;   // retried from _rx_timer_tick
            }
        } else {
            printk("UART%u: DMA pool exhausted, falling back to IRQ path\n", _serial_num);
        }
    }

    /* _initialized must be true BEFORE the interrupt path is enabled, or the first
     * interrupt arrives against a half-built driver. */
    _initialized = true;

    if (!_use_async) {
        uart_irq_callback_user_data_set(_dev, _irq_handler, this);
        uart_irq_err_enable(_dev);
        uart_irq_rx_enable(_dev);
    }

#if defined(CONFIG_UART_MCUX_LPUART)
    /* OPTION_RXINV has no runtime path anywhere in Zephyr's mcux_lpuart driver, so
     * inversion is applied by writing the LPUART register directly. */
    const uint32_t lpuart_base = _ap_lpuart_base(_dev);
    if (lpuart_base != 0) {
        const bool inverted = (_last_options & OPTION_RXINV) != 0;
        volatile uint32_t *const stat = (volatile uint32_t *)(lpuart_base + 0x14u);
        volatile uint32_t *const ctrl = (volatile uint32_t *)(lpuart_base + 0x18u);
        if (inverted) {
            *stat |= (1u << 28);   /* LPUART_STAT_RXINV_MASK */
        } else {
            *stat &= ~(1u << 28);
        }
        if ((_last_options & OPTION_TXINV) != 0) {
            *ctrl |= (1u << 28);   /* LPUART_CTRL_TXINV_MASK */
        } else {
            *ctrl &= ~(1u << 28);
        }
    }
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart6), okay)
    if (_dev == DEVICE_DT_GET(DT_NODELABEL(lpuart6))) {
        const bool inverted = (_last_options & OPTION_RXINV) != 0;

        /* Pull direction must track inversion, not stay fixed: a floating inverted line
         * idles at the wrong level and frames garbage. */
        volatile uint32_t *const pad = (volatile uint32_t *)0x400e82f4u;
        *pad = (*pad & ~(0x3u << 2)) | ((inverted ? 0x2u : 0x1u) << 2);
    }
#endif  /* lpuart6 */
#endif  /* CONFIG_UART_MCUX_LPUART */

    if (!_uart_timer_registered) {
        /* ONE registration covers both TX and RX duty (_uart_timer_tick calls both). */
        hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &UARTDriver::_uart_timer_tick, void));
        _uart_timer_registered = true;
    }

    _owner_tid = k_current_get();
#endif

    _baudrate = baud;
    _initialized = true;
}

void UARTDriver::configure_parity(uint8_t v)
{
    _parity = v;
}

void UARTDriver::set_stop_bits(int n)
{
    _stop_bits = n;
}

bool UARTDriver::set_options(uint16_t options)
{
    _last_options = options;
#if defined(CONFIG_UART_MCUX_LPUART)
    if (_ap_lpuart_base(_dev) != 0) {
        /* RXINV/TXINV genuinely implemented for every LPUART, unlike the generic path. */
        if (_initialized) {
            _begin(_baudrate, 0, 0);
        }
        return (options & ~(OPTION_RXINV | OPTION_TXINV)) == 0;
    }
#endif
    return options == 0;   /* AP_HAL::UARTDriver's own default-stub convention */
}

void UARTDriver::set_flow_control(enum flow_control flow_control_setting)
{
    _flow_control = flow_control_setting;
#ifdef __ZEPHYR__
    /* AP_SerialManager calls this both before AND after begin(), so it must be safe
     * on an unopened port. */
    if (_initialized) {
        _begin(_baudrate, 0, 0);
    }
#endif
}

void UARTDriver::_end()
{
#ifdef __ZEPHYR__
    if (_dev != nullptr) {
        if (_use_async) {
            uart_rx_disable(_dev);
            uart_tx_abort(_dev);
        } else {
            uart_irq_rx_disable(_dev);
            uart_irq_tx_disable(_dev);
            uart_irq_err_disable(_dev);
            uart_irq_callback_user_data_set(_dev, nullptr, nullptr);
        }
    }
#endif

    _readbuf.clear();
    _initialized = false;
}

void UARTDriver::_flush()
{
#ifdef __ZEPHYR__
    if (_dev == nullptr || !_initialized) {
        return;
    }
    /* BOUNDED. Both loops previously spun with no timeout, which is a hang. */
    const int64_t flush_deadline = k_uptime_get() + 50;   // 50ms cap
    while (_writebuf.available() > 0 && k_uptime_get() < flush_deadline) {
        k_yield();
    }
    if (_use_async) {
        while (_tx_dma_busy && k_uptime_get() < flush_deadline) {
            k_yield();
        }
    } else {
        while (uart_irq_tx_complete(_dev) == 0 && k_uptime_get() < flush_deadline) {
            k_yield();
        }
    }
#endif
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    return _initialized && _writebuf.available() > 0;
}

uint32_t UARTDriver::txspace()
{
    return _initialized ? _writebuf.space() : 0;
}

uint32_t UARTDriver::_available()
{
#ifdef __ZEPHYR__
    if (_owner_tid != nullptr && _owner_tid != k_current_get()) {
        return 0;
    }
#endif
    return _initialized ? _readbuf.available() : 0;
}

ssize_t UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized) {
        return -1;
    }

#ifdef __ZEPHYR__
    if (_owner_tid != nullptr && _owner_tid != k_current_get()) {
        return -1;
    }
#endif

    const uint32_t n = _readbuf.read(buffer, count);
    if (n > 0) {
        _receive_timestamp_update();
    }
    return n;
}

size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

#ifdef __ZEPHYR__
    if (_dev == nullptr) {
        return 0;
    }
    /* Buffered, interrupt-driven TX: byte-at-a-time uart_poll_out here
       blocked the caller for the full wire time (5 live UARTs dragged
       the vehicle loop to ~128 Hz). The ISR drains _writebuf via
       uart_fifo_fill and disables the TX interrupt when empty. */
    WITH_SEMAPHORE(_write_mutex);
    const uint32_t n = _writebuf.write(buffer, size);
    _dbg_tx_queued += n;
    if (n > 0) {
        if (_use_async) {
            _tx_dma_kick();
        } else {
            uart_irq_tx_enable(_dev);
        }
    }
    return n;
#else
    /* Native (non-Zephyr) build — e.g. native_sim waf binary.
     * Route serial 0 (console) straight to host stdout. */
    if (_serial_num == 0) {
        fwrite(buffer, 1, size, stdout);
        fflush(stdout);
    }
#endif

    return size;
}

bool UARTDriver::_discard_input()
{
    if (!_initialized) {
        return false;
    }
    _readbuf.clear();
    return true;
}

void UARTDriver::_receive_timestamp_update()
{
    _receive_timestamp[_receive_timestamp_idx ^ 1U] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1U;
}

uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0) {
        const uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + _available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

#if HAL_UART_STATS_ENABLED
void UARTDriver::uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms)
{
    const uint32_t tx = stats.tx.update(0);
    const uint32_t rx = stats.rx.update(_available());
    const uint32_t dr = stats.rx_dropped.update(_rx_dropped);
    const uint32_t dt = (dt_ms == 0U) ? 1U : dt_ms;

    str.printf("SERIAL%u: baud=%lu tx=%luB/s rx=%luB/s drop=%lu\n",
               (unsigned)_serial_num,
               (unsigned long)_baudrate,
               (unsigned long)((tx * 1000U) / dt),
               (unsigned long)((rx * 1000U) / dt),
               (unsigned long)((dr * 1000U) / dt));
}
#endif

#ifdef __ZEPHYR__
/*
  Async (eDMA) event callback. Runs in ISR context. One RX event per
  filled buffer or idle timeout - not per byte - and one TX event per
  staged chunk. Mirrors the role _irq_handler plays for the FIFO path.
 */
void UARTDriver::_async_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    UARTDriver *self = static_cast<UARTDriver *>(user_data);
    if (self == nullptr || self->_dev != dev) {
        return;
    }

    switch (evt->type) {
    case UART_RX_RDY: {
        self->_dbg_rx_events++;
        self->_dbg_rx_bytes += evt->data.rx.len;
        const uint32_t written = self->_readbuf.write(
            evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);
        if (written < evt->data.rx.len) {
            self->_rx_dropped += evt->data.rx.len - written;
        }
        self->_receive_timestamp_update();
        break;
    }
    case UART_RX_BUF_REQUEST: {
        /* two rotating buffers: buf[0] was armed at _begin(), so the first
           request gets buf[1], then strict alternation */
        self->_rx_dma_next_is_1 = !self->_rx_dma_next_is_1;
        uint8_t *next = self->_rx_dma_next_is_1 ? self->_rx_dma_buf[1] : self->_rx_dma_buf[0];
        uart_rx_buf_rsp(dev, next, RX_DMA_BUF_SIZE);
        break;
    }
    case UART_RX_BUF_RELEASED:
        break;
    case UART_RX_STOPPED:
        /* line error (framing/overrun/parity). RX_DISABLED follows;
           the restart there covers recovery. */
        self->_rx_dropped++;
        break;
    case UART_RX_DISABLED:
        /* request a thread-context restart - uart_rx_enable() from ISR
           context is not guaranteed safe on every driver */
        self->_rx_need_restart = true;
        break;
    case UART_TX_DONE:
    case UART_TX_ABORTED:
        /* staged bytes were already consumed from the ring at kick time;
           just free the stage and send the next chunk if one is waiting */
        self->_dbg_tx_done += evt->data.tx.len;
        self->_tx_dma_busy = false;
        if (self->_writebuf.available() > 0) {
            self->_tx_dma_kick();
        }
        break;
    default:
        break;
    }
}

/*
  Stage up to one chunk from the TX ring into the __nocache buffer and
  start an async (eDMA) transmit. Safe from thread and ISR context; the
  irq_lock closes the ISR-vs-thread race on _tx_dma_busy.
 */
void UARTDriver::_tx_dma_kick()
{
    const unsigned int key = irq_lock();
    if (_tx_dma_busy || !_initialized) {
        irq_unlock(key);
        return;
    }
    const uint32_t n = _writebuf.peekbytes(_tx_dma_buf, TX_DMA_BUF_SIZE);
    if (n == 0) {
        irq_unlock(key);
        return;
    }
    _writebuf.advance(n);
    _tx_dma_busy = true;
    irq_unlock(key);

    if (uart_tx(_dev, _tx_dma_buf, n, SYS_FOREVER_US) != 0) {
        /* driver busy or error: bytes in the stage are lost this round;
           the fallback tick retries with fresh ring content */
        _tx_dma_busy = false;
        _dbg_tx_fail++;
    } else {
        _dbg_tx_dma += n;
    }
}

void UARTDriver::_irq_handler(const struct device *dev, void *user_data)
{
    /* Read-and-clear error flags (ORE/FE/NE/PE) before anything else. */
    (void)uart_err_check(dev);

    UARTDriver *self = static_cast<UARTDriver *>(user_data);
    if (self == nullptr || !self->_initialized || self->_dev != dev) {
        return;
    }

    while (uart_irq_update(dev), uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            self->_drain_rx_fifo();
        }

        if (uart_irq_tx_ready(dev)) {
            self->_fill_tx_fifo();
        }

    }
}

void UARTDriver::_fill_tx_fifo()
{
    uint8_t tmp[32];

    /* USB CDC with no host session (DTR low): hand NOTHING to the class. */
    uint32_t dtr = 0;
    if (uart_line_ctrl_get(_dev, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr == 0) {
        uart_irq_tx_disable(_dev);
        return;
    }

    while (true) {
        const uint32_t n = _writebuf.peekbytes(tmp, sizeof(tmp));
        if (n == 0) {
            uart_irq_tx_disable(_dev);
            /* close the race with a writer that pushed bytes and
               called tx_enable between our peek and the disable —
               otherwise those bytes strand until the next write */
            if (_writebuf.available() > 0) {
                uart_irq_tx_enable(_dev);
                continue;
            }
            return;
        }
        const int sent = uart_fifo_fill(_dev, tmp, n);
        if (sent <= 0) {
            return;
        }
        _writebuf.advance(sent);
        if ((uint32_t)sent < n) {
            /* FIFO full — IRQ fires again when it drains */
            return;
        }
    }
}

void UARTDriver::_tx_timer_tick()
{
    if (_dev == nullptr || !_initialized) {
        return;
    }
    /* USB CDC with no host session (DTR low): discard TX rather than let it block. */
    uint32_t dtr = 0;
    if (uart_line_ctrl_get(_dev, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr == 0) {
        _writebuf.clear();
        return;
    }
    if (_use_async) {
        /* forward-progress guarantee for the async path: a uart_tx() that
           failed in _tx_dma_kick() leaves the ring holding data with no
           TX_DONE event coming - this tick retries it */
        if (!_tx_dma_busy && _writebuf.available() > 0) {
            _tx_dma_kick();
        }
        return;
    }
    /* Lock out the TX-ready ISR while we manually pump the FIFO from
       thread context - _fill_tx_fifo() is not written to be reentrant
       against itself. */
    const unsigned int key = irq_lock();
    _fill_tx_fifo();
    irq_unlock(key);
}

void UARTDriver::_drain_rx_fifo()
{
    uint8_t tmp[32];

    /* BOUNDED. This used to be an unbounded while(true) that exited only when the
     * hardware cooperated - a hang on any stuck flag. */
    static const uint16_t MAX_DRAIN_CHUNKS = 1024;   /* 32KB - never hit normally */

    for (uint16_t chunk = 0; chunk < MAX_DRAIN_CHUNKS; chunk++) {
        const int n = uart_fifo_read(_dev, tmp, sizeof(tmp));
        if (n <= 0) {
            break;
        }
        const uint32_t written = _readbuf.write(tmp, n);
        if (written < (uint32_t)n) {
            _rx_dropped += ((uint32_t)n - written);
        }
    }

    _receive_timestamp_update();
}

void UARTDriver::_rx_timer_tick()
{
    if (_dev == nullptr || !_initialized) {
        return;
    }
    if (_use_async) {
        /* RX data arrives via UART_RX_RDY events; the only tick duty is
           restarting reception after an error/disable (thread context -
           uart_rx_enable() from the ISR is not universally safe) */
        if (_rx_need_restart) {
            _rx_need_restart = false;
            _rx_dma_next_is_1 = false;
            /* uart_rx_enable() fails -EBUSY whenever the mcux_lpuart driver's async state is
             * still armed, so the retry is required rather than defensive. */
            uart_rx_disable(_dev);
            if (uart_rx_enable(_dev, _rx_dma_buf[0], RX_DMA_BUF_SIZE, RX_DMA_TIMEOUT_US) != 0) {
                _rx_need_restart = true;   // try again next tick
            }
        }
        return;
    }
    /* Lock out the RX ISR while we manually pump the FIFO from
       thread context - matches _tx_timer_tick() for TX */
    const unsigned int key = irq_lock();
    _drain_rx_fifo();
    irq_unlock(key);
}
#endif


#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

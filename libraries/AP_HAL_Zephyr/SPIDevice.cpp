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
#include "chain_profile.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "SPIDevice.h"
#include "bouncebuffer.h"
#include "hwdef.h"

#include <new>
#include <string.h>

#ifdef __ZEPHYR__
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>   /* k_cycle_get_32 / sys_clock_hw_cycles_per_sec */
#endif

/*
  Per-transfer peripheral reset.

  AP_HAL_ChibiOS returns the SPI block to its power-on register values before
  every transaction. SPIDevice::acquire_bus() calls SPIBus::stop_peripheral()
  and then start_peripheral(); the stop leaves the ChibiOS driver in SPI_STOP,
  so the following spiStart() takes spi_lld_start()'s first-time branch, which
  issues rccResetSPIn() before rccEnableSPIn(). Every transaction therefore
  begins from a known state rather than from whatever the previous one left.

  AP_HAL_Zephyr had no equivalent: Zephyr's drivers configure a bus once at
  init and leave it configured, and this HAL additionally reuses the same
  spi_config address between transfers (see the _cfg comment in SPIDevice.h),
  so at a steady speed the block was neither reset nor even reconfigured.

  Two things are needed to match, and the second is not optional: after a
  reset the block sits at register defaults while the Zephyr driver still
  believes it is configured, so the config pointer must also change to force
  spi_context_configured() to miss and the driver to program it again.

  How the reset is issued is per-SoC:

    STM32   the RCC reset bit, written directly - the same bit
            rccResetSPIn() writes. Zephyr's reset API cannot be used here;
            see the comment on the STM32 branch below.
    RT11xx  LPSPI has its own software reset, CR[RST], which is the documented
            way to return that block to its reset state; there is no separate
            clock-controller reset line for it.

    ESP32   SYSTEM_PERIP_RST_EN0, the register behind ESP-IDF's
            periph_module_reset(). This board had no SPIDEV at all until the
            ICM-42688-P was added to its hwdef, so nothing exercised it
            before.
*/
#ifndef AP_SPI_RESET_PER_TRANSFER
#if defined(CONFIG_SOC_SERIES_STM32H7X) || defined(CONFIG_SOC_SERIES_IMXRT11XX) || \
    defined(CONFIG_SOC_SERIES_ESP32S3)
#define AP_SPI_RESET_PER_TRANSFER 1
#else
#define AP_SPI_RESET_PER_TRANSFER 0
#endif
#endif

#if AP_SPI_RESET_PER_TRANSFER && defined(__ZEPHYR__)

/* Read over SWD to confirm the path runs. g_spi_reset_miss counts transfers
   whose bus was not found in the table - that case does nothing, leaves SPI
   working perfectly, and would otherwise be invisible. */
volatile uint32_t g_spi_reset_count;
volatile uint32_t g_spi_reset_miss;

#if defined(CONFIG_SOC_SERIES_STM32H7X)

/* Zephyr's reset API would be the tidy route, but st,stm32h7-spi.yaml does not
   declare a "resets" property, so a devicetree entry is rejected outright:

     devicetree error: 'resets' appears in /soc/spi@40013000 ... but is not
     declared in 'properties:' in .../st,stm32h7-spi.yaml

   Rather than patch the Zephyr submodule's binding for this, write the RCC
   reset bit directly - which is exactly what ChibiOS's rccResetSPIn() does.

   RCC at 0x58024400; the reset registers sit 0x68 below their matching enable
   registers (APB1LRSTR 0x90 / APB1LENR 0xE8, APB2RSTR 0x98 / APB2ENR 0xF0,
   APB4RSTR 0x9C / APB4ENR 0xF4), and a peripheral's reset bit is at the same
   position as its enable bit. */
#define AP_H7_RCC_BASE      0x58024400UL
#define AP_H7_APB1LRSTR     0x90U
#define AP_H7_APB2RSTR      0x98U
#define AP_H7_APB4RSTR      0x9CU

/* Keyed on the peripheral base from the devicetree rather than on a nodelabel,
   so this is a property of the SoC and not of any one board. */
static void ap_spi_reset_by_base(uintptr_t base)
{
    uint32_t reg, bit;
    switch (base) {
    case 0x40013000UL: reg = AP_H7_APB2RSTR;  bit = 12; break;  /* SPI1 */
    case 0x40003800UL: reg = AP_H7_APB1LRSTR; bit = 14; break;  /* SPI2 */
    case 0x40003C00UL: reg = AP_H7_APB1LRSTR; bit = 15; break;  /* SPI3 */
    case 0x40013400UL: reg = AP_H7_APB2RSTR;  bit = 13; break;  /* SPI4 */
    case 0x40015000UL: reg = AP_H7_APB2RSTR;  bit = 20; break;  /* SPI5 */
    case 0x58001400UL: reg = AP_H7_APB4RSTR;  bit =  5; break;  /* SPI6 */
    default: return;
    }
    volatile uint32_t *rstr = (volatile uint32_t *)(AP_H7_RCC_BASE + reg);
    *rstr |= (1UL << bit);
    *rstr &= ~(1UL << bit);
}

#define AP_SPI_RESET_ENTRY(node_id) { DEVICE_DT_GET(node_id), DT_REG_ADDR(node_id) },

static const struct {
    const struct device *bus;
    uintptr_t base;
} ap_spi_bus_resets[] = {
    DT_FOREACH_STATUS_OKAY(st_stm32_spi, AP_SPI_RESET_ENTRY)
};

static void ap_spi_bus_reset(const struct device *bus)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(ap_spi_bus_resets); i++) {
        if (ap_spi_bus_resets[i].bus == bus) {
            ap_spi_reset_by_base(ap_spi_bus_resets[i].base);
            g_spi_reset_count++;
            return;
        }
    }
    g_spi_reset_miss++;
}

#elif defined(CONFIG_SOC_SERIES_IMXRT11XX)

/* LPSPI CR is at +0x10; RST is bit 1. Held asserted briefly, as the reference
   manual requires the bit to be written back to 0 to release the block. */
#define AP_LPSPI_CR_OFFSET 0x10U
#define AP_LPSPI_CR_RST    (1U << 1)

#define AP_SPI_RESET_ENTRY(node_id) { DEVICE_DT_GET(node_id), DT_REG_ADDR(node_id) },

static const struct {
    const struct device *bus;
    uintptr_t base;
} ap_spi_bus_resets[] = {
    DT_FOREACH_STATUS_OKAY(nxp_lpspi, AP_SPI_RESET_ENTRY)
};

static void ap_spi_bus_reset(const struct device *bus)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(ap_spi_bus_resets); i++) {
        if (ap_spi_bus_resets[i].bus == bus) {
            volatile uint32_t *cr =
                (volatile uint32_t *)(ap_spi_bus_resets[i].base + AP_LPSPI_CR_OFFSET);
            *cr |= AP_LPSPI_CR_RST;
            *cr &= ~AP_LPSPI_CR_RST;
            g_spi_reset_count++;
            return;
        }
    }
    g_spi_reset_miss++;
}

#elif defined(CONFIG_SOC_SERIES_ESP32S3)

/* SYSTEM_PERIP_RST_EN0 holds one reset bit per peripheral - the register
   behind ESP-IDF's periph_module_reset(). Written directly rather than through
   that call because it lives in an esp_private/ header, and because the STM32
   branch above already establishes the pattern.

   DR_REG_SYSTEM_BASE 0x600C0000, PERIP_RST_EN0 at +0x20 (reg_base.h,
   system_reg.h); SYSTEM_SPI2_RST is BIT(6). */
#define AP_S3_PERIP_RST_EN0  0x600C0020UL
#define AP_S3_SPI2_RST       (1UL << 6)

static void ap_spi_reset_by_base(uintptr_t base)
{
    uint32_t bit;
    switch (base) {
    case 0x60024000UL: bit = AP_S3_SPI2_RST; break;   /* spi2 = FSPI */
    default: return;
    }
    volatile uint32_t *rst = (volatile uint32_t *)AP_S3_PERIP_RST_EN0;
    *rst |= bit;
    *rst &= ~bit;
}

#define AP_SPI_RESET_ENTRY(node_id) { DEVICE_DT_GET(node_id), DT_REG_ADDR(node_id) },

static const struct {
    const struct device *bus;
    uintptr_t base;
} ap_spi_bus_resets[] = {
    DT_FOREACH_STATUS_OKAY(espressif_esp32_spi, AP_SPI_RESET_ENTRY)
};

static void ap_spi_bus_reset(const struct device *bus)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(ap_spi_bus_resets); i++) {
        if (ap_spi_bus_resets[i].bus == bus) {
            ap_spi_reset_by_base(ap_spi_bus_resets[i].base);
            g_spi_reset_count++;
            return;
        }
    }
    g_spi_reset_miss++;
}

#endif  /* SoC */

#else   /* !AP_SPI_RESET_PER_TRANSFER */
static inline void ap_spi_bus_reset(const struct device *) {}
#endif

/* Bounce buffers live on the DeviceBus and are SEPARATE for TX and RX, exactly as
 * ChibiOS does, so a shared buffer cannot alias a transfer against itself. */

extern const AP_HAL::HAL& hal;

using namespace Zephyr;

struct SPIDeviceDesc {
    const char *name;
    SPIDevice::DeviceId id;
    uint8_t bus;
    uint8_t cs;
    uint32_t lowspeed_hz;
    uint32_t highspeed_hz;
};

/* SPI devices are now generated from hwdef.dat via zephyr_hwdef.py.
   The HAL_SPI_DEVICES_LIST macro in hwdef.h contains initializers for this array. */
static const SPIDeviceDesc spi_devices[] = {
#ifdef HAL_SPI_DEVICES_LIST
    HAL_SPI_DEVICES_LIST
#endif
};

#ifdef __ZEPHYR__
/* spi_dt_spec declarations, one per hwdef.dat SPIDEV entry whose name matches
   a DTS nodelabel. Generated by zephyr_hwdef.py into hwdef.h. */
#ifdef HAL_SPI_DT_SPEC_DECLS
HAL_SPI_DT_SPEC_DECLS
#endif

/* Name-based lookup — required because DeviceId (IMU0/1/2/BARO/FRAM) is not
   unique enough (e.g. two BARO devices: ms5611 and ms5611_ext share a bus role
   but are different physical parts on different SPI buses/CS pins). */
static const struct spi_dt_spec *spec_for_name(const char *name)
{
#ifdef HAL_SPI_DT_SPEC_LOOKUP
    HAL_SPI_DT_SPEC_LOOKUP(name)
#endif
    return nullptr;
}
#endif

SPIDevice::SPIDevice(const char *name, DeviceId id, uint8_t bus, uint8_t cs_index,
                     uint32_t lowspeed_hz, uint32_t highspeed_hz) :
    _name(name),
    _id(id),
    _speed_hz(highspeed_hz),
    _lowspeed_hz(lowspeed_hz),
    _highspeed_hz(highspeed_hz)
{
    set_device_bus(bus);
    set_device_address(cs_index);
    _bus = DeviceBus::get_bus(bus, (uint8_t)AP_HAL::Device::BUS_TYPE_SPI);
#ifdef __ZEPHYR__
    _spec = spec_for_name(_name);
#endif
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    if (_bus == nullptr) {
        return nullptr;
    }
    return _bus->register_periodic_callback(period_usec, cb, this);
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                         uint32_t period_usec)
{
    if (_bus == nullptr) {
        return false;
    }
    return _bus->adjust_timer(h, period_usec);
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
    _speed_hz = (speed == AP_HAL::Device::SPEED_HIGH) ? _highspeed_hz : _lowspeed_hz;
    return true;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    /* BUSN, not BUS: the non-N form writes profile slot 0 for EVERY bus, so
       with two bus threads transferring concurrently the phase fraction cannot
       be attributed to a bus and dividing by the COMBINED transfer rate
       underestimates per-transfer time. bus_num() is AP_HAL::Device's own
       accessor for the bus this device was constructed on. */
    AP_PHASE_BUSN(bus_num(), AP_PHASE_SPI_XFER);
    AP_PROF_TICK(AP_PROF_XFER_COUNT);
    AP_PROF_ADD(AP_PROF_XFER_BYTES, send_len + recv_len);
#ifdef __ZEPHYR__
    if (!_bus_ready) {
        if (_spec == nullptr || !spi_is_ready_dt(_spec)) {
            return false;
        }
        _bus_ready = true;
    }

    /* Reuse the persistent per-device config so the pointer stays stable -
       see the _cfg comment in SPIDevice.h. Rewriting the whole struct each
       time is fine (contents are identical); what must NOT change is its
       ADDRESS, which is what the driver compares. */
    if (!_cfg_init) {
        _cfg[0] = _spec->config;
        _cfg[1] = _spec->config;
        _cfg[0].frequency = _speed_hz;
        _cfg[1].frequency = _speed_hz;
        _cfg_freq = _speed_hz;
        _cfg_init = true;
    } else if (_cfg_freq != _speed_hz || AP_SPI_RESET_PER_TRANSFER) {
        /* Flip slots so the POINTER changes, which is the only way this driver
           notices - see SPIDevice.h. Needed on a real speed change, and on
           EVERY transfer once the block is being reset underneath the driver:
           after ap_spi_bus_reset() the hardware is at its reset values while
           the driver still holds the old config pointer, so without this flip
           it would skip reconfiguration and drive an unprogrammed block. */
        _cfg_idx ^= 1;
        _cfg[_cfg_idx] = _spec->config;
        _cfg[_cfg_idx].frequency = _speed_hz;
        _cfg_freq = _speed_hz;
    }
    struct spi_config &cfg = _cfg[_cfg_idx];

    /* ArduPilot transfer() semantics, matching AP_HAL_ChibiOS. */
    /* ChibiOS pattern (bouncebuffer.h): substitute DMA-safe buffers if the
       caller's are not, and do the cache maintenance the Zephyr LPSPI DMA
       driver omits. Done BEFORE the spi_buf structs so they are built with the
       final pointers. */
    const uint8_t *tx_buf = send;
    uint8_t *rx_buf = recv;
    bool combined = false;
    struct spi_buf tx_bufs[2];
    struct spi_buf rx_bufs[2];
    struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 2 };
    struct spi_buf_set rx_set = { .buffers = rx_bufs, .count = 2 };

    if (send_len > 0 && recv_len > 0 && send != nullptr && recv != nullptr &&
        _bus != nullptr) {
        /* Run the send-then-recv register transaction as ONE full-duplex transfer: as
         * two chunks the DMA RX channel only served the second, returning zeros. */
        uint8_t *tx_full;
        uint8_t *rx_full;
        const uint16_t total = send_len + recv_len;
        if (_bus->xfer_scratch(total, tx_full, rx_full)) {
            memcpy(tx_full, send, send_len);
            memset(tx_full + send_len, 0, recv_len);
            tx_bufs[0] = { .buf = tx_full, .len = total };
            rx_bufs[0] = { .buf = rx_full, .len = total };
            tx_set.count = 1;
            rx_set.count = 1;
            rx_buf = rx_full;
            combined = true;
        }
    }
    if (!combined) {
        if (_bus == nullptr || !_bus->bouncebuffer_setup(tx_buf, send_len, rx_buf, recv_len)) {
            return false;
        }
        tx_bufs[0] = { .buf = const_cast<uint8_t *>(tx_buf), .len = send_len };
        tx_bufs[1] = { .buf = nullptr, .len = recv_len };
        rx_bufs[0] = { .buf = nullptr, .len = send_len };
        rx_bufs[1] = { .buf = rx_buf, .len = recv_len };
    }

    /* ChibiOS resets the block in acquire_bus(), i.e. immediately before the
       transaction rather than after the previous one, so the peripheral spends
       the idle time in its reset state. Same ordering here. The config-pointer
       flip above is what makes the driver reprogram it afterwards. */
    ap_spi_bus_reset(_spec->bus);

    const uint32_t _t0 = k_cycle_get_32();
    const bool ok = spi_transceive(_spec->bus, &cfg, &tx_set, &rx_set) == 0;
    ap_prof_xfer_record(k_cycle_get_32() - _t0, sys_clock_hw_cycles_per_sec());
    if (combined) {
        if (ok) {
            memcpy(recv, rx_buf + send_len, recv_len);
        }
    } else {
        _bus->bouncebuffer_finish(tx_buf, rx_buf, recv_len);
    }
    return ok;
#else
    (void)send;
    (void)send_len;
    (void)recv;
    (void)recv_len;
    return true;
#endif
}

/* In-place full duplex - the form AP_InertialSensor_Invensensev3::read_fifo()
 * uses, where the RX buffer is the TX buffer. */
bool SPIDevice::transfer_fullduplex(uint8_t *send_recv, uint32_t len)
{
    return transfer_fullduplex(send_recv, send_recv, len);
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                                    uint32_t len)
{
    /* INSTRUMENTATION GAP, fixed 2026-08-05: this path carried no phase marker, so
     * its time landed in AP_PHASE_OTHER. */
    AP_PHASE_BUSN(bus_num(), AP_PHASE_SPI_XFER);
    AP_PROF_TICK(AP_PROF_XFER_COUNT);
    AP_PROF_ADD(AP_PROF_XFER_BYTES, len);
#ifdef __ZEPHYR__
    if (send == nullptr || recv == nullptr || len == 0U) {
        return false;
    }
    if (!_bus_ready) {
        if (_spec == nullptr || !spi_is_ready_dt(_spec)) {
            return false;
        }
        _bus_ready = true;
    }

    /* Reuse the persistent per-device config so the pointer stays stable -
       see the _cfg comment in SPIDevice.h. Rewriting the whole struct each
       time is fine (contents are identical); what must NOT change is its
       ADDRESS, which is what the driver compares. */
    if (!_cfg_init) {
        _cfg[0] = _spec->config;
        _cfg[1] = _spec->config;
        _cfg[0].frequency = _speed_hz;
        _cfg[1].frequency = _speed_hz;
        _cfg_freq = _speed_hz;
        _cfg_init = true;
    } else if (_cfg_freq != _speed_hz) {
        /* real speed change: flip slots so the POINTER changes too, which is
           the only way this driver notices - see SPIDevice.h */
        _cfg_idx ^= 1;
        _cfg[_cfg_idx] = _spec->config;
        _cfg[_cfg_idx].frequency = _speed_hz;
        _cfg_freq = _speed_hz;
    }
    struct spi_config &cfg = _cfg[_cfg_idx];

    const uint8_t *tx_buf = send;
    uint8_t *rx_buf = recv;
    if (_bus == nullptr || !_bus->bouncebuffer_setup(tx_buf, len, rx_buf, len)) {
        return false;
    }

    const struct spi_buf tx = {
        .buf = const_cast<uint8_t *>(tx_buf),
        .len = len,
    };
    const struct spi_buf_set tx_set = {
        .buffers = &tx,
        .count = 1,
    };

    struct spi_buf rx = {
        .buf = rx_buf,
        .len = len,
    };
    const struct spi_buf_set rx_set = {
        .buffers = &rx,
        .count = 1,
    };

    /* ChibiOS resets the block in acquire_bus(), i.e. immediately before the
       transaction rather than after the previous one, so the peripheral spends
       the idle time in its reset state. Same ordering here. The config-pointer
       flip above is what makes the driver reprogram it afterwards. */
    ap_spi_bus_reset(_spec->bus);

    const uint32_t _t0 = k_cycle_get_32();
    const bool ok = spi_transceive(_spec->bus, &cfg, &tx_set, &rx_set) == 0;
    ap_prof_xfer_record(k_cycle_get_32() - _t0, sys_clock_hw_cycles_per_sec());
    _bus->bouncebuffer_finish(tx_buf, rx_buf, len);
    return ok;
#else
    (void)send;
    (void)recv;
    (void)len;
    return true;
#endif
}

AP_HAL::SPIDevice *SPIDeviceManager::get_device_ptr(const char *name)
{
    if (name == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(spi_devices); i++) {
        if (strcmp(spi_devices[i].name, name) == 0) {
            AP_HAL::SPIDevice *dev = NEW_NOTHROW SPIDevice(
                spi_devices[i].name,
                spi_devices[i].id,
                spi_devices[i].bus,
                spi_devices[i].cs,
                spi_devices[i].lowspeed_hz,
                spi_devices[i].highspeed_hz);
            return dev;
        }
    }

    return nullptr;
}

uint8_t SPIDeviceManager::get_count()
{
    return ARRAY_SIZE(spi_devices);
}

const char *SPIDeviceManager::get_device_name(uint8_t idx)
{
    if (idx >= ARRAY_SIZE(spi_devices)) {
        return nullptr;
    }
    return spi_devices[idx].name;
}


#ifdef HAL_SPI_CHECK_CLOCK_FREQ
/*
  Measure the real SPI clock on every configured bus, the same bring-up check
  AP_HAL_ChibiOS carries in its own SPIDevice.cpp.

  The number in hwdef.dat is a ceiling, not a setting. What the silicon
  produces is the kernel clock feeding the peripheral divided by whatever
  divisor the driver picked to stay under that ceiling, so the two can differ
  by a lot and nothing reports it. On CubeOrangeZephyr the SPI buses do not even
  share a source: spi1 and spi2 take PLL1_Q while the board devicetree points
  spi4 at PLL3_Q, so a wrong PLL leaves one bus fast and another slow.

  Measured, not computed: 1024 bytes are clocked out and timed, and the answer
  is bits over seconds. That counts the driver's per-transfer overhead as well
  as the wire, so read it as a floor on the period rather than an exact SCK.
 */
/* Results also land here so they can be read over SWD. The console is USB CDC,
   which is not enumerated when this runs and drops what it cannot send, so the
   printk below is best-effort and these globals are the reliable copy. */
volatile uint32_t g_spi_clk_bus[8];
volatile uint32_t g_spi_clk_req[8];
volatile uint32_t g_spi_clk_meas[8];
volatile uint32_t g_spi_clk_n;

void SPIDevice::test_clock_freq(void)
{
#ifdef __ZEPHYR__
    g_spi_clk_n = 0;
    /* The console is USB CDC on every Zephyr board and the report is worthless
       if it lands in the ring buffer before the host has enumerated. */
    printk("Waiting for USB\n");
    for (uint8_t i=0; i<3; i++) {
        hal.scheduler->delay(1000);
        printk("Waiting %u\n", (unsigned)AP_HAL::millis());
    }

    const uint16_t len = 1024;
    uint8_t *buf1 = (uint8_t *)hal.util->malloc_type(len, AP_HAL::Util::MEM_DMA_SAFE);
    uint8_t *buf2 = (uint8_t *)hal.util->malloc_type(len, AP_HAL::Util::MEM_DMA_SAFE);
    if (buf1 == nullptr || buf2 == nullptr) {
        printk("SPI clock test: no DMA-safe buffer\n");
        hal.util->free_type(buf1, len, AP_HAL::Util::MEM_DMA_SAFE);
        hal.util->free_type(buf2, len, AP_HAL::Util::MEM_DMA_SAFE);
        return;
    }

    uint32_t done = 0;   // one bit per bus already measured
    for (uint8_t i=0; i<ARRAY_SIZE(spi_devices); i++) {
        const uint8_t bus = spi_devices[i].bus;
        if (bus >= 32 || (done & (1U<<bus)) != 0) {
            continue;
        }
        const struct spi_dt_spec *spec = spec_for_name(spi_devices[i].name);
        if (spec == nullptr || !device_is_ready(spec->bus)) {
            continue;
        }
        done |= 1U<<bus;

        /* Built here rather than copied from spec->config, which carries this
           device's chip select: the bytes must go out with nothing selected so
           no sensor mistakes 1024 zero bytes for register writes.
           Zero the WHOLE cs struct. Clearing only cs.gpio.port is not enough
           and is not safe - spi_cs_is_gpio() tests the separate cs.cs_is_gpio
           flag, so the driver still takes the CS path and dereferences the null
           port, which faults. The API doc claiming a NULL port "fully inhibits
           CS control" does not match this driver. */
        struct spi_config cfg = spec->config;
        cfg.cs = (struct spi_cs_control){};
        for (uint8_t pass=0; pass<2; pass++) {
            /* Two speeds: the low one keeps the divisor coarse enough to read
               clearly, the high one is what the sensors actually run at. */
            cfg.frequency = (pass == 0) ? 2000000UL : spi_devices[i].highspeed_hz;

            const struct spi_buf tx_buf = { .buf = buf1, .len = len };
            const struct spi_buf rx_buf = { .buf = buf2, .len = len };
            const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
            const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

            const uint32_t t0 = AP_HAL::micros();
            const int rc = spi_transceive(spec->bus, &cfg, &tx, &rx);
            const uint32_t t1 = AP_HAL::micros();
            if (rc != 0) {
                printk("SPI[%u] req=%u FAIL %d\n",
                       unsigned(bus), unsigned(cfg.frequency), rc);
                continue;
            }
            const uint32_t dt = t1 - t0;
            if (dt == 0) {
                printk("SPI[%u] req=%u measured too fast to time\n",
                       unsigned(bus), unsigned(cfg.frequency));
                continue;
            }
            const uint32_t measured = (uint32_t)(1000000ULL * len * 8ULL / (uint64_t)dt);
            if (g_spi_clk_n < 8) {
                g_spi_clk_bus[g_spi_clk_n] = bus;
                g_spi_clk_req[g_spi_clk_n] = cfg.frequency;
                g_spi_clk_meas[g_spi_clk_n] = measured;
                g_spi_clk_n++;
            }
            printk("SPI[%u] req=%u measured=%u\n",
                   unsigned(bus), unsigned(cfg.frequency), unsigned(measured));
        }
    }
    hal.util->free_type(buf1, len, AP_HAL::Util::MEM_DMA_SAFE);
    hal.util->free_type(buf2, len, AP_HAL::Util::MEM_DMA_SAFE);
#endif  // __ZEPHYR__
}
#endif  // HAL_SPI_CHECK_CLOCK_FREQ

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Notify_config.h"

#if AP_NOTIFY_ZEPHYR_LED_STRIP_ENABLED

/* Zephyr headers before any AP header that may pull AP_Math: AP_Math.h
   does `#undef MAX`, which breaks Zephyr macros expanding MAX at their
   use site (same rule as WiFiDriver.cpp / Scheduler.cpp). */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/sys_io.h>

#include "ZephyrLEDStrip.h"
#include <AP_HAL/AP_HAL.h>

/*
  Bit-banged SK6805/WS2812 emitter (2026-08-16). The ws2812-spi route
  (GPSPI2 as waveform shifter) never produced a single decoded frame on
  an ESP32-C6 board despite spi_write() success, a correct pinmux and an
  exact mirror of zephyr's in-tree esp32c3_rust reference - the pixel sat
  at its unpainted full-power white throughout. Direct GPIO with
  cycle-counted timing removes the SPI driver, the GPIO matrix signal
  routing and the serializer from the equation entirely; a notify LED
  updates at a few Hz, so ~30 us of irq-locked banging per change is
  negligible.
 */

#define STRIP_NODE DT_ALIAS(led_strip)

/* ESP32-C6 GPIO write-1-to-set / write-1-to-clear registers; base and
   OUT offset previously verified on this board via JTAG. A direct store
   is a single bus write - the gpio driver API costs too much for the
   300 ns T0H budget. */
#define C6_GPIO_OUT_W1TS 0x60091008UL
#define C6_GPIO_OUT_W1TC 0x6009100CUL

/* Espressif RISC-V cores have NO mcycle CSR (csrr mcycle = illegal
   instruction, measured: mcause 2, mtval 0xb00022f3). The vendor
   performance counter is CSR 0x7e2 (PCCR), enabled via 0x7e0/0x7e1
   (PCER/PCMR) - what esp_cpu_get_cycle_count() reads.

   These CSRs belong to the WiFi blob too: permanently rewriting them at
   init correlated exactly with esp_wifi stopping its own softAP seconds
   after every start (WIFI_EVENT_AP_STOP with no disable request,
   2026-08-16 - the AP worked in every build before the bit-bang landed).
   So the counter is enabled ONLY inside the irq-locked frame and the
   previous configuration is restored before interrupts return: on this
   single core the blob can never observe the change. */
static inline uint32_t rd_mcycle(void)
{
    uint32_t v;
    __asm__ volatile("csrr %0, 0x7e2" : "=r"(v));
    return v;
}

/* SK6805 timings at 160 MHz (6.25 ns/cycle): T0H 300 ns, T0L 900 ns,
   T1H 600 ns, T1L 600 ns. The pixel latches on >80 us of idle low.

   Two hard-won rules (bench, 2026-08-16):
   - ABSOLUTE deadline chain, not per-phase relative waits: store latency
     and loop-exit overhead added onto every phase with relative timing,
     stretching zeros into ones (value 3 lit as full scale).
   - Warm-up pass with mask 0 first: identical code path, no output -
     the real frame then runs from a hot icache. With a cold cache the
     XIP fetch stalls corrupted mid-frame timing (red decoded, the next
     frame latched garbage and stuck). */
static void sk6805_bang24(uint32_t pin_mask, uint32_t grb)
{
    const uint32_t T0H = 48, T0L = 144, T1H = 96, T1L = 96;
    for (int pass = 0; pass < 2; pass++) {
        const uint32_t mask = pass ? pin_mask : 0;
        const unsigned int key = irq_lock();
        uint32_t saved_pcer, saved_pcmr;
        __asm__ volatile("csrr %0, 0x7e0" : "=r"(saved_pcer));
        __asm__ volatile("csrr %0, 0x7e1" : "=r"(saved_pcmr));
        __asm__ volatile("csrwi 0x7e0, 1\n\tcsrwi 0x7e1, 1");
        uint32_t deadline = rd_mcycle();
        for (int i = 23; i >= 0; i--) {
            const bool one = (grb >> i) & 1U;
            sys_write32(mask, C6_GPIO_OUT_W1TS);
            deadline += one ? T1H : T0H;
            while ((int32_t)(rd_mcycle() - deadline) < 0) {
            }
            sys_write32(mask, C6_GPIO_OUT_W1TC);
            deadline += one ? T1L : T0L;
            while ((int32_t)(rd_mcycle() - deadline) < 0) {
            }
        }
        __asm__ volatile("csrw 0x7e0, %0" :: "r"(saved_pcer));
        __asm__ volatile("csrw 0x7e1, %0" :: "r"(saved_pcmr));
        irq_unlock(key);
    }
    k_busy_wait(120);   // latch: >80 us idle low
}

ZephyrLEDStrip::ZephyrLEDStrip() :
    RGBLed(0x00, 0x06, 0x03, 0x01)   // off, high, medium, low brightness
    // The SK6805 is searing at desk distance: 100%, 50% and 12.5% were
    // all rejected in turn; maintainer then asked for another 90% cut on
    // top (2026-08-16), landing boot (medium, NTF_LED_BRIGHT 2) at ~1.2%
    // of full scale. The param still picks between these levels at
    // runtime, so further taste-tuning needs no rebuild.
{
}

bool ZephyrLEDStrip::init(void)
{
    const struct device *gpio = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio)) {
        printk("ZephyrLEDStrip: gpio0 NOT READY\n");
        return false;
    }
#if defined(AP_ZEPHYR_LED_DATA_PIN)
    /* claim the data pad as plain GPIO (also un-routes any leftover
       GPIO-matrix signal), idle low */
    gpio_pin_configure(gpio, AP_ZEPHYR_LED_DATA_PIN, GPIO_OUTPUT_INACTIVE);
    strip = (const struct device *)1;   // marks init done for hw_set_rgb
#else
#error "AP_ZEPHYR_LED_DATA_PIN required for the bit-banged LED strip"
#endif
    /* paint black FIRST (warms the icache for the timing loop too),
       and only then enable the LED power rail: the SK6805 shows raw
       full-white from power-on until a frame is decoded */
    hw_set_rgb(0, 0, 0);
    hw_set_rgb(0, 0, 0);
#if defined(AP_ZEPHYR_LED_SLP_PIN)
    gpio_pin_configure(gpio, AP_ZEPHYR_LED_SLP_PIN, GPIO_OUTPUT_ACTIVE);
#endif
    printk("ZephyrLEDStrip: bit-bang init done (data IO%u)\n",
           (unsigned)AP_ZEPHYR_LED_DATA_PIN);
    return hw_set_rgb(0, 0, 0);
}

bool ZephyrLEDStrip::hw_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (strip == nullptr) {
        return false;
    }
    const uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    sk6805_bang24(1UL << AP_ZEPHYR_LED_DATA_PIN, grb);
    static uint8_t logged;
    if (logged < 3) {
        logged++;
        printk("ZephyrLEDStrip: bang rgb(%u,%u,%u)\n", r, g, b);
    }
    return true;
}

#endif  // AP_NOTIFY_ZEPHYR_LED_STRIP_ENABLED

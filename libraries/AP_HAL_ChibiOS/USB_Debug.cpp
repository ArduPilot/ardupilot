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
 * with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * This GDB monitor was written for ArduPilot. Its USB OTG reset, FIFO and
 * endpoint sequences adapt the ChibiOS STM32 OTGv1 driver, Copyright (C)
 * 2006-2026 Giovanni Di Sirio, licensed under Apache-2.0:
 * https://github.com/ArduPilot/ChibiOS/blob/9aebaf4a40442277d01bd9dd38c13713fc45ef41/os/hal/ports/STM32/LLD/OTGv1/hal_usb_lld.c
 *
 * STMicroelectronics' STM32H7 USB drivers (Copyright 2017 STMicroelectronics,
 * BSD-3-Clause) were consulted for reset and endpoint handling. Adam Green's
 * CrashCatcher (Apache-2.0), already credited in hwdef/common, was consulted
 * for exception frames and stack switching. No third-party GDB stub was imported.
 * https://github.com/STMicroelectronics/stm32h7xx-hal-driver/tree/7e541d92019e18f98d211fc4ab9197ec8e8105f6
 */

#include <AP_HAL/AP_HAL.h>
#include "USB_Debug.h"

#if AP_USB_DEBUG_ENABLED

#include <AP_Common/AP_Common.h>
#include "hal.h"
#include "UARTDriver.h"
#include "Scheduler.h"
#include "hwdef/common/usbcfg.h"
#include "hwdef/common/stm32_util.h"
#include "hwdef/common/watchdog.h"
#include <string.h>

#if !defined(STM32H7) || !HAL_HAVE_DUAL_USB_CDC || !STM32_USB_USE_OTG1
#error USB debug requires STM32H7 OTG1 dual CDC
#endif

// Extra hwdef files can enable the monitor without the waf option checks.
#if EXT_FLASH_SIZE_MB > 0
#error USB debug does not support external-flash boards
#endif

#if defined(HAL_GPIO_PIN_EXT_WDOG)
#error USB debug does not support external-watchdog boards
#endif

#if !CH_CFG_USE_REGISTRY
#error USB debug requires the ChibiOS thread registry
#endif

#pragma GCC target ("general-regs-only")

extern const AP_HAL::HAL& hal;

namespace {
static constexpr uint32_t smaller(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}

constexpr IRQn_Type USB_IRQ = IRQn_Type(STM32_OTG1_NUMBER);
constexpr uint8_t DEBUG_EP = 4;
constexpr uint8_t GCS_EP = 2;
constexpr uint32_t PACKET_SIZE = 512;
constexpr char HEX[] = "0123456789abcdef";
// UART workers and the priority-zero USB handler share these bounded queues.
// Mask IRQs only for one USB packet, including reset of the queue indices.
class GCSBuffer {
public:
    uint32_t available() const { return uint16_t(head-tail); }
    uint32_t space() const { return sizeof(data)-available(); }
    void clear() { head = tail = 0; }
    uint32_t write(const uint8_t *src, uint32_t size) {
        size = smaller(smaller(size, 64), space());
        for (uint32_t i = 0; i < size; i++) {
            data[(head+i) % sizeof(data)] = src[i];
        }
        head += size;
        return size;
    }
    uint32_t read(uint8_t *dest, uint32_t size) {
        size = smaller(smaller(size, 64), available());
        for (uint32_t i = 0; i < size; i++) {
            dest[i] = data[(tail+i) % sizeof(data)];
        }
        tail += size;
        return size;
    }
private:
    uint8_t data[1024];
    volatile uint16_t head, tail;
};

// All mutable monitor state lives here. Keep this zero-initialized in BSS;
// initialise nonzero defaults on attachment, rather than storing buffers in flash.
struct USBDebugState {
    // Exception entry switches to a private stack before entering C++.
    alignas(8) uint8_t stack[4096];
    // r4-r11, original MSP/PSP, EXC_RETURN, PRIMASK, BASEPRI, CONTROL.
    uint32_t context[14];
    volatile bool probing;
    uint32_t fp_registers[33];
    uint32_t fp_available;
    volatile uint32_t fault_enabled;
    uint32_t registers[50];

    // Attachment and exception-to-main-loop handoff.
    volatile bool active;
    volatile bool initial_stop;
    bool interrupt_requested;
    bool stopped;
    bool resume_requested;
    bool detach_requested;
    bool session_active;
    uint8_t matched;
    uint32_t saved_demcr;
    uint8_t saved_debug_priority, saved_usb_priority;
    uint32_t saved_vtor;
    uint32_t last_cycle;
    uint64_t startup_cycles;

    // Comparator ownership and the temporary step used to leave a breakpoint.
    uint32_t breakpoint_address[8];
    uint32_t breakpoint_count;
    int step_over, step_ram;
    struct RAMBreakpoint {
        uint32_t address;
        uint16_t original;
    } ram_breakpoints[8];
    bool step_requested;
    bool step_report;
    uint32_t step_basepri;
    uint8_t stop_signal;
    bool software_breakpoint;
    struct Watchpoint {
        uint32_t address;
        uint8_t type, mask;
    } watchpoints[4];
    uint8_t watchpoint_count, watch_type;
    uint32_t watch_address;
    uint32_t fault_exception, fault_cfsr, fault_hfsr, fault_mmfar, fault_bfar;

    // A snapshot is valid only until the stopped CPU resumes.
    uint32_t stopped_thread, selected_thread;
    uint32_t selected_registers[50];
    uint32_t register_address[50];
    uint64_t register_mask;
    uint32_t thread_ids[64];
    uint8_t thread_count, thread_cursor;

    // USB reset advances the generation, cancelling any partially parsed packet.
    stm32_otg_t *otg;
    volatile bool configured;
    uint32_t usb_generation;
    uint32_t configuration;
    uint32_t saved_gusbcfg, saved_gccfg;
    const uint8_t *control_data;
    uint32_t control_left;
    bool control_zlp;
    bool control_out_status;
    uint8_t setup[8];
    uint8_t control_reply[8];
    uint8_t line_coding[2][7];
    uint8_t rx[64];
    uint32_t rx_len, rx_pos;
    bool out_complete;
    char packet[PACKET_SIZE + 1];
    char reply[PACKET_SIZE + 1];
    uint8_t tx_frame[PACKET_SIZE+4];

    // The first CDC keeps GCS traffic queued while application threads are stopped.
    bool gcs_out_armed, gcs_rx_received, gcs_tx_busy, gcs_tx_zlp;
    GCSBuffer gcs_rx, gcs_tx;

    // Only the USB vector changes during attachment. Copy the full table so
    // ChibiOS keeps its normal ISR without a submodule change or kernel calls
    // from our priority-zero handler. H7's table fits in one aligned 1 KB block.
    alignas(1024) uint32_t vectors[16 + CORTEX_NUM_VECTORS];
};

static_assert(sizeof(USBDebugState::vectors) <= 1024, "USB debug vector table alignment");

// Naked entry points use basic assembly, so pin and check their byte offsets.
static_assert(offsetof(USBDebugState, context) == 4096, "debug context offset");
static_assert(offsetof(USBDebugState, probing) == 4152, "debug probe offset");
static_assert(offsetof(USBDebugState, fp_registers) == 4156, "debug FP offset");
static_assert(offsetof(USBDebugState, fp_available) == 4288, "debug FP availability offset");
static_assert(offsetof(USBDebugState, fault_enabled) == 4292, "debug fault gate offset");
static USBDebugState usb_debug asm("usb_debug") __attribute__((used));
}

extern "C" {
void usb_debug_exception();
void AP_HardFault_Handler();
void HardFault_Handler();
void USB_Debug_Handler();
void MemManage_Handler() __attribute__((nothrow, alias("HardFault_Handler")));
void BusFault_Handler() __attribute__((nothrow, alias("HardFault_Handler")));
void UsageFault_Handler() __attribute__((nothrow, alias("HardFault_Handler")));
void DebugMon_Handler() __attribute__((nothrow, alias("USB_Debug_Handler")));
int usb_debug_read_byte(uint32_t address);
int usb_debug_write_byte(uint32_t address, uint8_t value);

// The USB IRQ services the raw controller while running; DebugMonitor handles breakpoints
// and single steps. Neither may call ChibiOS: priority zero can preempt its locks.
// Both exceptions run at the same priority, so polling state is never reentered.
// Hardware has already saved r0-r3/r12/LR/PC/xPSR (and reserved any FP frame).
__attribute__((naked, noinline)) void USB_Debug_Handler()
{
    asm volatile(
        "ldr r0,=(usb_debug+4096)\n"
        "stmia r0!,{r4-r11}\n"
        "mrs r1,msp\n"
        "str r1,[r0,#0]\n"
        "mrs r1,psp\n"
        "str r1,[r0,#4]\n"
        "str lr,[r0,#8]\n"
        "mrs r1,primask\n"
        "cpsid i\n"
        "str r1,[r0,#12]\n"
        "mrs r1,basepri\n"
        "str r1,[r0,#16]\n"
        "mrs r1,control\n"
        "str r1,[r0,#20]\n"
        "ldr r0,=(usb_debug+4292)\n"
        "movs r1,#0\n"
        "str r1,[r0]\n"
        // VMRS completes lazy stacking before we inspect the interrupted FP frame.
        "ldr r2,=0xe000ed88\n"
        "ldr r2,[r2]\n"
        "and r2,r2,#0xf00000\n"
        "cmp r2,#0xf00000\n"
        "ite eq\n"
        "moveq r2,#1\n"
        "movne r2,#0\n"
        "ldr r0,=(usb_debug+4288)\n"
        "str r2,[r0]\n"
        "cbz r2,1f\n"
        "vmrs r1,fpscr\n"
        "ldr r0,=(usb_debug+4156)\n"
        "vstmia r0!,{s0-s31}\n"
        "str r1,[r0]\n"
        "1: ldr sp,=(usb_debug+4096)\n"
        "bl usb_debug_exception\n"
        "ldr r0,=(usb_debug+4288)\n"
        "ldr r1,[r0]\n"
        "cbz r1,2f\n"
        "ldr r0,=(usb_debug+4156)\n"
        "vldmia r0!,{s0-s31}\n"
        "ldr r1,[r0]\n"
        "vmsr fpscr,r1\n"
        "2:\n"
        "ldr r0,=(usb_debug+4096)\n"
        "ldmia r0!,{r4-r11}\n"
        "ldr r1,[r0,#0]\n"
        "msr msp,r1\n"
        "ldr r1,[r0,#4]\n"
        "msr psp,r1\n"
        "ldr lr,[r0,#8]\n"
        "ldr r1,[r0,#16]\n"
        "msr basepri,r1\n"
        "ldr r1,[r0,#20]\n"
        "msr control,r1\n"
        "isb\n"
        "ldr r1,[r0,#12]\n"
        "msr primask,r1\n"
        "bx lr\n");
}

__attribute__((naked, noinline)) int usb_debug_read_byte(uint32_t address)
{
    asm volatile(
        ".global usb_debug_load\n"
        "usb_debug_load: ldrb r0,[r0]\n"
        "bx lr\n"
        ".global usb_debug_read_fault\n"
        "usb_debug_read_fault: mov.w r0,#-1\n"
        "bx lr\n");
}

// Drain the store before leaving the guarded sequence: on Cortex-M7 a buffered
// write fault can arrive after STRB. The fault exit also drains pending writes.
__attribute__((naked, noinline)) int usb_debug_write_byte(uint32_t address, uint8_t value)
{
    asm volatile(
        ".global usb_debug_store\n"
        "usb_debug_store: strb r1,[r0]\n"
        "dsb\n"
        "movs r0,#0\n"
        ".global usb_debug_store_end\n"
        "usb_debug_store_end: bx lr\n"
        ".global usb_debug_write_fault\n"
        "usb_debug_write_fault: dsb\n"
        "mov.w r0,#-1\n"
        "bx lr\n");
}

// PRIMASK escalates faults from guarded accesses to HardFault. Recover ONLY
// a fault at the load or within the bounded store sequence. Application faults
// enter the monitor only while attached and outside monitor code; otherwise
// retain the existing crash dump / watchdog handling.
__attribute__((naked)) void HardFault_Handler()
{
    asm volatile(
        "ldr r0,=(usb_debug+4152)\n"
        "ldrb r0,[r0]\n"
        "cbz r0,1f\n"
        "tst lr,#4\n"
        "ite eq\n"
        "mrseq r0,msp\n"
        "mrsne r0,psp\n"
        "ldr r1,[r0,#24]\n"
        "ldr r2,=usb_debug_load\n"
        "cmp r1,r2\n"
        "bne 2f\n"
        "ldr r1,=usb_debug_read_fault\n"
        "b 3f\n"
        "2: ldr r2,=usb_debug_store\n"
        "cmp r1,r2\n"
        "blo 1f\n"
        "ldr r2,=usb_debug_store_end\n"
        "cmp r1,r2\n"
        "bhi 1f\n"
        "ldr r1,=usb_debug_write_fault\n"
        "3:\n"
        "str r1,[r0,#24]\n"
        "bx lr\n"
        "1: ldr r0,=(usb_debug+4292)\n"
        "ldr r0,[r0]\n"
        "cbz r0,4f\n"
        // Stacking/unstacking faults have no trustworthy exception frame.
        "ldr r0,=0xe000ed28\n"
        "ldr r0,[r0]\n"
        "ldr r1,=0x3838\n"
        "tst r0,r1\n"
        "bne 4f\n"
        "b USB_Debug_Handler\n"
        "4: b AP_HardFault_Handler\n");
}
}

namespace {
// Stream the repetitive register description into the requested RSP slice.
// Keep only one register element on the stack, rather than the whole XML in flash.
static void target_description(uint32_t offset, uint32_t length)
{
    length = smaller(length, PACKET_SIZE-1);
    char *out = usb_debug.reply+1;
    bool more = false;
    const auto append = [&](const char *text) {
        uint32_t size = strlen(text);
        const uint32_t skip = smaller(offset, size);
        offset -= skip;
        text += skip;
        size -= skip;
        const uint32_t count = smaller(length, size);
        memcpy(out, text, count);
        out += count;
        length -= count;
        more |= size > count;
    };
    append("<?xml version=\"1.0\"?><target><architecture>arm</architecture>"
           "<feature name=\"org.gnu.gdb.arm.m-profile\">");
    char reg[64];
    for (uint8_t i = 0; i < 13; i++) {
        hal.util->snprintf(reg, sizeof(reg), "<reg name=\"r%u\" bitsize=\"32\"/>", unsigned(i));
        append(reg);
    }
    append("<reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>"
           "<reg name=\"lr\" bitsize=\"32\"/>"
           "<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>"
           "<reg name=\"xpsr\" bitsize=\"32\" regnum=\"25\"/>"
           "</feature><feature name=\"org.gnu.gdb.arm.vfp\">");
    for (uint8_t i = 0; i < 16; i++) {
        hal.util->snprintf(reg, sizeof(reg),
                           "<reg name=\"d%u\" bitsize=\"64\" type=\"ieee_double\" regnum=\"%u\"/>",
                           unsigned(i), unsigned(26+i));
        append(reg);
    }
    append("<reg name=\"fpscr\" bitsize=\"32\" regnum=\"42\"/></feature></target>");
    *out = 0;
    usb_debug.reply[0] = more ? 'm' : 'l';
    if (offset != 0) {
        strcpy(usb_debug.reply, "E01");
    }
}

static void delay_cycles(uint32_t cycles)
{
    const uint32_t start = DWT->CYCCNT;
    while (uint32_t(DWT->CYCCNT - start) < cycles) {
        stm32_watchdog_pat();
    }
}

// A broken controller must not leave the watchdog fed indefinitely during init.
static void wait_bits(volatile uint32_t &reg, uint32_t mask, uint32_t value)
{
    const uint32_t start = DWT->CYCCNT;
    while ((reg & mask) != value) {
        if (uint32_t(DWT->CYCCNT - start) > STM32_SYS_D1CPRE_CK/10U) {
            NVIC_SystemReset();
        }
        stm32_watchdog_pat();
    }
}

static void arm_out(uint8_t ep)
{
    usb_debug.otg->oe[ep].DOEPTSIZ = (ep == 0 ? (3U << 29) : 0) | (1U << 19) | 64;
    usb_debug.otg->oe[ep].DOEPCTL |= DOEPCTL_EPENA | DOEPCTL_CNAK;
}

static void transmit(uint8_t ep, const uint8_t *data, uint32_t size)
{
    usb_debug.otg->ie[ep].DIEPINT = 0xffffffff;
    usb_debug.otg->ie[ep].DIEPTSIZ = (1U << 19) | size;
    usb_debug.otg->ie[ep].DIEPCTL |= DIEPCTL_EPENA | DIEPCTL_CNAK;
    for (uint32_t i = 0; i < size; i += 4) {
        uint32_t word = 0;
        for (uint32_t j = 0; j < 4 && i + j < size; j++) {
            word |= uint32_t(data[i + j]) << (8*j);
        }
        usb_debug.otg->FIFO[ep][0] = word;
    }
}

static void control_next()
{
    const uint32_t n = smaller(usb_debug.control_left, 64U);
    if (n != 0 || usb_debug.control_zlp) {
        transmit(0, usb_debug.control_data, n);
        usb_debug.control_data += n;
        usb_debug.control_left -= n;
        if (n == 0) {
            usb_debug.control_zlp = false;
        }
    } else if (usb_debug.setup[0] & 0x80U) {
        // A control read ends with an OUT status packet. Arm it only after
        // the last IN packet completes, without restarting an active receive.
        arm_out(0);
    }
}

static void control_send(const uint8_t *data, uint32_t size, uint32_t requested)
{
    usb_debug.control_data = data;
    usb_debug.control_left = smaller(size, requested);
    usb_debug.control_zlp = usb_debug.control_left == 0 || (size < requested && (size % 64) == 0);
    control_next();
}

static void reset_endpoints()
{
    ++usb_debug.usb_generation;
    usb_debug.configured = false;
    usb_debug.configuration = 0;
    usb_debug.rx_len = usb_debug.rx_pos = 0;
    usb_debug.control_left = 0;
    usb_debug.control_zlp = false;
    usb_debug.control_out_status = false;
    usb_debug.out_complete = false;
    usb_debug.gcs_rx.clear();
    usb_debug.gcs_tx.clear();
    usb_debug.gcs_out_armed = usb_debug.gcs_rx_received = usb_debug.gcs_tx_busy = usb_debug.gcs_tx_zlp = false;
    for (uint8_t ep = 0; ep <= USBD1.otgparams->num_endpoints; ep++) {
        if (usb_debug.otg->ie[ep].DIEPCTL & DIEPCTL_EPENA) {
            usb_debug.otg->ie[ep].DIEPCTL |= DIEPCTL_EPDIS | DIEPCTL_SNAK;
        }
        usb_debug.otg->oe[ep].DOEPCTL |= DOEPCTL_SNAK;
        if (usb_debug.otg->oe[ep].DOEPCTL & DOEPCTL_EPENA) {
            usb_debug.otg->oe[ep].DOEPCTL |= DOEPCTL_EPDIS;
        }
    }
    usb_debug.otg->GRSTCTL = GRSTCTL_RXFFLSH;
    wait_bits(usb_debug.otg->GRSTCTL, GRSTCTL_RXFFLSH, 0);
    usb_debug.otg->GRSTCTL = GRSTCTL_TXFNUM(16) | GRSTCTL_TXFFLSH;
    wait_bits(usb_debug.otg->GRSTCTL, GRSTCTL_TXFFLSH, 0);
    delay_cycles(18);
    usb_debug.otg->DCFG = (usb_debug.otg->DCFG & ~(0x7fU << 4)) | 3U;
    // H74x/H75x OTG cores have 4 KiB of FIFO RAM; this layout uses 384 words.
    // ChibiOS's generic 320-word limit does not describe the H7 hardware.
    // ST confirmed the capacity and corrected AN4879 (Table 4, Rev 12):
    // https://community.st.com/stm32-mcus-products-25/correct-otg-fs-fifo-size-for-stm32h753-163943
    usb_debug.otg->GRXFSIZ = 256;
    usb_debug.otg->DIEPTXF0 = (32U << 16) | 256;
    usb_debug.otg->DIEPTXF[DEBUG_EP-1] = (32U << 16) | 288;
    usb_debug.otg->DIEPTXF[GCS_EP-1] = (32U << 16) | 320;
    usb_debug.otg->DIEPTXF[0] = (16U << 16) | 352;
    usb_debug.otg->DIEPTXF[2] = (16U << 16) | 368;
    usb_debug.otg->DIEPMSK = DIEPMSK_XFRCM;
    usb_debug.otg->DOEPMSK = DOEPMSK_XFRCM | DOEPMSK_STUPM;
    usb_debug.otg->DAINTMSK = (1U << 16) | 1;
    for (uint8_t ep = 0; ep <= DEBUG_EP; ep++) {
        usb_debug.otg->ie[ep].DIEPINT = 0xffffffff;
        usb_debug.otg->oe[ep].DOEPINT = 0xffffffff;
    }
    usb_debug.otg->ie[0].DIEPCTL = DIEPCTL_USBAEP;
    usb_debug.otg->oe[0].DOEPCTL = DOEPCTL_USBAEP;
    arm_out(0);
}

static void setup_request()
{
    const uint16_t value = usb_debug.setup[2] | (uint16_t(usb_debug.setup[3]) << 8);
    const uint16_t index = usb_debug.setup[4] | (uint16_t(usb_debug.setup[5]) << 8);
    const uint16_t length = usb_debug.setup[6] | (uint16_t(usb_debug.setup[7]) << 8);
    usb_debug.control_left = 0;
    usb_debug.control_zlp = false;
    if ((usb_debug.setup[0] & 0x60) == 0) {
        switch (usb_debug.setup[1]) {
        case 6: { // GET_DESCRIPTOR: use the application's exact descriptors.
            const USBDescriptor *desc = usbcfg.get_descriptor_cb(&USBD1, value >> 8, value & 0xff, index);
            if (desc != nullptr) {
                control_send(desc->ud_string, desc->ud_size, length);
                return;
            }
            break;
        }
        case 5: // SET_ADDRESS
            // Like ChibiOS USB_EARLY_SET_ADDRESS, program DAD before the
            // status stage; the OTG core handles the address transition.
            usb_debug.otg->DCFG = (usb_debug.otg->DCFG & ~(0x7fU << 4)) | ((value & 0x7fU) << 4);
            control_send(usb_debug.control_reply, 0, 0);
            return;
        case 9: // SET_CONFIGURATION
            if (value > 1) {
                break;
            }
            ++usb_debug.usb_generation;
            usb_debug.rx_len = usb_debug.rx_pos = 0;
            usb_debug.out_complete = false;
            usb_debug.gcs_rx.clear();
            usb_debug.gcs_tx.clear();
            usb_debug.gcs_out_armed = usb_debug.gcs_rx_received = usb_debug.gcs_tx_busy = usb_debug.gcs_tx_zlp = false;
            usb_debug.configuration = value;
            usb_debug.configured = value == 1;
            for (uint8_t ep : {GCS_EP, DEBUG_EP}) {
                // Abandon completions from the previous configuration too;
                // a cancelled send must not leave the USB IRQ asserted.
                usb_debug.otg->ie[ep].DIEPINT = 0xffffffff;
                usb_debug.otg->oe[ep].DOEPINT = 0xffffffff;
                if (usb_debug.configured) {
                    usb_debug.otg->ie[ep].DIEPCTL = 64 | DIEPCTL_USBAEP | DIEPCTL_EPTYP_BULK |
                        (ep << 22) | DIEPCTL_SD0PID;
                    usb_debug.otg->oe[ep].DOEPCTL = 64 | DOEPCTL_USBAEP | DOEPCTL_EPTYP_BULK | DOEPCTL_SD0PID;
                    usb_debug.otg->DAINTMSK |= (1U << ep) | (1U << (16+ep));
                    arm_out(ep);
                } else {
                    usb_debug.otg->DAINTMSK &= ~((1U << ep) | (1U << (16+ep)));
                    usb_debug.otg->ie[ep].DIEPCTL |= DIEPCTL_EPDIS | DIEPCTL_SNAK;
                    usb_debug.otg->oe[ep].DOEPCTL |= DOEPCTL_EPDIS | DOEPCTL_SNAK;
                }
            }
            // The descriptors also advertise CDC notification IN endpoints.
            // Keep them enabled and NAKing even though there are no serial
            // status changes to report. Windows polls these endpoints and
            // attempts pipe recovery if an advertised endpoint is disabled.
            for (uint8_t ep : {uint8_t(1), uint8_t(3)}) {
                if (usb_debug.configured) {
                    usb_debug.otg->ie[ep].DIEPCTL = 16 | DIEPCTL_USBAEP | DIEPCTL_EPTYP_INTR |
                        (ep << 22) | DIEPCTL_SD0PID | DIEPCTL_SNAK;
                } else {
                    usb_debug.otg->ie[ep].DIEPCTL |= DIEPCTL_EPDIS | DIEPCTL_SNAK;
                }
            }
            usb_debug.gcs_out_armed = usb_debug.configured;
            control_send(usb_debug.control_reply, 0, 0);
            return;
        case 8: // GET_CONFIGURATION
            usb_debug.control_reply[0] = usb_debug.configuration;
            control_send(usb_debug.control_reply, 1, length);
            return;
        case 0: // GET_STATUS
            usb_debug.control_reply[0] = usb_debug.control_reply[1] = 0;
            control_send(usb_debug.control_reply, 2, length);
            return;
        case 1: // CLEAR_FEATURE(ENDPOINT_HALT), used by Windows pipe recovery.
            if (usb_debug.setup[0] != 2 || value != 0 || length != 0 ||
                (index & ~0x8fU) || (index & 15U) == 0 || (index & 15U) > DEBUG_EP ||
                (!(index & 0x80U) && (index & 15U) != GCS_EP && (index & 15U) != DEBUG_EP)) {
                break;
            }
            if (index & 0x80U) {
                auto &ctl = usb_debug.otg->ie[index & 15U].DIEPCTL;
                ctl = (ctl & ~DIEPCTL_STALL) | DIEPCTL_SD0PID;
            } else {
                auto &ctl = usb_debug.otg->oe[index & 15U].DOEPCTL;
                ctl = (ctl & ~DOEPCTL_STALL) | DOEPCTL_SD0PID;
            }
            control_send(usb_debug.control_reply, 0, 0);
            return;
        case 10: // GET_INTERFACE
            usb_debug.control_reply[0] = 0;
            control_send(usb_debug.control_reply, 1, length);
            return;
        case 11: // SET_INTERFACE
            control_send(usb_debug.control_reply, 0, 0);
            return;
        }
    } else if ((usb_debug.setup[0] & 0x60) == 0x20 && (index == 0 || index == 2)) {
        switch (usb_debug.setup[1]) {
        case 0x21: // GET_LINE_CODING
            control_send(usb_debug.line_coding[index/2], sizeof(usb_debug.line_coding[0]), length);
            return;
        case 0x20: // SET_LINE_CODING: the data stage is received below.
            arm_out(0);
            return;
        case 0x22: // SET_CONTROL_LINE_STATE
        case 0x23: // SEND_BREAK
            control_send(usb_debug.control_reply, 0, 0);
            return;
        }
    }
    usb_debug.otg->ie[0].DIEPCTL |= DIEPCTL_STALL;
    usb_debug.otg->oe[0].DOEPCTL |= DOEPCTL_STALL;
}

static void usb_poll()
{
    if (usb_debug.stopped) {
        stm32_watchdog_pat();
    }
    const uint32_t now = DWT->CYCCNT;
    usb_debug.startup_cycles += uint32_t(now-usb_debug.last_cycle);
    usb_debug.last_cycle = now;
    if (usb_debug.stopped && !usb_debug.session_active && usb_debug.startup_cycles > uint64_t(STM32_SYS_D1CPRE_CK)*30) {
        NVIC_SystemReset();
    }
    const uint32_t status = usb_debug.otg->GINTSTS;
    if (status & GINTSTS_SOF) {
        usb_debug.otg->GINTSTS = GINTSTS_SOF;
    }
    if (status & GINTSTS_USBRST) {
        usb_debug.otg->GINTSTS = GINTSTS_USBRST;
        reset_endpoints();
        return;
    }
    if (status & GINTSTS_ENUMDNE) {
        usb_debug.otg->GINTSTS = GINTSTS_ENUMDNE;
    }
    // Completion flags can already describe the next SETUP while the FIFO
    // still contains status entries from the previous control transfer. Drain
    // those entries before using STUP, so back-to-back CDC requests (notably
    // Windows' SET_LINE_CODING followed by GET_LINE_CODING) use the new setup.
    while (usb_debug.otg->GINTSTS & GINTSTS_RXFLVL) {
        const uint32_t receive = usb_debug.otg->GRXSTSP;
        const uint32_t ep = receive & 15;
        const uint32_t count = (receive >> 4) & 0x7ff;
        const uint32_t kind = (receive >> 17) & 15;
        uint8_t data[64];
        for (uint32_t i = 0; i < count; i += 4) {
            const uint32_t word = usb_debug.otg->FIFO[0][0];
            for (uint32_t j = 0; j < 4 && i+j < count && i+j < sizeof(data); j++) {
                data[i+j] = word >> (8*j);
            }
        }
        if (kind == 6 && ep == 0 && count == 8) {
            memcpy(usb_debug.setup, data, 8);
            usb_debug.control_out_status = false;
        } else if (kind == 2 && ep == DEBUG_EP && count <= sizeof(usb_debug.rx)) {
            // One OUT packet remains NAKed until the RSP parser consumes it.
            memcpy(usb_debug.rx, data, count);
            usb_debug.rx_len = count;
            usb_debug.rx_pos = 0;
        } else if (kind == 2 && ep == GCS_EP && count <= sizeof(data)) {
            usb_debug.gcs_rx.write(data, count);
            usb_debug.gcs_rx_received = true;
        } else if (kind == 2 && ep == 0 && usb_debug.setup[1] == 0x20 && count == 7 &&
                   (usb_debug.setup[4] == 0 || usb_debug.setup[4] == 2) && usb_debug.setup[5] == 0) {
            memcpy(usb_debug.line_coding[usb_debug.setup[4]/2], data, 7);
            usb_debug.control_out_status = true;
        }
    }
    const uint32_t out0 = usb_debug.otg->oe[0].DOEPINT;
    if (out0) {
        usb_debug.otg->oe[0].DOEPINT = out0;
        if (out0 & DOEPINT_STUP) {
            setup_request();
        }
        // The RX FIFO data event precedes completion of the OUT transfer.
        // Wait for XFRC before starting its IN status stage, as ChibiOS does.
        if ((out0 & DOEPINT_XFRC) && usb_debug.control_out_status) {
            usb_debug.control_out_status = false;
            control_send(usb_debug.control_reply, 0, 0);
        }
    }
    const uint32_t in0 = usb_debug.otg->ie[0].DIEPINT;
    if (in0 & DIEPINT_XFRC) {
        usb_debug.otg->ie[0].DIEPINT = DIEPINT_XFRC;
        control_next();
    }
    if (usb_debug.otg->oe[DEBUG_EP].DOEPINT & DOEPINT_XFRC) {
        usb_debug.otg->oe[DEBUG_EP].DOEPINT = DOEPINT_XFRC;
        usb_debug.out_complete = true;
    }
    if (usb_debug.otg->oe[GCS_EP].DOEPINT & DOEPINT_XFRC) {
        usb_debug.otg->oe[GCS_EP].DOEPINT = DOEPINT_XFRC;
        usb_debug.gcs_out_armed = false;
    }
    if (usb_debug.otg->ie[GCS_EP].DIEPINT & DIEPINT_XFRC) {
        usb_debug.otg->ie[GCS_EP].DIEPINT = DIEPINT_XFRC;
        usb_debug.gcs_tx_busy = false;
    }
    if (usb_debug.configured) {
        // NAK when full, including during a debugger stop. No bytes are dropped.
        if (!usb_debug.gcs_out_armed && usb_debug.gcs_rx_received && usb_debug.gcs_rx.space() >= 64) {
            arm_out(GCS_EP);
            usb_debug.gcs_out_armed = true;
            usb_debug.gcs_rx_received = false;
        }
        if (!usb_debug.gcs_tx_busy && (usb_debug.gcs_tx.available() || usb_debug.gcs_tx_zlp)) {
            uint8_t data[64];
            const uint32_t size = usb_debug.gcs_tx.read(data, sizeof(data));
            transmit(GCS_EP, data, size);
            usb_debug.gcs_tx_busy = true;
            usb_debug.gcs_tx_zlp = size == 64;
        }
    }
}

static int receive_byte()
{
    const uint32_t generation = usb_debug.usb_generation;
    while (usb_debug.rx_pos == usb_debug.rx_len) {
        if (usb_debug.configured && usb_debug.out_complete) {
            usb_debug.rx_len = usb_debug.rx_pos = 0;
            usb_debug.out_complete = false;
            arm_out(DEBUG_EP);
        }
        usb_poll();
        if (generation != usb_debug.usb_generation) {
            return -1;
        }
    }
    return usb_debug.rx[usb_debug.rx_pos++];
}

static bool send_bytes(const uint8_t *data, uint32_t size)
{
    const uint32_t generation = usb_debug.usb_generation;
    // A short packet completes the host's bulk read, which can exceed 64 bytes.
    // End an exact packet multiple with a ZLP, including retransmitted replies.
    bool zlp = size != 0 && size % 64U == 0;
    while ((size != 0 || zlp) && usb_debug.configured) {
        const uint32_t n = smaller(size, 64U);
        if (n == 0) {
            zlp = false;
        }
        transmit(DEBUG_EP, data, n);
        while (!(usb_debug.otg->ie[DEBUG_EP].DIEPINT & DIEPINT_XFRC) && usb_debug.configured) {
            usb_poll();
            if (generation != usb_debug.usb_generation) {
                return false;
            }
        }
        if (!usb_debug.configured) {
            return false;
        }
        // Unlike timer polling, the USB IRQ is level-triggered. A completed
        // final reply must not leave EP4's IN interrupt asserted on continue.
        usb_debug.otg->ie[DEBUG_EP].DIEPINT = DIEPINT_XFRC;
        data += n;
        size -= n;
    }
    return size == 0 && !zlp;
}

static bool hex_number(const char *&p, uint32_t &value)
{
    value = 0;
    unsigned digits = 0;
    uint8_t digit;
    while (hex_char_to_nibble(*p, digit)) {
        if (++digits > 8) {
            return false;
        }
        value = (value << 4) | digit;
        p++;
    }
    return digits != 0;
}

static char *encode_word(char *out, uint32_t word)
{
    for (uint8_t i = 0; i < 4; i++) {
        *out++ = HEX[(word >> (8*i+4)) & 15];
        *out++ = HEX[(word >> (8*i)) & 15];
    }
    *out = 0;
    return out;
}

static bool valid_memory(uint32_t address, uint32_t length, bool write = false)
{
    struct Region { void *address; uint32_t size; uint32_t flags; };
    static const Region regions[] = { HAL_MEMORY_REGIONS };
    if (length > (write ? PACKET_SIZE : PACKET_SIZE/2) || address + length < address) {
        return false;
    }
    if (write) {
        // Includes the live monitor stack, packet buffers and saved CPU context.
        const uint32_t monitor = uint32_t(&usb_debug);
        if (address < monitor + sizeof(usb_debug) && address + length > monitor) {
            return false;
        }
    } else if (address >= 0x08000000U && address + length <= 0x08000000U + BOARD_FLASH_SIZE*1024U) {
        return true;
    }
    for (const auto &region : regions) {
        const uint32_t start = uint32_t(region.address);
        if (address >= start && address + length <= start + region.size) {
            return true;
        }
    }
    return false;
}

static volatile uint32_t &fp_control = *(volatile uint32_t *)0xe0002000;
static volatile uint32_t *const fp_comparator = (volatile uint32_t *)0xe0002008;
extern "C" char __usb_debug_text_start, __usb_debug_text_end;

static bool probe_copy(void *dest, uint32_t address, uint32_t size)
{
    if (!valid_memory(address, size)) {
        return false;
    }
    auto *bytes = (uint8_t *)dest;
    const uint32_t cfsr = SCB->CFSR;
    const uint32_t hfsr = SCB->HFSR;
    for (uint32_t i = 0; i < size; i++) {
        usb_debug.probing = true;
        const int value = usb_debug_read_byte(address+i);
        usb_debug.probing = false;
        if (value < 0) {
            SCB->CFSR = SCB->CFSR & ~cfsr;
            SCB->HFSR = SCB->HFSR & ~hfsr;
            return false;
        }
        bytes[i] = value;
    }
    return true;
}

static void snapshot_threads()
{
    usb_debug.stopped_thread = 1;
    usb_debug.thread_count = 0;
    // A zero-latency stop can catch a registry update or an interrupt context.
    // In that case expose the real CPU frame only, not an inconsistent registry.
    if ((usb_debug.registers[16] & 0x1ffU) == 0 && usb_debug.context[12] == 0) {
        auto *header = REG_HEADER(currcore);
        auto *node = header->next;
        auto *previous = header;
        while (node != header && usb_debug.thread_count < ARRAY_SIZE(usb_debug.thread_ids)) {
            thread_t thread;
            const uint32_t address = uint32_t(node) - offsetof(thread_t, rqueue);
            if (!probe_copy(&thread, address, sizeof(thread)) || thread.rqueue.prev != previous) {
                usb_debug.thread_count = 0;
                break;
            }
            usb_debug.thread_ids[usb_debug.thread_count++] = address;
            if (address == uint32_t(currcore->rlist.current)) {
                usb_debug.stopped_thread = address;
            }
            previous = node;
            node = thread.rqueue.next;
        }
        if (node != header || usb_debug.stopped_thread == 1) {
            usb_debug.thread_count = 0;
            usb_debug.stopped_thread = 1;
        }
    }
    if (usb_debug.thread_count == 0) {
        usb_debug.thread_ids[usb_debug.thread_count++] = usb_debug.stopped_thread;
    }
    usb_debug.selected_thread = usb_debug.stopped_thread;
}

static bool known_thread(uint32_t id)
{
    for (uint8_t i = 0; i < usb_debug.thread_count; i++) {
        if (usb_debug.thread_ids[i] == id) {
            return true;
        }
    }
    return false;
}

static void select_registers()
{
    memset(usb_debug.register_address, 0, sizeof(usb_debug.register_address));
    usb_debug.register_mask = usb_debug.fp_available ? ((1ULL << 50)-1) : 0x1ffff;
    if (usb_debug.selected_thread == usb_debug.stopped_thread) {
        memcpy(usb_debug.selected_registers, usb_debug.registers, sizeof(usb_debug.selected_registers));
        return;
    }
    usb_debug.register_mask = 0;
    thread_t thread;
    port_intctx context;
    if (!probe_copy(&thread, usb_debug.selected_thread, sizeof(thread)) ||
        !probe_copy(&context, uint32_t(thread.ctx.sp), sizeof(context))) {
        return;
    }
    // __port_switch saves callee-saved registers and its call return address.
    // Caller-saved registers and xPSR are unavailable for a blocked thread.
    memcpy(usb_debug.selected_registers+4, &context.r4, 8*sizeof(uint32_t));
    usb_debug.selected_registers[13] = uint32_t(thread.ctx.sp) + sizeof(context);
    usb_debug.selected_registers[15] = context.lr & ~1U;
    usb_debug.register_mask = 0xff0U | (1U << 13) | (1U << 15);
    for (uint8_t i = 4; i <= 11; i++) {
        usb_debug.register_address[i] = uint32_t(thread.ctx.sp) + offsetof(port_intctx, r4) + 4*(i-4);
    }
    usb_debug.register_address[15] = uint32_t(thread.ctx.sp) + offsetof(port_intctx, lr);
#if CORTEX_USE_FPU
    memcpy(usb_debug.selected_registers+33, &context.s16, 16*sizeof(uint32_t));
    for (uint8_t i = 33; i < 49; i++) {
        usb_debug.register_address[i] = uint32_t(thread.ctx.sp) + offsetof(port_intctx, s16) + 4*(i-33);
        usb_debug.register_mask |= 1ULL << i;
    }
#endif
#if !PORT_USE_SYSCALL
    // A tail-called switch from the IRQ epilogue leaves the original exception
    // frame immediately above the saved context. Expose the interrupted thread.
    if (usb_debug.selected_registers[15] >= (uint32_t(__port_switch_from_isr) & ~1U) &&
        usb_debug.selected_registers[15] <= (uint32_t(__port_exit_from_isr) & ~1U)) {
        port_extctx frame;
        if (probe_copy(&frame, usb_debug.selected_registers[13], sizeof(frame)) &&
            (frame.xpsr & 0x010001ffU) == 0x01000000U) {
            memcpy(usb_debug.selected_registers, &frame.r0, 4*sizeof(uint32_t));
            usb_debug.selected_registers[12] = frame.r12;
            usb_debug.selected_registers[13] += sizeof(frame) + ((frame.xpsr & 512) ? 4 : 0);
            usb_debug.selected_registers[14] = frame.lr_thd;
            usb_debug.selected_registers[15] = frame.pc;
            usb_debug.selected_registers[16] = frame.xpsr;
            usb_debug.register_mask |= 0x1ffff;
            const uint32_t base = uint32_t(thread.ctx.sp) + sizeof(context);
            for (uint8_t i = 0; i < 4; i++) {
                usb_debug.register_address[i] = base + 4*i;
            }
            usb_debug.register_address[12] = base + 16;
            usb_debug.register_address[14] = base + 20;
            usb_debug.register_address[15] = base + 24;
            usb_debug.register_address[16] = base + 28;
#if CORTEX_USE_FPU
            memcpy(usb_debug.selected_registers+17, &frame.s0, 16*sizeof(uint32_t));
            usb_debug.selected_registers[49] = frame.fpscr;
            for (uint8_t i = 17; i < 33; i++) {
                usb_debug.register_address[i] = base + offsetof(port_extctx, s0) + 4*(i-17);
                usb_debug.register_mask |= 1ULL << i;
            }
            usb_debug.register_address[49] = base + offsetof(port_extctx, fpscr);
            usb_debug.register_mask |= 1ULL << 49;
#endif
        }
    }
#endif
}

static char *encode_register(char *out, uint8_t reg)
{
    if (usb_debug.register_mask & (1ULL << reg)) {
        return encode_word(out, usb_debug.selected_registers[reg]);
    }
    memcpy(out, "xxxxxxxx", 8);
    out[8] = 0;
    return out+8;
}

static char *encode_hex(char *out, uint32_t value)
{
    // HAL integer formatting uses only stack state, without allocation or locks.
    return out + hal.util->snprintf(out, 9, "%lx", (unsigned long)value);
}

static void stop_reply()
{
    strcpy(usb_debug.reply, "T00thread:");
    usb_debug.reply[1] = HEX[usb_debug.stop_signal >> 4];
    usb_debug.reply[2] = HEX[usb_debug.stop_signal & 15];
    char *out = encode_hex(usb_debug.reply+10, usb_debug.stopped_thread);
    *out++ = ';';
    *out = 0;
    if (usb_debug.watch_type) {
        const char *reason = usb_debug.watch_type == 2 ? "watch:" : usb_debug.watch_type == 3 ? "rwatch:" : "awatch:";
        strcpy(out, reason);
        out = encode_hex(out+strlen(reason), usb_debug.watch_address);
        strcpy(out, ";");
    }
}

// DWT comparators match an aligned power-of-two address range. Keep them off
// while the monitor reads application memory, so debugger inspection never
// triggers another watchpoint. FUNCTION.MATCHED is sampled before disabling.
static volatile uint32_t *dwt_comparator(uint8_t index)
{
    return (volatile uint32_t *)(0xe0001020U + 16U*index);
}

static void program_watchpoints(bool enable)
{
    for (uint8_t i = 0; i < usb_debug.watchpoint_count; i++) {
        auto *reg = dwt_comparator(i);
        reg[2] = 0;
        if (enable && usb_debug.watchpoints[i].type) {
            reg[0] = usb_debug.watchpoints[i].address;
            reg[1] = usb_debug.watchpoints[i].mask;
            const uint8_t type = usb_debug.watchpoints[i].type;
            reg[2] = type == 2 ? 6 : type == 3 ? 5 : 7;
        }
    }
    __DSB();
    __ISB();
}

static bool watchpoint(bool insert, uint8_t type, uint32_t address, uint32_t size)
{
    if (!size || size > 32 || (size & (size-1)) || (address & (size-1)) || address+size < address) {
        return false;
    }
    for (uint8_t i = 0; i < usb_debug.watchpoint_count; i++) {
        auto &wp = usb_debug.watchpoints[i];
        if (wp.type == type && wp.address == address && (1U << wp.mask) == size) {
            if (!insert) {
                wp.type = 0;
            }
            return true;
        }
    }
    if (!insert) {
        return true;
    }
    for (uint8_t i = 0; i < usb_debug.watchpoint_count; i++) {
        auto &wp = usb_debug.watchpoints[i];
        if (wp.type == 0) {
            uint8_t mask = 0;
            while ((1U << mask) < size) {
                mask++;
            }
            auto *reg = dwt_comparator(i);
            reg[1] = mask;
            if (reg[1] != mask) {
                return false;
            }
            wp.address = address;
            wp.mask = mask;
            wp.type = type;
            return true;
        }
    }
    return false;
}

static uint32_t breakpoint_value(uint32_t address)
{
    if ((fp_control >> 28) == 0) {
        return (address & 0x1ffffffcU) | ((address & 2U) ? (2U << 30) : (1U << 30)) | 1U;
    }
    return (address & ~1U) | 1U;
}

static void program_breakpoints()
{
    for (uint32_t i = 0; i < usb_debug.breakpoint_count; i++) {
        fp_comparator[i] = (usb_debug.breakpoint_address[i] && int(i) != usb_debug.step_over) ?
            breakpoint_value(usb_debug.breakpoint_address[i]) : 0;
    }
    __DSB();
    __ISB();
}

static bool breakpoint(bool insert, uint32_t address, uint32_t kind)
{
    if ((address & 1U) || (kind != 2 && kind != 4) || address < 0x08000000U ||
        address+4 > 0x08000000U + BOARD_FLASH_SIZE*1024U ||
        (address >= uint32_t(&__usb_debug_text_start) && address < uint32_t(&__usb_debug_text_end))) {
        return false;
    }
    for (uint32_t i = 0; i < usb_debug.breakpoint_count; i++) {
        if (usb_debug.breakpoint_address[i] == address) {
            if (!insert) {
                usb_debug.breakpoint_address[i] = 0;
            }
            return true;
        }
    }
    if (!insert) {
        return true;
    }
    for (uint32_t i = 0; i < usb_debug.breakpoint_count; i++) {
        if (usb_debug.breakpoint_address[i] == 0) {
            usb_debug.breakpoint_address[i] = address;
            return true;
        }
    }
    return false;
}

static void sync_code(uint32_t address, uint32_t length)
{
    const uint32_t start = address & ~31U;
    stm32_cacheBufferFlush((void *)start, ((address+length+31U) & ~31U) - start);
    __DSB();
    SCB->ICIALLU = 0;
    __DSB();
    __ISB();
}

static bool probe_write(uint32_t address, const void *data, uint32_t length)
{
    if (!valid_memory(address, length, true)) {
        return false;
    }
    const auto *bytes = (const uint8_t *)data;
    const uint32_t cfsr = SCB->CFSR, hfsr = SCB->HFSR;
    for (uint32_t i = 0; i < length; i++) {
        usb_debug.probing = true;
        const int result = usb_debug_write_byte(address+i, bytes[i]);
        usb_debug.probing = false;
        if (result < 0) {
            SCB->CFSR = SCB->CFSR & ~cfsr;
            SCB->HFSR = SCB->HFSR & ~hfsr;
            return false;
        }
    }
    return true;
}

static bool patch_ram_breakpoint(uint8_t index, bool enable)
{
    const auto &bp = usb_debug.ram_breakpoints[index];
    const uint16_t instruction = enable ? 0xbe00 : bp.original;
    const bool ok = probe_write(bp.address, &instruction, sizeof(instruction));
    sync_code(bp.address, sizeof(instruction));
    return ok;
}

static bool ram_breakpoint(bool insert, uint32_t address, uint32_t kind)
{
    if ((address & 1U) || (kind != 2 && kind != 4) || !valid_memory(address, 2, true)) {
        return false;
    }
    for (uint8_t i = 0; i < ARRAY_SIZE(usb_debug.ram_breakpoints); i++) {
        auto &bp = usb_debug.ram_breakpoints[i];
        if (bp.address == address) {
            if (!insert) {
                if (!patch_ram_breakpoint(i, false)) {
                    return false;
                }
                bp.address = 0;
            }
            return true;
        }
    }
    if (!insert) {
        return true;
    }
    for (uint8_t i = 0; i < ARRAY_SIZE(usb_debug.ram_breakpoints); i++) {
        auto &bp = usb_debug.ram_breakpoints[i];
        if (!bp.address && probe_copy(&bp.original, address, 2)) {
            bp.address = address;
            if (!patch_ram_breakpoint(i, true)) {
                patch_ram_breakpoint(i, false);
                bp.address = 0;
                return false;
            }
            return true;
        }
    }
    return false;
}

static bool prepare_resume(bool single_step)
{
    if (usb_debug.fault_exception) {
        if (single_step) {
            return false;
        }
        // Retry the faulting instruction, or the PC explicitly selected by GDB.
        // Never silently advance past a fault. A bad repair faults again.
        SCB->CFSR = usb_debug.fault_cfsr;
        SCB->HFSR = usb_debug.fault_hfsr;
    }
    usb_debug.step_ram = -1;
    for (uint8_t i = 0; i < ARRAY_SIZE(usb_debug.ram_breakpoints); i++) {
        if (usb_debug.ram_breakpoints[i].address == usb_debug.registers[15]) {
            if (!patch_ram_breakpoint(i, false)) {
                return false;
            }
            usb_debug.step_ram = i;
            usb_debug.software_breakpoint = false;
            break;
        }
    }
    if (usb_debug.software_breakpoint) {
        auto *frame = (uint32_t *)usb_debug.context[(usb_debug.context[10] & 4) ? 9 : 8];
        frame[6] += 2;
        usb_debug.registers[15] = frame[6];
        usb_debug.software_breakpoint = false;
        if (single_step) {
            stop_reply();
            return true;
        }
    }
    usb_debug.step_over = -1;
    for (uint32_t i = 0; i < usb_debug.breakpoint_count; i++) {
        if (usb_debug.breakpoint_address[i] == usb_debug.registers[15]) {
            usb_debug.step_over = i;
            break;
        }
    }
    usb_debug.step_requested = single_step || usb_debug.step_over >= 0 || usb_debug.step_ram >= 0;
    if (usb_debug.step_requested) {
        uint16_t instruction;
        if (!probe_copy(&instruction, usb_debug.registers[15], sizeof(instruction)) ||
            (usb_debug.registers[16] & 0x1ffU) != 0 || usb_debug.context[11] != 0 ||
            (instruction & 0xff00U) == 0xdf00U ||  // SVC
            (instruction & 0xff00U) == 0xbe00U ||  // BKPT
            (instruction & 0xffe0U) == 0xb660U ||  // CPS
            (instruction & 0xfff0U) == 0xf380U ||  // MSR
            instruction == 0xbf20U || instruction == 0xbf30U) { // WFE/WFI
            if (usb_debug.step_ram >= 0) {
                patch_ram_breakpoint(usb_debug.step_ram, true);
                usb_debug.step_ram = -1;
            }
            usb_debug.step_over = -1;
            usb_debug.step_requested = false;
            return false;
        }
        usb_debug.step_basepri = usb_debug.context[12];
        // Keep configurable IRQs below DebugMon masked for this instruction.
        usb_debug.context[12] = 1U << (8U-__NVIC_PRIO_BITS);
        NVIC_DisableIRQ(USB_IRQ);
        CoreDebug->DEMCR |= CoreDebug_DEMCR_MON_STEP_Msk;
    }
    usb_debug.step_report = single_step;
    usb_debug.resume_requested = true;
    return true;
}

static void write_memory(const char *p, uint32_t packet_length, bool binary)
{
    uint32_t address, length;
    if (!hex_number(p, address) || *p++ != ',' || !hex_number(p, length) || *p++ != ':' ||
        length > PACKET_SIZE || packet_length - uint32_t(p-usb_debug.packet) != length*(binary ? 1U : 2U)) {
        strcpy(usb_debug.reply, "E01");
        return;
    }
    // GDB probes binary-write support with an empty transfer.
    if (length == 0) {
        strcpy(usb_debug.reply, "OK");
        return;
    }
    if (!valid_memory(address, length, true)) {
        strcpy(usb_debug.reply, "E01");
        return;
    }
    // Decode into the consumed packet prefix, before changing application memory.
    // The ASCII payload fits fewer than 256 bytes in a PACKET_SIZE-byte packet.
    if (!binary) {
        if (length > UINT8_MAX ||
            !hex_charpairs_to_uint8s(p, length, (uint8_t *)usb_debug.packet)) {
            strcpy(usb_debug.reply, "E01");
            return;
        }
        p = usb_debug.packet;
    }
    const uint32_t cfsr = SCB->CFSR;
    const uint32_t hfsr = SCB->HFSR;
    for (uint32_t i = 0; i < length; i++) {
        const uint8_t value = uint8_t(p[i]);
        bool shadowed = false;
        for (auto &bp : usb_debug.ram_breakpoints) {
            if (bp.address && address+i >= bp.address && address+i < bp.address+2) {
                const uint8_t shift = 8*(address+i-bp.address);
                bp.original = (bp.original & ~(255U << shift)) | (uint16_t(value) << shift);
                shadowed = true;
                break;
            }
        }
        if (shadowed) {
            continue;
        }
        usb_debug.probing = true;
        const int result = usb_debug_write_byte(address+i, value);
        usb_debug.probing = false;
        if (result < 0) {
            SCB->CFSR = SCB->CFSR & ~cfsr;
            SCB->HFSR = SCB->HFSR & ~hfsr;
            // A bus fault may leave a prefix written; never report success.
            strcpy(usb_debug.reply, "E01");
            return;
        }
    }
    sync_code(address, length);
    strcpy(usb_debug.reply, "OK");
}

// Validate a whole register packet before changing any context. Core registers
// of the stopped CPU live in our exception frame; blocked threads instead have
// only the registers that ChibiOS actually saved. Their SP is descriptive and
// cannot be moved without also relocating the scheduler's private stack frames.
static bool apply_registers(const uint32_t *values, uint64_t changed)
{
    const bool current = usb_debug.selected_thread == usb_debug.stopped_thread;
    for (uint8_t i = 0; i < 50; i++) {
        if (!(changed & (1ULL << i))) {
            continue;
        }
        if (!(usb_debug.register_mask & (1ULL << i)) ||
            (!current && (!usb_debug.register_address[i] ||
                          !valid_memory(usb_debug.register_address[i], 4, true)))) {
            return false;
        }
    }
    if ((changed & (1ULL << 16)) &&
        ((values[16] ^ usb_debug.selected_registers[16]) & ~0xfe0ffc00U)) {
        return false;
    }
    if ((changed & (1ULL << 15)) &&
        (!valid_memory(values[15] & ~1U, 2) ||
         ((values[15] & ~1U) >= uint32_t(&__usb_debug_text_start) &&
          (values[15] & ~1U) < uint32_t(&__usb_debug_text_end)))) {
        return false;
    }
    if (current) {
        const uint8_t stack = (usb_debug.context[10] & 4) ? 9 : 8;
        auto *frame = (uint32_t *)usb_debug.context[stack];
        const uint32_t size = ((usb_debug.context[10] & 16) ? 32 : 104) + ((frame[7] & 512) ? 4 : 0);
        if (changed & (1ULL << 13)) {
            const uint32_t address = values[13] - size;
            if ((values[13] & 7U) != (usb_debug.registers[13] & 7U) || values[13] < size ||
                !valid_memory(address, size, true)) {
                return false;
            }
            // GDB's inferior-call setup moves SP to reserve arguments and its
            // dummy frame. Preserve the architectural exception-frame layout.
            uint32_t saved_frame[27];
            memcpy(saved_frame, frame, size);
            if (!probe_write(address, saved_frame, size)) {
                return false;
            }
            frame = (uint32_t *)address;
            usb_debug.context[stack] = address;
        }
        memcpy(usb_debug.registers, values, sizeof(usb_debug.registers));
        usb_debug.registers[15] &= ~1U;
        if (changed & (1ULL << 15)) {
            usb_debug.software_breakpoint = false;
        }
        memcpy(frame, values, 4*sizeof(uint32_t));
        memcpy(usb_debug.context, values+4, 8*sizeof(uint32_t));
        frame[4] = values[12];
        frame[5] = values[14];
        frame[6] = values[15] & ~1U;
        frame[7] = values[16];
        if (usb_debug.fp_available) {
            memcpy(usb_debug.fp_registers, values+17, sizeof(usb_debug.fp_registers));
            if (!(usb_debug.context[10] & 16)) {
                memcpy(frame+8, values+17, 16*sizeof(uint32_t));
                frame[24] = values[49];
            }
        }
    } else {
        for (uint8_t i = 0; i < 50; i++) {
            if (changed & (1ULL << i)) {
                uint32_t value = values[i];
                if (i == 15) {
                    // A switch return address requires Thumb bit 0 set; an
                    // exception PC requires it clear. Preserve the saved form.
                    uint32_t previous;
                    if (!probe_copy(&previous, usb_debug.register_address[i], sizeof(previous))) {
                        return false;
                    }
                    value = (value & ~1U) | (previous & 1U);
                }
                if (!probe_write(usb_debug.register_address[i], &value, sizeof(value))) {
                    return false;
                }
            }
        }
    }
    return true;
}

static void write_registers(const char *p, bool all)
{
    select_registers();
    uint32_t values[50];
    memcpy(values, usb_debug.selected_registers, sizeof(values));
    uint32_t first = 0, count = 50;
    if (!all) {
        uint32_t reg;
        if (!hex_number(p, reg) || *p++ != '=' || !(reg < 16 || (reg >= 25 && reg <= 42))) {
            strcpy(usb_debug.reply, "E01");
            return;
        }
        first = reg < 16 ? reg : reg == 25 ? 16 : reg == 42 ? 49 : 17+2*(reg-26);
        count = (reg >= 26 && reg <= 41) ? 2 : 1;
    }
    if (strlen(p) != 8*count) {
        strcpy(usb_debug.reply, "E01");
        return;
    }
    uint64_t changed = 0;
    for (uint32_t i = first; i < first+count; i++, p += 8) {
        if (all && strncmp(p, "xxxxxxxx", 8) == 0 && !(usb_debug.register_mask & (1ULL << i))) {
            continue;
        }
        uint32_t value;
        // RSP register bytes and STM32 words are both little-endian.
        if (!hex_charpairs_to_uint8s(p, sizeof(value), (uint8_t *)&value)) {
            strcpy(usb_debug.reply, "E01");
            return;
        }
        if (!all || value != values[i]) {
            changed |= 1ULL << i;
        }
        values[i] = value;
    }
    strcpy(usb_debug.reply, apply_registers(values, changed) ? "OK" : "E01");
}

static void command(uint32_t packet_length)
{
    strcpy(usb_debug.reply, "");
    const char *p = usb_debug.packet+1;
    uint32_t a, n;
    if (strcmp(usb_debug.packet, "?") == 0) {
        stop_reply();
    } else if (strcmp(usb_debug.packet, "g") == 0) {
        char *out = usb_debug.reply;
        select_registers();
        for (uint8_t reg = 0; reg < 50; reg++) {
            out = encode_register(out, reg);
        }
    } else if (usb_debug.packet[0] == 'p' && hex_number(p, a) && *p == 0 && (a < 16 || (a >= 25 && a <= 42))) {
        select_registers();
        const uint8_t reg = a < 16 ? a : a == 25 ? 16 : a == 42 ? 49 : 17+2*(a-26);
        char *out = encode_register(usb_debug.reply, reg);
        if (a >= 26 && a <= 41) {
            encode_register(out, reg+1);
        }
    } else if (usb_debug.packet[0] == 'm') {
        if (!hex_number(p, a) || *p++ != ',' || !hex_number(p, n) || *p != 0 || !valid_memory(a, n)) {
            strcpy(usb_debug.reply, "E01");
            return;
        }
        for (uint32_t i = 0; i < n; i++) {
            const uint32_t cfsr = SCB->CFSR;
            const uint32_t hfsr = SCB->HFSR;
            usb_debug.probing = true;
            int value = usb_debug_read_byte(a+i);
            usb_debug.probing = false;
            if (value < 0) {
                SCB->CFSR = SCB->CFSR & ~cfsr;
                SCB->HFSR = SCB->HFSR & ~hfsr;
                strcpy(usb_debug.reply, "E01");
                return;
            }
            for (const auto &bp : usb_debug.ram_breakpoints) {
                if (bp.address && a+i >= bp.address && a+i < bp.address+2) {
                    value = (bp.original >> (8*(a+i-bp.address))) & 255;
                }
            }
            usb_debug.reply[2*i] = HEX[value >> 4];
            usb_debug.reply[2*i+1] = HEX[value & 15];
        }
        usb_debug.reply[2*n] = 0;
    } else if (usb_debug.packet[0] == 'M' || usb_debug.packet[0] == 'X') {
        write_memory(p, packet_length, usb_debug.packet[0] == 'X');
    } else if (usb_debug.packet[0] == 'P' || usb_debug.packet[0] == 'G') {
        write_registers(p, usb_debug.packet[0] == 'G');
    } else if (strncmp(usb_debug.packet, "qSupported", 10) == 0) {
        strcpy(usb_debug.reply, "PacketSize=200;qXfer:features:read+;hwbreak+;ap-usb-debug+");
    } else if (strncmp(usb_debug.packet, "qXfer:features:read:target.xml:", 31) == 0) {
        p = usb_debug.packet+31;
        if (!hex_number(p, a) || *p++ != ',' || !hex_number(p, n) || *p) {
            strcpy(usb_debug.reply, "E01");
            return;
        }
        target_description(a, n);
    } else if (strcmp(usb_debug.packet, "qfThreadInfo") == 0 || strcmp(usb_debug.packet, "qsThreadInfo") == 0) {
        if (usb_debug.packet[1] == 'f') {
            usb_debug.thread_cursor = 0;
        }
        if (usb_debug.thread_cursor == usb_debug.thread_count) {
            strcpy(usb_debug.reply, "l");
        } else {
            usb_debug.reply[0] = 'm';
            char *out = usb_debug.reply+1;
            while (usb_debug.thread_cursor < usb_debug.thread_count && out+9 < usb_debug.reply+PACKET_SIZE) {
                if (out != usb_debug.reply+1) {
                    *out++ = ',';
                }
                out = encode_hex(out, usb_debug.thread_ids[usb_debug.thread_cursor++]);
            }
        }
    } else if (strncmp(usb_debug.packet, "qThreadExtraInfo,", 17) == 0) {
        p = usb_debug.packet+17;
        if (!hex_number(p, a) || *p || !known_thread(a)) {
            strcpy(usb_debug.reply, "E01");
            return;
        }
        char name[40] = "CPU (kernel/interrupt stop)";
        if (a != 1) {
            thread_t thread;
            if (!probe_copy(&thread, a, sizeof(thread))) {
                strcpy(usb_debug.reply, "E01");
                return;
            }
            unsigned i = 0;
            for (; i < sizeof(name)-1; i++) {
                if (!probe_copy(&name[i], uint32_t(thread.name)+i, 1) || name[i] == 0) {
                    break;
                }
            }
            name[i] = 0;
            if (!name[0]) {
                strcpy(name, "unnamed");
            }
        }
        for (unsigned i = 0; name[i]; i++) {
            usb_debug.reply[2*i] = HEX[uint8_t(name[i]) >> 4];
            usb_debug.reply[2*i+1] = HEX[name[i] & 15];
            usb_debug.reply[2*i+2] = 0;
        }
    } else if (strcmp(usb_debug.packet, "qC") == 0) {
        strcpy(usb_debug.reply, "QC");
        encode_hex(usb_debug.reply+2, usb_debug.stopped_thread);
    } else if (strcmp(usb_debug.packet, "qAttached") == 0) {
        strcpy(usb_debug.reply, "1");
    } else if (usb_debug.packet[0] == 'H' && (usb_debug.packet[1] == 'g' || usb_debug.packet[1] == 'c')) {
        p = usb_debug.packet+2;
        if (strcmp(p, "-1") == 0 || strcmp(p, "0") == 0) {
            a = usb_debug.stopped_thread;
        } else if (!hex_number(p, a) || *p || !known_thread(a)) {
            strcpy(usb_debug.reply, "E01");
            return;
        }
        if (usb_debug.packet[1] == 'g') {
            usb_debug.selected_thread = a;
        } else if (a != usb_debug.stopped_thread) {
            // All-stop resume only: scheduling a chosen blocked thread is not supported.
            strcpy(usb_debug.reply, "E01");
            return;
        }
        strcpy(usb_debug.reply, "OK");
    } else if (usb_debug.packet[0] == 'T' && hex_number(p, a) && !*p) {
        strcpy(usb_debug.reply, known_thread(a) ? "OK" : "E01");
    } else if ((usb_debug.packet[0] == 'Z' || usb_debug.packet[0] == 'z') &&
               usb_debug.packet[1] >= '0' && usb_debug.packet[1] <= '4' && usb_debug.packet[2] == ',') {
        p = usb_debug.packet+3;
        if (!hex_number(p, a) || *p++ != ',' || !hex_number(p, n) || *p ||
            !(usb_debug.packet[1] == '0' && valid_memory(a, 2, true) ? ram_breakpoint(usb_debug.packet[0] == 'Z', a, n) :
              usb_debug.packet[1] <= '1' ? breakpoint(usb_debug.packet[0] == 'Z', a, n) :
              watchpoint(usb_debug.packet[0] == 'Z', usb_debug.packet[1]-'0', a, n))) {
            strcpy(usb_debug.reply, "E01");
        } else {
            strcpy(usb_debug.reply, "OK");
        }
    } else if (strcmp(usb_debug.packet, "c") == 0 || strcmp(usb_debug.packet, "s") == 0) {
        if (!prepare_resume(usb_debug.packet[0] == 's')) {
            strcpy(usb_debug.reply, "E01");
        }
    } else if (strcmp(usb_debug.packet, "D") == 0 || strcmp(usb_debug.packet, "D;1") == 0) {
        for (uint8_t i = 0; i < ARRAY_SIZE(usb_debug.ram_breakpoints); i++) {
            auto &bp = usb_debug.ram_breakpoints[i];
            if (bp.address) {
                if (!patch_ram_breakpoint(i, false)) {
                    strcpy(usb_debug.reply, "E01");
                    return;
                }
                if (bp.address == usb_debug.registers[15]) {
                    usb_debug.software_breakpoint = false;
                }
                bp.address = 0;
            }
        }
        if (usb_debug.software_breakpoint) {
            auto *frame = (uint32_t *)usb_debug.context[(usb_debug.context[10] & 4) ? 9 : 8];
            frame[6] += 2;
            usb_debug.software_breakpoint = false;
        }
        strcpy(usb_debug.reply, "OK");
        usb_debug.detach_requested = true;
    } else if (strcmp(usb_debug.packet, "qRcmd,6661756c74") == 0) {
        char status[160];
        const int length = snprintf(status, sizeof(status),
                                    "exception=%lu CFSR=%08lx HFSR=%08lx MMFAR=%08lx BFAR=%08lx\n",
                                    (unsigned long)usb_debug.fault_exception, (unsigned long)usb_debug.fault_cfsr,
                                    (unsigned long)usb_debug.fault_hfsr, (unsigned long)usb_debug.fault_mmfar,
                                    (unsigned long)usb_debug.fault_bfar);
        for (int i = 0; i < length; i++) {
            usb_debug.reply[2*i] = HEX[uint8_t(status[i]) >> 4];
            usb_debug.reply[2*i+1] = HEX[status[i] & 15];
        }
        usb_debug.reply[2*length] = 0;
    } else if (strcmp(usb_debug.packet, "qRcmd,7265736574") == 0 || strcmp(usb_debug.packet, "k") == 0) {
        NVIC_SystemReset();
    }
}

static void send_reply()
{
    uint8_t sum = 0;
    uint32_t length = 1;
    usb_debug.tx_frame[0] = '$';
    for (const char *p = usb_debug.reply; *p; p++) {
        sum += uint8_t(*p);
        usb_debug.tx_frame[length++] = *p;
    }
    usb_debug.tx_frame[length++] = '#';
    usb_debug.tx_frame[length++] = HEX[sum >> 4];
    usb_debug.tx_frame[length++] = HEX[sum & 15];
    send_bytes(usb_debug.tx_frame, length);
}

static void remote_loop()
{
    while (!usb_debug.resume_requested && !usb_debug.detach_requested) {
        int c = receive_byte();
        if (c == '-') {
            send_reply();
            continue;
        }
        if (c == 3) {
            stop_reply();
            send_reply();
            continue;
        }
        if (c != '$') {
            continue;
        }
        uint8_t sum = 0;
        uint32_t count = 0;
        bool escaped = false;
        bool overflow = false;
        while (true) {
            c = receive_byte();
            if (c < 0) {
                break;
            }
            if (c == '#' && !escaped) {
                break;
            }
            sum += c;
            if (c == '}' && !escaped) {
                escaped = true;
                continue;
            }
            if (escaped) {
                c ^= 0x20;
                escaped = false;
            }
            if (count < PACKET_SIZE) {
                usb_debug.packet[count++] = c;
            } else {
                overflow = true;
            }
        }
        if (c < 0) {
            continue;
        }
        const char checksum[2] = {char(receive_byte()), char(receive_byte())};
        uint8_t received_sum;
        if (!hex_twochars_to_uint8(checksum, received_sum) || received_sum != sum || overflow) {
            send_bytes((const uint8_t *)"-", 1);
            continue;
        }
        usb_debug.packet[count] = 0;
        usb_debug.session_active = true;
        if (!send_bytes((const uint8_t *)"+", 1)) {
            continue;
        }
        command(count);
        if (!usb_debug.resume_requested) {
            send_reply();
        }
    }
}
}


namespace {
// Normal UART workers are locked before acquiring the raw controller. Keep
// the PHY configuration, but discard ChibiOS transfers with one re-enumeration.
static void start_transport()
{
    // Default CDC line coding is 115200 8N1 on both interfaces.
    static const uint8_t default_line_coding[7] = {0x00, 0xc2, 0x01, 0, 0, 0, 8};
    for (auto &coding : usb_debug.line_coding) {
        memcpy(coding, default_line_coding, sizeof(coding));
    }
    usb_debug.otg = USBD1.otg;
    usb_debug.saved_gusbcfg = usb_debug.otg->GUSBCFG;
    usb_debug.saved_gccfg = usb_debug.otg->GCCFG;
    usbDisconnectBus(&USBD1);
    chSysLock();
    sduSuspendHookI(&SDU1);
    sduSuspendHookI(&SDU2);
    chSysUnlock();
    usb_debug.saved_usb_priority = NVIC_GetPriority(USB_IRQ);
    usbStop(&USBD1);
    usb_debug.saved_vtor = SCB->VTOR;
    memcpy(usb_debug.vectors, (const void *)usb_debug.saved_vtor, sizeof(usb_debug.vectors));
    usb_debug.vectors[16 + STM32_OTG1_NUMBER] = uint32_t(&USB_Debug_Handler);
    // The IRQ stays disabled until the transport and monitor are both ready.
    // Vector fetches must see the table even on boards with cacheable BSS.
    SCB_CleanDCache_by_Addr(usb_debug.vectors, sizeof(usb_debug.vectors));
    __DSB();
    SCB->VTOR = uint32_t(usb_debug.vectors);
    __DSB();
    __ISB();
    NVIC_SetPriority(USB_IRQ, 0);
    rccEnableOTG_FS(true);
    NVIC_ClearPendingIRQ(IRQn_Type(STM32_OTG1_NUMBER));
    usb_debug.otg->PCGCCTL = 0;
    usb_debug.otg->DCTL |= DCTL_SDIS;
    delay_cycles(STM32_SYS_D1CPRE_CK/10U);
    usb_debug.otg->GAHBCFG = 0;
    wait_bits(usb_debug.otg->GRSTCTL, GRSTCTL_AHBIDL, GRSTCTL_AHBIDL);
    usb_debug.otg->GRSTCTL = GRSTCTL_CSRST;
    wait_bits(usb_debug.otg->GRSTCTL, GRSTCTL_CSRST, 0);
    delay_cycles(18);
    wait_bits(usb_debug.otg->GRSTCTL, GRSTCTL_AHBIDL, GRSTCTL_AHBIDL);
    usb_debug.otg->GUSBCFG = usb_debug.saved_gusbcfg;
    usb_debug.otg->GCCFG = usb_debug.saved_gccfg;
    delay_cycles(STM32_SYS_D1CPRE_CK/20U);
    usb_debug.otg->GINTMSK = 0;
    usb_debug.otg->GINTSTS = 0xffffffff;
    reset_endpoints();
    usb_debug.otg->DCTL &= ~DCTL_SDIS;
}

static void running_poll()
{
    // One bounded pass per USB IRQ. SOF also drains queued GCS output when
    // no transfers are arriving. Only Ctrl-C is meaningful while running;
    // all-stop GDB sends ordinary requests after the resulting stop reply.
    usb_poll();
    while (usb_debug.rx_pos < usb_debug.rx_len) {
        if (usb_debug.rx[usb_debug.rx_pos++] == 3) {
            usb_debug.interrupt_requested = true;
        }
    }
    if (usb_debug.configured && usb_debug.out_complete) {
        usb_debug.rx_len = usb_debug.rx_pos = 0;
        usb_debug.out_complete = false;
        arm_out(DEBUG_EP);
    }
}
}

extern "C" void usb_debug_exception()
{
    usb_debug.fault_enabled = 0;
    // Comparators stay disabled throughout all debugger code, including helpers.
    fp_control = 2;
    if (!usb_debug.active) {
        return;
    }
    // Preserve a match sampled by the USB IRQ until the pending DebugMonitor runs.
    for (uint8_t i = 0; i < usb_debug.watchpoint_count; i++) {
        if (dwt_comparator(i)[2] & (1U << 24)) {
            usb_debug.watch_type = usb_debug.watchpoints[i].type;
            usb_debug.watch_address = usb_debug.watchpoints[i].address;
        }
    }
    program_watchpoints(false);
    const uint32_t exception = __get_IPSR();
    const bool fault = exception >= 3 && exception <= 6;
    const bool usb_irq = exception == 16U + STM32_OTG1_NUMBER;
    if (usb_irq && usb_debug.initial_stop) {
        // A level-triggered USB source must be serviced even when initial
        // attachment has to wait for an interrupted kernel ISR to return.
        usb_poll();
    } else if (usb_irq) {
        running_poll();
        if (!usb_debug.interrupt_requested) {
            usb_debug.fault_enabled = 1;
            program_watchpoints(true);
            fp_control = 3;
            return;
        }
    }
    const uint32_t exc_return = usb_debug.context[10];
    auto *frame = (uint32_t *)usb_debug.context[(exc_return & 4) ? 9 : 8];
    memcpy(usb_debug.registers, frame, 4*sizeof(uint32_t));
    memcpy(usb_debug.registers+4, usb_debug.context, 8*sizeof(uint32_t));
    usb_debug.registers[12] = frame[4];
    usb_debug.registers[13] = uint32_t(frame) + ((exc_return & 16) ? 32 : 104) + ((frame[7] & 512) ? 4 : 0);
    usb_debug.registers[14] = frame[5];
    usb_debug.registers[15] = frame[6];
    usb_debug.registers[16] = frame[7];
    if (!(exc_return & 16) && usb_debug.fp_available) {
        // Exception entry can reset the live FPSCR for handler use. VMRS in
        // the entry shim completes lazy stacking; the interrupted low FP
        // registers and FPSCR must then come from that architectural frame.
        memcpy(usb_debug.fp_registers, frame+8, 16*sizeof(uint32_t));
        usb_debug.fp_registers[32] = frame[24];
    }
    memcpy(usb_debug.registers+17, usb_debug.fp_registers, sizeof(usb_debug.fp_registers));

    const uint32_t dfsr = SCB->DFSR;
    SCB->DFSR = dfsr;
    // Continue from a breakpoint first executes one instruction with its comparator
    // disabled. Restore the interrupt mask and comparators before normal execution.
    if (!usb_irq && usb_debug.step_requested) {
        CoreDebug->DEMCR &= ~CoreDebug_DEMCR_MON_STEP_Msk;
        usb_debug.context[12] = usb_debug.step_basepri;
        usb_debug.step_requested = false;
        if (usb_debug.step_ram >= 0) {
            if (!patch_ram_breakpoint(usb_debug.step_ram, true)) {
                usb_debug.step_report = true;
            }
            usb_debug.step_ram = -1;
        }
        usb_debug.step_over = -1;
        program_breakpoints();
        NVIC_EnableIRQ(USB_IRQ);
        if (!fault && !usb_debug.step_report && (dfsr & SCB_DFSR_HALTED_Msk) && !(dfsr & (SCB_DFSR_BKPT_Msk | SCB_DFSR_DWTTRAP_Msk))) {
            usb_debug.fault_enabled = 1;
            program_watchpoints(true);
            fp_control = 3;
            return;
        }
    }
    // Initial attach should expose a stable thread registry, not a kernel ISR.
    if (!fault && usb_debug.initial_stop && ((frame[7] & 0x1ffU) || usb_debug.context[12])) {
        usb_debug.fault_enabled = 1;
        program_watchpoints(true);
        fp_control = 3;
        return;
    }
    usb_debug.stopped = true;
    usb_debug.startup_cycles = 0;
    usb_debug.last_cycle = DWT->CYCCNT;
    if (!(dfsr & SCB_DFSR_DWTTRAP_Msk)) {
        usb_debug.watch_type = 0;
    }
    usb_debug.fault_exception = fault ? exception : 0;
    usb_debug.fault_cfsr = fault ? SCB->CFSR : 0;
    usb_debug.fault_hfsr = fault ? SCB->HFSR : 0;
    usb_debug.fault_mmfar = fault ? SCB->MMFAR : 0;
    usb_debug.fault_bfar = fault ? SCB->BFAR : 0;
    usb_debug.stop_signal = fault ? ((SCB->CFSR & 0xffff0000U) ? 4 : 11) : usb_debug.interrupt_requested ? 2 : 5;
    uint16_t instruction = 0;
    usb_debug.software_breakpoint = !usb_irq && (dfsr & SCB_DFSR_BKPT_Msk) &&
        probe_copy(&instruction, frame[6], sizeof(instruction)) && (instruction & 0xff00U) == 0xbe00U;
    for (const auto &bp : usb_debug.ram_breakpoints) {
        if (bp.address && bp.address == frame[6]) {
            usb_debug.software_breakpoint = false;
        }
    }
    usb_debug.interrupt_requested = false;
    usb_debug.initial_stop = false;
    usb_debug.resume_requested = false;
    snapshot_threads();
    if (usb_debug.session_active) {
        stop_reply();
        send_reply();
    }
    remote_loop();
    usb_debug.watch_type = 0;
    // No kernel calls or timekeeping updates are safe in this handler.
    static_cast<ChibiOS::Scheduler *>(hal.scheduler)->usb_debug_resume();
    usb_debug.stopped = false;
    if (usb_debug.detach_requested) {
        NVIC_DisableIRQ(USB_IRQ);
        CoreDebug->DEMCR = usb_debug.saved_demcr;
        for (uint32_t i = 0; i < usb_debug.breakpoint_count; i++) {
            fp_comparator[i] = 0;
        }
        return;
    }
    usb_debug.fault_enabled = 1;
    program_breakpoints();
    program_watchpoints(true);
    fp_control = 3;
}

bool ChibiOS::usb_debug_active()
{
    return usb_debug.active;
}

bool ChibiOS::usb_debug_configured()
{
    return usb_debug.active && usb_debug.configured;
}

uint32_t ChibiOS::usb_debug_gcs_read(uint8_t *data, uint32_t size)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t count = usb_debug.configured ? usb_debug.gcs_rx.read(data, size) : 0;
    __set_PRIMASK(primask);
    return count;
}

uint32_t ChibiOS::usb_debug_gcs_write(const uint8_t *data, uint32_t size)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t count = usb_debug.configured ? usb_debug.gcs_tx.write(data, size) : 0;
    __set_PRIMASK(primask);
    return count;
}

void ChibiOS::usb_debug_startup_wait()
{
#if AP_USB_DEBUG_STARTUP_WAIT_ENABLED
    // USB and the scheduler are already running, but vehicle setup has not
    // started. The usual launcher trigger attaches here; continue leaves the
    // initial stop and lets setup proceed. Keep the watchdog alive while waiting.
    while (!usb_debug.active || usb_debug.initial_stop) {
        usb_debug_poll();
        stm32_watchdog_pat();
        chThdSleepMilliseconds(1);
    }
#endif
}

void ChibiOS::usb_debug_poll()
{
    if (usb_debug.active) {
        if (!usb_debug.detach_requested) {
            return;
        }
        UARTDriver::usb_debug_lock(true);
        usb_debug.otg->DCTL |= DCTL_SDIS;
        chThdSleepMilliseconds(100);
        sduStop(&SDU1);
        sduStop(&SDU2);
        // Detach disabled our IRQ before leaving the monitor. Restore the
        // original table before usbStart() re-enables the ChibiOS USB ISR.
        NVIC_DisableIRQ(USB_IRQ);
        NVIC_ClearPendingIRQ(USB_IRQ);
        SCB->VTOR = usb_debug.saved_vtor;
        __DSB();
        __ISB();
        NVIC_SetPriority(USB_IRQ, usb_debug.saved_usb_priority);
        sduStart(&SDU1, &serusbcfg1);
        sduStart(&SDU2, &serusbcfg2);
        usbStart(&USBD1, &usbcfg);
        usbConnectBus(&USBD1);
        NVIC_SetPriority(DebugMonitor_IRQn, usb_debug.saved_debug_priority);
        usb_debug.active = false;
        UARTDriver::usb_debug_lock(false);
        return;
    }
    if (USBD1.state != USB_ACTIVE || __get_PRIMASK() || __get_BASEPRI() || __get_FAULTMASK()) {
        return;
    }
    const char trigger[] = "APUSBDBG\n";
    uint8_t byte;
    while (chnReadTimeout(&SDU2, &byte, 1, TIME_IMMEDIATE) == 1) {
        usb_debug.matched = byte == trigger[usb_debug.matched] ? usb_debug.matched+1 : (byte == trigger[0] ? 1 : 0);
        if (usb_debug.matched != sizeof(trigger)-1) {
            continue;
        }
        usb_debug.matched = 0;
        if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
            continue;
        }
        UARTDriver::usb_debug_lock(true);
        start_transport();
        usb_debug.resume_requested = usb_debug.detach_requested = usb_debug.session_active = usb_debug.step_requested = false;
        usb_debug.interrupt_requested = false;
        usb_debug.initial_stop = true;
        usb_debug.step_over = usb_debug.step_ram = -1;
        memset(usb_debug.ram_breakpoints, 0, sizeof(usb_debug.ram_breakpoints));
        usb_debug.saved_demcr = CoreDebug->DEMCR;
        usb_debug.saved_debug_priority = NVIC_GetPriority(DebugMonitor_IRQn);
        NVIC_SetPriority(DebugMonitor_IRQn, 0);
        usb_debug.breakpoint_count = smaller(((fp_control >> 4) & 15U) | ((fp_control >> 8) & 0x70U), 8);
        usb_debug.watchpoint_count = smaller(DWT->CTRL >> 28, ARRAY_SIZE(usb_debug.watchpoints));
        memset(usb_debug.watchpoints, 0, sizeof(usb_debug.watchpoints));
        program_watchpoints(false);
        memset(usb_debug.breakpoint_address, 0, sizeof(usb_debug.breakpoint_address));
        program_breakpoints();
        CoreDebug->DEMCR = (usb_debug.saved_demcr & ~(CoreDebug_DEMCR_MON_STEP_Msk | CoreDebug_DEMCR_MON_PEND_Msk)) |
            CoreDebug_DEMCR_MON_EN_Msk;
        usb_debug.fault_enabled = 1;
        usb_debug.active = true;
        UARTDriver::usb_debug_lock(false);
        usb_debug.otg->GINTMSK = GINTMSK_SOFM | GINTMSK_USBRSTM | GINTMSK_ENUMDNEM |
                                 GINTMSK_RXFLVLM | GINTMSK_IEPM | GINTMSK_OEPM;
        usb_debug.otg->GAHBCFG = GAHBCFG_GINTMSK;
        NVIC_EnableIRQ(USB_IRQ);
        break;
    }
}
#endif // AP_USB_DEBUG_ENABLED

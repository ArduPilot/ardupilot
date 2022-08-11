/*
  ArduPilot bootloader protocol

  based on bl.c from https://github.com/PX4/Bootloader.

  Ported to ChibiOS for ArduPilot by Andrew Tridgell
 */

/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include "ch.h"
#include "hal.h"
#include "hwdef.h"

#include "bl_protocol.h"
#include "support.h"
#include "can.h"
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#if EXT_FLASH_SIZE_MB
#include <AP_FlashIface/AP_FlashIface_JEDEC.h>
#endif
#include <AP_CheckFirmware/AP_CheckFirmware.h>

// #pragma GCC optimize("O0")


// bootloader flash update protocol.
//
// Command format:
//
//      <opcode>[<command_data>]<EOC>
//
// Reply format:
//
//      [<reply_data>]<INSYNC><status>
//
// The <opcode> and <status> values come from the PROTO_ defines below,
// the <*_data> fields is described only for opcodes that transfer data;
// in all other cases the field is omitted.
//
// Expected workflow (protocol 3) is:
//
// GET_SYNC		verify that the board is present
// GET_DEVICE		determine which board (select firmware to upload)
// CHIP_ERASE		erase the program area and reset address counter
// loop:
//      PROG_MULTI      program bytes
// GET_CRC		verify CRC of entire flashable area
// RESET		finalise flash programming, reset chip and starts application
//

#define BL_PROTOCOL_VERSION 		5		// The revision of the bootloader protocol
// protocol bytes
#define PROTO_INSYNC				0x12    // 'in sync' byte sent before status
#define PROTO_EOC					0x20    // end of command

// Reply bytes
#define PROTO_OK					0x10    // INSYNC/OK      - 'ok' response
#define PROTO_FAILED				0x11    // INSYNC/FAILED  - 'fail' response
#define PROTO_INVALID				0x13	// INSYNC/INVALID - 'invalid' response for bad commands
#define PROTO_BAD_SILICON_REV 		0x14 	// On the F4 series there is an issue with < Rev 3 silicon
// see https://pixhawk.org/help/errata
// Command bytes
#define PROTO_GET_SYNC				0x21    // NOP for re-establishing sync
#define PROTO_GET_DEVICE			0x22    // get device ID bytes
#define PROTO_CHIP_ERASE			0x23    // erase program area and reset program address
#define PROTO_PROG_MULTI			0x27    // write bytes at program address and increment
#define PROTO_READ_MULTI			0x28    // read bytes at address and increment
#define PROTO_GET_CRC				0x29	// compute & return a CRC
#define PROTO_GET_OTP				0x2a	// read a byte from OTP at the given address
#define PROTO_GET_SN				0x2b    // read a word from UDID area ( Serial)  at the given address
#define PROTO_GET_CHIP				0x2c    // read chip version (MCU IDCODE)
#define PROTO_SET_DELAY				0x2d    // set minimum boot delay
#define PROTO_GET_CHIP_DES			0x2e    // read chip version In ASCII
#define PROTO_BOOT					0x30    // boot the application
#define PROTO_DEBUG					0x31    // emit debug information - format not defined
#define PROTO_SET_BAUD				0x33    // baud rate on uart

// External Flash programming 
#define PROTO_EXTF_ERASE            0x34	// Erase sectors from external flash
#define PROTO_EXTF_PROG_MULTI       0x35    // write bytes at external flash program address and increment
#define PROTO_EXTF_READ_MULTI       0x36    // read bytes at address and increment
#define PROTO_EXTF_GET_CRC          0x37	// compute & return a CRC of data in external flash

#define PROTO_PROG_MULTI_MAX    64	// maximum PROG_MULTI size
#define PROTO_READ_MULTI_MAX    255	// size of the size field

/* argument values for PROTO_GET_DEVICE */
#define PROTO_DEVICE_BL_REV	1	// bootloader revision
#define PROTO_DEVICE_BOARD_ID	2	// board ID
#define PROTO_DEVICE_BOARD_REV	3	// board revision
#define PROTO_DEVICE_FW_SIZE	4	// size of flashable area
#define PROTO_DEVICE_VEC_AREA	5	// contents of reserved vectors 7-10
#define PROTO_DEVICE_EXTF_SIZE  6   // size of available external flash
// all except PROTO_DEVICE_VEC_AREA and PROTO_DEVICE_BOARD_REV should be done
#define CHECK_GET_DEVICE_FINISHED(x)   ((x & (0xB)) == 0xB)

// interrupt vector table for STM32
#define SCB_VTOR 0xE000ED08

static virtual_timer_t systick_vt;

/*
  millisecond timer array
 */
#define NTIMERS		    2
#define TIMER_BL_WAIT	0
#define TIMER_LED	    1

static enum led_state {LED_BLINK, LED_ON, LED_OFF} led_state;

volatile unsigned timer[NTIMERS];

// keep back 32 bytes at the front of flash. This is long enough to allow for aligned
// access on STM32H7
#define RESERVE_LEAD_WORDS 8


#if EXT_FLASH_SIZE_MB
extern AP_FlashIface_JEDEC ext_flash;
#endif

#ifndef BOOT_FROM_EXT_FLASH
#define BOOT_FROM_EXT_FLASH 0
#endif

/*
  1ms timer tick callback
 */
static void sys_tick_handler(void *ctx)
{
    chSysLockFromISR();
    chVTSetI(&systick_vt, chTimeMS2I(1), sys_tick_handler, nullptr);
    chSysUnlockFromISR();
    uint8_t i;
    for (i = 0; i < NTIMERS; i++)
        if (timer[i] > 0) {
            timer[i]--;
        }

    if ((led_state == LED_BLINK) && (timer[TIMER_LED] == 0)) {
        led_toggle(LED_BOOTLOADER);
        timer[TIMER_LED] = 50;
    }
}

static void delay(unsigned msec)
{
    chThdSleep(chTimeMS2I(msec));
}

static void
led_set(enum led_state state)
{
    led_state = state;

    switch (state) {
    case LED_OFF:
        led_off(LED_BOOTLOADER);
        break;

    case LED_ON:
        led_on(LED_BOOTLOADER);
        break;

    case LED_BLINK:
        /* restart the blink state machine ASAP */
        timer[TIMER_LED] = 0;
        break;
    }
}

static void
do_jump(uint32_t stacktop, uint32_t entrypoint)
{
#if defined(STM32F7) || defined(STM32H7)
    // disable caches on F7 before starting program
    __DSB();
    __ISB();
    SCB_DisableDCache();
    SCB_DisableICache();
#endif

    chSysLock();    

    // we set sp as well as msp to avoid an issue with loading NuttX
    asm volatile(
        "mov sp, %0	\n"
        "msr msp, %0	\n"
        "bx	%1	\n"
        : : "r"(stacktop), "r"(entrypoint) :);
}

#ifndef APP_START_ADDRESS
#define APP_START_ADDRESS (FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)*1024U)
#endif

void
jump_to_app()
{
    const uint32_t *app_base = (const uint32_t *)(APP_START_ADDRESS);

#if AP_CHECK_FIRMWARE_ENABLED
    const auto ok = check_good_firmware();
    if (ok != check_fw_result_t::CHECK_FW_OK) {
        // bad firmware, don't try and boot
        return;
    }
#endif
    
    // If we have QSPI chip start it
#if EXT_FLASH_SIZE_MB
    uint8_t* ext_flash_start_addr;
    if (!ext_flash.start_xip_mode((void**)&ext_flash_start_addr)) {
        return;
    }
#endif
    /*
     * We hold back the programming of the lead words until the upload
     * is marked complete by the host. So if they are not 0xffffffff,
     * we should try booting it.
     */
    for (uint8_t i=0; i<RESERVE_LEAD_WORDS; i++) {
        if (app_base[i] == 0xffffffff) {
            goto exit;
        }
    }

    /*
     * The second word of the app is the entrypoint; it must point within the
     * flash area (or we have a bad flash).
     */
    if (app_base[1] < APP_START_ADDRESS) {
        goto exit;
    }

#if BOOT_FROM_EXT_FLASH
    if (app_base[1] >= (APP_START_ADDRESS + board_info.extf_size)) {
        goto exit;
    }
#else
    if (app_base[1] >= (APP_START_ADDRESS + board_info.fw_size)) {
        goto exit;
    }
#endif

#if HAL_USE_CAN == TRUE ||  HAL_NUM_CAN_IFACES
    // for CAN firmware we start the watchdog before we run the
    // application code, to ensure we catch a bad firmare. If we get a
    // watchdog reset and the firmware hasn't changed the RTC flag to
    // indicate that it has been running OK for 30s then we will stay
    // in bootloader
#ifndef DISABLE_WATCHDOG
    stm32_watchdog_init();
#endif
    stm32_watchdog_pat();
#endif

    flash_set_keep_unlocked(false);
    
    led_set(LED_OFF);

    // resetting the clocks is needed for loading NuttX
#if defined(STM32H7)
    rccDisableAPB1L(~0);
    rccDisableAPB1H(~0);
#elif defined(STM32G4)
    rccDisableAPB1R1(~0);
    rccDisableAPB1R2(~0);
#elif defined(STM32L4)
    rccDisableAPB1R1(~0);
    rccDisableAPB1R2(~0);
#else
    rccDisableAPB1(~0);
#endif
    rccDisableAPB2(~0);
#if HAL_USE_SERIAL_USB == TRUE    
    rccResetOTG_FS();
#if defined(rccResetOTG_HS)
    rccResetOTG_HS();
#endif
#endif
    
    // disable all interrupt sources
    port_disable();

    /* switch exception handlers to the application */
    *(volatile uint32_t *)SCB_VTOR = APP_START_ADDRESS;

    /* extract the stack and entrypoint from the app vector table and go */
    do_jump(app_base[0], app_base[1]);
exit:
#if EXT_FLASH_SIZE_MB
    ext_flash.stop_xip_mode();
#endif
    return;
}

static void
sync_response(void)
{
    uint8_t data[] = {
        PROTO_INSYNC,	// "in sync"
        PROTO_OK	// "OK"
    };

    cout(data, sizeof(data));
}

static void
invalid_response(void)
{
    uint8_t data[] = {
        PROTO_INSYNC,	// "in sync"
        PROTO_INVALID	// "invalid command"
    };

    cout(data, sizeof(data));
}

static void
failure_response(void)
{
    uint8_t data[] = {
        PROTO_INSYNC,	// "in sync"
        PROTO_FAILED	// "command failed"
    };

    cout(data, sizeof(data));
}

/**
 * Function to wait for EOC
 *
 * @param timeout length of time in ms to wait for the EOC to be received
 * @return true if the EOC is returned within the timeout perio, else false
 */
inline static bool
wait_for_eoc(unsigned timeout)
{
    return cin(timeout) == PROTO_EOC;
}

static void
cout_word(uint32_t val)
{
    cout((uint8_t *)&val, 4);
}

#define TEST_FLASH 0

#if TEST_FLASH
static void test_flash()
{
    uint32_t loop = 1;
    bool init_done = false;
    while (true) {
        uint32_t addr = 0;
        uint32_t page = 0;
        while (true) {
            uint32_t v[8];
            for (uint8_t i=0; i<8; i++) {
                v[i] = (page<<16) + loop;
            }
            if (flash_func_sector_size(page) == 0) {
                continue;
            }
            uint32_t num_writes = flash_func_sector_size(page) / sizeof(v);
            uprintf("page %u size %u addr=0x%08x v=0x%08x\n",
                    page, flash_func_sector_size(page), addr, v[0]); delay(10);
            if (init_done) {
                for (uint32_t j=0; j<flash_func_sector_size(page)/4; j++) {
                    uint32_t v1 = (page<<16) + (loop-1);
                    uint32_t v2 = flash_func_read_word(addr+j*4);
                    if (v2 != v1) {
                        uprintf("read error at 0x%08x v=0x%08x v2=0x%08x\n", addr+j*4, v1, v2);
                        break;
                    }
                }
            }
            if (!flash_func_erase_sector(page)) {
                uprintf("erase of %u failed\n", page);
            }
            for (uint32_t j=0; j<num_writes; j++) {
                if (!flash_func_write_words(addr+j*sizeof(v), v, ARRAY_SIZE(v))) {
                    uprintf("write failed at 0x%08x\n", addr+j*sizeof(v));
                    break;
                }
            }
            addr += flash_func_sector_size(page);
            page++;
            if (flash_func_sector_size(page) == 0) {
                break;
            }
        }
        init_done = true;
        delay(1000);
        loop++;
    }
}
#endif

void
bootloader(unsigned timeout)
{
#if TEST_FLASH
    test_flash();
#endif

    uint32_t	address = board_info.fw_size;	/* force erase before upload will work */
#if EXT_FLASH_SIZE_MB
    uint32_t	extf_address = board_info.extf_size;	/* force erase before upload will work */
#endif
    uint32_t	read_address = 0;
    uint32_t	first_words[RESERVE_LEAD_WORDS];
    bool done_sync = false;
    uint8_t done_get_device_flags = 0;
    bool done_erase = false;
    static bool done_timer_init;
    unsigned original_timeout = timeout;

    memset(first_words, 0xFF, sizeof(first_words));

    if (!done_timer_init) {
        done_timer_init = true;
        chVTObjectInit(&systick_vt);
        chVTSet(&systick_vt, chTimeMS2I(1), sys_tick_handler, nullptr);
    }

    /* if we are working with a timeout, start it running */
    if (timeout) {
        timer[TIMER_BL_WAIT] = timeout;
    }

    /* make the LED blink while we are idle */
    led_set(LED_BLINK);

    while (true) {
        volatile int c;
        int arg;
        static union {
            uint8_t		c[256];
            uint32_t	w[64];
        } flash_buffer;

        // Wait for a command byte
        led_off(LED_ACTIVITY);

        do {
            /* if we have a timeout and the timer has expired, return now */
            if (timeout && !timer[TIMER_BL_WAIT]) {
                return;
            }

            /* try to get a byte from the host */
            c = cin(0);
#if HAL_USE_CAN == TRUE || HAL_NUM_CAN_IFACES
            if (c < 0) {
                can_update();
            }
#endif
        } while (c < 0);

        led_on(LED_ACTIVITY);

        // handle the command byte
        switch (c) {

        // sync
        //
        // command:		GET_SYNC/EOC
        // reply:		INSYNC/OK
        //
        case PROTO_GET_SYNC:

            /* expect EOC */
            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }
            done_sync = true;
            break;

        // get device info
        //
        // command:		GET_DEVICE/<arg:1>/EOC
        // BL_REV reply:	<revision:4>/INSYNC/EOC
        // BOARD_ID reply:	<board type:4>/INSYNC/EOC
        // BOARD_REV reply:	<board rev:4>/INSYNC/EOC
        // FW_SIZE reply:	<firmware size:4>/INSYNC/EOC
        // VEC_AREA reply	<vectors 7-10:16>/INSYNC/EOC
        // bad arg reply:	INSYNC/INVALID
        //
        case PROTO_GET_DEVICE:
            /* expect arg then EOC */
            arg = cin(1000);

            if (arg < 0) {
                goto cmd_bad;
            }

            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }

            // reset read pointer
            read_address = 0;

            switch (arg) {
            case PROTO_DEVICE_BL_REV: {
                uint32_t bl_proto_rev = BL_PROTOCOL_VERSION;
                cout((uint8_t *)&bl_proto_rev, sizeof(bl_proto_rev));
                break;
            }

            case PROTO_DEVICE_BOARD_ID:
                cout((uint8_t *)&board_info.board_type, sizeof(board_info.board_type));
                break;

            case PROTO_DEVICE_BOARD_REV:
                cout((uint8_t *)&board_info.board_rev, sizeof(board_info.board_rev));
                break;

            case PROTO_DEVICE_FW_SIZE:
                cout((uint8_t *)&board_info.fw_size, sizeof(board_info.fw_size));
                break;

            case PROTO_DEVICE_VEC_AREA:
                for (unsigned p = 7; p <= 10; p++) {
                    uint32_t bytes = flash_func_read_word(p * 4);

                    cout((uint8_t *)&bytes, sizeof(bytes));
                }

                break;

            case PROTO_DEVICE_EXTF_SIZE:
                cout((uint8_t *)&board_info.extf_size, sizeof(board_info.extf_size));
                break;

            default:
                goto cmd_bad;
            }
            done_get_device_flags |= (1<<(arg-1)); // set the flags for use when resetting timeout 
            break;

        // erase and prepare for programming
        //
        // command:		ERASE/EOC
        // success reply:	INSYNC/OK
        // erase failure:	INSYNC/FAILURE
        //
        case PROTO_CHIP_ERASE:

            if (!done_sync || !CHECK_GET_DEVICE_FINISHED(done_get_device_flags)) {
                // lower chance of random data on a uart triggering erase
                goto cmd_bad;
            }

            /* expect EOC */
            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }

            // once erase is done there is no going back, set timeout
            // to zero
            done_erase = true;
            timeout = 0;
            
            flash_set_keep_unlocked(true);

            // clear the bootloader LED while erasing - it stops blinking at random
            // and that's confusing
            led_set(LED_OFF);

            // erase all sectors
            for (uint8_t i = 0; flash_func_sector_size(i) != 0; i++) {
                if (!flash_func_erase_sector(i)) {
                    goto cmd_fail;
                }
            }

            // enable the LED while verifying the erase
            led_set(LED_ON);

            // verify the erase
            for (address = 0; address < board_info.fw_size; address += 4) {
                if (flash_func_read_word(address) != 0xffffffff) {
                    goto cmd_fail;
                }
            }

            address = 0;

            // resume blinking
            led_set(LED_BLINK);
            break;

        // program data from start of the flash
        //
        // command:		EXTF_ERASE/<len:4>/EOC
        // success reply:	INSYNC/OK
        // invalid reply:	INSYNC/INVALID
        // readback failure:	INSYNC/FAILURE
        //
        case PROTO_EXTF_ERASE:
#if EXT_FLASH_SIZE_MB
        {
            if (!done_sync || !CHECK_GET_DEVICE_FINISHED(done_get_device_flags)) {
                // lower chance of random data on a uart triggering erase
                goto cmd_bad;
            }
            uint32_t cmd_erase_bytes;
            if (cin_word(&cmd_erase_bytes, 100)) {
                goto cmd_bad;
            }

            // expect EOC
            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }
            uint32_t erased_bytes = 0;
            uint32_t sector_number = EXT_FLASH_RESERVE_START_KB * 1024 / ext_flash.get_sector_size();
            uint8_t pct_done = 0;
            if (cmd_erase_bytes > (ext_flash.get_sector_size() * ext_flash.get_sector_count())) {
                uprintf("Requested to erase more than we can\n");
                goto cmd_bad;
            }
            uprintf("Erase Command Received\n");
            sync_response();
            cout(&pct_done, sizeof(pct_done));
            // Flash all sectors that encompass the erase_bytes
            while (erased_bytes < cmd_erase_bytes) {
                uint32_t delay_ms = 0, timeout_ms = 0;
                if (!ext_flash.start_sector_erase(sector_number, delay_ms, timeout_ms)) {
                    goto cmd_fail;
                }
                uint32_t next_check_ms = AP_HAL::millis() + delay_ms;
                while (true) {
                    cout(&pct_done, sizeof(pct_done));
                    if (AP_HAL::millis() > next_check_ms) {
                        if (!ext_flash.is_device_busy()) {
                            pct_done = erased_bytes*100/cmd_erase_bytes;
                            uprintf("PCT DONE: %d\n", pct_done);
                            break;
                        }
                        if ((AP_HAL::millis() + timeout_ms) > next_check_ms) {
                            // We are out of time, return error
                            goto cmd_fail;
                        }
                        next_check_ms = AP_HAL::millis()+delay_ms;
                    }
                    chThdSleep(chTimeMS2I(delay_ms));
                }
                erased_bytes += ext_flash.get_sector_size();
                sector_number++;
            }
            pct_done = 100;
            extf_address = 0;
            cout(&pct_done, sizeof(pct_done));
        }
#else
            goto cmd_bad;
#endif // EXT_FLASH_SIZE_MB
            break;

        // program bytes at current external flash address
        //
        // command:		PROG_MULTI/<len:1>/<data:len>/EOC
        // success reply:	INSYNC/OK
        // invalid reply:	INSYNC/INVALID
        // readback failure:	INSYNC/FAILURE
        //
        case PROTO_EXTF_PROG_MULTI:
        {
#if EXT_FLASH_SIZE_MB
            if (!done_sync || !CHECK_GET_DEVICE_FINISHED(done_get_device_flags)) {
                // lower chance of random data on a uart triggering erase
                goto cmd_bad;
            }

            // expect count
            led_set(LED_OFF);

            arg = cin(50);

            if (arg < 0) {
                goto cmd_bad;
            }

            if ((extf_address + arg) > board_info.extf_size) {
                goto cmd_bad;
            }

            if (arg > sizeof(flash_buffer.c)) {
                goto cmd_bad;
            }

            for (int i = 0; i < arg; i++) {
                c = cin(1000);

                if (c < 0) {
                    goto cmd_bad;
                }

                flash_buffer.c[i] = c;
            }

            if (!wait_for_eoc(200)) {
                goto cmd_bad;
            }

            uint32_t offset = 0;
            uint32_t size = arg;
#if BOOT_FROM_EXT_FLASH
            // save the first words and don't program it until everything else is done
            if (extf_address < sizeof(first_words)) {
                uint8_t n = MIN(sizeof(first_words)-extf_address, arg);
                memcpy(&first_words[extf_address/4], &flash_buffer.w[0], n);
                // replace first words with 1 bits we can overwrite later
                memset(&flash_buffer.w[0], 0xFF, n);
            }
#endif
            uint32_t programming;
            uint32_t delay_us = 0, timeout_us = 0;
            uint64_t start_time_us;
            while (true) {
                if (size == 0) {
                    extf_address += arg;
                    break;
                }
                if (!ext_flash.start_program_offset(extf_address+offset+EXT_FLASH_RESERVE_START_KB*1024,
                    &flash_buffer.c[offset], size, programming, delay_us, timeout_us)) {
                    // uprintf("ext flash write command failed\n");
                    goto cmd_fail;
                }
                start_time_us = AP_HAL::micros64();
                // prepare for next run
                offset += programming;
                size -= programming;
                while (true) {
                    if (AP_HAL::micros64() > (start_time_us+delay_us)) {
                        if (!ext_flash.is_device_busy()) {
                            // uprintf("flash program Successful, elapsed %ld us\n", uint32_t(AP_HAL::micros64() - start_time_us));
                            break;
                        } else {
                            // uprintf("Typical flash program time reached, Still Busy?!\n");
                        }
                    }
                }
            }
#endif
            break;
        }

        // program bytes at current address
        //
        // command:		PROG_MULTI/<len:1>/<data:len>/EOC
        // success reply:	INSYNC/OK
        // invalid reply:	INSYNC/INVALID
        // readback failure:	INSYNC/FAILURE
        //
        case PROTO_PROG_MULTI:		// program bytes
            if (!done_sync || !CHECK_GET_DEVICE_FINISHED(done_get_device_flags)) {
                // lower chance of random data on a uart triggering erase
                goto cmd_bad;
            }

            // expect count
            led_set(LED_OFF);

            arg = cin(50);

            if (arg < 0) {
                goto cmd_bad;
            }

            // sanity-check arguments
            if (arg % 4) {
                goto cmd_bad;
            }

            if ((address + arg) > board_info.fw_size) {
                goto cmd_bad;
            }

            if (arg > sizeof(flash_buffer.c)) {
                goto cmd_bad;
            }

            for (int i = 0; i < arg; i++) {
                c = cin(1000);

                if (c < 0) {
                    goto cmd_bad;
                }

                flash_buffer.c[i] = c;
            }

            if (!wait_for_eoc(200)) {
                goto cmd_bad;
            }

            // save the first words and don't program it until everything else is done
#if !BOOT_FROM_EXT_FLASH
            if (address < sizeof(first_words)) {
                uint8_t n = MIN(sizeof(first_words)-address, arg);
                memcpy(&first_words[address/4], &flash_buffer.w[0], n);
                // replace first words with 1 bits we can overwrite later
                memset(&flash_buffer.w[0], 0xFF, n);
            }
#endif
            arg /= 4;
            // program the words
            if (!flash_write_buffer(address, flash_buffer.w, arg)) {
                goto cmd_fail;
            }
            address += arg * 4;
            break;

        // fetch CRC of the entire flash area
        //
        // command:			GET_CRC/EOC
        // reply:			<crc:4>/INSYNC/OK
        //
        case PROTO_GET_CRC: {
            // expect EOC
            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }

            if (!flash_write_flush()) {
                goto cmd_bad;
            }

            // compute CRC of the programmed area
            uint32_t sum = 0;

            for (unsigned p = 0; p < board_info.fw_size; p += 4) {
                uint32_t bytes;

#if !BOOT_FROM_EXT_FLASH
                if (p < sizeof(first_words) && first_words[0] != 0xFFFFFFFF) {
                    bytes = first_words[p/4];
                } else
#endif
                {
                    bytes = flash_func_read_word(p);
                }
                sum = crc32_small(sum, (uint8_t *)&bytes, sizeof(bytes));
            }

            cout_word(sum);
            break;
        }

        // fetch CRC of the external flash area
        //
        // command:		    EXTF_GET_CRC/<len:4>/EOC
        // reply:			<crc:4>/INSYNC/OK
        //
        case PROTO_EXTF_GET_CRC: {
#if EXT_FLASH_SIZE_MB
            // expect EOC
            uint32_t cmd_verify_bytes;
            if (cin_word(&cmd_verify_bytes, 100)) {
                goto cmd_bad;
            }

            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }

            // compute CRC of the programmed area
            uint32_t sum = 0;
            uint8_t rembytes = cmd_verify_bytes % 4;
            for (unsigned p = 0; p < (cmd_verify_bytes-rembytes); p+=4) {
                uint32_t bytes;

#if BOOT_FROM_EXT_FLASH
                if (p < sizeof(first_words) && first_words[0] != 0xFFFFFFFF) {
                    bytes = first_words[p/4];
                } else
#endif
                {
                    ext_flash.read(p+EXT_FLASH_RESERVE_START_KB*1024, (uint8_t *)&bytes, sizeof(bytes));
                }
                sum = crc32_small(sum, (uint8_t *)&bytes, sizeof(bytes));
            }
            if (rembytes) {
                uint8_t bytes[3];
                ext_flash.read(EXT_FLASH_RESERVE_START_KB*1024+cmd_verify_bytes-rembytes, bytes, rembytes);
                sum = crc32_small(sum, bytes, rembytes);
            }
            cout_word(sum);
            break;
#endif
        }

        // read a word from the OTP
        //
        // command:			GET_OTP/<addr:4>/EOC
        // reply:			<value:4>/INSYNC/OK
        case PROTO_GET_OTP:
            // expect argument
        {
            uint32_t index = 0;

            if (cin_word(&index, 100)) {
                goto cmd_bad;
            }

            // expect EOC
            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }

            cout_word(flash_func_read_otp(index));
        }
        break;

        // read the SN from the UDID
        //
        // command:			GET_SN/<addr:4>/EOC
        // reply:			<value:4>/INSYNC/OK
        case PROTO_GET_SN:
            // expect argument
        {
            uint32_t index = 0;

            if (cin_word(&index, 100)) {
                goto cmd_bad;
            }

            // expect EOC
            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }

            cout_word(flash_func_read_sn(index));
        }
        break;

        // read the chip ID code
        //
        // command:			GET_CHIP/EOC
        // reply:			<value:4>/INSYNC/OK
        case PROTO_GET_CHIP: {
            // expect EOC
            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }

            cout_word(get_mcu_id());
        }
        break;

        // read the chip  description
        //
        // command:			GET_CHIP_DES/EOC
        // reply:			<value:4>/INSYNC/OK
        case PROTO_GET_CHIP_DES: {
            uint8_t buffer[MAX_DES_LENGTH];
            unsigned len = MAX_DES_LENGTH;

            // expect EOC
            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }

            len = get_mcu_desc(len, buffer);
            cout_word(len);
            cout(buffer, len);
        }
        break;

#ifdef BOOT_DELAY_ADDRESS

        case PROTO_SET_DELAY: {
            /*
              Allow for the bootloader to setup a
              boot delay signature which tells the
              board to delay for at least a
              specified number of seconds on boot.
             */
            int v = cin(100);

            if (v < 0) {
                goto cmd_bad;
            }

            uint8_t boot_delay = v & 0xFF;

            if (boot_delay > BOOT_DELAY_MAX) {
                goto cmd_bad;
            }

            // expect EOC
            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }

            uint32_t sig1 = flash_func_read_word(BOOT_DELAY_ADDRESS);
            uint32_t sig2 = flash_func_read_word(BOOT_DELAY_ADDRESS + 4);

            if (sig1 != BOOT_DELAY_SIGNATURE1 ||
                sig2 != BOOT_DELAY_SIGNATURE2) {
                goto cmd_bad;
            }

            uint32_t value = (BOOT_DELAY_SIGNATURE1 & 0xFFFFFF00) | boot_delay;
            flash_func_write_word(BOOT_DELAY_ADDRESS, value);

            if (flash_func_read_word(BOOT_DELAY_ADDRESS) != value) {
                goto cmd_fail;
            }
        }
        break;
#endif

        case PROTO_READ_MULTI: {
            arg = cin(50);
            if (arg < 0) {
                goto cmd_bad;
            }
            if (arg % 4) {
                goto cmd_bad;
            }
            if ((read_address + arg) > board_info.fw_size) {
                goto cmd_bad;
            }
            // expect EOC
            if (!wait_for_eoc(2)) {
                goto cmd_bad;
            }
            arg /= 4;

            while (arg-- > 0) {
                cout_word(flash_func_read_word(read_address));
                read_address += 4;
            }
            break;
        }

        // finalise programming and boot the system
        //
        // command:			BOOT/EOC
        // reply:			INSYNC/OK
        //
        case PROTO_BOOT:

            // expect EOC
            if (!wait_for_eoc(1000)) {
                goto cmd_bad;
            }

            if (!flash_write_flush()) {
                goto cmd_fail;
            }

            // program the deferred first word
            if (first_words[0] != 0xffffffff) {
#if !BOOT_FROM_EXT_FLASH
                if (!flash_write_buffer(0, first_words, RESERVE_LEAD_WORDS)) {
                    goto cmd_fail;
                }
#else
                uint32_t programming;
                uint32_t delay_us;
                uint32_t timeout_us;
                if (!ext_flash.start_program_offset(EXT_FLASH_RESERVE_START_KB*1024, (const uint8_t*)first_words, sizeof(first_words), programming, delay_us, timeout_us)) {
                    // uprintf("ext flash write command failed\n");
                    goto cmd_fail;
                }
                uint64_t start_time_us = AP_HAL::micros64();
                while (true) {
                    if (AP_HAL::micros64() > (start_time_us+delay_us)) {
                        if (!ext_flash.is_device_busy()) {
                            // uprintf("flash program Successful, elapsed %ld us\n", uint32_t(AP_HAL::micros64() - start_time_us));
                            break;
                        } else {
                            // uprintf("Typical flash program time reached, Still Busy?!\n");
                        }
                    }
                }
#endif
                // revert in case the flash was bad...
                memset(first_words, 0xff, sizeof(first_words));
            }

            // send a sync and wait for it to be collected
            sync_response();
            delay(100);

            // quiesce and jump to the app
            return;

        case PROTO_DEBUG:
            // XXX reserved for ad-hoc debugging as required
            break;

		case PROTO_SET_BAUD: {
            if (!done_sync || !CHECK_GET_DEVICE_FINISHED(done_get_device_flags)) {
                // prevent timeout going to zero on noise
                goto cmd_bad;
            }
            /* expect arg then EOC */
            uint32_t baud = 0;

            if (cin_word(&baud, 100)) {
                goto cmd_bad;
            }

			if (!wait_for_eoc(2)) {
                goto cmd_bad;
			}

            // send the sync response for this command
            sync_response();

            delay(5);

            // set the baudrate
            port_setbaud(baud);

            lock_bl_port();
            timeout = 0;
            
            // this is different to what every other case in this
            // switch does!  Most go through sync_response down the
            // bottom, but we need to undertake an action after
            // returning the response...
            continue;
        }
            
        default:
            continue;
        }

        // we got a good command on this port, lock to the port
        lock_bl_port();
        
        // once we get both a valid sync and valid get_device then kill
        // the timeout
        if (done_sync && CHECK_GET_DEVICE_FINISHED(done_get_device_flags)) {
            timeout = 0;
        }

        // send the sync response for this command
        sync_response();
        continue;
cmd_bad:
        // if we get a bad command it could be line noise on a
        // uart. Set timeout back to original timeout so we don't get
        // stuck in the bootloader
        if (!done_erase) {
            timeout = original_timeout;
        }
        // send an 'invalid' response but don't kill the timeout - could be garbage
        invalid_response();
        continue;

cmd_fail:
        // send a 'command failed' response but don't kill the timeout - could be garbage
        failure_response();
        continue;
    }
}

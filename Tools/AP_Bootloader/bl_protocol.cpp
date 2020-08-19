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

#define PROTO_PROG_MULTI_MAX    64	// maximum PROG_MULTI size
#define PROTO_READ_MULTI_MAX    255	// size of the size field

/* argument values for PROTO_GET_DEVICE */
#define PROTO_DEVICE_BL_REV	1	// bootloader revision
#define PROTO_DEVICE_BOARD_ID	2	// board ID
#define PROTO_DEVICE_BOARD_REV	3	// board revision
#define PROTO_DEVICE_FW_SIZE	4	// size of flashable area
#define PROTO_DEVICE_VEC_AREA	5	// contents of reserved vectors 7-10

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

#define APP_START_ADDRESS (FLASH_LOAD_ADDRESS + FLASH_BOOTLOADER_LOAD_KB*1024U)

void
jump_to_app()
{
    const uint32_t *app_base = (const uint32_t *)(APP_START_ADDRESS);

    /*
     * We hold back the programming of the lead words until the upload
     * is marked complete by the host. So if they are not 0xffffffff,
     * we should try booting it.
     */
    for (uint8_t i=0; i<RESERVE_LEAD_WORDS; i++) {
        if (app_base[i] == 0xffffffff) {
            return;
        }
    }

    /*
     * The second word of the app is the entrypoint; it must point within the
     * flash area (or we have a bad flash).
     */
    if (app_base[1] < APP_START_ADDRESS) {
        return;
    }

    if (app_base[1] >= (APP_START_ADDRESS + board_info.fw_size)) {
        return;
    }

#if HAL_USE_CAN == TRUE
    // for CAN firmware we start the watchdog before we run the
    // application code, to ensure we catch a bad firmare. If we get a
    // watchdog reset and the firmware hasn't changed the RTC flag to
    // indicate that it has been running OK for 30s then we will stay
    // in bootloader
    stm32_watchdog_init();
    stm32_watchdog_pat();
#endif

    flash_set_keep_unlocked(false);
    
    led_set(LED_OFF);

    // resetting the clocks is needed for loading NuttX
#if defined(STM32H7)
    rccDisableAPB1L(~0);
    rccDisableAPB1H(~0);
#else
    rccDisableAPB1(~0);
#endif
    rccDisableAPB2(~0);
#if HAL_USE_SERIAL_USB == TRUE    
    rccResetOTG_FS();
    rccResetOTG_HS();
#endif
    
    // disable all interrupt sources
    port_disable();

    /* switch exception handlers to the application */
    *(volatile uint32_t *)SCB_VTOR = APP_START_ADDRESS;

    /* extract the stack and entrypoint from the app vector table and go */
    do_jump(app_base[0], app_base[1]);
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

/*
  we use a write buffer for flashing, both for efficiency and to
  ensure that we only ever do 32 byte aligned writes on STM32H7. If
  you attempt to do writes on a H7 of less than 32 bytes or not
  aligned then the flash can end up in a CRC error state, which can
  generate a hardware fault (a double ECC error) on flash read, even
  after a power cycle
 */
static struct {
    uint32_t buffer[8];
    uint32_t address;
    uint8_t n;
} fbuf;

/*
  flush the write buffer
 */
static bool flash_write_flush(void)
{
    if (fbuf.n == 0) {
        return true;
    }
    fbuf.n = 0;
    return flash_func_write_words(fbuf.address, fbuf.buffer, ARRAY_SIZE(fbuf.buffer));
}

/*
  write to flash with buffering to 32 bytes alignment
 */
static bool flash_write_buffer(uint32_t address, const uint32_t *v, uint8_t nwords)
{
    if (fbuf.n > 0 && address != fbuf.address + fbuf.n*4) {
        if (!flash_write_flush()) {
            return false;
        }
    }
    while (nwords > 0) {
        if (fbuf.n == 0) {
            fbuf.address = address;
            memset(fbuf.buffer, 0xff, sizeof(fbuf.buffer));
        }
        uint8_t n = MIN(ARRAY_SIZE(fbuf.buffer)-fbuf.n, nwords);
        memcpy(&fbuf.buffer[fbuf.n], v, n*4);
        address += n*4;
        v += n;
        nwords -= n;
        fbuf.n += n;
        if (fbuf.n == ARRAY_SIZE(fbuf.buffer)) {
            if (!flash_write_flush()) {
                return false;
            }
        }
    }
    return true;
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
    uint32_t	read_address = 0;
    uint32_t	first_words[RESERVE_LEAD_WORDS];
    bool done_sync = false;
    bool done_get_device = false;
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
#if HAL_USE_CAN == TRUE
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

            default:
                goto cmd_bad;
            }
            done_get_device = true;
            break;

        // erase and prepare for programming
        //
        // command:		ERASE/EOC
        // success reply:	INSYNC/OK
        // erase failure:	INSYNC/FAILURE
        //
        case PROTO_CHIP_ERASE:

            if (!done_sync || !done_get_device) {
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

        // program bytes at current address
        //
        // command:		PROG_MULTI/<len:1>/<data:len>/EOC
        // success reply:	INSYNC/OK
        // invalid reply:	INSYNC/INVALID
        // readback failure:	INSYNC/FAILURE
        //
        case PROTO_PROG_MULTI:		// program bytes
            if (!done_sync || !done_get_device) {
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
            if (address < sizeof(first_words)) {
                uint8_t n = MIN(sizeof(first_words)-address, arg);
                memcpy(&first_words[address/4], &flash_buffer.w[0], n);
                // replace first words with 1 bits we can overwrite later
                memset(&flash_buffer.w[0], 0xFF, n);
            }

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

                if (p < sizeof(first_words) && first_words[0] != 0xFFFFFFFF) {
                    bytes = first_words[p/4];
                } else {
                    bytes = flash_func_read_word(p);
                }
                sum = crc32_small(sum, (uint8_t *)&bytes, sizeof(bytes));
            }

            cout_word(sum);
            break;
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
                if (!flash_write_buffer(0, first_words, RESERVE_LEAD_WORDS)) {
                    goto cmd_fail;
                }
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
            if (!done_sync || !done_get_device) {
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
        if (done_sync && done_get_device) {
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

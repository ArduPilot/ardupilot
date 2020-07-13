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
 * MAX7456 driver partially based on betaflight and inav max7456.c implemention.
 * Many thanks to their authors.
 */

#include <AP_OSD/AP_OSD_MAX7456.h>

#include <AP_HAL/Util.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Scheduler.h>
#include <AP_ROMFS/AP_ROMFS.h>

#include <utility>

#define VIDEO_COLUMNS             30
#define NVM_RAM_SIZE              54

//MAX7456 registers
#define MAX7456ADD_READ         0x80
#define MAX7456ADD_VM0          0x00
#define MAX7456ADD_VM1          0x01
#define MAX7456ADD_HOS          0x02
#define MAX7456ADD_VOS          0x03
#define MAX7456ADD_DMM          0x04
#define MAX7456ADD_DMAH         0x05
#define MAX7456ADD_DMAL         0x06
#define MAX7456ADD_DMDI         0x07
#define MAX7456ADD_CMM          0x08
#define MAX7456ADD_CMAH         0x09
#define MAX7456ADD_CMAL         0x0a
#define MAX7456ADD_CMDI         0x0b
#define MAX7456ADD_OSDM         0x0c
#define MAX7456ADD_RB0          0x10
#define MAX7456ADD_OSDBL        0x6c
#define MAX7456ADD_STAT         0xA0
#define MAX7456ADD_CMDO         0xC0

// VM0 register bits
#define VIDEO_BUFFER_DISABLE        0x01
#define MAX7456_RESET               0x02
#define VERTICAL_SYNC_NEXT_VSYNC    0x04
#define OSD_ENABLE                  0x08
#define VIDEO_MODE_PAL              0x40
#define VIDEO_MODE_NTSC             0x00
#define VIDEO_MODE_MASK             0x40

#define VIDEO_MODE_IS_PAL(val)      (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_PAL)
#define VIDEO_MODE_IS_NTSC(val) (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_NTSC)

// VM1 register bits
// duty cycle is on_off
#define BLINK_DUTY_CYCLE_50_50 0x00
#define BLINK_DUTY_CYCLE_33_66 0x01
#define BLINK_DUTY_CYCLE_25_75 0x02
#define BLINK_DUTY_CYCLE_75_25 0x03

// blinking time
#define BLINK_TIME_0 0x00
#define BLINK_TIME_1 0x04
#define BLINK_TIME_2 0x08
#define BLINK_TIME_3 0x0C

// background mode brightness (percent)
#define BACKGROUND_BRIGHTNESS_0  (0x00 << 4)
#define BACKGROUND_BRIGHTNESS_7  (0x01 << 4)
#define BACKGROUND_BRIGHTNESS_14 (0x02 << 4)
#define BACKGROUND_BRIGHTNESS_21 (0x03 << 4)
#define BACKGROUND_BRIGHTNESS_28 (0x04 << 4)
#define BACKGROUND_BRIGHTNESS_35 (0x05 << 4)
#define BACKGROUND_BRIGHTNESS_42 (0x06 << 4)
#define BACKGROUND_BRIGHTNESS_49 (0x07 << 4)

// STAT register bits
#define STAT_PAL      0x01
#define STAT_NTSC     0x02
#define STAT_LOS      0x04
#define STAT_NVR_BUSY 0x20

#define STAT_IS_PAL(val)  ((val) & STAT_PAL)
#define STAT_IS_NTSC(val) ((val) & STAT_NTSC)
#define STAT_IS_LOS(val)  ((val) & STAT_LOS)

#define VIN_IS_PAL(val)  (!STAT_IS_LOS(val) && STAT_IS_PAL(val))
#define VIN_IS_NTSC(val)  (!STAT_IS_LOS(val) && STAT_IS_NTSC(val))

// There are occasions that NTSC is not detected even with !LOS (AB7456 specific?)
// When this happens, lower 3 bits of STAT register is read as zero.
// To cope with this case, this macro defines !LOS && !PAL as NTSC.
// Should be compatible with MAX7456 and non-problematic case.
#define VIN_IS_NTSC_ALT(val)  (!STAT_IS_LOS(val) && !STAT_IS_PAL(val))

//CMM register bits
#define WRITE_NVR               0xA0
#define READ_NVR                0x50

// DMM special bits
#define DMM_BLINK (1 << 4)
#define DMM_INVERT_PIXEL_COLOR (1 << 3)
#define DMM_CLEAR_DISPLAY (1 << 2)
#define DMM_CLEAR_DISPLAY_VERT (DMM_CLEAR_DISPLAY | 1 << 1)
#define DMM_AUTOINCREMENT (1 << 0)

// time to check video signal format
#define VIDEO_SIGNAL_CHECK_INTERVAL_MS 1000
//time to wait for input to stabilize
#define VIDEO_SIGNAL_DEBOUNCE_MS 100
//time to wait nvm flash complete
#define MAX_NVM_WAIT 10000

//black and white level
#ifndef WHITEBRIGHTNESS
#define WHITEBRIGHTNESS 0x01
#endif
#ifndef BLACKBRIGHTNESS
#define BLACKBRIGHTNESS 0x00
#endif
#define BWBRIGHTNESS ((BLACKBRIGHTNESS << 2) | WHITEBRIGHTNESS)


extern const AP_HAL::HAL &hal;

AP_OSD_MAX7456::AP_OSD_MAX7456(AP_OSD &osd, AP_HAL::OwnPtr<AP_HAL::Device> dev):
    AP_OSD_Backend(osd), _dev(std::move(dev))
{
    video_signal_reg = VIDEO_MODE_PAL | OSD_ENABLE;
    video_lines = video_lines_pal;
}

bool AP_OSD_MAX7456::init()
{
    uint8_t status = 0xFF;

    _dev->get_semaphore()->take_blocking();
    _dev->write_register(MAX7456ADD_VM0, MAX7456_RESET);
    hal.scheduler->delay(1);
    _dev->read_registers(MAX7456ADD_VM0|MAX7456ADD_READ, &status, 1);
    _dev->get_semaphore()->give();
    if (status != 0) {
        return false;
    }
    return update_font();
}

bool AP_OSD_MAX7456::update_font()
{
    uint32_t font_size;
    uint8_t updated_chars = 0;
    char fontname[] = "font0.bin";
    last_font = get_font_num();
    fontname[4] = last_font + '0';
    const uint8_t *font_data = AP_ROMFS::find_decompress(fontname, font_size);
    if (font_data == nullptr || font_size != NVM_RAM_SIZE * 256) {
        return false;
    }

    for (uint16_t chr=0; chr < 256; chr++) {
        const uint8_t* chr_font_data = font_data + chr*NVM_RAM_SIZE;
        //check if char already up to date
        if (!check_font_char(chr, chr_font_data)) {
            //update char inside max7456 NVM
            if (!update_font_char(chr, chr_font_data)) {
                hal.console->printf("AP_OSD: error during font char update\n");
                AP_ROMFS::free(font_data);
                return false;
            }
            updated_chars++;
        }
    }
    if (updated_chars > 0) {
        hal.console->printf("AP_OSD: updated %d symbols.\n", updated_chars);
    }
    hal.console->printf("AP_OSD: osd font is up to date.\n");
    AP_ROMFS::free(font_data);
    return true;
}

//compare char chr inside MAX7456 NVM with font_data
bool AP_OSD_MAX7456::check_font_char(uint8_t chr, const uint8_t* font_data)
{
    buffer_offset = 0;
    //send request to read data from NVM
    buffer_add_cmd(MAX7456ADD_VM0, 0);
    buffer_add_cmd(MAX7456ADD_CMAH, chr);
    buffer_add_cmd(MAX7456ADD_CMM, READ_NVR);
    for (uint16_t x = 0; x < NVM_RAM_SIZE; x++) {
        buffer_add_cmd(MAX7456ADD_CMAL, x);
        buffer_add_cmd(MAX7456ADD_CMDO, 0xFF);
    }
    _dev->get_semaphore()->take_blocking();
    _dev->transfer(buffer, buffer_offset, buffer, buffer_offset);
    _dev->get_semaphore()->give();

    //skip response from MAX7456ADD_VM0/MAX7456ADD_CMAH...
    buffer_offset = 9;
    for (uint16_t x = 0; x < NVM_RAM_SIZE; x++) {
        if (buffer[buffer_offset] != font_data[x]) {
            return false;
        }
        //one byte per MAX7456ADD_CMAL/MAX7456ADD_CMDO pair
        buffer_offset += 4;
    }
    return true;
}

bool AP_OSD_MAX7456::update_font_char(uint8_t chr, const uint8_t* font_data)
{
    uint16_t retry;
    buffer_offset = 0;
    buffer_add_cmd(MAX7456ADD_VM0, 0);
    buffer_add_cmd(MAX7456ADD_CMAH, chr);
    for (uint16_t x = 0; x < NVM_RAM_SIZE; x++) {
        buffer_add_cmd(MAX7456ADD_CMAL, x);
        buffer_add_cmd(MAX7456ADD_CMDI, font_data[x]);
    }
    buffer_add_cmd(MAX7456ADD_CMM, WRITE_NVR);

    _dev->get_semaphore()->take_blocking();
    _dev->transfer(buffer, buffer_offset, nullptr, 0);
    _dev->get_semaphore()->give();

    for (retry = 0; retry < MAX_NVM_WAIT; retry++) {
        uint8_t status = 0xFF;
        hal.scheduler->delay(15);
        _dev->get_semaphore()->take_blocking();
        _dev->read_registers(MAX7456ADD_STAT, &status, 1);
        _dev->get_semaphore()->give();
        if ((status & STAT_NVR_BUSY) == 0x00) {
            break;
        }
    }
    return retry != MAX_NVM_WAIT;
}

AP_OSD_Backend *AP_OSD_MAX7456::probe(AP_OSD &osd, AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_OSD_MAX7456 *backend = new AP_OSD_MAX7456(osd, std::move(dev));
    if (!backend) {
        return nullptr;
    }
    if (!backend->init()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

void AP_OSD_MAX7456::buffer_add_cmd(uint8_t reg, uint8_t arg)
{
    if (buffer_offset < spi_buffer_size - 1) {
        buffer[buffer_offset++] = reg;
        buffer[buffer_offset++] = arg;
    }
}

//Thanks to betaflight for the max stall/reboot detection approach and ntsc/pal autodetection
void AP_OSD_MAX7456::check_reinit()
{
    uint8_t check = 0xFF;
    _dev->get_semaphore()->take_blocking();

    _dev->read_registers(MAX7456ADD_VM0|MAX7456ADD_READ, &check, 1);

    uint32_t now = AP_HAL::millis();

    // Stall check
    if (check != video_signal_reg) {
        reinit();
    } else if ((now - last_signal_check) > VIDEO_SIGNAL_CHECK_INTERVAL_MS) {
        uint8_t sense;

        // Adjust output format based on the current input format
        _dev->read_registers(MAX7456ADD_STAT, &sense, 1);

        if (sense & STAT_LOS) {
            video_detect_time = 0;
        } else {
            if ((VIN_IS_PAL(sense) && VIDEO_MODE_IS_NTSC(video_signal_reg))
                || (VIN_IS_NTSC_ALT(sense) && VIDEO_MODE_IS_PAL(video_signal_reg))) {
                if (video_detect_time) {
                    if (AP_HAL::millis() - video_detect_time > VIDEO_SIGNAL_DEBOUNCE_MS) {
                        reinit();
                    }
                } else {
                    // Wait for signal to stabilize
                    video_detect_time = AP_HAL::millis();
                }
            }
        }
        last_signal_check = now;
    }
    _dev->get_semaphore()->give();
}

void AP_OSD_MAX7456::reinit()
{
    uint8_t sense;

    //do not init MAX before camera power up correctly
    if (AP_HAL::millis() < 1500) {
        return;
    }

    //check input signal format
    _dev->read_registers(MAX7456ADD_STAT, &sense, 1);
    if (VIN_IS_PAL(sense)) {
        video_signal_reg = VIDEO_MODE_PAL | OSD_ENABLE;
        video_lines = video_lines_pal;
    } else {
        video_signal_reg = VIDEO_MODE_NTSC | OSD_ENABLE;
        video_lines = video_lines_ntsc;
    }

    // set all rows to same character black/white level
    for (uint8_t x = 0; x < video_lines_pal; x++) {
        _dev->write_register(MAX7456ADD_RB0 + x, BWBRIGHTNESS);
    }

    // make sure the Max7456 is enabled
    _dev->write_register(MAX7456ADD_VM0, video_signal_reg);
    _dev->write_register(MAX7456ADD_VM1, BLINK_DUTY_CYCLE_50_50 | BLINK_TIME_3 | BACKGROUND_BRIGHTNESS_28);
    _dev->write_register(MAX7456ADD_DMM, DMM_CLEAR_DISPLAY);

    //write osd position
    int8_t hos = constrain_int16(_osd.h_offset, 0, 63);
    int8_t vos = constrain_int16(_osd.v_offset, 0, 31);
    _dev->write_register(MAX7456ADD_HOS, hos);
    _dev->write_register(MAX7456ADD_VOS, vos);
    last_v_offset = _osd.v_offset;
    last_h_offset = _osd.h_offset;

    // force redrawing all screen
    memset(shadow_frame, 0xFF, sizeof(shadow_frame));

    initialized = true;
}

void AP_OSD_MAX7456::flush()
{
    if (last_font != get_font_num()) {
        update_font();
    }

    // check for offset changes
    if (last_v_offset != _osd.v_offset) {
        int8_t vos = constrain_int16(_osd.v_offset, 0, 31);
        _dev->get_semaphore()->take_blocking();
        _dev->write_register(MAX7456ADD_VOS, vos);
        _dev->get_semaphore()->give();
        last_v_offset = _osd.v_offset;
    }
    if (last_h_offset != _osd.h_offset) {
        int8_t hos = constrain_int16(_osd.h_offset, 0, 63);
        _dev->get_semaphore()->take_blocking();
        _dev->write_register(MAX7456ADD_HOS, hos);
        _dev->get_semaphore()->give();
        last_h_offset = _osd.h_offset;
    }

    check_reinit();
    transfer_frame();
}

void AP_OSD_MAX7456::transfer_frame()
{
    uint16_t previous_pos = UINT16_MAX - 1;
    bool autoincrement = false;
    if (!initialized) {
        return;
    }

    buffer_offset = 0;
    for (uint8_t y=0; y<video_lines; y++) {
        for (uint8_t x=0; x<video_columns; x++) {
            if (!is_dirty(x, y)) {
                continue;
            }
            //ensure space for 1 char and escape sequence
            if (buffer_offset >= spi_buffer_size - 32) {
                break;
            }
            shadow_frame[y][x] = frame[y][x];
            uint8_t chr = frame[y][x];
            uint16_t pos = y * video_columns + x;
            bool position_changed = ((previous_pos + 1) != pos);

            if (position_changed && autoincrement) {
                //it is impossible to write to MAX7456ADD_DMAH/MAX7456ADD_DMAL in autoincrement mode
                //so, exit autoincrement mode
                buffer_add_cmd(MAX7456ADD_DMDI, 0xFF);
                buffer_add_cmd(MAX7456ADD_DMM, 0);
                autoincrement = false;
            }

            if (!autoincrement) {
                buffer_add_cmd(MAX7456ADD_DMAH, pos >> 8);
                buffer_add_cmd(MAX7456ADD_DMAL, pos & 0xFF);
            }

            if (!autoincrement && is_dirty(x+1, y)) {
                //(re)enter autoincrement mode
                buffer_add_cmd(MAX7456ADD_DMM, DMM_AUTOINCREMENT);
                autoincrement = true;
            }

            buffer_add_cmd(MAX7456ADD_DMDI, chr);
            previous_pos = pos;
        }
    }
    if (autoincrement) {
        buffer_add_cmd(MAX7456ADD_DMDI, 0xFF);
        buffer_add_cmd(MAX7456ADD_DMM, 0);
        autoincrement = false;
    }

    if (buffer_offset > 0) {
        _dev->get_semaphore()->take_blocking();
        _dev->transfer(buffer, buffer_offset, nullptr, 0);
        _dev->get_semaphore()->give();
    }
}

bool AP_OSD_MAX7456::is_dirty(uint8_t x, uint8_t y)
{
    if (y>=video_lines || x>=video_columns) {
        return false;
    }
    return frame[y][x] != shadow_frame[y][x];
}

void AP_OSD_MAX7456::clear()
{
    AP_OSD_Backend::clear();
    memset(frame, ' ', sizeof(frame));
}

void AP_OSD_MAX7456::write(uint8_t x, uint8_t y, const char* text)
{
    if (y >= video_lines_pal || text == nullptr) {
        return;
    }
    while ((x < VIDEO_COLUMNS) && (*text != 0)) {
        frame[y][x] = *text;
        ++text;
        ++x;
    }
}

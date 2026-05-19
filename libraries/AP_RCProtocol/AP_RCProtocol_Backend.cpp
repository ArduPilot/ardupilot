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

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_ENABLED

#include "AP_RCProtocol.h"
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_VideoTX/AP_VideoTX_config.h>

// for video TX configuration:
#if AP_VIDEOTX_ENABLED
#include <AP_VideoTX/AP_VideoTX.h>
#include "spm_srxl.h"
#endif



AP_RCProtocol_Backend::AP_RCProtocol_Backend(AP_RCProtocol &_frontend) :
    frontend(_frontend)
{}

bool AP_RCProtocol_Backend::new_input()
{
    bool ret = rc_input_count != last_rc_input_count;
    if (ret) {
        last_rc_input_count = rc_input_count;
    }
    return ret;
}

uint8_t AP_RCProtocol_Backend::num_channels() const
{
    return _num_channels;
}

uint16_t AP_RCProtocol_Backend::read(uint8_t chan)
{
    return _pwm_values[chan];
}

void AP_RCProtocol_Backend::read(uint16_t *pwm, uint8_t n)
{
    if (n >= MAX_RCIN_CHANNELS) {
        n = MAX_RCIN_CHANNELS;
    }
    memcpy(pwm, _pwm_values, n*sizeof(pwm[0]));
}

/*
  provide input from a backend
 */
void AP_RCProtocol_Backend::add_input(uint8_t num_values, uint16_t *values, bool in_failsafe, int16_t _rssi, int16_t _rx_link_quality)
{
    num_values = MIN(num_values, MAX_RCIN_CHANNELS);
    memcpy(_pwm_values, values, num_values*sizeof(uint16_t));
    _num_channels = num_values;
    rc_frame_count++;
    frontend.set_failsafe_active(in_failsafe);
#if !AP_RC_CHANNEL_ENABLED
    // failsafed is sorted out in AP_IOMCU.cpp
    in_failsafe = false;
#else
    if (rc().option_is_enabled(RC_Channels::Option::IGNORE_FAILSAFE)) {
        in_failsafe = false;
    }
#endif
    if (!in_failsafe) {
        rc_input_count++;
    }
    rssi = _rssi;
    rx_link_quality = _rx_link_quality;
}


/*
  decode channels from the standard 11bit format (used by CRSF, SBUS, FPort and FPort2)
  must be used on multiples of 8 channels
*/
void AP_RCProtocol_Backend::decode_11bit_channels(const uint8_t* data, uint8_t nchannels, uint16_t *values, uint16_t mult, uint16_t div, uint16_t offset)
{
#define CHANNEL_SCALE(x) ((int32_t(x) * mult) / div + offset)
    while (nchannels >= 8) {
        const Channels11Bit_8Chan* channels = (const Channels11Bit_8Chan*)data;
        values[0] = CHANNEL_SCALE(channels->ch0);
        values[1] = CHANNEL_SCALE(channels->ch1);
        values[2] = CHANNEL_SCALE(channels->ch2);
        values[3] = CHANNEL_SCALE(channels->ch3);
        values[4] = CHANNEL_SCALE(channels->ch4);
        values[5] = CHANNEL_SCALE(channels->ch5);
        values[6] = CHANNEL_SCALE(channels->ch6);
        values[7] = CHANNEL_SCALE(channels->ch7);

        nchannels -= 8;
        data += sizeof(*channels);
        values += 8;
    }
}

#if AP_VIDEOTX_ENABLED
// configure the video transmitter, the input values are Spektrum-oriented
void AP_RCProtocol_Backend::configure_vtx(uint8_t band, uint8_t channel, uint8_t power, uint8_t pitmode)
{
    AP_VideoTX& vtx = AP::vtx();
    // VTX Band (0 = Fatshark, 1 = Raceband, 2 = E, 3 = B, 4 = A)
    // map to TBS band A, B, E, Race, Airwave, LoRace
    switch (band) {
    case VTX_BAND_FATSHARK:
        vtx.set_configured_band(AP_VideoTX::VideoBand::FATSHARK);
        break;
    case VTX_BAND_RACEBAND:
        vtx.set_configured_band(AP_VideoTX::VideoBand::RACEBAND);
        break;
    case VTX_BAND_E_BAND:
        vtx.set_configured_band(AP_VideoTX::VideoBand::BAND_E);
        break;
    case VTX_BAND_B_BAND:
        vtx.set_configured_band(AP_VideoTX::VideoBand::BAND_B);
        break;
    case VTX_BAND_A_BAND:
        vtx.set_configured_band(AP_VideoTX::VideoBand::BAND_A);
        break;
    default:
        break;
    }
    // VTX Channel (0-7)
    vtx.set_configured_channel(channel);
    if (pitmode) {
        vtx.set_configured_options(vtx.get_options() | uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
    } else {
        vtx.set_configured_options(vtx.get_options() & ~uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
    }

    switch (power) {
    case VTX_POWER_1MW_14MW:
    case VTX_POWER_15MW_25MW:
        vtx.set_configured_power_mw(25);
        break;
    case VTX_POWER_26MW_99MW:
    case VTX_POWER_100MW_299MW:
        vtx.set_configured_power_mw(100);
        break;
    case VTX_POWER_300MW_600MW:
        vtx.set_configured_power_mw(400);
        break;
    case VTX_POWER_601_PLUS:
        vtx.set_configured_power_mw(800);
        break;
    default:
        break;
    }
}
#endif  // AP_VIDEOTX_ENABLED

/*
  optionally log RC input data
 */
void AP_RCProtocol_Backend::log_data(AP_RCProtocol::rcprotocol_t prot, uint32_t timestamp, const uint8_t *data, uint8_t len) const
{
#if HAL_LOGGING_ENABLED && AP_RC_CHANNEL_ENABLED

#if (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
    if (RC_Channels::get_singleton() == nullptr) { // allow running without RC_Channels if we are doing the examples
        return;
    }
#endif
    if (rc().option_is_enabled(RC_Channels::Option::LOG_RAW_DATA)) {
        uint32_t u32[10] {};
        if (len > sizeof(u32)) {
            len = sizeof(u32);
        }
        memcpy(u32, data, len);
// @LoggerMessage: RCDA
// @Description: Raw RC data
// @Field: TimeUS: Time since system startup
// @Field: TS: data arrival timestamp
// @Field: Prot: Protocol currently being decoded
// @Field: Len: Number of valid bytes in message
// @Field: U0: first quartet of bytes
// @Field: U1: second quartet of bytes
// @Field: U2: third quartet of bytes
// @Field: U3: fourth quartet of bytes
// @Field: U4: fifth quartet of bytes
// @Field: U5: sixth quartet of bytes
// @Field: U6: seventh quartet of bytes
// @Field: U7: eight quartet of bytes
// @Field: U8: ninth quartet of bytes
// @Field: U9: tenth quartet of bytes
        AP::logger().WriteStreaming("RCDA", "TimeUS,TS,Prot,Len,U0,U1,U2,U3,U4,U5,U6,U7,U8,U9", "QIBBIIIIIIIIII",
                           AP_HAL::micros64(),
                           timestamp,
                           (uint8_t)prot,
                           len,
                           u32[0], u32[1], u32[2], u32[3], u32[4],
                           u32[5], u32[6], u32[7], u32[8], u32[9]);
    }
#endif  // HAL_LOGGING_ENABLED && AP_RC_CHANNEL_ENABLED
}

#endif  // AP_RCPROTOCOL_ENABLED

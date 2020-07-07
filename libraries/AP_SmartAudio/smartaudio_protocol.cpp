/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

//#include "platform.h"

//#include "common/crc.h"

#include <AP_Math/AP_Math.h>
#include "stdio.h"

#include "smartaudio_protocol.h"


#define U16BIGENDIAN(bytes) (bytes << 8) | ((bytes >> 8) & 0xFF)


static void smartaudioFrameInit(const uint8_t command, smartaudioFrameHeader_t *header, const uint8_t payloadLength)
{
    header->syncByte = SMARTAUDIO_SYNC_BYTE;
    header->headerByte= SMARTAUDIO_HEADER_BYTE;
    header->length = payloadLength;
    header->command = command;

}

static void smartaudioUnpackOperationMode(smartaudioSettings_t *settings, const uint8_t operationMode, const bool settingsResponse)
{
    if (settingsResponse) {
        // operation mode bit order is different between 'Get Settings' and 'Set Mode' responses.
        settings->userFrequencyMode = operationMode & 0x01;
        settings->pitmodeEnabled = operationMode & 0x02;
        settings->pitmodeInRangeActive = operationMode & 0x04;
        settings->pitmodeOutRangeActive = operationMode & 0x08;
        settings->unlocked = operationMode & 0x10;
    } else {
        settings->pitmodeInRangeActive = operationMode & 0x01;
        settings->pitmodeOutRangeActive = operationMode & 0x02;
        settings->pitmodeEnabled = operationMode & 0x04;
        settings->unlocked = operationMode & 0x08;
    }
}

static void smartaudioUnpackFrequency(smartaudioSettings_t *settings, const uint16_t frequency)
{
    if (frequency & SMARTAUDIO_GET_PITMODE_FREQ) {
        settings->pitmodeFrequency = U16BIGENDIAN(frequency & SMARTAUDIO_FREQUENCY_MASK);
    } else {
        settings->frequency = U16BIGENDIAN(frequency & SMARTAUDIO_FREQUENCY_MASK);
    }
}

static void smartaudioUnpackSettings(smartaudioSettings_t *settings, const smartaudioSettingsResponseFrame_t *frame)
{
    settings->channel = frame->channel;
    settings->power = frame->power;
    smartaudioUnpackFrequency(settings, frame->frequency);
    smartaudioUnpackOperationMode(settings, frame->operationMode, true);
}

static uint8_t smartaudioPackOperationMode(const smartaudioSettings_t *settings)
{
    uint8_t operationMode = 0;
    operationMode |= settings->pitmodeInRangeActive << 0;
    operationMode |= settings->pitmodeOutRangeActive << 1;
    operationMode |= settings->pitmodeEnabled << 2;
    operationMode |= settings->unlocked << 3;
    return operationMode;
}

size_t smartaudioFrameGetSettings(smartaudioFrame_t *smartaudioFrame)
{
    smartaudioCommandOnlyFrame_t *frame = (smartaudioCommandOnlyFrame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_GET_SETTINGS, &frame->header, 0);
    //frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioCommandOnlyFrame_t) - sizeof(frame->crc));
    frame->crc=crc_crc8_dvb_s2((const uint8_t *)frame, sizeof(smartaudioCommandOnlyFrame_t) - sizeof(frame->crc));
    return sizeof(smartaudioCommandOnlyFrame_t);
}

size_t smartaudioFrameGetPitmodeFrequency(smartaudioFrame_t *smartaudioFrame)
{
    smartaudioU16Frame_t *frame = (smartaudioU16Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_FREQUENCY, &frame->header, sizeof(frame->payload));
    frame->payload = SMARTAUDIO_SET_PITMODE_FREQ;
    //frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    frame->crc = crc_crc8_dvb_s2((const uint8_t *)frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU16Frame_t);
}

size_t smartaudioFrameSetPower(smartaudioFrame_t *smartaudioFrame, const uint8_t power)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_POWER, &frame->header, sizeof(frame->payload));
    frame->payload = power;
    //frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    frame->crc = crc_crc8_dvb_s2((const uint8_t *)frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

size_t smartaudioFrameSetBandChannel(smartaudioFrame_t *smartaudioFrame, const uint8_t band, const uint8_t channel)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_CHANNEL, &frame->header, sizeof(frame->payload));
    frame->payload = SMARTAUDIO_BANDCHAN_TO_INDEX(band, channel);
    //frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    frame->crc = crc_crc8_dvb_s2( (const uint8_t *)frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

size_t smartaudioFrameSetFrequency(smartaudioFrame_t *smartaudioFrame, const uint16_t frequency, const bool pitmodeFrequency)
{
    smartaudioU16Frame_t *frame = (smartaudioU16Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_FREQUENCY, &frame->header, sizeof(frame->payload));
    frame->payload = U16BIGENDIAN(frequency | (pitmodeFrequency ? SMARTAUDIO_SET_PITMODE_FREQ : 0x00));
    //frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    frame->crc = crc_crc8_dvb_s2((const uint8_t *)frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU16Frame_t);
}

size_t smartaudioFrameSetOperationMode(smartaudioFrame_t *smartaudioFrame, const smartaudioSettings_t *settings)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_MODE, &frame->header, sizeof(frame->payload));
    frame->payload = smartaudioPackOperationMode(settings);
    //frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    frame->crc = crc_crc8_dvb_s2((const uint8_t *)frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

bool smartaudioParseResponseBuffer(smartaudioSettings_t *settings, const uint8_t *buffer)
{
    const smartaudioFrameHeader_t *header = (const smartaudioFrameHeader_t *)buffer;
    const uint8_t fullFrameLength = sizeof(smartaudioFrameHeader_t) + header->length;
    const uint8_t headerPayloadLength = fullFrameLength - 1; // subtract crc byte from length
    //const uint8_t *endPtr = buffer + fullFrameLength;

    // if (crc8_dvb_s2_update(*endPtr, buffer, headerPayloadLength) || header->startCode != SMARTAUDIO_START_CODE) {
    //     return false;
    // }

    if (crc_crc8_dvb_s2( buffer, headerPayloadLength) || ( header->syncByte !=  SMARTAUDIO_SYNC_BYTE && header->headerByte!= SMARTAUDIO_HEADER_BYTE) /*&& )SMARTAUDIO_START_CODE*/) {
         return false;
     }
    switch (header->command) {
        case SMARTAUDIO_RSP_GET_SETTINGS_V1: {
                const smartaudioSettingsResponseFrame_t *resp = (const smartaudioSettingsResponseFrame_t *)buffer;
                settings->version = 1;
                smartaudioUnpackSettings(settings, resp);
            }
            break;
        case SMARTAUDIO_RSP_GET_SETTINGS_V2: {
                const smartaudioSettingsResponseFrame_t *resp = (const smartaudioSettingsResponseFrame_t *)buffer;
                settings->version = 2;
                smartaudioUnpackSettings(settings, resp);
            }
            break;
        case SMARTAUDIO_RSP_SET_POWER: {
                const smartaudioU16ResponseFrame_t *resp = (const smartaudioU16ResponseFrame_t *)buffer;
                settings->channel = (resp->payload >> 8) & 0xFF;
                settings->power = resp->payload & 0xFF;
            }
            break;
        case SMARTAUDIO_RSP_SET_CHANNEL: {
                const smartaudioU8ResponseFrame_t *resp = (const smartaudioU8ResponseFrame_t *)buffer;
                settings->channel = resp->payload;
            }
            break;
        case SMARTAUDIO_RSP_SET_FREQUENCY: {
                const smartaudioU16ResponseFrame_t *resp = (const smartaudioU16ResponseFrame_t *)buffer;
                smartaudioUnpackFrequency(settings, resp->payload);
            }
            break;
        case SMARTAUDIO_RSP_SET_MODE: {
                const smartaudioU8ResponseFrame_t *resp = (const smartaudioU8ResponseFrame_t*)buffer;
                smartaudioUnpackOperationMode(settings, resp->payload, false);
            }
            break;
        default:
            return false;
    }
    return true;
}

/*
  Code for handling MAVLink2 signing

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

#include "GCS.h"

extern const AP_HAL::HAL& hal;

// storage object
StorageAccess GCS_MAVLINK::_signing_storage(StorageManager::StorageKeys);

// magic for versioning of the structure
#define SIGNING_KEY_MAGIC 0x3852fcd1

// structure stored in FRAM
struct SigningKey {
    uint32_t magic;
    uint64_t timestamp;
    uint8_t secret_key[32];
};

// shared signing_streams structure
mavlink_signing_streams_t GCS_MAVLINK::signing_streams;

// last time we saved the timestamp
uint32_t GCS_MAVLINK::last_signing_save_ms;

bool GCS_MAVLINK::signing_key_save(const struct SigningKey &key)
{
    if (_signing_storage.size() < sizeof(key)) {
        return false;
    }
    return _signing_storage.write_block(0, &key, sizeof(key));
}

bool GCS_MAVLINK::signing_key_load(struct SigningKey &key)
{
    if (_signing_storage.size() < sizeof(key)) {
        return false;
    }
    if (!_signing_storage.read_block(&key, 0, sizeof(key))) {
        return false;
    }
    if (key.magic != SIGNING_KEY_MAGIC) {
        return false;
    }
    return true;
}

/*
  handle a setup_signing message
 */
void GCS_MAVLINK::handle_setup_signing(const mavlink_message_t *msg)
{
    // decode
    mavlink_setup_signing_t packet;
    mavlink_msg_setup_signing_decode(msg, &packet);

    struct SigningKey key;
    key.magic = SIGNING_KEY_MAGIC;
    key.timestamp = packet.initial_timestamp;
    memcpy(key.secret_key, packet.secret_key, 32);

    if (!signing_key_save(key)) {
        hal.console->printf("Failed to save signing key");
        return;
    }

    // activate it immediately
    load_signing_key();
}


/*
  callback to accept unsigned (or incorrectly signed) packets
 */
extern "C" {

static const uint32_t accept_list[] = {
    MAVLINK_MSG_ID_RADIO_STATUS,
    MAVLINK_MSG_ID_RADIO
};
    
static bool accept_unsigned_callback(const mavlink_status_t *status, uint32_t msgId)
{
    if (status == mavlink_get_channel_status(MAVLINK_COMM_0)) {
        // always accept channel 0, assumed to be secure channel. This
        // is USB on PX4 boards
        return true;
    }
    for (uint8_t i=0; i<ARRAY_SIZE(accept_list); i++) {
        if (accept_list[i] == msgId) {
            return true;
        }
    }
    return false;
}
}

/*
  load signing key
 */
void GCS_MAVLINK::load_signing_key(void)
{
    struct SigningKey key;
    if (!signing_key_load(key)) {
        return;
    }
    mavlink_status_t *status = mavlink_get_channel_status(chan);
    if (status == nullptr) {
        hal.console->printf("Failed to load signing key - no status");
        return;        
    }
    memcpy(signing.secret_key, key.secret_key, 32);
    signing.link_id = (uint8_t)chan;
    // use a timestamp 1 minute past the last recorded
    // timestamp. Combined with saving the key once every 30s this
    // prevents a window for replay attacks
    signing.timestamp = key.timestamp + 60UL * 100UL * 1000UL;
    signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
    signing.accept_unsigned_callback = accept_unsigned_callback;

    // if timestamp and key are all zero then we disable signing
    bool all_zero = (key.timestamp == 0);
    for (uint8_t i=0; i<sizeof(key.secret_key); i++) {
        if (signing.secret_key[i] != 0) {
            all_zero = false;
        }
    }
    
    // enable signing on all channels
    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        mavlink_status_t *cstatus = mavlink_get_channel_status((mavlink_channel_t)(MAVLINK_COMM_0 + i));
        if (cstatus != nullptr) {
            if (all_zero) {
                // disable signing
                cstatus->signing = nullptr;
                cstatus->signing_streams = nullptr;
            } else {
                cstatus->signing = &signing;
                cstatus->signing_streams = &signing_streams;
            }
        }
    }
}

/*
  update signing timestamp. This is called when we get GPS lock
  timestamp_usec is since 1/1/1970 (the epoch)
 */
void GCS_MAVLINK::update_signing_timestamp(uint64_t timestamp_usec)
{
    uint64_t signing_timestamp = (timestamp_usec / (1000*1000ULL));
    // this is the offset from 1/1/1970 to 1/1/2015
    const uint64_t epoch_offset = 1420070400;
    if (signing_timestamp > epoch_offset) {
        signing_timestamp -= epoch_offset;
    }

    // convert to 10usec units
    signing_timestamp *= 100 * 1000ULL;

    // increase signing timestamp on any links that have signing
    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        mavlink_status_t *status = mavlink_get_channel_status(chan);
        if (status && status->signing && status->signing->timestamp < signing_timestamp) {
            status->signing->timestamp = signing_timestamp;
        }
    }

    // save to stable storage
    save_signing_timestamp(true);
}


/*
  save the signing timestamp periodically
 */
void GCS_MAVLINK::save_signing_timestamp(bool force_save_now)
{
    uint32_t now = AP_HAL::millis();
    // we save the timestamp every 30s, unless forced by a GPS update
    if (!force_save_now &&  now - last_signing_save_ms < 30*1000UL) {
        return;
    }

    last_signing_save_ms = now;

    struct SigningKey key;
    if (!signing_key_load(key)) {
        return;
    }
    bool need_save = false;

    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        const mavlink_status_t *status = mavlink_get_channel_status(chan);
        if (status && status->signing && status->signing->timestamp > key.timestamp) {
            key.timestamp = status->signing->timestamp;
            need_save = true;
        }
    }
    if (need_save) {
        // save updated key
        signing_key_save(key);
    }
}

/*
  return true if signing is enabled on this channel
 */
bool GCS_MAVLINK::signing_enabled(void) const
{
    const mavlink_status_t *status = mavlink_get_channel_status(chan);
    if (status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING)) {
        return true;
    }
    return false;
}

/*
  return packet overhead in bytes for a channel
 */
uint8_t GCS_MAVLINK::packet_overhead_chan(mavlink_channel_t chan)
{
    /*
      reserve 100 bytes for parameters when a GCS fails to fetch a
      parameter due to lack of buffer space. The reservation lasts 2
      seconds
     */
    uint8_t reserved_space = 0;
    if (reserve_param_space_start_ms != 0 &&
        AP_HAL::millis() - reserve_param_space_start_ms < 2000) {
        reserved_space = 100;
    } else {
        reserve_param_space_start_ms = 0;
    }
    
    const mavlink_status_t *status = mavlink_get_channel_status(chan);
    if (status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING)) {
        return MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_SIGNATURE_BLOCK_LEN + reserved_space;
    }
    return MAVLINK_NUM_NON_PAYLOAD_BYTES + reserved_space;
}


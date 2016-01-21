// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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

#if MAVLINK_PROTOCOL_VERSION >= 2

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
    return _signing_storage.read_block(&key, 0, sizeof(key));
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

static const struct {
    uint8_t dialect;
    uint16_t msgId;
} accept_list[] = {
    { MAVLINK_MSG_ID_RADIO_STATUS_TUPLE }
};
    
static bool accept_unsigned_callback(const mavlink_status_t *status,
                                     uint8_t dialect, uint16_t msgId)
{
    if (status == mavlink_get_channel_status(MAVLINK_COMM_0)) {
        // always accept channel 0, assumed to be secure channel. This
        // is USB on PX4 boards
        return true;
    }
    for (uint8_t i=0; i<ARRAY_SIZE(accept_list); i++) {
        if (accept_list[i].dialect == dialect &&
            accept_list[i].msgId == msgId) {
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
        hal.console->printf("Failed to load signing key");
        return;
    }
    mavlink_status_t *status = mavlink_get_channel_status(chan);
    if (status == NULL) {
        hal.console->printf("Failed to load signing key - no status");
        return;        
    }
    memcpy(signing.secret_key, key.secret_key, 32);
    signing.link_id = (uint8_t)chan;
    signing.timestamp = key.timestamp;
    signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
    signing.accept_unsigned_callback = accept_unsigned_callback;
    
    status->signing = &signing;
    status->signing_streams = &signing_streams;
}

/*
  update signing timestamp. This is called when we get GPS lock
  timestamp_usec is since 1/1/1970 (the epoch)
 */
void GCS_MAVLINK::update_signing_timestamp(uint64_t timestamp_usec)
{
    uint64_t signing_timestamp = (timestamp_usec / 1000*1000UL);
    const uint64_t epoch_offset = 1420070400;
    if (signing_timestamp > epoch_offset) {
        signing_timestamp -= epoch_offset;
    }
    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        mavlink_status_t *status = mavlink_get_channel_status(chan);
        if (status && status->signing && status->signing->timestamp < signing_timestamp) {
            status->signing->timestamp = signing_timestamp;
        }
    }
}

#else
void GCS_MAVLINK::update_signing_timestamp(uint64_t timestamp_usec) {}
#endif // MAVLINK_PROTOCOL_VERSION

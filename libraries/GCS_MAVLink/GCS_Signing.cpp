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
    signing.timestamp = 1;
    signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
    
    status->signing = &signing;
}

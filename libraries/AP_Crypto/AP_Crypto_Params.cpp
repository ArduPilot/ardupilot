/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR ANY PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Crypto_Params.h"

#if AP_CRYPTO_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_CheckFirmware/monocypher.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Crypto/AP_Crypto.h>

extern const AP_HAL::HAL& hal;

AP_Crypto_Params *AP_Crypto_Params::_singleton = nullptr;

const struct AP_Param::GroupInfo AP_Crypto_Params::var_info[] = {
    // @Param: LEIGH_KEY
    // @DisplayName: Encryption Key
    // @Description: Encryption key for file encryption. INT32 value used to derive 32-byte encryption key via BLAKE2b. Supports full INT32 range including values like 74768361. When set via MAVLink, the INT32 value is hashed with salt "LEIGH_KEY_SALT_1" using BLAKE2b to produce a 32-byte key which is stored in secure storage. When LEIGH_CRYPT_LVL is 0, this parameter's value is readable via MAVLink; otherwise, it returns 0 for security.
    // @Range: -2147483648 2147483647
    // @Units: INT_32
    // @User: Standard
    AP_GROUPINFO("LEIGH_KEY", 1, AP_Crypto_Params, leigh_key, 0),
    
    // @Param: LEIGH_CRYPT_LVL
    // @DisplayName: Encryption Control Level
    // @Description: Bitmask to selectively control what gets encrypted. Bit 0 (0x01): Enable encryption for log files written to SD card. Bit 1 (0x02): Enable encryption for USB terminal output (log downloads). Value 0 disables all AP_CRYPTO encryption when writing log files and USB terminal output. When set to 0, LEIGH_KEY parameter value becomes readable via MAVLink for debugging purposes.
    // @Range: 0 3
    // @Bitmask: 0:Log Files,1:USB Terminal
    // @User: Standard
    // @Param: LEIGH_CRYPT_LVL
    // @DisplayName: Encryption Control Level
    // @Description: Bitmask to selectively control what gets encrypted. Bit 0 (0x01): Enable encryption for log files written to SD card. Bit 1 (0x02): Enable encryption for USB terminal output (log downloads). Value 0 disables all AP_CRYPTO encryption when writing log files and USB terminal output. When set to 0, LEIGH_KEY parameter value becomes readable via MAVLink for debugging purposes.
    // @Range: 0 3
    // @Bitmask: 0:Log Files,1:USB Terminal
    // @User: Standard
    AP_GROUPINFO("LEIGH_CRYPT_LVL", 2, AP_Crypto_Params, leigh_crypt_lvl, 0),
    
    AP_GROUPEND
};

AP_Crypto_Params::AP_Crypto_Params()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Crypto_Params must be singleton");
    }
    _singleton = this;
}

void AP_Crypto_Params::handle_key_set(int32_t value)
{
    // Derive a 32-byte key from the INT32 value
    // Use BLAKE2b to hash the value with a salt
    uint8_t key[32];
    const uint8_t salt[16] = {
        0x4c, 0x45, 0x49, 0x47, 0x48, 0x5f, 0x4b, 0x45,
        0x59, 0x5f, 0x53, 0x41, 0x4c, 0x54, 0x5f, 0x31
    }; // "LEIGH_KEY_SALT_1" in ASCII
    
    // Hash the INT32 value + salt to get 32-byte key
    crypto_blake2b_ctx ctx;
    crypto_blake2b_general_init(&ctx, 32, nullptr, 0); // 32-byte output
    crypto_blake2b_update(&ctx, (const uint8_t*)&value, sizeof(value));
    crypto_blake2b_update(&ctx, salt, sizeof(salt));
    crypto_blake2b_final(&ctx, key);
    
    // Store the derived key
    AP_Crypto::store_key(key);
}

bool AP_Crypto_Params::is_encryption_enabled(uint32_t feature) const
{
    if (_singleton == nullptr) {
        return false;
    }
    // Check if the specified feature bit is set in the bitmask
    return (_singleton->leigh_crypt_lvl.get() & feature) != 0;
}

namespace AP {
    AP_Crypto_Params &crypto_params()
    {
        return *AP_Crypto_Params::get_singleton();
    }
}

#endif  // AP_CRYPTO_ENABLED


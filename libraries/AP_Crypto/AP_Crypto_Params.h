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
#pragma once

#include "AP_Crypto_config.h"

#if AP_CRYPTO_ENABLED

#include <AP_Param/AP_Param.h>

/*
  Bitmask constants for LEIGH_CRYPT_LVL parameter
  These bits control what gets encrypted:
*/
#define LEIGH_CRYPT_LOG_FILES     0x01  // Bit 0: Encrypt log files written to SD card
#define LEIGH_CRYPT_USB_TERMINAL  0x02  // Bit 1: Encrypt USB terminal output (log downloads)

/*
  AP_Crypto_Params - Parameter class for AP_Crypto

  This class manages the LEIGH_KEY parameter which allows setting
  the encryption key via MAVLink PARAM_SET, but always returns 0
  when read to prevent key disclosure.
 */
class AP_Crypto_Params
{
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_Crypto_Params();

    // Get the parameter instance
    static AP_Crypto_Params *get_singleton(void) {
        return _singleton;
    }

    // The LEIGH_KEY parameter (INT32 - 32-bit value)
    // When read via MAVLink, always returns 0
    // When set via MAVLink, extracts key and stores it
    AP_Int32 leigh_key;

    // The LEIGH_CRYPT_LVL parameter (INT32 - bitmask)
    // Bitmask to selectively control what gets encrypted:
    // Bit 0 (0x01): Enable encryption for log files written to SD card
    // Bit 1 (0x02): Enable encryption for USB terminal output (log downloads)
    // Value 0: Disables all AP_CRYPTO encryption
    AP_Int32 leigh_crypt_lvl;

    // Extract key from INT32 value and store it
    // The INT32 value is used as a seed to derive/generate the key
    void handle_key_set(int32_t value);
    
    // Check if encryption is enabled for a specific feature
    // feature: bitmask value (e.g., LEIGH_CRYPT_LOG_FILES, LEIGH_CRYPT_USB_TERMINAL)
    bool is_encryption_enabled(uint32_t feature) const;

private:
    static AP_Crypto_Params *_singleton;
};

namespace AP {
    AP_Crypto_Params &crypto_params();
}

// Global instance - must be created before AP_Param::setup() is called
// This should be created as a static instance in a vehicle's Parameters class
// or as a global variable that gets initialized early

#endif  // AP_CRYPTO_ENABLED


/*
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
#pragma once

#include "AP_Crypto_config.h"

#if defined(AP_CRYPTO_ENABLED) && AP_CRYPTO_ENABLED

#include <stdint.h>

/*
  AP_Crypto - Key storage and retrieval functions for ArduPilot

  Provides secure key storage and retrieval functionality.
  Encryption/decryption is handled by AP_Crypto_Simple.
 */
class AP_Crypto
{
public:
    /*
      Key storage and retrieval functions
     */
    
    /*
      Store encryption key in persistent storage
      
      @param key: 32-byte encryption key (raw bytes)
      @return: true on success, false on failure
     */
    static bool store_key(const uint8_t key[32]);
    
    /*
      Retrieve encryption key from persistent storage
      
      @param key: Output buffer for 32-byte key
      @return: true if key was found and retrieved, false otherwise
     */
    static bool retrieve_key(uint8_t key[32]);
    
    /*
      Check if a key is stored in persistent storage
      
      @return: true if key exists, false otherwise
     */
    static bool has_stored_key(void);
    
    /*
      Generate and store a new encryption key
      
      @param key: Output buffer for generated key (optional, can be nullptr)
      @return: true on success, false on failure
     */
    static bool generate_and_store_key(uint8_t key[32] = nullptr);
    
    /*
      Derive key from board ID (DISABLED - always returns false)
      
      @param key: Output buffer for 32-byte key (unused)
      @return: always returns false (functionality disabled)
     */
    static bool derive_key_from_board_id(uint8_t key[32]);
};

#endif  // AP_CRYPTO_ENABLED

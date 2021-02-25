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
#pragma once

#if defined(SECURE) && SECURE==1 && defined(KEY_FLASH_PAGE) && !defined(HAL_BOOTLOADER_BUILD)
#include <wolfssl/options.h>
#include <wolfssl/wolfcrypt/rsa.h>
#include <wolfssl/wolfcrypt/sha256.h>
#include <wolfssl/wolfcrypt/signature.h>
#include <wolfssl/wolfcrypt/asn.h>
#include <wolfssl/wolfcrypt/error-crypt.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/EventSource.h>
#endif
#ifndef DIGEST_VALUE_LEN
#define DIGEST_VALUE_LEN 32 // For SHA256
#endif

#define MAX_KEYDER_SIZE  1200

class KeyManager {
public:
    KeyManager();

    // get singleton instance
    static KeyManager *get_singleton() {
        return _singleton;
    }
    void init();
    void reset_sha256(wc_Sha256* sha_handle);
    void update_sha256(wc_Sha256* sha_handle, const char* data, uint16_t data_len);
    void final_sha256(wc_Sha256* sha_handle, uint8_t* hash);
    void load_server_pubkey();
    int verify_hash_with_server_pkey(const char* hashed_data, uint16_t hashed_data_len, const uint8_t* signature, uint16_t signature_len);
    bool sign_data_with_ap_key(uint8_t* data, uint32_t data_len, uint8_t* signature);
    void change_pin(uint32_t old_pin, uint32_t new_pin);
    bool is_secure();
    bool set_input_pin(uint32_t pin);

private:
    void _generate_private_key(void);
    void _secure_thread(void);
    void _save_public_key(void);
    bool _check_and_initialise_private_key(void);
    bool _flash_read(uint32_t addr, void *data, uint32_t length);

    HAL_EventHandle event_handle;

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    static ChibiOS::EventSource evt_src;
#endif

    struct secure_block_t {
        uint32_t dersize;
        uint32_t keyder_crc;
        uint8_t der[MAX_KEYDER_SIZE];
        uint8_t pin[DIGEST_VALUE_LEN];
        uint8_t num_incorrect_pins;
    } __attribute__ ((aligned (32))) *secure_block;

    RsaKey ap_key;
    uint8_t *_input_pin;
    wc_Sha256 *pin_sha_handle; 
    RsaKey server_pubkey;
    bool _server_key_loaded = false;
    static KeyManager *_singleton;
    uint8_t _num_pin_tries;
};


namespace AP {
    KeyManager &keymgr();
};

#endif
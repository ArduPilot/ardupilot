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
 * Code by Siddharth Bharat Purohit
 */

#include <AP_HAL/AP_HAL.h>

//Check if the build is for Registered Flight Module
#if defined(SECURE) && SECURE==1 && defined(KEY_FLASH_PAGE) && !defined(HAL_BOOTLOADER_BUILD)

#include "KeyManager.h"
#include <AP_ROMFS/AP_ROMFS.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>
#include <string.h>

#include <flash.h>
#include <stm32_util.h>

ChibiOS::EventSource KeyManager::evt_src;

#ifndef AP_SERVER_PUBLIC_KEY_FILE
#define AP_SERVER_PUBLIC_KEY_FILE "server_pubkey.der"
#endif

#ifndef AP_SELF_PUBLIC_KEY_FILE
#define AP_SELF_PUBLIC_KEY_FILE HAL_BOARD_STORAGE_DIRECTORY "/self_pubkey.der"
#endif

#define MAX_NUM_INCORRECT_PINS 20

extern const AP_HAL::HAL& hal;

#ifndef KEY_FLASH_PAGE
#define KEY_FLASH_PAGE 0
#endif

#ifndef HAL_KEYREGION_SIZE
#define HAL_KEYREGION_SIZE (16*1024)
#endif

static KeyManager _keymgr;

#define EVT_SECURE_BLOCK_UPDATE 1U

KeyManager::KeyManager()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many KeyManager modules");
        return;
    }
    _singleton = this;
}

void KeyManager::init() {
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&KeyManager::_secure_thread, void), 
                                                    "SECURE", 512, AP_HAL::Scheduler::PRIORITY_TIMER, 0,
                                                    AP_HAL::Util::MEM_SECURE);

    // check if we already generated key during last run
    if (!_check_and_initialise_private_key() && stm32_flash_is_rdp_enabled()) {
        //Generate new key in case of failure
        //Run private keygen thread
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&KeyManager::_generate_private_key, void), 
                                                         "KEYGEN", 16384, AP_HAL::Scheduler::PRIORITY_KEYMGR, 0)) {
            AP_HAL::panic("KeyManager: Failed to allocate Private Key Thread!\n");
        }
    }
    // Load Server's Public Key for Verification
#ifdef HAL_DIGITAL_SKY_RFM
    load_server_pubkey();
#endif
}

void KeyManager::_secure_thread()
{
    event_handle.set_source(&evt_src);
    event_handle.register_event(EVT_SECURE_BLOCK_UPDATE);

    while (true) {
        if (event_handle.wait(10000000)) { // wait here for a long time
            size_t base_address = hal.flash->getpageaddr(KEY_FLASH_PAGE);
            //Erase Key Flash Sector
            GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Erasing Secure Flash Page.\n");

            stm32_flash_keep_unlocked(true);

            while (!hal.flash->erasepage(KEY_FLASH_PAGE)) {
                hal.scheduler->delay(100);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Updating Secure Flash Region, %u Bytes\n", sizeof(secure_block_t));
            bool finished = false;
            while(!finished) {
                finished = hal.flash->write(base_address, secure_block, sizeof(secure_block_t));
                hal.scheduler->delay(100);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Secure Flash Region Updated\n");

            stm32_flash_keep_unlocked(false);
        }
        hal.scheduler->delay(1);
    }
}

bool KeyManager::_check_and_initialise_private_key()
{
    uint32_t calc_crc = 0xACDC;
    word32 idx = 0;
    int ret;
    uint32_t base_address = hal.flash->getpageaddr(KEY_FLASH_PAGE);

    if (secure_block == nullptr) {
        secure_block = (secure_block_t*)hal.util->malloc_type(sizeof(secure_block_t), AP_HAL::Util::MEM_SECURE);
        if (secure_block == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Failed to allocate secure block.\n");
            hal.scheduler->delay(1);
            return false;
        }
    }

    _flash_read(base_address, secure_block, sizeof(secure_block_t));

    //check sanity for 2048bit RSA Key's dersize
    if (secure_block->dersize < 1100 || secure_block->dersize > 1200) {
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Invalid Key Length in Flash.\n");
        hal.scheduler->delay(1);
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Found Private Key in Flash.\n");
    //Initialise Rsa Key structure
    // we allow insecure memory here
    ret = wc_InitRsaKey(&ap_key, NULL);
    if (ret != 0) {
        return false;
    }

    //check key is good
    calc_crc = crc_crc32(calc_crc, secure_block->der, secure_block->dersize);
    if (secure_block->keyder_crc != calc_crc) {
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Invalid Key CRC!\n");
        hal.scheduler->delay(1);
        return false;
    }
    ret = wc_RsaPrivateKeyDecode(secure_block->der, &idx, &ap_key, secure_block->dersize);
    if (ret != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT,  "KeyManager: Failed to load Key from Flash.\n");
        hal.scheduler->delay(1);
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Successfully Loaded Private Key.\n");

    _save_public_key();
    return true;
}

void KeyManager::_save_public_key()
{
    uint32_t dersize = 1200;
    // we don't need secure memory
    // for public key
    //Generate Public Key File
    GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Extracting Public Key.\n");
    uint8_t *publickey_der = new uint8_t[dersize];
    if (publickey_der == nullptr) {
        AP_HAL::panic("KeyManager: Failed to Allocate buffer for Public Key.");
        return;
    }

    int ret = wc_RsaKeyToPublicDer(&ap_key, publickey_der, dersize);
    if (ret < 0) {
        char error_str[WOLFSSL_MAX_ERROR_SZ];
        wc_ErrorString(ret, error_str);
        AP_HAL::panic("KeyManager: Failed to Convert RSA PublicKey %d %s\n", ret, error_str);
        return;
    }
    dersize = ret;
    //Save public key to file
    int pubkey_fd = AP::FS().open(AP_SELF_PUBLIC_KEY_FILE, O_WRONLY|O_TRUNC|O_CREAT);
    if (pubkey_fd < 0) {
        AP_HAL::panic("KeyManager: Failed to create file for Public Key Storage.");
        return;
    }

    if (AP::FS().write(pubkey_fd, publickey_der, dersize) < 0) {
        AP_HAL::panic("KeyManager: Failed to write public key");
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Saved Public Key to SDCard.\n");
    AP::FS().close(pubkey_fd);
    delete[] publickey_der;

}

void KeyManager::_generate_private_key()
{
    WC_RNG rng;
    int ret;

    //Initialise Random Number Generator
    ret = wc_InitRng(&rng);
    if (ret != 0) {
        AP_HAL::panic("KeyManager: Failed to Initialize Random Number Generator\n");
        return;
    }
    //Initialise Rsa Key structure
    ret = wc_InitRsaKey(&ap_key, NULL);
    if (ret != 0) {
        AP_HAL::panic("KeyManager: Failed to Initialize RSA Key\n");
        return;
    }
    //Generate RSA Key
    GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Generating Private Key\n");
    hal.scheduler->delay(1);
    ret = wc_MakeRsaKey(&ap_key, 2048, 65537, &rng);
    if (ret != 0) {
        char error_str[WOLFSSL_MAX_ERROR_SZ];
        wc_ErrorString(ret, error_str);
        AP_HAL::panic("KeyManager: Failed to Generate RSA Key Pair: %d %s\n", ret, error_str);
        return;
    }
    //Generate Key in DER format
    GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Creating DER Private Key.\n");
    hal.scheduler->delay(1);

    if (secure_block == nullptr) {
        AP_HAL::panic("KeyManager: Secure Block not allocated.");
        return;
    }

    secure_block->dersize = 1200;
    secure_block->keyder_crc = 0xACDC;

    secure_block->dersize = wc_RsaKeyToDer(&ap_key, secure_block->der, secure_block->dersize);
    if (ret != 0) {
        AP_HAL::panic("KeyManager: Failed to convert RSA Key to DER");
        return;
    }

    //Generate CRC hash or der key for sanity check
    secure_block->keyder_crc = crc_crc32(secure_block->keyder_crc, secure_block->der, secure_block->dersize);

    evt_src.signal(EVT_SECURE_BLOCK_UPDATE);
    _save_public_key();

    wc_FreeRng(&rng);
}

bool KeyManager::set_input_pin(uint32_t pin)
{
    if (_input_pin == nullptr) {
        _input_pin = (uint8_t*)hal.util->malloc_type(DIGEST_VALUE_LEN, AP_HAL::Util::MEM_SECURE);
        if (_input_pin == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Failed to allocate input pin.\n");
            return false;
        }
    }

    if (secure_block->num_incorrect_pins >= MAX_NUM_INCORRECT_PINS) {
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Num Tries Exceeded, You have been locked out!\n");
        return false;
    }

    pin_sha_handle = (wc_Sha256*)hal.util->malloc_type(sizeof(wc_Sha256), AP_HAL::Util::MEM_SECURE);
    reset_sha256(pin_sha_handle);
    update_sha256(pin_sha_handle, (const char*)&pin, sizeof(pin));
    final_sha256(pin_sha_handle, _input_pin);
    free(pin_sha_handle);

    if (!is_secure()) {
        // write to flash, this invalid key event
        // TODO: We can significantly reduce/eliminate the flash erase requirement,
        // by using unwritten parts of Flash to record incorrect pin count
        secure_block->num_incorrect_pins++;
        evt_src.signal(EVT_SECURE_BLOCK_UPDATE);
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Incorrect Pin, Num Tries Remaining: %d \n", MAX_NUM_INCORRECT_PINS - secure_block->num_incorrect_pins);
    } else if (secure_block->num_incorrect_pins != 0) {
        secure_block->num_incorrect_pins = 0;
        evt_src.signal(EVT_SECURE_BLOCK_UPDATE);
    }
    return true;
}

bool KeyManager::is_secure()
{
    if (secure_block->pin == nullptr || _input_pin == nullptr) {
        return false;
    }
    for (uint8_t i = 0; i < DIGEST_VALUE_LEN; i++) {
        if (secure_block->pin[i] != _input_pin[i]) {
            return false;
        }
    }
    return true;
}

void KeyManager::change_pin(uint32_t old_pin, uint32_t new_pin)
{
    if (secure_block == nullptr) {
        return;
    }
    if (new_pin > 1000000) {
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Only numbers between 1000 and 1000000 are supported.\n");
        return;
    }
    if (old_pin < 1000) {
        if (stm32_flash_is_rdp_enabled()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Can't set initial key in Secured State.\n");
            return;
        }
        for (uint8_t i=0; i < DIGEST_VALUE_LEN; i++) {
            if (secure_block->pin[i] != 0xFF) {
                GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Pin Exists.\n");
                return;
            }
        }
    } else {
        if (!set_input_pin(old_pin)) {
            return;
        }
    }

    pin_sha_handle = (wc_Sha256*)hal.util->malloc_type(sizeof(wc_Sha256), AP_HAL::Util::MEM_SECURE);
    reset_sha256(pin_sha_handle);
    update_sha256(pin_sha_handle, (const char*)&new_pin, sizeof(new_pin));
    final_sha256(pin_sha_handle, secure_block->pin);
    free(pin_sha_handle);
    
    secure_block->num_incorrect_pins = 0;
    evt_src.signal(EVT_SECURE_BLOCK_UPDATE);
    return;
}

void KeyManager::reset_sha256(wc_Sha256* sha_handle)
{
    if (sha_handle == nullptr) {
        return;
    }
    wc_Sha256Free(sha_handle);
    wc_InitSha256(sha_handle);
}

void KeyManager::update_sha256(wc_Sha256* sha_handle, const char* data, uint16_t data_len)
{
    if (sha_handle == nullptr) {
        return;
    }
    wc_Sha256Update(sha_handle, (uint8_t*)data, data_len);
}

void KeyManager::final_sha256(wc_Sha256* sha_handle, uint8_t* hash)
{
    if (sha_handle == nullptr) {
        return;
    }
    wc_Sha256Final(sha_handle, hash);
}

//Reads the embedded server public key and loads into the raw structure
void KeyManager::load_server_pubkey()
{
    word32 idx = 0;
    uint32_t server_pubkey_dersize = 0;
    const uint8_t *server_pubkey_der = AP_ROMFS::find_decompress(AP_SERVER_PUBLIC_KEY_FILE, server_pubkey_dersize);
    if (server_pubkey_der == NULL) {
        AP_HAL::panic("Failed to find Server Public Key!");;
    }
    int ret = wc_InitRsaKey(&server_pubkey, 0);
    if (ret == 0) {
        ret = wc_RsaPublicKeyDecode(server_pubkey_der, &idx, &server_pubkey, server_pubkey_dersize);
    }
    if (ret != 0) {
        AP_HAL::panic("Failed to load Server Public Key!");
    }
    _server_key_loaded = true;
    GCS_SEND_TEXT(MAV_SEVERITY_ALERT,  "KeyManager: Server Public Key Loaded.\n");
}

//Verifies if the given hash is authentic wrt server's public key
//return from this method is similar as in Openssl's Verify Key method
int KeyManager::verify_hash_with_server_pkey(const char* hashed_data, uint16_t hashed_data_len, const uint8_t* signature, uint16_t signature_len)
{
    int ret = 0;
    if (!_server_key_loaded) {
        return -1;
    }
    byte *digest_buf;
    word32 digest_len;

    /* Check arguments */
    if (hashed_data == NULL || hashed_data_len <= 0 || signature == NULL || signature_len <= 0) {
        ret = -1;
        goto end;
    }

    /* Validate signature len (1 to max is okay) */
    if ((int)signature_len > wc_SignatureGetSize(WC_SIGNATURE_TYPE_RSA_W_ENC, &server_pubkey, sizeof(server_pubkey))) {
        ret = -1;
        goto end;
    }

    //create der encode from the raw data digest we recieved
    ret = wc_HashGetOID(WC_HASH_TYPE_SHA256);
    if (ret > 0) {
        int oid = ret;

        /* Allocate buffer for hash and max DER encoded */
        digest_len = signature_len + MAX_DER_DIGEST_SZ;
        digest_buf = (byte*)malloc(digest_len);
        if (digest_buf) {
            ret = wc_EncodeSignature(digest_buf, (const uint8_t*)hashed_data, hashed_data_len, oid);
            if (ret > 0) {
                digest_len = ret;
            }
            else {
                free(digest_buf);
            }
        }
        else {
            ret = -1;
            goto end;
        }
    } else {
        ret = -1;
        goto end;
    }

    {
        word32 plain_len = digest_len;
        byte *plain_data;

        /* Make sure the plain text output is at least key size */
        if (plain_len < signature_len) {
            plain_len = signature_len;
        }
        plain_data = (byte*)malloc(plain_len);
        if (plain_data) {
            /* Perform verification of signature using provided RSA key */
            do {
            if (ret >= 0)
                ret = wc_RsaSSL_Verify(signature, signature_len, plain_data,
                    plain_len, &server_pubkey);
            } while (ret == WC_PENDING_E);
            if (ret >= 0) {
                if ((word32)ret == digest_len &&
                        memcmp(plain_data, digest_buf, digest_len) == 0) {
                    ret = 1; /* Success */
                }
                else {
                    ret = 0;
                }
            }
            free(plain_data);
        }
    }

end:
    return ret;
}

bool KeyManager::_flash_read(uint32_t addr, void *data, uint32_t length)
{
    memcpy(data, (void*)addr, length);
    return true;
}

bool KeyManager::sign_data_with_ap_key(uint8_t* data, uint32_t data_len, uint8_t* signature) {
    WC_RNG rng;
    int ret = 0;

    ret = wc_InitRng(&rng);
    if (ret != 0) {
        return false;
    }

    byte encoded_buf[DIGEST_VALUE_LEN+44];
    word32 encoded_buf_len = sizeof(encoded_buf);

    encoded_buf_len = wc_EncodeSignature(encoded_buf, data, data_len, SHA256h);

    ret = wc_RsaSSL_Sign(encoded_buf, encoded_buf_len, signature, 256, &ap_key, &rng);
    if (ret < 0) {
        char error_str[WOLFSSL_MAX_ERROR_SZ];
        wc_ErrorString(ret, error_str);
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "KeyManager: Failed to Generate RSA Key Pair: %d %s\n", ret, error_str);
        wc_FreeRng(&rng);
        return false;
    }
    wc_FreeRng(&rng);
    return true;
}

// singleton instance
KeyManager* KeyManager::_singleton;

namespace AP {

KeyManager &keymgr()
{
    return *KeyManager::get_singleton();
}

}

void show_gen_printf(const char* chr)
{
    hal.console->printf("%s",chr);
}

#endif //HAL_IS_REGISTERED_FLIGHT_MODULE

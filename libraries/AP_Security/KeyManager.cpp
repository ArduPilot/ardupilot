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
#ifdef HAL_DIGITAL_SKY_RFM

#include "KeyManager.h"
#include <AP_ROMFS/AP_ROMFS.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>
#include <string.h>

#ifndef AP_SERVER_PUBLIC_KEY_FILE
#define AP_SERVER_PUBLIC_KEY_FILE "server_pubkey.der"
#endif

extern const AP_HAL::HAL& hal;

static KeyManager _keymgr;


KeyManager::KeyManager()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many KeyManager modules");
        return;
    }
    _singleton = this;
}

void KeyManager::init() {
    // Load Server's Public Key for Verification
    load_server_pubkey();
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
#include <npnt.h>
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

#endif //#ifdef HAL_DIGITAL_SKY_RFM

#include "AP_DiskCrypto_SW.h"

#if AP_DISKCRYPTO_ENABLED

#include <string.h>

void AP_DiskCrypto_SW::set_key(const uint8_t key[32])
{
    AES_init_ctx(&_ctx, key);
}

void AP_DiskCrypto_SW::encrypt_block(const uint8_t in[16], uint8_t out[16])
{
    if (out != in) {
        memcpy(out, in, 16);
    }
    AES_ECB_encrypt(&_ctx, out);
}

void AP_DiskCrypto_SW::decrypt_block(const uint8_t in[16], uint8_t out[16])
{
    if (out != in) {
        memcpy(out, in, 16);
    }
    AES_ECB_decrypt(&_ctx, out);
}

#endif  // AP_DISKCRYPTO_ENABLED

/*
   Simple XOR Encryption - Streaming Implementation
   
   Supports large files by processing data in chunks
 */

#include "AP_Crypto_Simple_Streaming.h"

#if AP_CRYPTO_ENABLED

#include <string.h>
#include <AP_CheckFirmware/monocypher.h>  // For BLAKE2b (or implement SHA-256)

bool AP_Crypto_Simple_Streaming::streaming_encrypt_init(SimpleXORStreamEncrypt *ctx, const uint8_t key[32])
{
    if (ctx == nullptr || key == nullptr) {
        return false;
    }
    
    memset(ctx, 0, sizeof(*ctx));
    memcpy(ctx->key, key, 32);
    ctx->counter = 0;
    ctx->bytes_encrypted = 0;
    ctx->initialized = true;
    
    return true;
}

ssize_t AP_Crypto_Simple_Streaming::streaming_encrypt_write(SimpleXORStreamEncrypt *ctx,
                                                            const uint8_t *plaintext, size_t plaintext_len,
                                                            uint8_t *ciphertext_out, size_t ciphertext_max)
{
    if (ctx == nullptr || !ctx->initialized || plaintext == nullptr || ciphertext_out == nullptr) {
        return -1;
    }
    
    if (plaintext_len > ciphertext_max) {
        return -1;
    }
    
    size_t offset = 0;
    
    while (offset < plaintext_len) {
        // Generate keystream block for current counter
        uint8_t keystream[32];
        if (!AP_Crypto_Simple::generate_keystream_block(ctx->key, ctx->counter, keystream)) {
            return -1;
        }
        
        // XOR with plaintext
        size_t block_size = (plaintext_len - offset < 32) ? (plaintext_len - offset) : 32;
        for (size_t i = 0; i < block_size; i++) {
            ciphertext_out[offset + i] = plaintext[offset + i] ^ keystream[i];
        }
        
        offset += block_size;
        ctx->counter++;
    }
    
    ctx->bytes_encrypted += plaintext_len;
    return plaintext_len;
}

void AP_Crypto_Simple_Streaming::streaming_encrypt_cleanup(SimpleXORStreamEncrypt *ctx)
{
    if (ctx == nullptr) {
        return;
    }
    
    // Wipe sensitive data
    memset(ctx->key, 0, sizeof(ctx->key));
    memset(ctx, 0, sizeof(*ctx));
}

bool AP_Crypto_Simple_Streaming::streaming_decrypt_init(SimpleXORStreamDecrypt *ctx, const uint8_t key[32])
{
    if (ctx == nullptr || key == nullptr) {
        return false;
    }
    
    memset(ctx, 0, sizeof(*ctx));
    memcpy(ctx->key, key, 32);
    ctx->counter = 0;
    ctx->bytes_decrypted = 0;
    ctx->initialized = true;
    
    return true;
}

ssize_t AP_Crypto_Simple_Streaming::streaming_decrypt_read(SimpleXORStreamDecrypt *ctx,
                                                           const uint8_t *ciphertext, size_t ciphertext_len,
                                                           uint8_t *plaintext_out, size_t plaintext_max)
{
    // Decryption is identical to encryption (XOR is symmetric)
    if (ctx == nullptr || !ctx->initialized || ciphertext == nullptr || plaintext_out == nullptr) {
        return -1;
    }
    
    if (ciphertext_len > plaintext_max) {
        return -1;
    }
    
    size_t offset = 0;
    
    while (offset < ciphertext_len) {
        // Generate keystream block for current counter
        uint8_t keystream[32];
        if (!AP_Crypto_Simple::generate_keystream_block(ctx->key, ctx->counter, keystream)) {
            return -1;
        }
        
        // XOR with ciphertext
        size_t block_size = (ciphertext_len - offset < 32) ? (ciphertext_len - offset) : 32;
        for (size_t i = 0; i < block_size; i++) {
            plaintext_out[offset + i] = ciphertext[offset + i] ^ keystream[i];
        }
        
        offset += block_size;
        ctx->counter++;
    }
    
    ctx->bytes_decrypted += ciphertext_len;
    return ciphertext_len;
}

void AP_Crypto_Simple_Streaming::streaming_decrypt_cleanup(SimpleXORStreamDecrypt *ctx)
{
    if (ctx == nullptr) {
        return;
    }
    
    // Wipe sensitive data
    memset(ctx->key, 0, sizeof(ctx->key));
    memset(ctx, 0, sizeof(*ctx));
}

#endif  // AP_CRYPTO_ENABLED



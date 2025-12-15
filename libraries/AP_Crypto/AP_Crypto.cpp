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

#include "AP_Crypto.h"

#if defined(AP_CRYPTO_ENABLED) && AP_CRYPTO_ENABLED

#include <AP_CheckFirmware/monocypher.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <StorageManager/StorageManager.h>
#include <string.h>
#include <sys/stat.h>
#include <limits.h>

extern const AP_HAL::HAL& hal;

// Base64URL alphabet (RFC 4648 Section 5)
static const char base64url_table[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

// Fernet token format (simplified, using ChaCha20-Poly1305):
// [Version: 1 byte] [Timestamp: 8 bytes] [Nonce: 24 bytes] [Ciphertext: variable] [MAC: 16 bytes]
// All encoded as base64url

#define FERNET_VERSION 0x80
#define FERNET_TIMESTAMP_SIZE 8
#define FERNET_NONCE_SIZE 24
#define FERNET_MAC_SIZE 16
#define FERNET_HEADER_SIZE (1 + FERNET_TIMESTAMP_SIZE + FERNET_NONCE_SIZE)

size_t AP_Crypto::base64url_encode(const uint8_t *data, size_t data_len,
                                   char *output, size_t output_max)
{
    // Base64URL encoding without padding (as per RFC 4648 Section 5)
    size_t output_len = ((data_len + 2) / 3) * 4;
    if (output_max < output_len + 1) {
        return 0; // Buffer too small
    }
    
    size_t i = 0;
    size_t j = 0;
    
    while (i < data_len) {
        uint32_t b0 = data[i++];
        uint32_t b1 = (i < data_len) ? data[i++] : 0;
        uint32_t b2 = (i < data_len) ? data[i++] : 0;
        
        uint32_t bitmap = (b0 << 16) | (b1 << 8) | b2;
        
        output[j++] = base64url_table[(bitmap >> 18) & 0x3F];
        output[j++] = base64url_table[(bitmap >> 12) & 0x3F];
        
        if (i - 2 < data_len) {
            output[j++] = base64url_table[(bitmap >> 6) & 0x3F];
        }
        
        if (i - 1 < data_len) {
            output[j++] = base64url_table[bitmap & 0x3F];
        }
    }
    
    output[j] = '\0';
    return j;
}

size_t AP_Crypto::base64url_decode(const char *input, size_t input_len,
                                   uint8_t *output, size_t output_max)
{
    // Build decode table
    uint8_t dtable[256];
    memset(dtable, 0x80, 256);
    for (size_t i = 0; i < 64; i++) {
        dtable[(uint8_t)base64url_table[i]] = i;
    }
    
    // Count valid characters (skip padding and invalid chars)
    size_t valid_count = 0;
    for (size_t i = 0; i < input_len; i++) {
        if (dtable[(uint8_t)input[i]] != 0x80) {
            valid_count++;
        }
    }
    
    if (valid_count == 0) {
        return 0;
    }
    
    // Calculate output length (handle padding)
    size_t output_len = (valid_count * 3) / 4;
    if (output_len > output_max) {
        return 0;
    }
    
    size_t j = 0;
    uint8_t block[4];
    size_t block_idx = 0;
    
    for (size_t i = 0; i < input_len && j < output_len; i++) {
        uint8_t c = dtable[(uint8_t)input[i]];
        if (c == 0x80) {
            continue; // Skip invalid characters
        }
        
        block[block_idx++] = c;
        
        if (block_idx == 4) {
            output[j++] = (block[0] << 2) | (block[1] >> 4);
            if (j < output_len) {
                output[j++] = (block[1] << 4) | (block[2] >> 2);
            }
            if (j < output_len) {
                output[j++] = (block[2] << 6) | block[3];
            }
            block_idx = 0;
        }
    }
    
    // Handle remaining bytes
    if (block_idx > 0) {
        output[j++] = (block[0] << 2) | (block[1] >> 4);
        if (block_idx > 2 && j < output_len) {
            output[j++] = (block[1] << 4) | (block[2] >> 2);
        }
    }
    
    return j;
}

bool AP_Crypto::decode_key(const char *key_b64, uint8_t *key_bytes)
{
    size_t decoded_len = base64url_decode(key_b64, strlen(key_b64), key_bytes, 32);
    return decoded_len == 32;
}

bool AP_Crypto::generate_key(char *key_out, size_t key_out_max)
{
    if (key_out_max < 44) { // 32 bytes base64url encoded = 44 chars + null
        return false;
    }
    
    // Generate 32 random bytes
    uint8_t key_bytes[32];
    for (size_t i = 0; i < 32; i++) {
        // Use a combination of time and micros for randomness
        // In production, use proper RNG
        uint32_t seed = AP_HAL::millis() ^ (AP_HAL::micros() << (i % 16));
        key_bytes[i] = (uint8_t)(seed ^ (i * 0x9E3779B9));
    }
    
    // Encode as base64url
    size_t encoded_len = base64url_encode(key_bytes, 32, key_out, key_out_max);
    return encoded_len > 0;
}

int AP_Crypto::encode(const char *key,
                     const uint8_t *plaintext, size_t plaintext_len,
                     char *ciphertext, size_t ciphertext_max)
{
    if (key == nullptr || plaintext == nullptr || ciphertext == nullptr) {
        return -1;
    }
    
    // Decode key from base64url
    uint8_t key_bytes[32];
    if (!decode_key(key, key_bytes)) {
        return -1;
    }
    
    // Generate nonce (24 bytes for ChaCha20-Poly1305)
    uint8_t nonce[24];
    uint32_t seed = AP_HAL::millis() ^ AP_HAL::micros();
    for (size_t i = 0; i < 24; i++) {
        nonce[i] = (uint8_t)(seed ^ (i * 0x9E3779B9));
        seed = (seed << 1) | (seed >> 31);
    }
    
    // Get current timestamp (8 bytes, big-endian)
    uint64_t timestamp = AP_HAL::millis64();
    uint8_t timestamp_bytes[8];
    for (int i = 7; i >= 0; i--) {
        timestamp_bytes[i] = timestamp & 0xFF;
        timestamp >>= 8;
    }
    
    // Encrypt using ChaCha20-Poly1305
    uint8_t mac[16];
    uint8_t *ciphertext_bytes = (uint8_t*)hal.util->malloc_type(plaintext_len, AP_HAL::Util::MEM_DMA_SAFE);
    if (ciphertext_bytes == nullptr) {
        return -1;
    }
    
    crypto_lock(mac, ciphertext_bytes, key_bytes, nonce, plaintext, plaintext_len);
    
    // Build token: version + timestamp + nonce + ciphertext + mac
    size_t token_size = FERNET_HEADER_SIZE + plaintext_len + FERNET_MAC_SIZE;
    uint8_t *token = (uint8_t*)hal.util->malloc_type(token_size, AP_HAL::Util::MEM_DMA_SAFE);
    if (token == nullptr) {
        hal.util->free_type(ciphertext_bytes, plaintext_len, AP_HAL::Util::MEM_DMA_SAFE);
        return -1;
    }
    
    size_t pos = 0;
    token[pos++] = FERNET_VERSION;
    memcpy(&token[pos], timestamp_bytes, FERNET_TIMESTAMP_SIZE);
    pos += FERNET_TIMESTAMP_SIZE;
    memcpy(&token[pos], nonce, FERNET_NONCE_SIZE);
    pos += FERNET_NONCE_SIZE;
    memcpy(&token[pos], ciphertext_bytes, plaintext_len);
    pos += plaintext_len;
    memcpy(&token[pos], mac, FERNET_MAC_SIZE);
    pos += FERNET_MAC_SIZE;
    
    // Encode token as base64url
    size_t encoded_len = base64url_encode(token, token_size, ciphertext, ciphertext_max);
    
    hal.util->free_type(ciphertext_bytes, plaintext_len, AP_HAL::Util::MEM_DMA_SAFE);
    hal.util->free_type(token, token_size, AP_HAL::Util::MEM_DMA_SAFE);
    
    if (encoded_len == 0) {
        return -1;
    }
    
    return encoded_len;
}

int AP_Crypto::decode(const char *key,
                     const char *ciphertext, size_t ciphertext_len,
                     uint8_t *plaintext, size_t plaintext_max)
{
    if (key == nullptr || ciphertext == nullptr || plaintext == nullptr) {
        return -1;
    }
    
    // Decode key from base64url
    uint8_t key_bytes[32];
    if (!decode_key(key, key_bytes)) {
        return -1;
    }
    
    // Decode token from base64url
    size_t max_token_size = (ciphertext_len * 3) / 4 + 4; // Base64 expansion
    uint8_t *token = (uint8_t*)hal.util->malloc_type(max_token_size, AP_HAL::Util::MEM_DMA_SAFE);
    if (token == nullptr) {
        return -1;
    }
    
    size_t token_size = base64url_decode(ciphertext, ciphertext_len, token, max_token_size);
    if (token_size < FERNET_HEADER_SIZE + FERNET_MAC_SIZE) {
        hal.util->free_type(token, max_token_size, AP_HAL::Util::MEM_DMA_SAFE);
        return -1;
    }
    
    // Parse token
    size_t pos = 0;
    if (token[pos++] != FERNET_VERSION) {
        hal.util->free_type(token, max_token_size, AP_HAL::Util::MEM_DMA_SAFE);
        return -1; // Wrong version
    }
    
    // Skip timestamp (we don't validate it for now)
    pos += FERNET_TIMESTAMP_SIZE;
    
    // Extract nonce
    uint8_t nonce[24];
    memcpy(nonce, &token[pos], FERNET_NONCE_SIZE);
    pos += FERNET_NONCE_SIZE;
    
    // Extract ciphertext and MAC
    size_t ciphertext_data_len = token_size - FERNET_HEADER_SIZE - FERNET_MAC_SIZE;
    if (ciphertext_data_len > plaintext_max) {
        hal.util->free_type(token, max_token_size, AP_HAL::Util::MEM_DMA_SAFE);
        return -1;
    }
    
    uint8_t *ciphertext_data = &token[pos];
    uint8_t *mac = &token[pos + ciphertext_data_len];
    
    // Decrypt
    int result = crypto_unlock(plaintext, key_bytes, nonce, mac,
                              ciphertext_data, ciphertext_data_len);
    
    hal.util->free_type(token, max_token_size, AP_HAL::Util::MEM_DMA_SAFE);
    
    if (result != 0) {
        return -1; // Decryption/MAC verification failed
    }
    
    return ciphertext_data_len;
}

bool AP_Crypto::streaming_encrypt_init(StreamingEncrypt *ctx, const char *key)
{
    if (ctx == nullptr || key == nullptr) {
        return false;
    }
    
    memset(ctx, 0, sizeof(*ctx));
    
    // Decode key from base64url
    if (!decode_key(key, ctx->key)) {
        return false;
    }
    
    // Generate nonce (24 bytes for ChaCha20-Poly1305)
    uint32_t seed = AP_HAL::millis() ^ AP_HAL::micros();
    for (size_t i = 0; i < 24; i++) {
        ctx->nonce[i] = (uint8_t)(seed ^ (i * 0x9E3779B9));
        seed = (seed << 1) | (seed >> 31);
    }
    
    // Get current timestamp (8 bytes, big-endian)
    uint64_t timestamp = AP_HAL::millis64();
    for (int i = 7; i >= 0; i--) {
        ctx->timestamp[i] = timestamp & 0xFF;
        timestamp >>= 8;
    }
    
    // Derive keys for streaming encryption (same as crypto_lock does)
    // Derive sub-key from main key and first 16 bytes of nonce
    crypto_hchacha20(ctx->sub_key, ctx->key, ctx->nonce);
    
    // Generate auth key from sub-key and remaining 8 bytes of nonce
    crypto_chacha20(ctx->auth_key, nullptr, 64, ctx->sub_key, ctx->nonce + 16);
    
    // Initialize Poly1305 for incremental MAC computation
    crypto_poly1305_init(&ctx->poly1305_ctx, ctx->auth_key);
    
    // Initialize streaming state
    ctx->encrypt_counter = 1; // ChaCha20-CTR starts at 1 (0 is used for auth key)
    ctx->bytes_encrypted = 0;
    ctx->initialized = true;
    
    return true;
}

bool AP_Crypto::streaming_encrypt_write_header(StreamingEncrypt *ctx, int fd)
{
    if (ctx == nullptr || fd < 0 || !ctx->initialized) {
        return false;
    }
    
    // Write header: version + timestamp + nonce
    uint8_t header[FERNET_HEADER_SIZE];
    size_t pos = 0;
    header[pos++] = FERNET_VERSION;
    memcpy(&header[pos], ctx->timestamp, FERNET_TIMESTAMP_SIZE);
    pos += FERNET_TIMESTAMP_SIZE;
    memcpy(&header[pos], ctx->nonce, FERNET_NONCE_SIZE);
    pos += FERNET_NONCE_SIZE;
    
    ssize_t written = AP::FS().write(fd, header, sizeof(header));
    return written == sizeof(header);
}

ssize_t AP_Crypto::streaming_encrypt_write(StreamingEncrypt *ctx, int fd,
                                          const uint8_t *plaintext, size_t plaintext_len)
{
    if (ctx == nullptr || fd < 0 || plaintext == nullptr || plaintext_len == 0 || !ctx->initialized) {
        return -1;
    }
    
    // Allocate buffer for encrypted chunk
    uint8_t *ciphertext = (uint8_t*)hal.util->malloc_type(plaintext_len, AP_HAL::Util::MEM_DMA_SAFE);
    if (ciphertext == nullptr) {
        return -1;
    }
    
    // Encrypt this chunk using ChaCha20-CTR
    // Use the remaining 8 bytes of nonce (nonce[16:24]) for ChaCha20-CTR
    uint64_t new_counter = crypto_chacha20_ctr(
        ciphertext,              // output ciphertext
        plaintext,               // input plaintext
        plaintext_len,           // size
        ctx->sub_key,            // key (derived sub-key)
        ctx->nonce + 16,         // nonce (last 8 bytes)
        ctx->encrypt_counter     // counter
    );
    
    // Update Poly1305 MAC with this ciphertext chunk
    crypto_poly1305_update(&ctx->poly1305_ctx, ciphertext, plaintext_len);
    
    // Write encrypted chunk to file
    ssize_t written = AP::FS().write(fd, ciphertext, plaintext_len);
    
    hal.util->free_type(ciphertext, plaintext_len, AP_HAL::Util::MEM_DMA_SAFE);
    
    if (written <= 0) {
        return -1;
    }
    
    // Update state
    ctx->encrypt_counter = new_counter;
    ctx->bytes_encrypted += plaintext_len;
    
    return written;
}

bool AP_Crypto::streaming_encrypt_finalize(StreamingEncrypt *ctx, int fd)
{
    if (ctx == nullptr || fd < 0 || !ctx->initialized) {
        return false;
    }
    
    // Finalize Poly1305 MAC
    uint8_t mac[FERNET_MAC_SIZE];
    crypto_poly1305_final(&ctx->poly1305_ctx, mac);
    
    // Write MAC at the end of the file
    ssize_t written = AP::FS().write(fd, mac, FERNET_MAC_SIZE);
    if (written != FERNET_MAC_SIZE) {
        return false;
    }
    
    // Sync file
    AP::FS().fsync(fd);
    
    return true;
}

void AP_Crypto::streaming_encrypt_cleanup(StreamingEncrypt *ctx)
{
    if (ctx == nullptr) {
        return;
    }
    
    // Wipe sensitive data
    crypto_wipe(ctx->key, sizeof(ctx->key));
    crypto_wipe(ctx->sub_key, sizeof(ctx->sub_key));
    crypto_wipe(ctx->auth_key, sizeof(ctx->auth_key));
    memset(ctx, 0, sizeof(*ctx));
}

bool AP_Crypto::streaming_decrypt_init(StreamingDecrypt *ctx, const char *key, int fd)
{
    if (ctx == nullptr || key == nullptr || fd < 0) {
        return false;
    }
    
    memset(ctx, 0, sizeof(*ctx));
    
    // Decode key from base64url
    if (!decode_key(key, ctx->key)) {
        return false;
    }
    
    // Read header: version + timestamp + nonce
    uint8_t header[FERNET_HEADER_SIZE];
    ssize_t bytes_read = AP::FS().read(fd, header, sizeof(header));
    if (bytes_read != sizeof(header)) {
        return false;
    }
    
    size_t pos = 0;
    if (header[pos++] != FERNET_VERSION) {
        return false; // Wrong version
    }
    
    // Skip timestamp (we don't validate it for now)
    pos += FERNET_TIMESTAMP_SIZE;
    
    // Extract nonce
    memcpy(ctx->nonce, &header[pos], FERNET_NONCE_SIZE);
    pos += FERNET_NONCE_SIZE;
    
    // Derive keys for streaming decryption (same as crypto_unlock does)
    crypto_hchacha20(ctx->sub_key, ctx->key, ctx->nonce);
    crypto_chacha20(ctx->auth_key, nullptr, 64, ctx->sub_key, ctx->nonce + 16);
    
    // Initialize Poly1305 for incremental MAC computation
    crypto_poly1305_init(&ctx->poly1305_ctx, ctx->auth_key);
    
    // Initialize streaming state
    ctx->decrypt_counter = 1; // ChaCha20-CTR starts at 1
    ctx->bytes_decrypted = 0;
    ctx->initialized = true;
    
    // Try to seek to end to get file size
    int32_t file_end = AP::FS().lseek(fd, 0, SEEK_END);
    if (file_end >= 0 && (size_t)file_end >= FERNET_MAC_SIZE) {
        ctx->expected_mac_pos = file_end - FERNET_MAC_SIZE;
    } else {
        // Can't determine, will read until EOF
        ctx->expected_mac_pos = SIZE_MAX;
    }
    
    // Seek back to after header
    AP::FS().lseek(fd, FERNET_HEADER_SIZE, SEEK_SET);
    
    return true;
}

ssize_t AP_Crypto::streaming_decrypt_read(StreamingDecrypt *ctx, int fd,
                                         uint8_t *plaintext, size_t plaintext_max)
{
    if (ctx == nullptr || fd < 0 || plaintext == nullptr || plaintext_max == 0 || !ctx->initialized) {
        return -1;
    }
    
    // Check current position - if we're at MAC position, don't read more
    int32_t current_pos = AP::FS().lseek(fd, 0, SEEK_CUR);
    if (current_pos < 0) {
        return -1;
    }
    
    if (ctx->expected_mac_pos != SIZE_MAX && (size_t)current_pos >= ctx->expected_mac_pos) {
        return 0; // At MAC position, no more data
    }
    
    // Calculate how much to read (leave room for MAC)
    size_t max_read = plaintext_max;
    if (ctx->expected_mac_pos != SIZE_MAX) {
        size_t remaining = ctx->expected_mac_pos - (size_t)current_pos;
        max_read = (max_read < remaining) ? max_read : remaining;
    }
    
    // Read ciphertext chunk
    uint8_t *ciphertext = (uint8_t*)hal.util->malloc_type(max_read, AP_HAL::Util::MEM_DMA_SAFE);
    if (ciphertext == nullptr) {
        return -1;
    }
    
    ssize_t bytes_read = AP::FS().read(fd, ciphertext, max_read);
    if (bytes_read <= 0) {
        hal.util->free_type(ciphertext, max_read, AP_HAL::Util::MEM_DMA_SAFE);
        return bytes_read; // EOF or error
    }
    
    // Update Poly1305 MAC with this ciphertext chunk
    crypto_poly1305_update(&ctx->poly1305_ctx, ciphertext, bytes_read);
    
    // Decrypt this chunk using ChaCha20-CTR
    uint64_t new_counter = crypto_chacha20_ctr(
        plaintext,              // output plaintext
        ciphertext,             // input ciphertext
        bytes_read,             // size
        ctx->sub_key,           // key (derived sub-key)
        ctx->nonce + 16,        // nonce (last 8 bytes)
        ctx->decrypt_counter    // counter
    );
    
    hal.util->free_type(ciphertext, max_read, AP_HAL::Util::MEM_DMA_SAFE);
    
    // Update state
    ctx->decrypt_counter = new_counter;
    ctx->bytes_decrypted += bytes_read;
    
    return bytes_read;
}

bool AP_Crypto::streaming_decrypt_finalize(StreamingDecrypt *ctx, int fd)
{
    if (ctx == nullptr || fd < 0 || !ctx->initialized) {
        return false;
    }
    
    // Read MAC from end of file
    uint8_t mac[FERNET_MAC_SIZE];
    ssize_t bytes_read = AP::FS().read(fd, mac, FERNET_MAC_SIZE);
    if (bytes_read != FERNET_MAC_SIZE) {
        return false;
    }
    
    // Finalize Poly1305 MAC
    uint8_t computed_mac[FERNET_MAC_SIZE];
    crypto_poly1305_final(&ctx->poly1305_ctx, computed_mac);
    
    // Verify MAC
    if (crypto_verify16(mac, computed_mac) != 0) {
        return false; // MAC verification failed
    }
    
    return true;
}

void AP_Crypto::streaming_decrypt_cleanup(StreamingDecrypt *ctx)
{
    if (ctx == nullptr) {
        return;
    }
    
    // Wipe sensitive data
    crypto_wipe(ctx->key, sizeof(ctx->key));
    crypto_wipe(ctx->sub_key, sizeof(ctx->sub_key));
    crypto_wipe(ctx->auth_key, sizeof(ctx->auth_key));
    memset(ctx, 0, sizeof(*ctx));
}

// Key storage constants
#define CRYPTO_KEY_MAGIC 0x43525950  // "CRYP" in ASCII
#define CRYPTO_KEY_VERSION 1
#define CRYPTO_KEY_STORAGE_OFFSET 0  // Offset within StorageKeys area
#define CRYPTO_KEY_STORAGE_SIZE 32   // 32 bytes for key

struct crypto_key_header {
    uint32_t magic;      // CRYPTO_KEY_MAGIC
    uint32_t version;    // CRYPTO_KEY_VERSION
    uint8_t key[32];     // The actual key
};

bool AP_Crypto::store_key(const uint8_t key[32])
{
    if (key == nullptr) {
        return false;
    }
    
    StorageAccess storage(StorageManager::StorageKeys);
    if (storage.size() < sizeof(crypto_key_header)) {
        return false; // Not enough storage space
    }
    
    crypto_key_header header;
    header.magic = CRYPTO_KEY_MAGIC;
    header.version = CRYPTO_KEY_VERSION;
    memcpy(header.key, key, 32);
    
    // Write to storage
    if (!storage.write_block(CRYPTO_KEY_STORAGE_OFFSET, &header, sizeof(header))) {
        return false;
    }
    
    return true;
}

bool AP_Crypto::retrieve_key(uint8_t key[32])
{
    if (key == nullptr) {
        return false;
    }
    
    StorageAccess storage(StorageManager::StorageKeys);
    if (storage.size() < sizeof(crypto_key_header)) {
        return false; // Not enough storage space
    }
    
    crypto_key_header header;
    if (!storage.read_block(&header, CRYPTO_KEY_STORAGE_OFFSET, sizeof(header))) {
        return false;
    }
    
    // Verify magic and version
    if (header.magic != CRYPTO_KEY_MAGIC || header.version != CRYPTO_KEY_VERSION) {
        return false; // Invalid or uninitialized key
    }
    
    memcpy(key, header.key, 32);
    return true;
}

bool AP_Crypto::has_stored_key(void)
{
    StorageAccess storage(StorageManager::StorageKeys);
    if (storage.size() < sizeof(crypto_key_header)) {
        return false;
    }
    
    uint32_t magic = storage.read_uint32(CRYPTO_KEY_STORAGE_OFFSET);
    return (magic == CRYPTO_KEY_MAGIC);
}

bool AP_Crypto::generate_and_store_key(uint8_t key[32])
{
    // Generate random key
    uint8_t new_key[32];
    for (size_t i = 0; i < 32; i++) {
        // Use a combination of time and micros for randomness
        // In production, use proper RNG
        uint32_t seed = AP_HAL::millis() ^ (AP_HAL::micros() << (i % 16));
        new_key[i] = (uint8_t)(seed ^ (i * 0x9E3779B9));
        // Mix in more entropy
        seed = (seed << 1) | (seed >> 31);
        new_key[i] ^= (uint8_t)(seed & 0xFF);
    }
    
    // Store the key
    if (!store_key(new_key)) {
        return false;
    }
    
    // Return key if requested
    if (key != nullptr) {
        memcpy(key, new_key, 32);
    }
    
    return true;
}

bool AP_Crypto::derive_key_from_board_id(uint8_t key[32])
{
    // Board ID key derivation is disabled
    // Keys must be explicitly stored via AP_Crypto::store_key() or use the default key
    (void)key; // Suppress unused parameter warning
    return false;
}

#endif  // AP_CRYPTO_ENABLED


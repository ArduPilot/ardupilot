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

#if AP_CRYPTO_ENABLED

#include <stdint.h>
#include <stddef.h>
#include <AP_CheckFirmware/monocypher.h>
#include <AP_Filesystem/AP_Filesystem.h>

/*
  AP_Crypto - Cryptographic functions for ArduPilot

  Provides Fernet-compatible encryption/decryption for ASCII files.
  Note: Uses ChaCha20-Poly1305 instead of AES-128-CBC+HMAC-SHA256 for
  better performance and smaller code size. The format is similar but
  not 100% compatible with Python's cryptography.fernet.
 */
class AP_Crypto
{
public:
    /*
      Encode (encrypt) ASCII data using Fernet-compatible format
      
      @param key: 32-byte encryption key (base64url-encoded Fernet key)
      @param plaintext: Input plaintext data (ASCII)
      @param plaintext_len: Length of plaintext
      @param ciphertext: Output buffer for encrypted data (base64url-encoded)
      @param ciphertext_max: Maximum size of ciphertext buffer
      @return: Length of ciphertext written, or -1 on error
     */
    static int encode(const char *key,
                     const uint8_t *plaintext, size_t plaintext_len,
                     char *ciphertext, size_t ciphertext_max);

    /*
      Decode (decrypt) Fernet-compatible encrypted data
      
      @param key: 32-byte encryption key (base64url-encoded Fernet key)
      @param ciphertext: Input encrypted data (base64url-encoded)
      @param ciphertext_len: Length of ciphertext
      @param plaintext: Output buffer for decrypted data
      @param plaintext_max: Maximum size of plaintext buffer
      @return: Length of plaintext written, or -1 on error
     */
    static int decode(const char *key,
                     const char *ciphertext, size_t ciphertext_len,
                     uint8_t *plaintext, size_t plaintext_max);

    /*
      Generate a Fernet key (base64url-encoded 32-byte key)
      
      @param key_out: Output buffer (must be at least 44 bytes for base64url encoding)
      @param key_out_max: Maximum size of output buffer
      @return: true on success, false on failure
     */
    static bool generate_key(char *key_out, size_t key_out_max);

    /*
      Streaming encryption context for large files
     */
    struct StreamingEncrypt {
        uint8_t key[32];              // Decoded encryption key
        uint8_t nonce[24];            // Nonce for encryption
        uint8_t timestamp[8];         // Timestamp (big-endian)
        uint8_t sub_key[32];         // Derived sub-key for ChaCha20
        uint8_t auth_key[64];        // Auth key for Poly1305
        crypto_poly1305_ctx poly1305_ctx; // Poly1305 context for incremental MAC
        uint64_t encrypt_counter;     // Counter for ChaCha20-CTR encryption
        size_t bytes_encrypted;       // Total bytes encrypted so far
        bool initialized;             // Whether context is initialized
    };

    /*
      Initialize streaming encryption context
      
      @param ctx: Streaming encryption context
      @param key: Base64url-encoded 32-byte key
      @return: true on success, false on failure
     */
    static bool streaming_encrypt_init(StreamingEncrypt *ctx, const char *key);

    /*
      Write header for streaming encryption (version + timestamp + nonce)
      This should be called once before writing encrypted data
      
      @param ctx: Streaming encryption context
      @param fd: File descriptor to write to
      @return: true on success, false on failure
     */
    static bool streaming_encrypt_write_header(StreamingEncrypt *ctx, int fd);

    /*
      Encrypt and write a chunk of data (streaming)
      
      @param ctx: Streaming encryption context
      @param fd: File descriptor to write to
      @param plaintext: Plaintext data to encrypt
      @param plaintext_len: Length of plaintext data
      @return: Number of bytes written, or -1 on error
     */
    static ssize_t streaming_encrypt_write(StreamingEncrypt *ctx, int fd,
                                           const uint8_t *plaintext, size_t plaintext_len);

    /*
      Finalize streaming encryption (write MAC)
      
      @param ctx: Streaming encryption context
      @param fd: File descriptor to write to
      @return: true on success, false on failure
     */
    static bool streaming_encrypt_finalize(StreamingEncrypt *ctx, int fd);

    /*
      Cleanup streaming encryption context
      
      @param ctx: Streaming encryption context
     */
    static void streaming_encrypt_cleanup(StreamingEncrypt *ctx);

    /*
      Streaming decryption context for large files
     */
    struct StreamingDecrypt {
        uint8_t key[32];              // Decoded decryption key
        uint8_t nonce[24];            // Nonce from header
        uint8_t sub_key[32];         // Derived sub-key for ChaCha20
        uint8_t auth_key[64];        // Auth key for Poly1305
        crypto_poly1305_ctx poly1305_ctx; // Poly1305 context for incremental MAC
        uint64_t decrypt_counter;     // Counter for ChaCha20-CTR decryption
        size_t bytes_decrypted;       // Total bytes decrypted so far
        size_t expected_mac_pos;      // Expected position of MAC in file
        bool initialized;             // Whether context is initialized
    };

    /*
      Initialize streaming decryption context from file header
      
      @param ctx: Streaming decryption context
      @param key: Base64url-encoded 32-byte key
      @param fd: File descriptor (must be positioned at start of file)
      @return: true on success, false on failure
     */
    static bool streaming_decrypt_init(StreamingDecrypt *ctx, const char *key, int fd);

    /*
      Decrypt and read a chunk of data (streaming)
      
      @param ctx: Streaming decryption context
      @param fd: File descriptor to read from
      @param plaintext: Output buffer for decrypted data
      @param plaintext_max: Maximum size of plaintext buffer
      @return: Number of bytes decrypted, or -1 on error
     */
    static ssize_t streaming_decrypt_read(StreamingDecrypt *ctx, int fd,
                                         uint8_t *plaintext, size_t plaintext_max);

    /*
      Finalize streaming decryption (verify MAC)
      
      @param ctx: Streaming decryption context
      @param fd: File descriptor to read from
      @return: true on success (MAC verified), false on failure
     */
    static bool streaming_decrypt_finalize(StreamingDecrypt *ctx, int fd);

    /*
      Cleanup streaming decryption context
      
      @param ctx: Streaming decryption context
     */
    static void streaming_decrypt_cleanup(StreamingDecrypt *ctx);

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

private:
    // Internal helper to decode base64url-encoded key
    static bool decode_key(const char *key_b64, uint8_t *key_bytes);
    
    // Internal helper to encode to base64url
    static size_t base64url_encode(const uint8_t *data, size_t data_len,
                                   char *output, size_t output_max);
    
    // Internal helper to decode from base64url
    static size_t base64url_decode(const char *input, size_t input_len,
                                   uint8_t *output, size_t output_max);
};

#endif  // AP_CRYPTO_ENABLED


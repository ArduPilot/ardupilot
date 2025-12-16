/*
   Simple XOR Encryption - Streaming Support for Large Files
   
   This provides streaming encryption/decryption for large files like log files.
   It uses the same simple XOR algorithm but processes data in chunks.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#if defined(AP_CRYPTO_ENABLED) && AP_CRYPTO_ENABLED

/*
  Streaming encryption context for simple XOR cipher
  Supports large files by processing data in chunks
 */
struct SimpleXORStreamEncrypt {
    uint8_t key[32];           // 32-byte encryption key
    uint64_t counter;           // Current block counter (starts at 0)
    size_t bytes_encrypted;     // Total bytes encrypted so far
    bool initialized;          // Whether context is initialized
};

/*
  Streaming decryption context for simple XOR cipher
  Supports large files by processing data in chunks
 */
struct SimpleXORStreamDecrypt {
    uint8_t key[32];           // 32-byte decryption key
    uint64_t counter;           // Current block counter (starts at 0)
    size_t bytes_decrypted;     // Total bytes decrypted so far
    bool initialized;          // Whether context is initialized
};

class AP_Crypto_Simple_Streaming
{
public:
    /*
      Initialize streaming encryption context
      
      @param ctx: Encryption context (must be allocated by caller)
      @param key: 32-byte encryption key
      @return: true on success, false on failure
     */
    static bool streaming_encrypt_init(SimpleXORStreamEncrypt *ctx, const uint8_t key[32]);
    
    /*
      Encrypt a chunk of data (streaming)
      
      @param ctx: Encryption context
      @param plaintext: Input plaintext data
      @param plaintext_len: Length of plaintext
      @param ciphertext_out: Output buffer for encrypted data
      @param ciphertext_max: Maximum size of ciphertext buffer
      @return: Number of bytes encrypted, or -1 on error
     */
    static ssize_t streaming_encrypt_write(SimpleXORStreamEncrypt *ctx,
                                           const uint8_t *plaintext, size_t plaintext_len,
                                           uint8_t *ciphertext_out, size_t ciphertext_max);
    
    /*
      Cleanup streaming encryption context
      
      @param ctx: Encryption context
     */
    static void streaming_encrypt_cleanup(SimpleXORStreamEncrypt *ctx);
    
    /*
      Initialize streaming decryption context
      
      @param ctx: Decryption context (must be allocated by caller)
      @param key: 32-byte decryption key
      @return: true on success, false on failure
     */
    static bool streaming_decrypt_init(SimpleXORStreamDecrypt *ctx, const uint8_t key[32]);
    
    /*
      Decrypt a chunk of data (streaming)
      
      @param ctx: Decryption context
      @param ciphertext: Input encrypted data
      @param ciphertext_len: Length of ciphertext
      @param plaintext_out: Output buffer for decrypted data
      @param plaintext_max: Maximum size of plaintext buffer
      @return: Number of bytes decrypted, or -1 on error
     */
    static ssize_t streaming_decrypt_read(SimpleXORStreamDecrypt *ctx,
                                         const uint8_t *ciphertext, size_t ciphertext_len,
                                         uint8_t *plaintext_out, size_t plaintext_max);
    
    /*
      Cleanup streaming decryption context
      
      @param ctx: Decryption context
     */
    static void streaming_decrypt_cleanup(SimpleXORStreamDecrypt *ctx);
};

#endif  // AP_CRYPTO_ENABLED





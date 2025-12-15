/*
   Simple XOR Encryption - No Crypto Libraries, No MAC
   
   This is a simple encryption scheme that uses only standard C++ libraries.
   It's designed to work on first try and be easily compatible with Python.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#if AP_CRYPTO_ENABLED

/*
  Simple XOR encryption using counter-based keystream
  
  Algorithm:
  1. Key derivation: SHA256(LEIGH_KEY_INT32_bytes + salt) -> 32-byte key
  2. Keystream: For each 32-byte block, SHA256(key + counter) -> 32-byte keystream
  3. Encryption: XOR plaintext with keystream block by block
  
  File format: [Ciphertext: variable length]
  - No nonce, no MAC, no header
  - Just encrypted data
 */

class AP_Crypto_Simple
{
public:
    /*
      Derive 32-byte key from LEIGH_KEY INT32 value
      
      @param leigh_key_value: LEIGH_KEY parameter value (INT32)
      @param key_out: Output buffer for 32-byte key
      @return: true on success
     */
    static bool derive_key_from_leigh_key(int32_t leigh_key_value, uint8_t key_out[32]);
    
    /*
      Generate 32-byte keystream block for given counter
      
      @param key: 32-byte encryption key
      @param counter: Block counter (starts at 0)
      @param keystream_out: Output buffer for 32-byte keystream
      @return: true on success
     */
    static bool generate_keystream_block(const uint8_t key[32], uint64_t counter, uint8_t keystream_out[32]);
    
    /*
      Encrypt data using simple XOR cipher
      
      @param key: 32-byte encryption key
      @param plaintext: Input plaintext data
      @param plaintext_len: Length of plaintext
      @param ciphertext_out: Output buffer for encrypted data
      @param ciphertext_max: Maximum size of ciphertext buffer
      @return: Length of ciphertext written, or -1 on error
     */
    static int encrypt_simple(const uint8_t key[32],
                              const uint8_t *plaintext, size_t plaintext_len,
                              uint8_t *ciphertext_out, size_t ciphertext_max);
    
    /*
      Decrypt data using simple XOR cipher
      
      @param key: 32-byte encryption key
      @param ciphertext: Input encrypted data
      @param ciphertext_len: Length of ciphertext
      @param plaintext_out: Output buffer for decrypted data
      @param plaintext_max: Maximum size of plaintext buffer
      @return: Length of plaintext written, or -1 on error
     */
    static int decrypt_simple(const uint8_t key[32],
                              const uint8_t *ciphertext, size_t ciphertext_len,
                              uint8_t *plaintext_out, size_t plaintext_max);
};

#endif  // AP_CRYPTO_ENABLED



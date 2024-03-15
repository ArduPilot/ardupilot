#pragma once

/*
 * MIT License
 * 
 * Copyright (c) 2024 Blake West
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Remove this to disable the file
// Also in mavlink_helpers.h
// #define MAVLINK_NO_ENCRYPTION
#ifndef MAVLINK_NO_ENCRYPTION
#include <openssl/evp.h>
#include <openssl/rand.h>
#endif
#ifdef MAVLINK_USE_CXX_NAMESPACE
namespace mavlink {
#endif

#ifndef MAVLINK_NO_ENCRYPTION

// Random key for testing, use an actual secure key in practice
static const uint32_t mavlink_chacha20_256_uint32_arr_key[8] = {
    0x80f346ad, 0x0b700f24, 0xb53fbc01, 0x09166e1a,
    0xd88a544e, 0xaa36c51a, 0x597436b3, 0x043c084c
};

#define MAVLINK_CHACHA20_ERROR   false
#define MAVLINK_CHACHA20_SUCCESS true
#define EVP_SUCCESS 1

// Convert to the type that is expected for the key.
static const u_char * mavlink_chacha20_256_key = (const u_char *) mavlink_chacha20_256_uint32_arr_key;

// Chacha 20 uses a 128 bit IV or 16 btes
// Random u_chars for IV
static u_char  mavlink_chacha20_256_iv[16] = {
    0x0a, 0xf1, 0xda, 0x2b,  
    0x21, 0x71, 0x24, 0x22, 
    0xcd, 0x2d, 0x61, 0x23,
    0xde, 0xad, 0xbe, 0xef}; // Except this line

/**
 * Encrypt the mavlink payload
 *
 * @param plain_text plain text to be encrypted
 * @param plain_text_len Size of plain text
 * @param cipher_text Encrypted payload returned
 * @param cipher_text_len Size of encrypted payload
 */
static inline bool mavlink_chacha20_256_encrypt(const u_char * plain_text, const uint8_t &plain_text_len, 
    u_char * cipher_text, uint8_t &cipher_text_len) {

    // Create an encryption context
    EVP_CIPHER_CTX *encryption_context = EVP_CIPHER_CTX_new();
    
    int _cipher_text_len, final_len;

    // If we have failed to create a context return
    if (!encryption_context) return MAVLINK_CHACHA20_ERROR;

    // Initialize encryption context with chacha20 and our IV + Key
    if (EVP_EncryptInit_ex(encryption_context, EVP_chacha20(), nullptr, 
        mavlink_chacha20_256_key, mavlink_chacha20_256_iv) != EVP_SUCCESS) {
        return MAVLINK_CHACHA20_ERROR; }

    // encrypt our plain text
    if (EVP_EncryptUpdate(encryption_context, cipher_text, &_cipher_text_len, plain_text, 
        plain_text_len) != EVP_SUCCESS) {
        return MAVLINK_CHACHA20_ERROR; }
    
    // Finalize encryption
    if (EVP_EncryptFinal_ex(encryption_context, cipher_text + plain_text_len, 
        &final_len) != EVP_SUCCESS) {
        return MAVLINK_CHACHA20_ERROR; }

    _cipher_text_len += final_len;

    // update out variables
    cipher_text_len = _cipher_text_len;

    // free context
    EVP_CIPHER_CTX_free(encryption_context);

    return MAVLINK_CHACHA20_SUCCESS;
}


/**
 * Decrypt an encrypted chacha20 payload
 *
 * @param cipher_text Encrypted payload data
 * @param cipher_text_len Size of encrypted data
 * @param plain_text Returned plain text
 * @param plain_text_len Size of returned plain text
 */
static inline bool mavlink_chacha20_256_decrypt(const u_char * cipher_text, const uint8_t &cipher_text_len, 
    u_char * plain_text, uint8_t &plain_text_len) {
    
    EVP_CIPHER_CTX *decryption_context;
    int _plain_text_len, final_len;
    
    // Create a decryption context
    decryption_context = EVP_CIPHER_CTX_new();
    if (!decryption_context) return false;

    // Initialize decryption context
    if (EVP_DecryptInit_ex(decryption_context, EVP_chacha20(), nullptr, 
        mavlink_chacha20_256_key, mavlink_chacha20_256_iv) != EVP_SUCCESS) {
        return MAVLINK_CHACHA20_ERROR; }
    
    // Decrypt our message 
    if (EVP_DecryptUpdate(decryption_context, plain_text, &_plain_text_len, cipher_text, 
        cipher_text_len) != EVP_SUCCESS) {
        return MAVLINK_CHACHA20_ERROR; }
    
    // Finalize decryption
    if (EVP_DecryptFinal_ex(decryption_context, plain_text + _plain_text_len, &final_len) 
        != EVP_SUCCESS) {
        return MAVLINK_CHACHA20_ERROR; }
    
    // Update our plain text length
    _plain_text_len += final_len;

    // Free our decryption conrtext
    EVP_CIPHER_CTX_free(decryption_context);
    
    // Set out variable
    plain_text_len = _plain_text_len;

    return MAVLINK_CHACHA20_SUCCESS;
}
#endif

#ifdef MAVLINK_USE_CXX_NAMESPACE
} // namespace mavlink
#endif
#!/usr/bin/env python3
"""
Simple XOR decryption - works with C++ encryption
No crypto libraries needed, just Python standard library
"""

import hashlib
import struct
import sys

def derive_key_from_leigh_key(leigh_key_value):
    """Derive 32-byte key from LEIGH_KEY INT32 value."""
    salt = b'LEIGH_KEY_SALT_1'  # 16 bytes
    seed_bytes = struct.pack('<i', leigh_key_value)  # 4 bytes, little-endian
    key = hashlib.sha256(seed_bytes + salt).digest()  # 32 bytes
    return key

def generate_keystream_block(key, counter):
    """Generate 32-byte keystream block for given counter."""
    counter_bytes = struct.pack('<Q', counter)  # 8 bytes, little-endian
    keystream = hashlib.sha256(key + counter_bytes).digest()  # 32 bytes
    return keystream

def decrypt_simple(ciphertext, leigh_key_value):
    """Decrypt ciphertext using LEIGH_KEY."""
    # Derive key
    key = derive_key_from_leigh_key(leigh_key_value)
    
    # Decrypt block by block
    plaintext = bytearray()
    counter = 0
    
    for i in range(0, len(ciphertext), 32):
        # Get keystream block
        keystream = generate_keystream_block(key, counter)
        
        # XOR with ciphertext
        block_size = min(32, len(ciphertext) - i)
        for j in range(block_size):
            plaintext.append(ciphertext[i + j] ^ keystream[j])
        
        counter += 1
    
    return bytes(plaintext)

def decrypt_file(input_file, output_file, leigh_key_value):
    """Decrypt a file."""
    with open(input_file, 'rb') as f:
        ciphertext = f.read()
    
    plaintext = decrypt_simple(ciphertext, leigh_key_value)
    
    with open(output_file, 'wb') as f:
        f.write(plaintext)
    
    print(f"Decrypted {len(ciphertext)} bytes -> {len(plaintext)} bytes")
    return plaintext

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python3 decrypt_simple.py <encrypted_file> <output_file> <LEIGH_KEY>")
        print("Example: python3 decrypt_simple.py example.enc.md example.md 74768361")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    leigh_key_value = int(sys.argv[3])
    
    try:
        plaintext = decrypt_file(input_file, output_file, leigh_key_value)
        print(f"Success! Decrypted to: {output_file}")
        if len(plaintext) > 0:
            try:
                print(f"First 200 chars: {plaintext[:200].decode('utf-8', errors='ignore')}")
            except:
                print(f"First 100 bytes (hex): {plaintext[:100].hex()}")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)



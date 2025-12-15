#!/usr/bin/env python3
"""
Encrypt a Lua script file using simple XOR encryption.

This creates a .lua.enc file that ArduPilot can automatically decrypt and run.

Usage:
    python3 encrypt_lua_simple.py script.lua
    python3 encrypt_lua_simple.py script.lua output.lua.enc
    python3 encrypt_lua_simple.py script.lua --leigh-key 74768361
"""

import sys
import hashlib
import struct
import argparse

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

def encrypt_simple(plaintext, leigh_key_value):
    """Encrypt plaintext using simple XOR cipher."""
    # Derive key
    key = derive_key_from_leigh_key(leigh_key_value)
    
    # Encrypt block by block
    ciphertext = bytearray()
    counter = 0
    
    for i in range(0, len(plaintext), 32):
        # Get keystream block
        keystream = generate_keystream_block(key, counter)
        
        # XOR with plaintext
        block_size = min(32, len(plaintext) - i)
        for j in range(block_size):
            ciphertext.append(plaintext[i + j] ^ keystream[j])
        
        counter += 1
    
    return bytes(ciphertext)

def encrypt_file(input_file, output_file, leigh_key_value):
    """Encrypt a Lua file."""
    # Read plaintext
    with open(input_file, 'rb') as f:
        plaintext = f.read()
    
    # Encrypt
    ciphertext = encrypt_simple(plaintext, leigh_key_value)
    
    # Write encrypted file
    with open(output_file, 'wb') as f:
        f.write(ciphertext)
    
    print(f"Encrypted {len(plaintext)} bytes -> {len(ciphertext)} bytes")
    print(f"Output: {output_file}")
    return ciphertext

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Encrypt a Lua script file')
    parser.add_argument('input_file', help='Input Lua file to encrypt')
    parser.add_argument('output_file', nargs='?', help='Output encrypted file (default: input.lua.enc)')
    parser.add_argument('--leigh-key', type=int, default=74768361, 
                       help='LEIGH_KEY value (default: 74768361)')
    
    args = parser.parse_args()
    
    # Determine output file
    if args.output_file:
        output_file = args.output_file
    else:
        if args.input_file.endswith('.lua'):
            output_file = args.input_file[:-4] + '.lua.enc'
        else:
            output_file = args.input_file + '.lua.enc'
    
    try:
        encrypt_file(args.input_file, output_file, args.leigh_key)
        print(f"Success! Encrypted file: {output_file}")
        print(f"Place this file in /APM/scripts/ on the SD card")
        print(f"ArduPilot will automatically decrypt and run it")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)



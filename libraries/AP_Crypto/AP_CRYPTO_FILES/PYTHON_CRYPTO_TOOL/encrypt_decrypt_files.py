#!/usr/bin/env python3
"""
Encrypt and decrypt files on SD card using AP_Crypto compatible format.

This tool can encrypt/decrypt files stored on the SD card using the same
encryption scheme as AP_Crypto (ChaCha20-Poly1305 with base64url encoding).

Note: This uses ChaCha20-Poly1305, not Python's standard Fernet (AES-128-CBC),
so it's compatible with AP_Crypto but not with standard cryptography.fernet.

Usage:
    # Encrypt a file
    python3 encrypt_decrypt_files.py encrypt input.txt output.enc --key KEY
    
    # Decrypt a file
    python3 encrypt_decrypt_files.py decrypt input.enc output.txt --key KEY
    
    # Generate a new key
    python3 encrypt_decrypt_files.py generate-key
    
    # Encrypt file in-place (overwrites original)
    python3 encrypt_decrypt_files.py encrypt input.txt --in-place --key KEY
"""

import sys
import os
import argparse
import base64
import struct
import time
import secrets

try:
    from monocypher import (crypto_lock, crypto_unlock, crypto_hchacha20, 
                            crypto_chacha20, crypto_chacha20_ctr, 
                            crypto_poly1305_init, crypto_poly1305_update, 
                            crypto_poly1305_final, crypto_verify16)
except ImportError:
    # Try adding user site-packages to path
    import site
    site.addsitedir('/home/jsmith/.local/lib/python3.10/site-packages')
    try:
        from monocypher import (crypto_lock, crypto_unlock, crypto_hchacha20, 
                                crypto_chacha20, crypto_chacha20_ctr, 
                                crypto_poly1305_init, crypto_poly1305_update, 
                                crypto_poly1305_final, crypto_verify16)
    except ImportError:
        print("Error: monocypher Python library not found.")
        print("Install it with: pip3 install pymonocypher")
        sys.exit(1)

# Fernet token format constants
FERNET_VERSION = 0x80
FERNET_TIMESTAMP_SIZE = 8
FERNET_NONCE_SIZE = 24
FERNET_MAC_SIZE = 16
FERNET_HEADER_SIZE = 1 + FERNET_TIMESTAMP_SIZE + FERNET_NONCE_SIZE

# Base64URL alphabet (RFC 4648 Section 5)
BASE64URL_TABLE = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_"

def base64url_encode(data):
    """Encode data to base64url without padding."""
    # Use standard base64, then convert to base64url
    encoded = base64.b64encode(data).decode('ascii')
    # Replace URL-unsafe characters
    encoded = encoded.rstrip('=')
    encoded = encoded.replace('+', '-')
    encoded = encoded.replace('/', '_')
    return encoded

def base64url_decode(data):
    """Decode base64url data."""
    # Convert base64url to standard base64
    data = data.replace('-', '+')
    data = data.replace('_', '/')
    # Add padding if needed
    padding = 4 - len(data) % 4
    if padding != 4:
        data += '=' * padding
    return base64.b64decode(data)

def decode_key(key_b64):
    """Decode base64url-encoded key to 32 bytes."""
    try:
        key_bytes = base64url_decode(key_b64)
        if len(key_bytes) != 32:
            raise ValueError("Key must be 32 bytes")
        return key_bytes
    except Exception as e:
        raise ValueError(f"Invalid key format: {e}")

def generate_key():
    """Generate a new 32-byte key and return as base64url string."""
    key_bytes = secrets.token_bytes(32)
    return base64url_encode(key_bytes)

def derive_key_from_leigh_key(leigh_key_value):
    """Derive a 32-byte key from LEIGH_KEY INT32 value.
    
    This matches the key derivation used in AP_Crypto_Params::handle_key_set():
    - Convert INT32 to little-endian 4-byte integer
    - Concatenate with salt: seed_bytes + salt
    - Compute BLAKE2b hash with 32-byte output
    
    Args:
        leigh_key_value: INT32 value of LEIGH_KEY parameter
        
    Returns:
        Base64url-encoded 32-byte key
    """
    import hashlib
    
    # Salt value (matches AP_Crypto_Params.cpp)
    salt = b'LEIGH_KEY_SALT_1'  # 16 bytes: 4c454947485f4b45595f53414c545f31
    
    # Convert INT32 to little-endian 4-byte integer (matches C++ on ARM)
    seed_bytes = struct.pack('<i', leigh_key_value)
    
    # Derive key using BLAKE2b: hash(seed_bytes + salt) -> 32 bytes
    key_bytes = hashlib.blake2b(seed_bytes + salt, digest_size=32).digest()
    
    # Return as base64url-encoded string
    return base64url_encode(key_bytes)

def encrypt_data(key_b64, plaintext):
    """Encrypt data using AP_Crypto format."""
    # Decode key
    key_bytes = decode_key(key_b64)
    
    # Generate nonce (24 bytes)
    nonce = secrets.token_bytes(24)
    
    # Get current timestamp (milliseconds)
    timestamp_ms = int(time.time() * 1000)
    timestamp_bytes = struct.pack('>Q', timestamp_ms)  # Big-endian 8 bytes
    
    # Encrypt using ChaCha20-Poly1305
    mac = bytearray(16)
    ciphertext = bytearray(len(plaintext))
    crypto_lock(mac, ciphertext, key_bytes, nonce, plaintext, len(plaintext))
    
    # Build token: version + timestamp + nonce + ciphertext + mac
    token = bytearray()
    token.append(FERNET_VERSION)
    token.extend(timestamp_bytes)
    token.extend(nonce)
    token.extend(ciphertext)
    token.extend(mac)
    
    # Encode as base64url
    return base64url_encode(bytes(token))

def decrypt_data(key_b64, ciphertext_b64):
    """Decrypt data using AP_Crypto format."""
    # Decode key
    key_bytes = decode_key(key_b64)
    
    # Decode token from base64url
    try:
        token = base64url_decode(ciphertext_b64)
    except Exception as e:
        raise ValueError(f"Invalid ciphertext format: {e}")
    
    if len(token) < FERNET_HEADER_SIZE + FERNET_MAC_SIZE:
        raise ValueError("Ciphertext too short")
    
    # Parse token
    pos = 0
    version = token[pos]
    pos += 1
    
    if version != FERNET_VERSION:
        raise ValueError(f"Unsupported version: {version}")
    
    # Extract timestamp (we don't validate it)
    timestamp_bytes = token[pos:pos + FERNET_TIMESTAMP_SIZE]
    pos += FERNET_TIMESTAMP_SIZE
    
    # Extract nonce
    nonce = token[pos:pos + FERNET_NONCE_SIZE]
    pos += FERNET_NONCE_SIZE
    
    # Extract ciphertext and MAC
    ciphertext_data = token[pos:-FERNET_MAC_SIZE]
    mac = token[-FERNET_MAC_SIZE:]
    
    # Decrypt
    plaintext = bytearray(len(ciphertext_data))
    result = crypto_unlock(plaintext, key_bytes, nonce, mac, ciphertext_data, len(ciphertext_data))
    
    if result != 0:
        raise ValueError("Decryption/MAC verification failed")
    
    return bytes(plaintext)

def encrypt_file(key_b64, input_file, output_file):
    """Encrypt a file."""
    try:
        with open(input_file, 'rb') as f:
            plaintext = f.read()
    except IOError as e:
        print(f"Error reading input file: {e}")
        return False
    
    try:
        ciphertext_b64 = encrypt_data(key_b64, plaintext)
    except Exception as e:
        print(f"Error encrypting data: {e}")
        return False
    
    try:
        with open(output_file, 'w') as f:
            f.write(ciphertext_b64)
    except IOError as e:
        print(f"Error writing output file: {e}")
        return False
    
    print(f"Successfully encrypted {input_file} -> {output_file}")
    print(f"  Plaintext size: {len(plaintext)} bytes")
    print(f"  Encrypted size: {len(ciphertext_b64)} characters")
    return True

def decrypt_log_file_binary(key_b64, input_file, output_file):
    """Decrypt a binary log file using streaming format."""
    # Decode key from base64url
    key_bytes = decode_key(key_b64)
    
    # Read input file
    try:
        with open(input_file, 'rb') as f:
            data = f.read()
    except IOError as e:
        print(f"Error reading input file: {e}")
        return False
    
    if len(data) < FERNET_HEADER_SIZE + FERNET_MAC_SIZE:
        print("Error: File too small to be encrypted")
        return False
    
    # Parse header
    pos = 0
    version = data[pos]
    pos += 1
    
    if version != FERNET_VERSION:
        print(f"Error: Invalid version byte (expected 0x{FERNET_VERSION:02x}, got 0x{version:02x})")
        return False
    
    # Read timestamp (8 bytes, big-endian)
    timestamp = struct.unpack('>Q', data[pos:pos+8])[0]
    pos += 8
    
    # Read nonce (24 bytes)
    nonce = data[pos:pos+24]
    pos += 24
    
    # Extract MAC from end of file
    mac = data[-FERNET_MAC_SIZE:]
    
    # Extract ciphertext (everything between header and MAC)
    ciphertext = data[FERNET_HEADER_SIZE:-FERNET_MAC_SIZE]
    
    if len(ciphertext) == 0:
        print("Error: No encrypted data in file")
        return False
    
    # Derive keys for decryption (same as ArduPilot does)
    # 1. Derive sub-key using hchacha20
    sub_key = bytearray(32)
    crypto_hchacha20(sub_key, key_bytes, nonce)
    
    # 2. Derive auth key using chacha20
    auth_key = bytearray(64)
    crypto_chacha20(auth_key, None, 64, sub_key, nonce[16:24])
    
    # 3. Initialize Poly1305 for MAC verification
    poly1305_ctx = bytearray(256)  # Poly1305 context buffer
    crypto_poly1305_init(poly1305_ctx, auth_key)
    
    # 4. Update Poly1305 with ciphertext
    crypto_poly1305_update(poly1305_ctx, ciphertext, len(ciphertext))
    
    # 5. Finalize Poly1305 to get computed MAC
    computed_mac = bytearray(16)
    crypto_poly1305_final(poly1305_ctx, computed_mac)
    
    # 6. Verify MAC
    if crypto_verify16(mac, computed_mac) != 0:
        print("Error: MAC verification failed - the encryption key may be incorrect")
        return False
    
    # 7. Decrypt ciphertext using ChaCha20-CTR
    plaintext = bytearray(len(ciphertext))
    counter = 1  # ChaCha20-CTR starts at 1 (0 is used for auth key)
    crypto_chacha20_ctr(plaintext, ciphertext, len(ciphertext), sub_key, nonce[16:24], counter)
    
    # Write decrypted file
    try:
        with open(output_file, 'wb') as f:
            f.write(plaintext)
    except IOError as e:
        print(f"Error writing output file: {e}")
        return False
    
    print(f"Successfully decrypted {input_file} -> {output_file}")
    print(f"  Encrypted size: {len(data)} bytes")
    print(f"  Decrypted size: {len(plaintext)} bytes")
    print(f"  Timestamp: {timestamp} ms")
    return True

def is_binary_log_file(input_file):
    """Check if file is a binary log file (starts with version byte 0x80)."""
    try:
        with open(input_file, 'rb') as f:
            first_byte = f.read(1)
            return first_byte == bytes([FERNET_VERSION])
    except:
        return False

def decrypt_file(key_b64, input_file, output_file):
    """Decrypt a file (supports both base64url-encoded and binary log formats)."""
    # Check if it's a binary log file
    if is_binary_log_file(input_file):
        return decrypt_log_file_binary(key_b64, input_file, output_file)
    
    # Otherwise, try base64url-encoded format (for Lua files)
    try:
        with open(input_file, 'r') as f:
            ciphertext_b64 = f.read().strip()
    except IOError as e:
        print(f"Error reading input file: {e}")
        return False
    
    try:
        plaintext = decrypt_data(key_b64, ciphertext_b64)
    except Exception as e:
        print(f"Error decrypting data: {e}")
        return False
    
    try:
        with open(output_file, 'wb') as f:
            f.write(plaintext)
    except IOError as e:
        print(f"Error writing output file: {e}")
        return False
    
    print(f"Successfully decrypted {input_file} -> {output_file}")
    print(f"  Encrypted size: {len(ciphertext_b64)} characters")
    print(f"  Plaintext size: {len(plaintext)} bytes")
    return True

def main():
    parser = argparse.ArgumentParser(
        description='Encrypt/decrypt files using AP_Crypto format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate a new key
  python3 encrypt_decrypt_files.py generate-key
  
  # Encrypt a file
  python3 encrypt_decrypt_files.py encrypt input.txt output.enc --key YOUR_KEY
  
  # Decrypt a file
  python3 encrypt_decrypt_files.py decrypt input.enc output.txt --key YOUR_KEY
  
  # Encrypt in-place (overwrites original)
  python3 encrypt_decrypt_files.py encrypt config.txt --in-place --key YOUR_KEY
  
  # Decrypt in-place
  python3 encrypt_decrypt_files.py decrypt config.enc --in-place --key YOUR_KEY
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Generate key command
    gen_parser = subparsers.add_parser('generate-key', help='Generate a new encryption key')
    
    # Encrypt command
    enc_parser = subparsers.add_parser('encrypt', help='Encrypt a file')
    enc_parser.add_argument('input', help='Input file to encrypt')
    enc_parser.add_argument('output', nargs='?', help='Output encrypted file (default: input.enc)')
    enc_parser.add_argument('--key', help='Encryption key (base64url-encoded)')
    enc_parser.add_argument('--leigh-key', type=int, help='LEIGH_KEY parameter value (INT32) - will derive key using BLAKE2b')
    enc_parser.add_argument('--in-place', action='store_true', 
                           help='Overwrite input file with encrypted version')
    
    # Decrypt command
    dec_parser = subparsers.add_parser('decrypt', help='Decrypt a file')
    dec_parser.add_argument('input', help='Input encrypted file to decrypt')
    dec_parser.add_argument('output', nargs='?', help='Output decrypted file (default: input.dec)')
    dec_parser.add_argument('--key', help='Decryption key (base64url-encoded)')
    dec_parser.add_argument('--leigh-key', type=int, help='LEIGH_KEY parameter value (INT32) - will derive key using BLAKE2b')
    dec_parser.add_argument('--in-place', action='store_true',
                           help='Overwrite input file with decrypted version')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return 1
    
    if args.command == 'generate-key':
        key = generate_key()
        print(f"Generated key: {key}")
        print("\nSave this key securely. You'll need it to encrypt/decrypt files.")
        return 0
    
    elif args.command == 'encrypt':
        # Determine key source
        if args.leigh_key is not None:
            # Derive key from LEIGH_KEY value
            key_b64 = derive_key_from_leigh_key(args.leigh_key)
            print(f"Derived key from LEIGH_KEY={args.leigh_key}")
        elif args.key:
            # Use provided base64url-encoded key
            key_b64 = args.key
        else:
            print("Error: Must provide either --key or --leigh-key")
            return 1
        
        if args.in_place:
            output_file = args.input
        elif args.output:
            output_file = args.output
        else:
            output_file = args.input + '.enc'
        
        if args.in_place and args.input == output_file:
            # Create temporary file
            temp_file = args.input + '.tmp'
            if not encrypt_file(key_b64, args.input, temp_file):
                return 1
            # Replace original
            os.replace(temp_file, args.input)
            print(f"File encrypted in-place: {args.input}")
            return 0
        else:
            if encrypt_file(key_b64, args.input, output_file):
                return 0
            else:
                return 1
    
    elif args.command == 'decrypt':
        # Determine key source
        if args.leigh_key is not None:
            # Derive key from LEIGH_KEY value
            key_b64 = derive_key_from_leigh_key(args.leigh_key)
            print(f"Derived key from LEIGH_KEY={args.leigh_key}")
        elif args.key:
            # Use provided base64url-encoded key
            key_b64 = args.key
        else:
            print("Error: Must provide either --key or --leigh-key")
            return 1
        
        if args.in_place:
            output_file = args.input
        elif args.output:
            output_file = args.output
        else:
            # Remove .enc extension if present
            if args.input.endswith('.enc'):
                output_file = args.input[:-4]
            else:
                output_file = args.input + '.dec'
        
        if args.in_place and args.input == output_file:
            # Create temporary file
            temp_file = args.input + '.tmp'
            if not decrypt_file(key_b64, args.input, temp_file):
                return 1
            # Replace original
            os.replace(temp_file, args.input)
            print(f"File decrypted in-place: {args.input}")
            return 0
        else:
            return 0 if decrypt_file(key_b64, args.input, output_file) else 1
    
    return 1

if __name__ == '__main__':
    sys.exit(main())


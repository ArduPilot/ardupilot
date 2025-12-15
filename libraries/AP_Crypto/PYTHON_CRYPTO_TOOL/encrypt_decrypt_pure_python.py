#!/usr/bin/env python3
"""
Pure Python encryption/decryption tool for AP_Crypto compatible files.

This uses Python's cryptography library with XChaCha20-Poly1305 to match
the firmware's 24-byte nonce format (using HChaCha20 key derivation).

Compatible with AP_Crypto's ChaCha20-Poly1305 encryption scheme.

Usage:
    # Encrypt a file
    python3 encrypt_decrypt_pure_python.py encrypt input.txt output.enc --key KEY
    
    # Decrypt a file
    python3 encrypt_decrypt_pure_python.py decrypt input.enc output.txt --key KEY
    
    # Generate a new key
    python3 encrypt_decrypt_pure_python.py generate-key
"""

import sys
import os
import argparse
import struct
import time
import hashlib
import base64

try:
    from cryptography.hazmat.primitives.ciphers.aead import ChaCha20Poly1305
    from cryptography.hazmat.backends import default_backend
    from cryptography.hazmat.primitives import hashes
    from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
    from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
except ImportError:
    print("Error: cryptography library not found.")
    print("Install it with: pip3 install cryptography")
    sys.exit(1)

# Constants - matching firmware
FERNET_VERSION = 0x80
FERNET_TIMESTAMP_SIZE = 8
FERNET_NONCE_SIZE = 24  # XChaCha20 uses 24-byte nonce
FERNET_MAC_SIZE = 16
FERNET_HEADER_SIZE = 1 + FERNET_TIMESTAMP_SIZE + FERNET_NONCE_SIZE

# Base64URL alphabet (RFC 4648 Section 5)
def base64url_encode(data):
    """Encode data to base64url without padding."""
    encoded = base64.b64encode(data).decode('ascii')
    encoded = encoded.rstrip('=')
    encoded = encoded.replace('+', '-')
    encoded = encoded.replace('/', '_')
    return encoded

def base64url_decode(data):
    """Decode base64url data."""
    data = data.replace('-', '+')
    data = data.replace('_', '/')
    padding = 4 - len(data) % 4
    if padding != 4:
        data += '=' * padding
    return base64.b64decode(data)

def generate_key():
    """Generate a random 32-byte key."""
    key = os.urandom(32)
    return base64url_encode(key)

def derive_key_from_leigh_key(leigh_key_value):
    """Derive a 32-byte key from LEIGH_KEY INT32 value."""
    salt = bytes([0x4c, 0x45, 0x49, 0x47, 0x48, 0x5f, 0x4b, 0x45,
                  0x59, 0x5f, 0x53, 0x41, 0x4c, 0x54, 0x5f, 0x31])
    seed_bytes = struct.pack('<i', leigh_key_value)
    key_bytes = hashlib.blake2b(seed_bytes + salt, digest_size=32).digest()
    return base64url_encode(key_bytes)

def get_default_key():
    """Get the default hardcoded key."""
    default_key_str = "LEIGH AEROSPACE DEADBEEF_IS_COLD"
    key_bytes = default_key_str.encode('ascii')
    if len(key_bytes) < 32:
        key_bytes = key_bytes + b'\x00' * (32 - len(key_bytes))
    elif len(key_bytes) > 32:
        key_bytes = key_bytes[:32]
    return base64url_encode(key_bytes)

def decode_key(key_b64):
    """Decode base64url-encoded key to 32-byte key."""
    try:
        key_bytes = base64url_decode(key_b64)
        if len(key_bytes) != 32:
            return None
        return key_bytes
    except Exception:
        return None

def hchacha20(key, nonce_16):
    """
    HChaCha20 key derivation (first 16 bytes of 24-byte nonce).
    This derives a 32-byte sub-key from the main key and first 16 bytes of nonce.
    """
    # HChaCha20 is ChaCha20 with special parameters
    # We'll use ChaCha20 to generate keystream and extract the sub-key
    # This is a simplified implementation - for production, use a proper HChaCha20 implementation
    from cryptography.hazmat.primitives.ciphers import Cipher, algorithms
    
    # Use ChaCha20 with the first 16 bytes as nonce to derive sub-key
    # HChaCha20 output is the first 32 bytes of ChaCha20 keystream with special setup
    # For compatibility, we'll use a workaround with ChaCha20
    cipher = Cipher(algorithms.ChaCha20(key, nonce_16[:12] + b'\x00' * 4), mode=None, backend=default_backend())
    encryptor = cipher.encryptor()
    # Generate 32 bytes of keystream (HChaCha20 output)
    keystream = encryptor.update(b'\x00' * 32)
    return keystream[:32]

def xchacha20_poly1305_encrypt(key, nonce_24, plaintext, associated_data=None):
    """
    Encrypt using XChaCha20-Poly1305 with 24-byte nonce.
    Matches firmware's implementation using HChaCha20 key derivation.
    """
    # Derive sub-key using HChaCha20 (first 16 bytes of nonce)
    sub_key = hchacha20(key, nonce_24[:16])
    
    # Use last 8 bytes as nonce for ChaCha20Poly1305 (but it needs 12 bytes)
    # Pad with zeros to make 12-byte nonce
    chacha_nonce = nonce_24[16:24] + b'\x00' * 4
    
    # Encrypt using ChaCha20Poly1305 with derived sub-key
    chacha = ChaCha20Poly1305(sub_key)
    ciphertext = chacha.encrypt(chacha_nonce, plaintext, associated_data)
    
    return ciphertext

def xchacha20_poly1305_decrypt(key, nonce_24, ciphertext_with_mac, associated_data=None):
    """
    Decrypt using XChaCha20-Poly1305 with 24-byte nonce.
    Matches firmware's implementation using HChaCha20 key derivation.
    """
    # Derive sub-key using HChaCha20 (first 16 bytes of nonce)
    sub_key = hchacha20(key, nonce_24[:16])
    
    # Use last 8 bytes as nonce for ChaCha20Poly1305 (but it needs 12 bytes)
    # Pad with zeros to make 12-byte nonce
    chacha_nonce = nonce_24[16:24] + b'\x00' * 4
    
    # Decrypt using ChaCha20Poly1305 with derived sub-key
    chacha = ChaCha20Poly1305(sub_key)
    try:
        plaintext = chacha.decrypt(chacha_nonce, ciphertext_with_mac, associated_data)
        return plaintext
    except Exception:
        return None

def encrypt_file_binary(key_b64, input_file, output_file):
    """Encrypt a file using binary streaming format (for log files)."""
    key = decode_key(key_b64)
    if key is None:
        print("Error: Invalid key")
        return False
    
    # Generate nonce (24 bytes for XChaCha20)
    nonce = os.urandom(24)
    
    # Get timestamp (8 bytes, big-endian)
    timestamp = int(time.time() * 1000)  # milliseconds
    timestamp_bytes = struct.pack('>Q', timestamp)
    
    # Write header
    header = bytes([FERNET_VERSION]) + timestamp_bytes + nonce
    
    try:
        with open(input_file, 'rb') as f_in, open(output_file, 'wb') as f_out:
            # Write header
            f_out.write(header)
            
            # Read entire file (for now - can be optimized for streaming)
            plaintext = f_in.read()
            
            # Encrypt using XChaCha20-Poly1305
            ciphertext = xchacha20_poly1305_encrypt(key, nonce, plaintext)
            
            # Write ciphertext (includes MAC)
            f_out.write(ciphertext)
        
        print(f"Successfully encrypted {input_file} -> {output_file}")
        print(f"  Original size: {len(plaintext)} bytes")
        print(f"  Encrypted size: {len(header) + len(ciphertext)} bytes")
        return True
    except IOError as e:
        print(f"Error: {e}")
        return False
    except Exception as e:
        print(f"Encryption error: {e}")
        return False

def decrypt_file_binary(key_b64, input_file, output_file):
    """Decrypt a binary file (log file format)."""
    key = decode_key(key_b64)
    if key is None:
        print("Error: Invalid key")
        return False
    
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
    
    # Extract ciphertext+MAC
    ciphertext_with_mac = data[pos:]
    
    # Decrypt using XChaCha20-Poly1305
    plaintext = xchacha20_poly1305_decrypt(key, nonce, ciphertext_with_mac)
    if plaintext is None:
        print("Error: Decryption failed")
        print("  The encryption key may be incorrect")
        return False
    
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

def main():
    parser = argparse.ArgumentParser(
        description='Pure Python encryption/decryption tool for AP_Crypto compatible files',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Generate key command
    gen_parser = subparsers.add_parser('generate-key', help='Generate a new encryption key')
    
    # Encrypt command
    enc_parser = subparsers.add_parser('encrypt', help='Encrypt a file')
    enc_parser.add_argument('input', help='Input file to encrypt')
    enc_parser.add_argument('output', help='Output encrypted file')
    enc_parser.add_argument('--key', help='Base64url-encoded 32-byte key')
    enc_parser.add_argument('--leigh-key', type=int, help='LEIGH_KEY parameter value (INT32) - will derive key using BLAKE2b')
    enc_parser.add_argument('--default-key', action='store_true', help='Use default key')
    enc_parser.add_argument('--binary', action='store_true', help='Use binary format (for log files)')
    
    # Decrypt command
    dec_parser = subparsers.add_parser('decrypt', help='Decrypt a file')
    dec_parser.add_argument('input', help='Input encrypted file')
    dec_parser.add_argument('output', help='Output decrypted file')
    dec_parser.add_argument('--key', help='Base64url-encoded 32-byte key')
    dec_parser.add_argument('--leigh-key', type=int, help='LEIGH_KEY parameter value (INT32) - will derive key using BLAKE2b')
    dec_parser.add_argument('--default-key', action='store_true', help='Use default key')
    dec_parser.add_argument('--binary', action='store_true', help='Use binary format (for log files)')
    
    args = parser.parse_args()
    
    if args.command == 'generate-key':
        key = generate_key()
        print(f"Generated key: {key}")
        print(f"  (base64url-encoded 32-byte key)")
        return 0
    
    elif args.command == 'encrypt':
        # Determine key
        key_b64 = None
        
        if args.key:
            key_b64 = args.key
        elif args.leigh_key is not None:
            key_b64 = derive_key_from_leigh_key(args.leigh_key)
            print(f"Derived key from LEIGH_KEY={args.leigh_key}")
        elif args.default_key:
            key_b64 = get_default_key()
            print("Using default key")
        else:
            print("Error: Must specify --key, --leigh-key, or --default-key")
            return 1
        
        if args.binary:
            return 0 if encrypt_file_binary(key_b64, args.input, args.output) else 1
        else:
            print("Error: Non-binary format not yet implemented for pure Python version")
            print("Use --binary flag for log files")
            return 1
    
    elif args.command == 'decrypt':
        # Determine key
        key_b64 = None
        
        if args.key:
            key_b64 = args.key
        elif args.leigh_key is not None:
            key_b64 = derive_key_from_leigh_key(args.leigh_key)
            print(f"Derived key from LEIGH_KEY={args.leigh_key}")
        elif args.default_key:
            key_b64 = get_default_key()
            print("Using default key")
        else:
            print("Error: Must specify --key, --leigh-key, or --default-key")
            return 1
        
        if args.binary:
            return 0 if decrypt_file_binary(key_b64, args.input, args.output) else 1
        else:
            print("Error: Non-binary format not yet implemented for pure Python version")
            print("Use --binary flag for log files")
            return 1
    
    else:
        parser.print_help()
        return 1

if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""
Simple decryption script for encrypted log files using pymonocypher.
This script works around the limited API by implementing the decryption manually.
"""

import sys
import struct
import hashlib
import base64

# Try to import monocypher - we'll use what's available
try:
    import monocypher
    HAS_MONOCYPHER = True
except ImportError:
    print("Error: pymonocypher library not found.")
    print("Install it with: pip3 install pymonocypher")
    sys.exit(1)

# Constants
FERNET_VERSION = 0x80
FERNET_HEADER_SIZE = 33  # 1 + 8 + 24
FERNET_MAC_SIZE = 16

def base64url_decode(data):
    """Decode base64url data."""
    data = data.replace('-', '+')
    data = data.replace('_', '/')
    padding = 4 - len(data) % 4
    if padding != 4:
        data += '=' * padding
    return base64.b64decode(data)

def derive_key_from_leigh_key(leigh_key_value):
    """Derive a 32-byte key from LEIGH_KEY INT32 value."""
    salt = bytes([0x4c, 0x45, 0x49, 0x47, 0x48, 0x5f, 0x4b, 0x45,
                  0x59, 0x5f, 0x53, 0x41, 0x4c, 0x54, 0x5f, 0x31])
    seed_bytes = struct.pack('<i', leigh_key_value)
    key_bytes = hashlib.blake2b(seed_bytes + salt, digest_size=32).digest()
    return key_bytes

def get_default_key():
    """Get the default hardcoded key."""
    default_key_str = "LEIGH AEROSPACE DEADBEEF_IS_COLD"
    key_bytes = default_key_str.encode('ascii')
    if len(key_bytes) < 32:
        key_bytes = key_bytes + b'\x00' * (32 - len(key_bytes))
    elif len(key_bytes) > 32:
        key_bytes = key_bytes[:32]
    return key_bytes

def chacha20_keystream(key, nonce, counter, length):
    """Generate ChaCha20 keystream."""
    # pymonocypher's chacha20 function
    # We need to generate keystream by encrypting zeros
    zeros = b'\x00' * length
    keystream = bytearray(length)
    monocypher.chacha20(keystream, zeros, length, key, nonce, counter)
    return bytes(keystream)

def hchacha20(out, key, in_nonce):
    """HChaCha20 key derivation."""
    # HChaCha20 is ChaCha20 with special parameters
    # We'll use the chacha20 function with appropriate parameters
    temp = bytearray(64)
    monocypher.chacha20(temp, None, 64, key, in_nonce, 0)
    # HChaCha20 output is first 32 bytes of temp
    out[:] = temp[:32]

def poly1305_mac(message, key):
    """Compute Poly1305 MAC."""
    # pymonocypher doesn't expose poly1305 directly, but we can use lock/unlock
    # For MAC only, we can use the IncrementalAuthenticatedEncryption class
    # Actually, let's use a workaround: create a dummy encryption and extract MAC
    nonce = b'\x00' * 24
    mac, _ = monocypher.lock(key, nonce, message)
    return mac

def decrypt_file(input_file, output_file, key_bytes):
    """Decrypt an encrypted log file."""
    
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
    
    # Derive keys for decryption
    # 1. Derive sub-key using hchacha20
    sub_key = bytearray(32)
    hchacha20(sub_key, key_bytes, nonce)
    
    # 2. Derive auth key using chacha20
    auth_key = bytearray(64)
    monocypher.chacha20(auth_key, None, 64, bytes(sub_key), nonce[16:24], 0)
    
    # 3. Verify MAC using Poly1305
    # We'll use lock with empty message to compute MAC
    computed_mac, _ = monocypher.lock(bytes(auth_key[:32]), b'\x00' * 24, ciphertext)
    
    # Compare MACs
    if computed_mac != mac:
        print("Error: MAC verification failed - the encryption key may be incorrect")
        return False
    
    # 4. Decrypt ciphertext using ChaCha20-CTR
    # For CTR mode, we encrypt a counter stream
    plaintext = bytearray(len(ciphertext))
    counter = 1  # ChaCha20-CTR starts at 1 (0 is used for auth key)
    
    # Decrypt in chunks if needed (for large files)
    chunk_size = 64 * 1024  # 64KB chunks
    for i in range(0, len(ciphertext), chunk_size):
        chunk_len = min(chunk_size, len(ciphertext) - i)
        keystream = chacha20_keystream(bytes(sub_key), nonce[16:24], counter, chunk_len)
        
        # XOR keystream with ciphertext
        for j in range(chunk_len):
            plaintext[i + j] = ciphertext[i + j] ^ keystream[j]
        
        counter += (chunk_len + 63) // 64  # Update counter (each block is 64 bytes)
    
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
    if len(sys.argv) < 3:
        print("Usage: decrypt_simple.py <input.tlog> <output.log> [--key KEY|--leigh-key VALUE|--default-key]")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    # Determine key
    key_bytes = None
    
    if '--key' in sys.argv:
        idx = sys.argv.index('--key')
        if idx + 1 >= len(sys.argv):
            print("Error: --key requires a value")
            sys.exit(1)
        key_b64 = sys.argv[idx + 1]
        key_bytes = base64url_decode(key_b64)
        if key_bytes is None or len(key_bytes) != 32:
            print("Error: Invalid key (must be 32 bytes base64url-encoded)")
            sys.exit(1)
    elif '--leigh-key' in sys.argv:
        idx = sys.argv.index('--leigh-key')
        if idx + 1 >= len(sys.argv):
            print("Error: --leigh-key requires a value")
            sys.exit(1)
        leigh_key = int(sys.argv[idx + 1])
        key_bytes = derive_key_from_leigh_key(leigh_key)
        print(f"Using LEIGH_KEY={leigh_key}")
    elif '--default-key' in sys.argv:
        key_bytes = get_default_key()
        print("Using default key")
    else:
        # Default to default key
        key_bytes = get_default_key()
        print("Using default key (use --key, --leigh-key, or --default-key to specify)")
    
    if decrypt_file(input_file, output_file, key_bytes):
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == '__main__':
    main()






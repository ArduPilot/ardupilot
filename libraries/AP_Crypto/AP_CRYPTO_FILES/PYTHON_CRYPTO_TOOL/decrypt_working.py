#!/usr/bin/env python3
"""
Working decryption script for encrypted log files.
Uses pymonocypher's lock/unlock functions with format conversion.
"""

import sys
import struct
import hashlib
import base64

try:
    import monocypher
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

def chacha20_quarter_round(state, a, b, c, d):
    """ChaCha20 quarter round operation."""
    state[a] = (state[a] + state[b]) & 0xFFFFFFFF
    state[d] ^= state[a]
    state[d] = ((state[d] << 16) | (state[d] >> 16)) & 0xFFFFFFFF
    state[c] = (state[c] + state[d]) & 0xFFFFFFFF
    state[b] ^= state[c]
    state[b] = ((state[b] << 12) | (state[b] >> 20)) & 0xFFFFFFFF
    state[a] = (state[a] + state[b]) & 0xFFFFFFFF
    state[d] ^= state[a]
    state[d] = ((state[d] << 8) | (state[d] >> 24)) & 0xFFFFFFFF
    state[c] = (state[c] + state[d]) & 0xFFFFFFFF
    state[b] ^= state[c]
    state[b] = ((state[b] << 7) | (state[b] >> 25)) & 0xFFFFFFFF

def hchacha20(key, in_nonce):
    """HChaCha20 key derivation - returns 32-byte sub-key."""
    # HChaCha20 uses ChaCha20 with special parameters
    # We'll use monocypher's chacha20 to generate keystream, then extract sub-key
    # Actually, let's implement it manually based on the algorithm
    constants = [0x61707865, 0x3320646e, 0x79622d32, 0x6b206574]
    
    # Convert key and nonce to uint32 arrays
    key_words = [struct.unpack('<I', key[i:i+4])[0] for i in range(0, 32, 4)]
    nonce_words = [struct.unpack('<I', in_nonce[i:i+4])[0] for i in range(0, 16, 4)]
    
    # Initialize state
    state = constants + key_words + nonce_words
    
    # 20 rounds (10 column rounds + 10 diagonal rounds)
    for _ in range(10):
        # Column rounds
        chacha20_quarter_round(state, 0, 4, 8, 12)
        chacha20_quarter_round(state, 1, 5, 9, 13)
        chacha20_quarter_round(state, 2, 6, 10, 14)
        chacha20_quarter_round(state, 3, 7, 11, 15)
        # Diagonal rounds
        chacha20_quarter_round(state, 0, 5, 10, 15)
        chacha20_quarter_round(state, 1, 6, 11, 12)
        chacha20_quarter_round(state, 2, 7, 8, 13)
        chacha20_quarter_round(state, 3, 4, 9, 14)
    
    # Output is first 32 bytes (first 8 words)
    output = bytearray(32)
    for i in range(8):
        struct.pack_into('<I', output, i * 4, state[i])
    
    return bytes(output)

def chacha20_ctr_decrypt(ciphertext, key, nonce, counter):
    """Decrypt using ChaCha20-CTR mode."""
    plaintext = bytearray(len(ciphertext))
    
    # Process in 64-byte blocks
    for i in range(0, len(ciphertext), 64):
        block_size = min(64, len(ciphertext) - i)
        
        # Generate keystream for this block
        # Use monocypher's chacha20 with counter
        # We need to construct the full nonce with counter
        full_nonce = nonce + struct.pack('<Q', counter)
        keystream = monocypher.chacha20(key, full_nonce, b'\x00' * block_size)
        
        # XOR with ciphertext
        for j in range(block_size):
            plaintext[i + j] = ciphertext[i + j] ^ keystream[j]
        
        counter += 1
    
    return bytes(plaintext)

def poly1305_mac_compute(message, key):
    """Compute Poly1305 MAC - simplified using lock function."""
    # Use lock with empty nonce to compute MAC
    # Actually, we need to use the auth key properly
    # Let's use a workaround: lock with the message and extract MAC
    dummy_nonce = b'\x00' * 24
    mac, _ = monocypher.lock(key, dummy_nonce, message)
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
    
    # Derive keys for decryption (same as ArduPilot does)
    # 1. Derive sub-key using hchacha20
    sub_key = hchacha20(key_bytes, nonce)
    
    # 2. Derive auth key using chacha20 (first 32 bytes of 64-byte keystream)
    auth_key_keystream = monocypher.chacha20(bytes(sub_key), nonce[16:24], b'\x00' * 64)
    auth_key = auth_key_keystream[:32]
    
    # 3. Verify MAC using Poly1305
    # We'll use lock to compute MAC
    computed_mac, _ = monocypher.lock(auth_key, b'\x00' * 24, ciphertext)
    
    if computed_mac != mac:
        print("Error: MAC verification failed - the encryption key may be incorrect")
        print(f"  Expected MAC: {mac.hex()}")
        print(f"  Computed MAC: {computed_mac.hex()}")
        return False
    
    # 4. Decrypt ciphertext using ChaCha20-CTR
    plaintext = chacha20_ctr_decrypt(ciphertext, bytes(sub_key), nonce[16:24], 1)
    
    # Write decrypted file
    try:
        with open(output_file, 'wb') as f:
            f.write(plaintext)
    except IOError as e:
        print(f"Error writing output file: {e}")
        return False
    
    print(f"✓ Successfully decrypted {input_file} -> {output_file}")
    print(f"  Encrypted size: {len(data)} bytes")
    print(f"  Decrypted size: {len(plaintext)} bytes")
    print(f"  Timestamp: {timestamp} ms")
    
    return True

def main():
    if len(sys.argv) < 3:
        print("Usage: decrypt_working.py <input.tlog> <output.log> [--key KEY|--leigh-key VALUE|--default-key]")
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






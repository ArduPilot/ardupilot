#!/usr/bin/env python3
"""
Efficiently test a range of LEIGH_KEY values for decryption.

This script tests LEIGH_KEY values in a given range and reports which ones
successfully decrypt the file.
"""

import sys
import struct
import hashlib
import argparse

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

def derive_key_from_leigh_key(leigh_key_value):
    """Derive a 32-byte key from LEIGH_KEY INT32 value."""
    salt = bytes([0x4c, 0x45, 0x49, 0x47, 0x48, 0x5f, 0x4b, 0x45,
                  0x59, 0x5f, 0x53, 0x41, 0x4c, 0x54, 0x5f, 0x31])
    seed_bytes = struct.pack('<i', leigh_key_value)
    key_bytes = hashlib.blake2b(seed_bytes + salt, digest_size=32).digest()
    return key_bytes

def test_leigh_key(input_file, leigh_key_value, show_progress=False):
    """Test if a LEIGH_KEY value successfully decrypts the file."""
    
    # Read file
    try:
        with open(input_file, 'rb') as f:
            data = f.read()
    except IOError as e:
        print(f"Error reading input file: {e}")
        return False, None
    
    if len(data) < FERNET_HEADER_SIZE + FERNET_MAC_SIZE:
        return False, None
    
    # Parse header
    version = data[0]
    if version != FERNET_VERSION:
        return False, None
    
    nonce = data[9:33]
    ciphertext = data[33:-16]
    mac = data[-16:]
    
    # Derive key
    key = derive_key_from_leigh_key(leigh_key_value)
    
    # Try to decrypt
    plaintext = monocypher.unlock(key, nonce, mac, ciphertext)
    
    if plaintext:
        return True, plaintext
    else:
        return False, None

def main():
    parser = argparse.ArgumentParser(
        description='Test a range of LEIGH_KEY values for decryption',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test range 74768350 to 74768370
  python3 test_leigh_key_range.py 00000002.tlog --start 74768350 --end 74768370
  
  # Test specific values
  python3 test_leigh_key_range.py 00000002.tlog --values 74768360 74768361 74768362
  
  # Test with progress updates every 100 values
  python3 test_leigh_key_range.py 00000002.tlog --start 0 --end 1000000 --progress 100
        """
    )
    parser.add_argument('input', help='Input encrypted .tlog file')
    parser.add_argument('--start', type=int, help='Start of LEIGH_KEY range')
    parser.add_argument('--end', type=int, help='End of LEIGH_KEY range (inclusive)')
    parser.add_argument('--values', type=int, nargs='+', help='Specific LEIGH_KEY values to test')
    parser.add_argument('--output', help='Output file for decrypted data (default: input_decrypted.log)')
    parser.add_argument('--progress', type=int, default=1000, 
                       help='Show progress every N values (default: 1000)')
    parser.add_argument('--quiet', action='store_true',
                       help='Only show successful results')
    
    args = parser.parse_args()
    
    # Determine values to test
    values_to_test = []
    if args.values:
        values_to_test = args.values
    elif args.start is not None and args.end is not None:
        values_to_test = range(args.start, args.end + 1)
    else:
        print("Error: Must specify --start/--end or --values")
        sys.exit(1)
    
    if not args.quiet:
        print(f"Testing {len(values_to_test)} LEIGH_KEY value(s)...")
        if args.start is not None:
            print(f"Range: {args.start} to {args.end}")
        print()
    
    # Determine output file
    if args.output:
        output_file = args.output
    else:
        if args.input.endswith('.tlog'):
            output_file = args.input[:-5] + '_decrypted.log'
        else:
            output_file = args.input + '_decrypted.log'
    
    # Test each value
    tested = 0
    for leigh_key in values_to_test:
        tested += 1
        
        # Show progress
        if not args.quiet and tested % args.progress == 0:
            print(f"Tested {tested}/{len(values_to_test)} values... (current: {leigh_key})", end='\r')
        
        # Test this value
        success, plaintext = test_leigh_key(args.input, leigh_key)
        
        if success:
            print(f"\n✓ SUCCESS with LEIGH_KEY={leigh_key}!")
            
            # Write decrypted file
            try:
                with open(output_file, 'wb') as f:
                    f.write(plaintext)
            except IOError as e:
                print(f"Error writing output file: {e}")
                sys.exit(1)
            
            print(f"  Decrypted file: {output_file}")
            print(f"  Decrypted size: {len(plaintext)} bytes")
            print(f"  LEIGH_KEY value: {leigh_key}")
            print(f"  Total values tested: {tested}")
            sys.exit(0)
    
    if not args.quiet:
        print(f"\n✗ All {tested} LEIGH_KEY values failed")
        print()
        print("Possible reasons:")
        print("  1. File was encrypted with a key stored in secure storage")
        print("  2. LEIGH_KEY value is outside the tested range")
        print("  3. File was encrypted with a different key derivation method")
        print()
        print("Suggestions:")
        print("  - Try a wider range")
        print("  - Extract key from firmware's secure storage")
        print("  - Check firmware logs/config for LEIGH_KEY value")
    
    sys.exit(1)

if __name__ == '__main__':
    main()






#!/usr/bin/env python3
"""
Calculate flash address for key storage extraction.

Usage:
    python3 calc_flash_address.py <STORAGE_FLASH_PAGE> <PAGE_SIZE_KB> [FLASH_BASE]

Example:
    python3 calc_flash_address.py 14 128        # STM32H7, page 14, 128KB pages
    python3 calc_flash_address.py 1 2          # STM32F4, page 1, 2KB pages
    python3 calc_flash_address.py 18 2 0x08000000  # STM32L4, explicit base
"""

import sys

# Default STM32 flash base address
DEFAULT_FLASH_BASE = 0x08000000

# Key offset within storage
KEY_OFFSET = 8064

def calculate_addresses(storage_page, page_size_kb, flash_base=DEFAULT_FLASH_BASE):
    """Calculate flash addresses for storage and key."""
    
    # Calculate page offset in bytes
    page_offset = storage_page * page_size_kb * 1024
    
    # Flash page address
    flash_page_addr = flash_base + page_offset
    
    # Key address
    key_address = flash_page_addr + KEY_OFFSET
    
    return flash_page_addr, key_address

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    
    try:
        storage_page = int(sys.argv[1])
        page_size_kb = int(sys.argv[2])
        flash_base = int(sys.argv[3], 16) if len(sys.argv) > 3 else DEFAULT_FLASH_BASE
    except ValueError as e:
        print(f"Error: Invalid argument - {e}")
        sys.exit(1)
    
    flash_page_addr, key_address = calculate_addresses(storage_page, page_size_kb, flash_base)
    
    print("=" * 70)
    print("Flash Address Calculation")
    print("=" * 70)
    print(f"Storage Flash Page: {storage_page}")
    print(f"Page Size: {page_size_kb} KB")
    print(f"Flash Base: 0x{flash_base:08X}")
    print()
    print(f"Flash Page Address: 0x{flash_page_addr:08X}")
    print(f"Key Offset: {KEY_OFFSET} bytes")
    print(f"Key Address: 0x{key_address:08X}")
    print()
    print("=" * 70)
    print("GDB Commands:")
    print("=" * 70)
    print(f"dump binary memory storage_block.bin 0x{key_address:08X} 0x{key_address+40:08X}")
    print()
    print("=" * 70)
    print("OpenOCD Command:")
    print("=" * 70)
    print(f"dump_image storage_block.bin 0x{key_address:08X} 40")
    print()
    print("=" * 70)
    print("J-Link Command:")
    print("=" * 70)
    print(f"savebin storage_block.bin 0x{key_address:08X} 40")
    print()
    print("=" * 70)
    print("After extraction:")
    print("=" * 70)
    print("python3 view_stored_key.py storage_block.bin")

if __name__ == "__main__":
    main()






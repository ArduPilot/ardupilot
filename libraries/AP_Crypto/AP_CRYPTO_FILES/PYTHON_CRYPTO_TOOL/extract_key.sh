#!/bin/bash
# Quick script to extract key from storage dump

if [ $# -eq 0 ]; then
    echo "Usage: $0 <key.bin or storage_block.bin>"
    echo ""
    echo "This script extracts the encryption key from a 40-byte storage dump."
    echo ""
    echo "Example:"
    echo "  $0 key.bin"
    echo "  $0 /path/to/storage_block.bin"
    exit 1
fi

FILE="$1"

if [ ! -f "$FILE" ]; then
    echo "Error: File '$FILE' not found"
    exit 1
fi

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Run the Python script
python3 "$SCRIPT_DIR/view_stored_key.py" "$FILE"






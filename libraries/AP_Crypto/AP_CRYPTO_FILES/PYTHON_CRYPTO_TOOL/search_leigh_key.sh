#!/bin/bash
# Search for LEIGH_KEY references in various file types

echo "Searching for LEIGH_KEY references..."
echo "======================================"
echo ""

# Search parameter files
echo "=== Parameter Files (.param, .parm) ==="
find . -type f \( -name "*.param" -o -name "*.parm" \) 2>/dev/null | while read file; do
    if grep -qi "LEIGH_KEY" "$file" 2>/dev/null; then
        echo "Found in: $file"
        grep -i "LEIGH_KEY" "$file" 2>/dev/null | head -3
        echo ""
    fi
done

# Search log files
echo "=== Log Files (.log, .tlog) ==="
find . -type f \( -name "*.log" -o -name "*.tlog" \) 2>/dev/null | while read file; do
    if grep -qi "LEIGH_KEY" "$file" 2>/dev/null; then
        echo "Found in: $file"
        grep -i "LEIGH_KEY" "$file" 2>/dev/null | head -3
        echo ""
    fi
done

# Search text files
echo "=== Text Files (.txt) ==="
find . -maxdepth 2 -type f -name "*.txt" 2>/dev/null | while read file; do
    if grep -qi "LEIGH_KEY" "$file" 2>/dev/null; then
        echo "Found in: $file"
        grep -i "LEIGH_KEY" "$file" 2>/dev/null | head -3
        echo ""
    fi
done

# Search for parameter value patterns (numbers near LEIGH_KEY)
echo "=== Parameter Value Patterns ==="
find . -type f \( -name "*.param" -o -name "*.parm" -o -name "*.txt" \) 2>/dev/null | while read file; do
    if grep -qiE "LEIGH_KEY.*[0-9]+|[0-9]+.*LEIGH_KEY" "$file" 2>/dev/null; then
        echo "Found pattern in: $file"
        grep -iE "LEIGH_KEY.*[0-9]+|[0-9]+.*LEIGH_KEY" "$file" 2>/dev/null | head -3
        echo ""
    fi
done

echo "Search complete."
echo ""
echo "Note: LEIGH_KEY parameter is hidden when read (returns 0),"
echo "      so finding it in parameter files may show 0 even if it was set."






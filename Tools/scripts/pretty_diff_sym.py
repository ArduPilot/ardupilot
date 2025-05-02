#!/usr/bin/env python

'''
This script intends to provide a pretty symbol size diff between two binaries.

AP_FLAKE8_CLEAN
'''

import subprocess


def run_nm(file):
    """Run nm on the specified file and return the output."""
    result = subprocess.run(['arm-none-eabi-nm', '-S', '-C', '--size-sort', file], capture_output=True, text=True)
    if result.returncode != 0:
        raise Exception(f"Error running nm on {file}: {result.stderr}")
    return result.stdout


def parse_nm_output(output):
    """Parse the nm output and return a dictionary of symbols and their sizes."""
    symbols = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 4:
            size = int(parts[1], 16)
            symbol = parts[3]
            symbols[symbol] = size
    return symbols


def compare_symbols(symbols1, symbols2):
    """Compare the symbols between two dictionaries and return the size differences."""
    diff = {}
    all_symbols = set(symbols1.keys()).union(symbols2.keys())
    for symbol in all_symbols:
        size1 = symbols1.get(symbol, 0)
        size2 = symbols2.get(symbol, 0)
        if size1 != size2:
            diff[symbol] = (size1, size2, size2 - size1)
    return diff


def main(file1, file2):
    output1 = run_nm(file1)
    output2 = run_nm(file2)

    symbols1 = parse_nm_output(output1)
    symbols2 = parse_nm_output(output2)

    diff = compare_symbols(symbols1, symbols2)

    print(f"Symbol size differences between {file1} and {file2}:")
    total = 0
    for symbol, (size1, size2, change) in diff.items():
        print("%4d %s %s -> %s" % (change, symbol, size1, size2))
        total += change
    print(f"Total: {total}")


if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <file1> <file2>")
        sys.exit(1)

    file1 = sys.argv[1]
    file2 = sys.argv[2]

    main(file1, file2)

#!/bin/sh
#
# cppcheck static analysis
# For Debian based: apt-get install cppcheck
#

num_cores=$(grep -c ^processor /proc/cpuinfo)
if [ -z "$num_cores" ]; then
    echo "Hey, it looks like we're not on Linux. Please fix this script to add support for this OS."
    num_cores=4
fi
echo "Number of threads for cppcheck: $num_cores"

# TODO: with future versions of cppcheck, add --library=glibc
cppcheck . --error-exitcode=1 --quiet --enable=all --platform=unix64 --std=c99 --std=c++11 \
           --inline-suppr --force --template=gcc -j$num_cores \
           -U__BIGGEST_ALIGNMENT__ -UUAVCAN_MEM_POOL_BLOCK_SIZE -UBIG_ENDIAN -UBYTE_ORDER \
           -Iinclude $@

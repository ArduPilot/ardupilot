#!/bin/sh

# Create build directory.
if [ -d build ]; then
    rm -rf build
fi
mkdir build && cd build;

# Compile profiles.
PROFILES="complete_profile core_profile tcp_profile udp_profile serial_profile custom_profile"
for P in $PROFILES
do
    cmake -DUCLIENT_CONFIG="${PWD}"/../test/memory/memory_map/$P.config ..;
    make;
    ../test/memory/memory_map/memory_map.sh $P
done

# Launch memory map analysis.
python3 ../test/memory/memory_map/memory_map_analysis.py ./memory_map.txt

# Launch stack analysis.
cmake -DUCLIENT_MEMORY_TESTS=ON -DUCLIENT_CONFIG="${PWD}"/../test/memory/consumption/performance_profile.config ..
make
../test/memory/consumption/stack_usage.sh

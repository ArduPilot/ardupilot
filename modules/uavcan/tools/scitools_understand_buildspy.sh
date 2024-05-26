#!/bin/sh
#
# This script generates project file for SciTools Understand via buildspy.
#

buildspy_dir="$1"
uavcan_dir="$(readlink -f $(dirname $0)/..)"

function die() { echo $1; exit 1; }

[ -z "$buildspy_dir" ] && die "Path to buildspy directory expected, e.g. ~/opt/scitools/bin/linux64/buildspy/"

compiler="$buildspy_dir/g++wrapper"
buildspy="$buildspy_dir/buildspy"

echo "Pathes:"
echo "compiler: $compiler"
echo "buildspy: $buildspy"
echo "uavcan:   $uavcan_dir"

read -p "Looks good? (y/N) " confirm
[[ $confirm == "y" ]] || die "Bye"

cmake "$uavcan_dir" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER="$compiler" || exit 1

$buildspy -db uavcan.udb -cmd make

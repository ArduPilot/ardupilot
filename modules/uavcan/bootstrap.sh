#!/usr/bin/env bash

# +----------------------------------------------------------+
# | BASH : Modifying Shell Behaviour
# |    (https://www.gnu.org/software/bash/manual)
# +----------------------------------------------------------+
# Treat unset variables and parameters other than the special 
# parameters ‘@’ or ‘*’ as an error when performing parameter 
# expansion. An error message will be written to the standard 
# error, and a non-interactive shell will exit.
set -o nounset

# Exit immediately if a pipeline returns a non-zero status.
set -o errexit

# If set, the return value of a pipeline is the value of the 
# last (rightmost) command to exit with a non-zero status, or 
# zero if all commands in the pipeline exit successfully.
set -o pipefail

# +----------------------------------------------------------+

sudo apt-get update
sudo apt-get -y install software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa -y
sudo apt-get update
sudo apt-get -y install cmake
sudo apt-get -y install python3
sudo apt-get -y install git
sudo apt-get -y install g++-5;
sudo apt-get -y install gcc-arm-embedded

# Export to tell cmake which native compilers to use.
export CXX="g++-5" CC="gcc-5";

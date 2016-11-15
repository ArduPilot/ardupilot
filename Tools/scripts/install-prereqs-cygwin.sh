#!/bin/bash

# Install pre-reqs for cywgin
BASE_PKGS="ccache,g++,gawk,git,make,wget,nano,automake"
PX4_PKGS="autoconf"
SITL_PKGS="libtool,libxml2-dev,libxslt1-dev,python-devel,procps,libexpat,libxslt-devel"

echo "Checking cygwin setup.exe name..."
cd c:
SETUP_CMD=$(ls c:/cygwin/setup-*.exe)
#if this fails it says "ls: cannot access ....: No such file or directory" which is currently unhandled



# install packages
# arguments, see https://cygwin.com/faq/faq.html#faq.setup.cli
echo -e "\n\nInstalling packages..."
$SETUP_CMD --no-shortcuts --quiet-mode --disable-buggy-antivirus --packages $BASE_PKGS,$PX4_PKGS,$SITL_PKGS

# reload profile so packages work immediately
. ~/.profile

echo -e "\n\nDone!"

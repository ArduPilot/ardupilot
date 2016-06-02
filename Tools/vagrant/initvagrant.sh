#!/bin/bash

set -e

echo "Initial setup of SITL-vagrant instance."

BASE_PKGS="gawk make git arduino-core curl"
SITL_PKGS="g++ python-pip python-matplotlib python-serial python-wxgtk2.8 python-scipy python-opencv python-numpy python-empy python-pyparsing ccache"
PYTHON_PKGS="pymavlink MAVProxy droneapi"
PX4_PKGS="python-serial python-argparse openocd flex bison libncurses5-dev \
          autoconf texinfo build-essential libftdi-dev libtool zlib1g-dev \
          zip genromfs cmake"
UBUNTU64_PKGS="libc6:i386 libgcc1:i386 gcc-4.9-base:i386 libstdc++5:i386 libstdc++6:i386"

# GNU Tools for ARM Embedded Processors
# (see https://launchpad.net/gcc-arm-embedded/)
ARM_ROOT="gcc-arm-none-eabi-4_9-2015q3"
ARM_TARBALL="$ARM_ROOT-20150921-linux.tar.bz2"
ARM_TARBALL_URL="http://firmware.ardupilot.org/Tools/PX4-tools/$ARM_TARBALL"

# Ardupilot Tools
ARDUPILOT_TOOLS="ardupilot/Tools/autotest"

sudo usermod -a -G dialout $USER

sudo apt-get -y remove modemmanager
sudo apt-get -y update
sudo apt-get -y install dos2unix g++-4.7 ccache python-lxml screen
sudo apt-get -y install $BASE_PKGS $SITL_PKGS $PX4_PKGS $UBUNTU64_PKGS
sudo pip -q install $PYTHON_PKGS
sudo pip install catkin_pkg


# ARM toolchain
if [ ! -d /opt/$ARM_ROOT ]; then
    (
        cd /opt;
        sudo wget -nv $ARM_TARBALL_URL;
        sudo tar xjf ${ARM_TARBALL};
        sudo rm ${ARM_TARBALL};
    )
fi

exportline="export PATH=/opt/$ARM_ROOT/bin:\$PATH"
if grep -Fxq "$exportline" /home/vagrant/.profile; then echo nothing to do ; else echo $exportline >> /home/vagrant/.profile; fi

echo "source /vagrant/Tools/vagrant/shellinit.sh" >>/home/vagrant/.profile
# This allows the PX4NuttX build to proceed when the underlying fs is on windows
# It is only marginally less efficient on Linux
echo "export PX4_WINTOOL=y" >>/home/vagrant/.profile
ln -fs /vagrant/Tools/vagrant/screenrc /home/vagrant/.screenrc

# build JSB sim
pushd /tmp
rm -rf jsbsim
git clone git://github.com/tridge/jsbsim.git
sudo apt-get install -y libtool automake autoconf libexpat1-dev
cd jsbsim
./autogen.sh
make -j2
sudo make install
popd

# Now you can run
# vagrant ssh -c "screen -d -R"

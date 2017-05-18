#!/bin/bash

# this script is run by the root user in the virtual machine

set -e
set -x

echo "Initial setup of SITL-vagrant instance."

BASE_PKGS="gawk make git arduino-core curl"
SITL_PKGS="g++ python-pip python-matplotlib python-serial python-wxgtk3.0 python-scipy python-opencv python-numpy python-empy python-pyparsing ccache"
PYTHON_PKGS="pymavlink MAVProxy droneapi future"
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

VAGRANT_USER=ubuntu

usermod -a -G dialout $VAGRANT_USER

apt-get -y remove modemmanager
apt-get -y update
apt-get -y install dos2unix g++-4.7 ccache python-lxml screen xterm gdb
apt-get -y install $BASE_PKGS $SITL_PKGS $PX4_PKGS $UBUNTU64_PKGS
pip -q install $PYTHON_PKGS
easy_install catkin_pkg


# ARM toolchain
if [ ! -d /opt/$ARM_ROOT ]; then
    (
        sudo -u $VAGRANT_USER wget -nv $ARM_TARBALL_URL
        pushd /opt
        tar xjf ${OLDPWD}/${ARM_TARBALL}
        popd
        rm ${ARM_TARBALL}
    )
fi

exportline="export PATH=/opt/$ARM_ROOT/bin:\$PATH"
DOT_PROFILE=/home/$VAGRANT_USER/.profile
PROFILE_TEXT=""
if grep -Fxq "$exportline" $DOT_PROFILE; then
    echo nothing to do
else
    PROFILE_TEXT="
$PROFILE_TEXT
$exportline
"
fi

PROFILE_TEXT="
$PROFILE_TEXT
source /vagrant/Tools/vagrant/shellinit.sh
# This allows the PX4NuttX build to proceed when the underlying fs is on windows
# It is only marginally less efficient on Linux
export PX4_WINTOOL=y
export PATH=\$PATH:\$HOME/jsbsim/src
"

echo "$PROFILE_TEXT" | sudo -u $VAGRANT_USER dd conv=notrunc oflag=append of=$DOT_PROFILE
sudo -u $VAGRANT_USER ln -fs /vagrant/Tools/vagrant/screenrc /home/$VAGRANT_USER/.screenrc

# build JSB sim
apt-get install -y libtool libtool-bin automake autoconf libexpat1-dev
sudo -u $VAGRANT_USER sh <<"EOF"
cd $HOME
rm -rf jsbsim
git clone https://github.com/tridge/jsbsim.git
cd jsbsim
./autogen.sh
make -j2
EOF

#Plant a marker for sim_vehicle that we're inside a vagrant box
touch /ardupilot.vagrant

# Now you can run
# vagrant ssh -c "screen -d -R"

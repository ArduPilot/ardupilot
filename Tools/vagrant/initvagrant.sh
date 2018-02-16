#!/bin/bash

# this script is run by the root user in the virtual machine

set -e
set -x

echo "Initial setup of SITL-vagrant instance."

VAGRANT_USER=ubuntu
if [ -e /home/vagrant ]; then
    # prefer vagrant user
    VAGRANT_USER=vagrant
fi

# artful rootfs is 2GB without resize:
sudo resize2fs /dev/sda1

usermod -a -G dialout $VAGRANT_USER

/vagrant/Tools/scripts/install-prereqs-ubuntu.sh -y

# run-in-terminal-window uses xterm:
apt-get install -y xterm

# valgrind support:
apt-get install -y valgrind

# gdb support:
apt-get install -y gdb

# gcov support:
apt-get install -y gcovr lcov

# install pexpect for autotest.py
pip install pexpect

sudo -u $VAGRANT_USER ln -fs /vagrant/Tools/vagrant/screenrc /home/$VAGRANT_USER/.screenrc

# build JSB sim
apt-get install -y libtool automake autoconf libexpat1-dev
#  libtool-bin
sudo --login -u $VAGRANT_USER /vagrant/Tools/scripts/build-jsbsim.sh

# adjust environment for every login shell:
DOT_PROFILE=/home/$VAGRANT_USER/.profile
echo "source /vagrant/Tools/vagrant/shellinit.sh" |
    sudo -u $VAGRANT_USER dd conv=notrunc oflag=append of=$DOT_PROFILE

#Plant a marker for sim_vehicle that we're inside a vagrant box
touch /ardupilot.vagrant

# Now you can run
# vagrant ssh -c "screen -d -R"

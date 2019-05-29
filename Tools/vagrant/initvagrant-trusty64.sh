#!/bin/bash

# this script is run by the root user in the virtual machine

set -e
set -x
set -u

echo "Initial setup of SITL-vagrant instance."

/vagrant/Tools/environment_install/install-prereqs-ubuntu.sh -y

# extra packages we desire on the VM but aren't prereqs for AP compilation:
sudo apt-get install -y valgrind gdb

VAGRANT_USER=vagrant

PROFILE_TEXT="
source /vagrant/Tools/vagrant/shellinit.sh
# This allows the PX4NuttX build to proceed when the underlying fs is on windows
# It is only marginally less efficient on Linux
export PX4_WINTOOL=y
export PATH=\$PATH:\$HOME/jsbsim/build/src
export BUILDLOGS=/tmp/buildlogs
"

DOT_PROFILE=/home/$VAGRANT_USER/.profile

echo "$PROFILE_TEXT" | sudo -u $VAGRANT_USER dd conv=notrunc oflag=append of=$DOT_PROFILE
sudo -u $VAGRANT_USER ln -fs /vagrant/Tools/vagrant/screenrc /home/$VAGRANT_USER/.screenrc

# build JSB sim
sudo -u $VAGRANT_USER /vagrant/Tools/scripts/install-jsbsim.sh /home/$VAGRANT_USER

#Plant a marker for sim_vehicle that we're inside a vagrant box
touch /ardupilot.vagrant

# Now you can run
# vagrant ssh -c "screen -d -R"

#!/bin/bash

set -e
set -x

# this should be install by default, but may as well:
sudo apt install python3

# add $HOME/bin into the binary search path so we can force python-is-python3:
mkdir $HOME/bin
rm -f $HOME/bin/python
ln -s /usr/bin/python3 $HOME/bin/python

echo 'export PATH=$HOME/bin:$PATH' >>$HOME/.profile

# apt Python packages (swiped from install-prereqs-ubuntu.sh):
sudo apt install -y python3-wxgtk4.0 python3-opencv python3-matplotlib python3-pip

# pip-install python packages (also swiped from install-prereqs-ubuntu.sh):
pip3 install future lxml pymavlink MAVProxy pexpect flake8==3.7.9 requests==2.27.1 monotonic==1.6 geocoder empy configparser==4.0.2 click==7.1.2 decorator==4.4.2 dronecan pygame intelhex empy

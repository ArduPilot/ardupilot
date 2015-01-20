#!/bin/bash

set -e 

echo "Initial setup of SITL-vagrant instance."
sudo apt-get -y update
sudo apt-get -y install dos2unix python-wxgtk2.8 python-scipy python-matplotlib python-opencv python-pip g++ g++-4.7 gawk git ccache  python-serial python-wxgtk2.8 python-lxml screen

sudo pip install pymavlink MAVProxy

echo "source /vagrant/Tools/vagrant/shellinit.sh" >>/home/vagrant/.profile
ln -s /vagrant/Tools/vagrant/screenrc /home/vagrant/.screenrc

echo "NOTE: Currently this vagrant file only support simulating copters and rovers, adding support for plane would be straightforward,"
echo "just add JSB sim per Tridge's instructions (and send in a pull-request)"

# 
# vagrant ssh -c "screen -d -R"
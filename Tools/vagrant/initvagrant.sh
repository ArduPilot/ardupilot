#!/bin/bash

set -e 

echo "Initial setup of SITL-vagrant instance."
sudo apt-get -y update
sudo apt-get -y install dos2unix python-wxgtk2.8 python-scipy python-matplotlib python-opencv python-pip g++ g++-4.7 gawk git ccache  python-serial python-wxgtk2.8 python-lxml screen

sudo pip install pymavlink MAVProxy

echo "source /vagrant/Tools/vagrant/shellinit.sh" >>/home/vagrant/.profile
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

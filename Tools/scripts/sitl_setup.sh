#############################################################
## Ardupilot SITL Setup Script
##
## If something fails you can uncomment the lines that was successfully
## by placing a '#' at the beginning of the line
##
## To be able to run this script in the terminal run:
## chmod +x instsim.sh
## first to mark the script as executable
##
##
## Svein A. Frotjold
## svefro@gmail.com
## this may work. i guarantee nothing.
#############################################################

echo 'This script requires the ardupilot source folder to be located in your home directory. /HOME/USERNAME/ardupilot'
echo 'ardupilot needs to be all lower case'
echo 'Run install-prereqs-ubuntu.sh before you run this script'
read -p "Do you want to run ''install-prereqs-ubuntu.sh'' now? (This will download everything without asking) (y/n) " RESP0
if [ "$RESP0" = "y" ]; then
    echo '######################################################'
    echo 'Running: install-prereqs-ubuntu.sh -y'
    echo '######################################################'
    cd ~/ardupilot/Tools/scripts
    chmod +x install-prereqs-ubuntu.sh
    ./install-prereqs-ubuntu.sh -y
fi


#######################
## Update Package List
#######################
echo '######################################################'
read -p "Do you want to run ''sudo apt-get update''? Do this if it's the first time you run this script (y/n) " RESP1
if [ "$RESP1" = "y" ]; then
    sudo apt-get update
fi

#######################
## Install Packages
##
##JNJO: Commented packages that are installed by install-prereqs-ubuntu.sh
##JNJO: Clone ArduPilot sourcecode in Ubuntu, eliminating the need for shared folder and compiling with Windows. (Necessary for install-prereqs-ubuntu.sh to work)
##
#######################


echo '######################################################'
read -p "Do you want to install common required software? (y/n) " RESP2
if [ "$RESP2" = "y" ]; then
#    sudo apt-get -y install python-pip
#    sudo apt-get -y install python-wxgtk2.8
#    sudo apt-get -y install python-matplotlib
#    sudo apt-get -y install python-opencv
#    sudo apt-get -y install g++
#    sudo apt-get -y install gawk
#    sudo apt-get -y install git
#    sudo apt-get -y install build-essential
     sudo apt-get -y install dos2unix
     sudo apt-get -y install g++-4.7
     sudo apt-get -y install git
fi

#######################

echo '######################################################'
read -p "Do you want to run install required packages for ArduPlane (y/n) " RESP3
if [ "$RESP3" = "y" ]; then
#######################
##
##JNJO: Uncommented packages that are installed by install-prereqs-ubuntu.sh
##JNJO: Added JSBsim Clone and Build
##
#######################
#    sudo apt-get -y install autoconf
#    sudo apt-get -y install libtool
    sudo apt-get -y install libexpat1-dev
    sudo apt-get -y install automake
    cd
    git clone git://github.com/tridge/jsbsim.git
    cd ~/jsbsim
    ./autogen.sh
    make
fi



#######################
## put directories in .bashrc only run this once!!!
## sample: echo 'PATH HERE' >> ~/.bashrc
## open .bashrc in your user folder to check that the lines is only added once
## the lines have been disabled with '#' by default. copy the lines below to the bottom of .bashrc and remove the '#' from all the lines
##
##JNJO: Added export PATH, so the paths work in this terminal instance.
##JNJO: Commented reload .bashrc (Doesn't work in the same terminal instance. Added .~/.profile at the end to update PATH)
##
#######################
echo '######################################################'
read -p "Do you want to insert required lines in .bashrc? (ONLY DO THIS ONCE) (y/n) " RESP4
if [ "$RESP4" = "y" ]; then
    cd
    echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest'  >> ~/.bashrc 
    echo 'export PATH=$PATH:/usr/local/lib/python2.7/dist-packages/MavProxy' >>~/.bashrc
    echo 'export PATH=$PATH:/usr/local/lib/python2.7/dist-packages/pymavlink/examples' >>~/.bashrc
    echo 'export PATH=$PATH:$HOME/jsbsim/src' >>~/.bashrc
    echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc
    export PATH=$PATH:$HOME/ardupilot/Tools/autotest
    export PATH=$PATH:/usr/local/lib/python2.7/dist-packages/MavProxy
    export PATH=$PATH:/usr/local/lib/python2.7/dist-packages/pymavlink/examples
    export PATH=$PATH:$HOME/jsbsim/src
    export PATH=/usr/lib/ccache:$PATH

fi


#######################
#Reload .bashrc
#######################
#echo "Reloading .bashrc"
#. ~/.bashrc


#######################
## Install Python Packages with pip
##
##JNJO: Commented packages that are installed by install-prereqs-ubuntu.sh
##
#######################

# sudo pip install pymavlink
# sudo pip install mavproxy


#######################

#######################
##JNJO: Added make configure.
##JNJO: Added chmod +x ~/sim.sh to make things easier. (Less typing....)
##JNJO: Added . ~/.profile (Looking at install-prereqs-ubuntu.sh, it seems to be preferred over .bashrc. Both ways appear to work.)
##JNJO: Added parameter change to copter_params.parm [ARMING_CHECK 0] (Several users gets stuck at this, and can't arm due to "Pre-arm: INS not calibrated")
##JNJO: Added instruction to get PATH statements in bashrc working, and reminder to use simrc.sh
#######################
echo '######################################################'
cd ~/ardupilot/ArduCopter
make configure

#cd
#chmod +x ~/sim.sh
#. ~/.profile

#echo 'ARMING_CHECK 0' >> ~/ardupilot/Tools/autotest/copter_params.parm
echo '######################################################'
cd
cd ~/ardupilot/Tools/autotest
chmod -x sim_start.sh

read -p "Now either close this terminal, and then open a new terminal before running /ardupilot/Tools/autotest/sim_start.sh.
Or type .~/.bashrc in this terminal, then run /ardupilot/Tools/autotest/sim_start.sh"
#EOF

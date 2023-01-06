#!/bin/bash
echo "---------- $0 start ----------"
set -e
# set -x

ROS_WS_ROOT=$HOME/ardupilot-ws
AP_GZ_ROOT=$HOME/ardupilot_gazebo

red=`tput setaf 1`
green=`tput setaf 2`
reset=`tput sgr0`

sep="##############################################"


function heading() {
    echo "$sep"
    echo $*
    echo "$sep"
}

ASSUME_YES=false

function maybe_prompt_user() {
    if $ASSUME_YES; then
        return 0
    else
        read -p "$1"
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            return 0
        else
            return 1
        fi
    fi
}

function usage
{
    echo "Usage: ./installROS.sh [[-p package] | [-h]]"
    echo "Install ROS1"
    echo "This script will select the ROS distribution according to the OS being used"
    echo "Installs desktop-full as default base package; Use -p to override"
    echo "-p | --package <packagename>  ROS package to install"
    echo "                              Multiple usage allowed"
    echo "                              Must include one of the following:"
    echo "                               ros-base"
    echo "                               desktop"
    echo "                               desktop-full"
    echo "-h | --help  This message"
}

function shouldInstallPackages
{
    echo "${red}Your package list did not include a recommended base package${reset}"
    echo "Please include one of the following:"
    echo "   ros-base"
    echo "   desktop"
    echo "   desktop-full"
    echo ""
    echo "ROS not installed"
}

function package_is_installed() {
    dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -c "ok installed"
}

# Iterate through command line inputs
packages=()
while [ "$1" != "" ]; do
    case $1 in
        -p | --package )        shift
                                packages+=("$1")
                                ;;
        -h | --help )           usage
                                exit
                                ;;
        * )                     usage
                                exit 1
    esac
    shift
done

# Install lsb-release as it is needed to check Ubuntu version
if ! package_is_installed "lsb-release"; then
    heading "Installing lsb-release"
    sudo apt install lsb-release -y
    echo "Done!"
fi

# Checking Ubuntu release to adapt software version to install
RELEASE_CODENAME=$(lsb_release -c -s)
PYTHON_V="python3"  # starting from ubuntu 20.04, python isn't symlink to default python interpreter


# echo $RELEASE_CODENAME

if [ ${RELEASE_CODENAME} == 'bionic' ] ; then
    #Ubuntu 18.04 - Melodic
    ROS_VERSION="melodic"
    PYTHON_V="python2"
    heading "${green}Detected Ubuntu 18.04, installing ROS Melodic${reset}"
elif [ ${RELEASE_CODENAME} == 'buster' ]; then
    #RPi Buster - Melodic
    ROS_VERSION="melodic"
    PYTHON_V="python2"
    heading "${green}Detected RPi Buster, installing ROS Melodic${reset}"
elif [ ${RELEASE_CODENAME} == 'focal' ]; then
    #Ubuntu 20.04 - Noetic
    ROS_VERSION="noetic"
    PYTHON_V="python3"
    heading "${green}Detected Ubuntu 20.04, installing ROS Noetic${reset}"
elif [ ${RELEASE_CODENAME} == 'jammy' ]; then
    #Ubuntu 22.04 - unsupported only ROS2
    heading "${red}Currently only ROS1 is supported. This Ubuntu release can only be used with ROS2.${reset}"
    exit 1
else
    # We assume an unsupported OS is being used.
    heading "${red}Unsupported OS detected. Please refer to the ROS webpage to find how to install ROS1 on your system if at all possible.${reset}"
    exit 1
fi

# Check to see if other packages were specified
# If not, set the default base package
if [ ${#packages[@]}  -eq 0 ] ; then
 packages+="desktop-full"
fi
echo "Packages to install: "${packages[@]}
# Check to see if we have a ROS base kinda thingie
hasBasePackage=false
for package in "${packages[@]}"; do
  if [[ $package == "ros-base" ]]; then
     delete=ros-base
     packages=( "${packages[@]/$delete}" )
     packages+=" ros-${ROS_VERSION}-ros-base"
     hasBasePackage=true
     break
  elif [[ $package == "desktop" ]]; then
     delete=desktop
     packages=( "${packages[@]/$delete}" )
     packages+=" ros-${ROS_VERSION}-desktop"    
     hasBasePackage=true
     break
  elif [[ $package == "desktop-full" ]]; then
     delete=desktop-full
     packages=( "${packages[@]/$delete}" )
     packages+=" ros-${ROS_VERSION}-desktop-full"    
     hasBasePackage=true
     break
  fi
done
if [ $hasBasePackage == false ] ; then
   shouldInstallPackages
   exit 1
fi

heading "${green}Adding Repositories and source lists${reset}"
#Lets start instaling stuff
sudo apt install software-properties-common -y
sudo apt-add-repository universe
sudo apt-add-repository multiverse
sudo apt-add-repository restricted

# Setup sources.lst
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# If you experience issues connecting to the keyserver, you can try substituting hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 in the previous command.
# Installation

heading "${green}Updating apt${reset}"
sudo apt update

heading "${green}Installing ROS${reset}"
# Here we loop through any packages passed on the command line
# Install packages ...
for package in "${packages[@]}"; do
  sudo apt install $package -y
done

# This is where you might start to modify the packages being installed, i.e.# sudo apt install ros-${ROS_VERSION}-{package_name}
# sudo apt install -y setpriv
# sudo apt install -y ros-${ROS_VERSION}-robot-upstart
# sudo apt install -y ros-${ROS_VERSION}-navigation

# Install MAVROS and the geographic libs
sudo apt install -y ros-${ROS_VERSION}-mavros

wget  https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# install other needed packages
sudo apt install build-essential cmake -y

#
# To find available packages:
# apt-cache search ros-melodic
# 
# Initialize rosdep

heading "${green}Installing rosdep${reset}"

sudo apt install ${PYTHON_V}-rosdep -y
# Certificates are messed up on earlier version Jetson for some reason
# Do not know if it is an issue with the Xavier, test by commenting out
# sudo c_rehash /etc/ssl/certs
# Initialize rosdep

heading "${green}Initializaing rosdep${reset}"

sudo rosdep init || true
# To find available packages, use:
rosdep update
# Use this to install dependencies of packages in a workspace
# rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
# Environment Setup - Don't add /opt/ros/${ROS_VERSION}/setup.bash if it's already in bashrc
if maybe_prompt_user "Do you want to add ROS_HOSTNAME and ROS_MASTER_URI to your .bashrc [N/y]?" ; then
    heading "${green}Adding setup.bash, ROS_MASTER_URI and ROS_HOSTNAME to .bashrc ${reset}"
    grep -q -F "ROS_HOSTNAME=$HOSTNAME.local" ~/.bashrc || echo "ROS_HOSTNAME=$HOSTNAME.local" >> ~/.bashrc
    grep -q -F "ROS_MASTER_URI=http://$HOSTNAME.local:11311" ~/.bashrc || echo "ROS_MASTER_URI=http://$HOSTNAME.local:11311" >> ~/.bashrc
    grep -q -F "source /opt/ros/${ROS_VERSION}/setup.bash" ~/.bashrc || echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> ~/.bashrc
    source ~/.bashrc
fi

heading "${green}Installing rosinstall tools${reset}"

sudo apt install ${PYTHON_V}-rosinstall ${PYTHON_V}-rosinstall-generator ${PYTHON_V}-wstool ${PYTHON_V}-catkin-tools -y

heading "${green}Installing Ardupilot-ROS workspace${reset}"

if maybe_prompt_user "Add ardupilot-ws to your home folder [N/y]?" ; then
    if [ ! -d $ROS_WS_ROOT ]; then
        mkdir -p $ROS_WS_ROOT/src
        pushd $ROS_WS_ROOT
        catkin init
        pushd src
        git clone https://github.com/ArduPilot/ardupilot_ros.git
        popd
        sudo apt update
        rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
        catkin build
        popd
    else
        heading "${red}ardupilot-ws already exists, skipping...${reset}"
    fi
    
else
    echo "Skipping adding ardupilot_ws to your home folder."
fi


if maybe_prompt_user "Add ardupilot_gazebo to your home folder [N/y]?" ; then
    if [ ! -d $AP_GZ_ROOT ]; then
        sudo apt install libgz-sim7-dev rapidjson-dev
        git clone https://github.com/ArduPilot/ardupilot_gazebo
        pushd $AP_GZ_ROOT
        mkdir build && pushd build
        cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
        make -j4
        popd
        popd
        echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$AP_GZ_ROOT/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
        echo 'export GZ_SIM_RESOURCE_PATH=$AP_GZ_ROOT/models:$AP_GZ_ROOT/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
    else
        heading "${red}ardupilot_gazebo already exists, skipping...${reset}"
    fi
    
else
    echo "Skipping adding ardupilot_gazebo to your home folder."
fi

heading "${green}Adding setup.bash, ROS_MASTER_URI and ROS_HOSTNAME to .bashrc ${reset}"
grep -q -F "source $ROS_WS_ROOT/devel/setup.bash" ~/.bashrc || echo "source $ROS_WS_ROOT/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

heading "${green}Installation complete! Please close this terminal and open a new one for changes to take effect!${reset}"

#!/bin/bash
##############################
# SITL Simple Startup Script
##############################


##############################
#Launch sim_vehicle.sh
#
# #echo 'ARMING_CHECK 0' >> ~/ardupilot/Tools/autotest/copter_params.parm
##############################
echo '######################################################'
echo 'SITL Simple Startup Script'
echo 'Mission Planner/APM Planner Windows port: 14550, Connect using UDP'
echo 'MavProxy will auto start in this terminal window'
echo 'MavProxy Windows port: 14551, Cunnect by running: 'mavproxy.py --master=192.168.1.13:14551''
echo 'Assuming VM IP Address: 192.168.1.13'
echo 'To change the ip address of the VM go to network connection in windows and change ifv4 ip address there'

ARDUPILOT_TOOLS_AUTOTEST_SIM_VEHICLE=~/ardupilot/Tools/autotest/sim_vehicle.sh
FIRMWARE=''
FIRMWARE_FOLDER=''
EXTRAPARAMS='-j 2 --console'
MAP='--map'
WIPE=''
OUT='--out=192.168.1.13:14550 --out=192.168.1.13:14551'

#    read -p "Do you want to disable Pre Arm Checks? (y/n) " RESPARM
#    if [ "$RESPARM" = "y" ]; then
#        echo 'ARMING_CHECK 0' >> ~/ardupilot/Tools/autotest/copter_params.parm
#    fi
#    read -p "Do you want to disable Pre Arm Checks? (y/n) " RESPARM
#    if [ "$RESPARM" = "y" ]; then
#        echo 'ARMING_CHECK 0' >> ~/ardupilot/Tools/autotest/ArduPlane.parm
#    fi
#    read -p "Do you want to disable Pre Arm Checks? (y/n) " RESPARM
#    if [ "$RESPARM" = "y" ]; then
#        echo 'ARMING_CHECK 0' >> ~/ardupilot/Tools/autotest/Rover.parm
#    fi
echo '######################################################'
read -p "Witch software do you want to simulate? Copter=c, Plane=p, Rover=r (c/p/r) " RESPM
if [ "$RESPM" = "c" ]; then
    FIRMWARE_FOLDER='~/ardupilot/ArduCopter'

    FIRMWARE='-v ArduCopter -f X'
fi

if [ "$RESPM" = "p" ]; then
    echo "Note: JSBSim needs to be installed and path added to .bashrc"
    FIRMWARE_FOLDER='cd ~/ardupilot/ArduPlane'
    FIRMWARE='-v ArduPlane'
fi

if [ "$RESPM" = "r" ]; then
    FIRMWARE_FOLDER='cd ~/ardupilot/APMrover2'
    FIRMWARE='-v APMrover2'
fi

echo '######################################################'
read -p "Do you want to reset parameters to default? (y/n) " RESP
    if [ "$RESP" = "y" ]; then
        WIPE='-w'
    else
        WIPE=''
    fi
echo '######################################################'
echo "We are Ready to start: "
echo "$ARDUPILOT_TOOLS_AUTOTEST_SIM_VEHICLE $WIPE $FIRMWARE $EXTRAPARAMS $MAP $OUT"
read -p "Are you Ready to start? (y/n)" RESPS
if [ "$RESPS" = "y" ]; then
    #cd $FIRMWARE_FOLDER
    $ARDUPILOT_TOOLS_AUTOTEST_SIM_VEHICLE $WIPE $FIRMWARE $EXTRAPARAMS $MAP $OUT
fi





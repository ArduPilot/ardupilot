#!/bin/bash
##############################
# SITL Simple Startup Script
##############################


##############################
#Launch sim_vehicle.sh
#
# #echo 'ARMING_CHECK 0' >> ~/ardupilot/Tools/autotest/copter_params.parm
##############################

echo 'SITL Simple Startup Script'
echo 'Mission Planner/APM Planner Windows port: 14550, Connect using UDP'
echo 'MavProxy will auto start in linux terminal'
echo 'MavProxy Windows port: 14551, Cunnect by running: 'mavproxy.py --master=192.168.1.13:14551''
echo 'Assuming VM IP Address: 192.168.1.13'
echo 'To change the ip address of the VM go to network connection in windows and change ifv4 ip address there'

read -p "Witch software do you want to simulate? Copter=c, Plane=p, Rover=r (c/p/r) " RESPM
if [ "$RESPM" = "c" ]; then
    cd ~/ardupilot/ArduCopter

#    read -p "Do you want to disable Pre Arm Checks? (y/n) " RESPARM
#    if [ "$RESPARM" = "y" ]; then
#        echo 'ARMING_CHECK 0' >> ~/ardupilot/Tools/autotest/copter_params.parm
#    fi

    read -p "Do you want to reset parameters to default? (y/n) " RESP
    if [ "$RESP" = "y" ]; then
        ~/ardupilot/Tools/autotest/sim_vehicle.sh -v ArduCopter -f X -j 2 -w --console --out=192.168.1.13:14550 --out=192.168.1.13:14551
    else
        ~/ardupilot/Tools/autotest/sim_vehicle.sh -v ArduCopter -f X -j 2 --console --out=192.168.1.13:14550 --out=192.168.1.13:14551
    fi
fi

if [ "$RESPM" = "p" ]; then
    echo "Plane is not tested"
    echo "Note: JSBSim needs to be installed and path added to .bashrc"
    cd ~/ardupilot/ArduPlane
    
#    read -p "Do you want to disable Pre Arm Checks? (y/n) " RESPARM
#    if [ "$RESPARM" = "y" ]; then
#        echo 'ARMING_CHECK 0' >> ~/ardupilot/Tools/autotest/ArduPlane.parm
#    fi
    
    read -p "Do you want to reset parameters to default? (y/n) " RESP
    if [ "$RESP" = "y" ]; then
        ~/ardupilot/Tools/autotest/sim_vehicle.sh -v ArduPlane -j 2 -w --console --out=192.168.1.13:14550 --out=192.168.1.13:14551
    else
        ~/ardupilot/Tools/autotest/sim_vehicle.sh -v ArduPlane -j 2 --console --out=192.168.1.13:14550 --out=192.168.1.13:14551
    fi
fi

if [ "$RESPM" = "r" ]; then
    cd ~/ardupilot/APMrover2
    read -p "Do you want to reset parameters to default? (y/n) " RESP
    
#    read -p "Do you want to disable Pre Arm Checks? (y/n) " RESPARM
#    if [ "$RESPARM" = "y" ]; then
#        echo 'ARMING_CHECK 0' >> ~/ardupilot/Tools/autotest/Rover.parm
#    fi
    
    if [ "$RESP" = "y" ]; then
        ~/ardupilot/Tools/autotest/sim_vehicle.sh -v APMrover2 -j 2 -w --console --out=192.168.1.13:14550 --out=192.168.1.13:14551
    else
        ~/ardupilot/Tools/autotest/sim_vehicle.sh -v APMrover2 -j 2 --console --out=192.168.1.13:14550 --out=192.168.1.13:14551
    fi
fi





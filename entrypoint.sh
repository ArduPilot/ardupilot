#!/usr/bin/env bash

PARAM_FILE_PATH=${1:-''}

echo export PATH=$PATH:$HOME/.local/bin >> ~/.bashrc
source ~/.bashrc

if [[ ${PARAM_FILE_PATH} == '' ]]
then
    Tools/autotest/sim_vehicle.py -v APMrover2 –model=rover-skid -w -L Ircherpark
else
    Tools/autotest/sim_vehicle.py -v APMrover2 -f gazebo-rover --wipe-eeprom --add-param-file=${PARAM_FILE_PATH} -m --mav10 --map --console -I1 -L Ircherpark
fi

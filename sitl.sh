#!/bin/bash

export PARAM_FILE=/home/chirag/ardupilot/Tools/autotest/default_params/gps_ek3_2source.param
#PARAM_FILE=$1
sim_vehicle.py -v APMrover2 -f gazebo-rover --wipe-eeprom --add-param-file=$PARAM_FILE -m --mav10 --map --console -I1 -L Ircherpark

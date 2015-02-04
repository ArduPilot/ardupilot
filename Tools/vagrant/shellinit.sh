# Init that is run every time a new session starts up

export APMROOT=/vagrant
export PATH=$APMROOT/Tools/autotest:$PATH

cd $APMROOT/ArduCopter

echo "Ardupilot environment ready.  Run 'sim_vehicle.sh' to start simulating an arducopter instance."
echo "or run 'make sitl' to just do a test build."
echo "NOTE: This vagrant build environment isn't currently intended for building PX4 loads, but if someone wants"
echo "to add that ability it wouldn't be too difficult."
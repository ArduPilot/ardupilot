# Init that is run every time a new session starts up

export APMROOT=/vagrant
export PATH=$APMROOT/Tools/autotest:$PATH

cd $APMROOT/ArduCopter

echo "Ardupilot environment ready.  Run 'sim_vehicle.py' to start simulating an arducopter instance."
echo "or run 'make sitl' to just do a test build."

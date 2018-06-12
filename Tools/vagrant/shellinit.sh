# Init that is run every time a new session starts up

# This allows the PX4NuttX build to proceed when the underlying fs is on windows
# It is only marginally less efficient on Linux
export PX4_WINTOOL=y
export PATH=$PATH:$HOME/jsbsim/src
export BUILDLOGS=/tmp/buildlogs

export APMROOT=/vagrant
export PATH=$APMROOT/Tools/autotest:$PATH

cd $APMROOT/ArduCopter

echo "Ardupilot environment ready.  Run 'sim_vehicle.py' to start simulating an arducopter instance."
echo "To build for fmuv2:"
echo "  cd /vagrant"
echo "  ./waf configure --board=px4-v2"
echo "  ./waf build --target=bin/arducopter"

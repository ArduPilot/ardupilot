# Init that is run every time a new session starts up

# This allows the PX4NuttX build to proceed when the underlying fs is on windows
# It is only marginally less efficient on Linux
export PX4_WINTOOL=y
export PATH=$PATH:$HOME/jsbsim/build/src
export BUILDLOGS=/tmp/buildlogs

export APMROOT=/vagrant
new=$HOME/.local/bin
case ":${PATH:=$new}:" in
    *:"$new":*)  ;;
    *) if [ -d "$HOME/.local/bin" ] ; then
          export PATH="$new:$PATH"
       fi ;;
esac

export PATH=$APMROOT/Tools/autotest:$PATH
export PATH=/usr/lib/ccache:$PATH

cd $APMROOT/ArduCopter

echo "Ardupilot environment ready.  Run 'sim_vehicle.py' to start simulating an arducopter instance."
echo "To build for fmuv2:"
echo "  cd /vagrant"
echo "  ./waf configure --board=Pixhawk1"
echo "  ./waf build --target=bin/arducopter"

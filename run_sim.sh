USAGE="Usage: ./run_sim.sh [mode=plane|drone] [interface=en0]"

INTERFACE="en0"
MODE="plane"

if [ $# -gt 0 ]; then
  if [ "$1" = "help" ]; then
    echo "$USAGE"
    exit 0
  fi

  MODE="$1"
  INTERFACE="$2"
fi


INT_LINE_NUM=`ifconfig | grep -n "$INTERFACE: " | cut -d':' -f1 | head -n 1`
INT_ADDRESS=`ifconfig | tail -n +$INT_LINE_NUM | grep "inet " | head -n 1 | cut -d' ' -f2`
COORDS_AGH="50.065,19.9218,0,0"
COORDS_AFYON="38.790284,30.483538,0,0"
VEHICLE="ArduPlane"
FRAME="plane"


if [[ $MODE == "plane" ]]; then
  VEHICLE="ArduPlane"
  FRAME="plane"
else 
  if [[ $MODE == "drone" ]]; then
    VEHICLE="ArduCopter"
    FRAME="quad"
  else
    echo "$USAGE"
    exit 0
  fi
fi

poetry run Tools/autotest/sim_vehicle.py -v $VEHICLE -f $FRAME -l $COORDS_AFYON --console --map --out=$INT_ADDRESS:14550
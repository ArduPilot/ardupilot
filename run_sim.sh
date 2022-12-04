USAGE="Usage: ./run_sim.sh [interface=en0]"

INTERFACE="en0"

if [ $# -gt 0 ]; then
  if [ "$1" = "help" ]; then
    echo "$USAGE"
    exit 0
  fi

  INTERFACE="$1"
fi


INT_LINE_NUM=`ifconfig | grep -n "$INTERFACE: " | cut -d':' -f1 | head -n 1`
INT_ADDRESS=`ifconfig | tail -n +$INT_LINE_NUM | grep "inet " | head -n 1 | cut -d' ' -f2`
COORDS_AGH="50.065,19.9218,0,0"
COORDS_AFYON="38.790284,30.483538,0,0"
poetry run Tools/autotest/sim_vehicle.py -v ArduPlane -f plane -l $COORDS_AFYON --console --map --out=$INT_ADDRESS:14550

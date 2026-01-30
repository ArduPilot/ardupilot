#!/bin/bash

# Parse command-line arguments
INSTANCE=0
while [[ $# -gt 0 ]]; do
    case $1 in
        -I)
            INSTANCE="$2"
            shift 2
            ;;
        *)
            shift
            ;;
    esac
done

# Calculate port and sysid based on instance
OUT_PORT=$((14550 + INSTANCE * 10))
MAVROS_PORT=$((14551 + INSTANCE * 10))
SYSID=$((1 + INSTANCE))

# Check if sim_vehicle.py exists
if [ ! -f "Tools/autotest/sim_vehicle.py" ]; then
    echo "Warning: Tools/autotest/sim_vehicle.py not found!"
    echo "Make sure you are running from the root directory containing this script."
    echo "This is so that you can reuse the eeprom file containing parameters saved from last time."
    exit 1
fi

python3 Tools/autotest/sim_vehicle.py -v ArduSub --model JSON:${AP_JSON_IP:-127.0.0.1} --out udp:127.0.0.1:$OUT_PORT --out udp:127.0.0.1:$MAVROS_PORT -L SGMarinaBarrage --add-param-file=params/sitl_json.parm -I $INSTANCE --sysid $SYSID 
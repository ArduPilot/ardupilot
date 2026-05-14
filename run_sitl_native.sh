OUT_PORT=14555
python3 Tools/autotest/sim_vehicle.py -v ArduSub --out udp:127.0.0.1:$OUT_PORT -L SGMarinaBarrage --mavproxy-args="--streamrate=-1"
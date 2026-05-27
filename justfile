launch_sitl_vanilla:
    sim_vehicle.py -A "--serial4=uart:/dev/ttyUSB0:921600" -v ArduCopter -f quad --add-param-file=./Tools/autotest/default_params/copter.parm \
        --console --map

launch_gazebo:
    gz sim -v4 -r iris_runway.sdf

launch_rf_gazebo:
    gz sim -v4 -r ./AL1000/rf_runway.sdf

launch_sitl_physics:
    sim_vehicle.py -A "--serial4=uart:/dev/ttyUSB0:921600" -v ArduCopter -f gazebo-iris --model JSON --map --console \
            --add-param-file=./AL1000/al1000.parm

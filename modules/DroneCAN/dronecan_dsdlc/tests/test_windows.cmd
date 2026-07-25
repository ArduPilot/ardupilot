# simple test of DSDL compilation for windows

python3 -m pip install -U empy pexpect dronecan

git clone https://github.com/DroneCAN/DSDL

echo "Testing generation"
python3 dronecan_dsdlc.py --output dsdl_generated DSDL/dronecan DSDL/uavcan DSDL/com DSDL/ardupilot 

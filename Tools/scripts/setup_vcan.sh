# Credits to Pavel Kirienko
# http://uavcan.org/Implementations/Pyuavcan/Tutorials/1._Setup/
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_bcm
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
sudo ifconfig vcan0 up
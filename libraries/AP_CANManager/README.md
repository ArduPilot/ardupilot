## Testing And Debugging

**Testing under SITL**

https://github.com/linux-can/can-utils contains a nice set of utility to do CAN related testings on Linux system. I used Ubuntu for this development, for Ubuntu systems you can simply download this tool using `sudo apt-get install can-utils`

Following are the common commands that can be used while testing or developing:
* Create Virtual CAN Interface:
```
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
sudo ip link add dev vcan1 type vcan
sudo ip link set up vcan1
```
* Route one CANSocket to another
```
sudo modprobe can-gw
sudo cangw -A -s vcan0 -d vcan1 -e
sudo cangw -A -s vcan1 -d vcan0 -e
```
* Delete routes
```
sudo cangw -D -s vcan0 -d vcan1 -e
sudo cangw -D -s vcan1 -d vcan0 -e
```
* Route SLCAN to VCAN, this allows connecting CAN devices to SITL run via CAN Adapter like the one running in Ardupilot itself.
```
sudo modprobe slcan
sudo slcan_attach -f -s8 -o /dev/ttyACM0
sudo slcand ttyACM0 slcan0
sudo ifconfig slcan0 up
sudo cangw -A -s vcan0 -d slcan0 -e
sudo cangw -A -s slcan0 -d vcan0 -e
```
* Dump can messages:
```
sudo candump vcan0
```

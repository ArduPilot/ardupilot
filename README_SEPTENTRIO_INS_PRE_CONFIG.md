# Pre-Config of the AsteRx-i or AsteRx-m2 dual antenna setup

If Septentrio INS is selected (GPS_TYPE = 17) a pre-config of the AsteRx-i or AsteRx-m2 dual antenna setup is required. This can be done trough RX-tools or connecting the AsteRx with a com port and sending the commands over serial communication.

The \n indicates a newline and is needed for the AsteRx to interpret a new command.

## AsteRx-i

The following commands has to be set:

Define on which com-port the imu is connected to. For example com3
    setIMUInput, COM3\n

Define the imu orientation with respect to your drone setup. For default setup use the following command
    setIMUOrientation, SensorDefault\n

    If the imu is not placed with de x-axis pointing to the front of the drone and placed horizontally and upside up, use following command
        setIMUOrientation, rotationAngle-x, rotationAngle-y, rotationAngel-z <CR>

        for example:
            90° rotation around the z-axis:
                setIMUOrientation, 0, 0, 90\n
            90° rotation around the x-axis and 270° around the z-axis:
                setIMUOrientation, 90, 0, 270\n

Define the antenna lever arm in the drone reference frame. This is the offset in x,y and z between the IMU reference point to the arp of the main GNNS antenna  
    setINSAntLeverArm, x, y, z\n

Optionally the point of interest can be defined. By default the position is calculated at the IMU reference point. If this is not the point of interest the x,y and z offset can be set to the desire point of interest (for example a camera)
    setINSPOILeverArm, POI1, x, y, z\n

Make sure the INS/GNSS integration filter is enabled (normally it is by default)
    setINSNAvConfig, on\n

## AsteRx-m2 dual antenna setup

If only the AsterRx-m2 dual antenna setup is used the only command to us in pre-configuration is the attitude offset. The offset is defined in degrees
    setAttitudeOffset, antennaOffset\n
    for example the aux1 antenna is on the right of the main antenna (Default):
    setAttitudeOffset, 90\n
    Or to the left of the main antenna:
    setAttitudeOffset, -90\n

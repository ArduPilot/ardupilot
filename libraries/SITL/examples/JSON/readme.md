The JSON SITL backend allows software to easily interface with ArduPilot using a standard JSON interface.

To launch the JSON backend run SITL with ```-f json:127.0.0.1``` where 127.0.0.1 is replaced with the IP the physics backend is running at.

Connection to SITL is made via a UDP link, port 9002 should be used to receive inputs from ArduPilot and port 9003 to return attitude information to ArduPilot.

SITL output
Data is output from SITL in a binary format:
```
    uint32_t frame_count
    float Speedup
    uint16_t pwm[16] 
```
The frame_count will increment for each output frame sent by ArduPilot, this count can be used to detect lost or duplicate frames. This count will be reset when SITL is re-started allowing the physics backend to reset the vehicle. If not input data is received after 10 seconds ArduPilot will re-send the output frame without incrementing the counter. This allows the physics model to be restarted and re-connect. Note that this may fill up the input buffer of the physics backend after some time. Speed up is the desired speed up as set with the SIM_SPEEDUP param, this is a target only. PWM is a array of 16 servo values in micro seconds, typically in the 1000 to 2000 range as set by the servo output functions.

SITL input
Data is received from the physics backend in a plain text JSON format. The data must contain the following fields:
```
    timestamp (us) physics time
    imu:
        gyro(roll, pitch, yaw) (radians/sec) body frame
        accel_body(north, east, down) (m/s^2) body frame

    position(north, east, down) (m) earth frame
    attitude(roll, pitch yaw) (radians)
    velocity(north, east,down) (m/s) earth frame
```
This is a example input frame, it should be preceded by and terminated with a carriage return ("\n") :
```
{"timestamp":2500,"imu":{"gyro":[0,0,0],"accel_body":[0,0,0]},"position":[0,0,0],"attitude":[0,0,0],"velocity":[0,0,0]}
```
The order of fields is not important, this is minimum required fields. In the future a support for a number of additional optional felids will be added to allow readings to be provided for sensors such as airspeed and lidar.

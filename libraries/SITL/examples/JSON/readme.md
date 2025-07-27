# JSON Simulation Format

The JSON SITL backend allows software to easily interface with ArduPilot using a standard JSON interface.

To launch the JSON backend run SITL with ```--model JSON:127.0.0.1``` where 127.0.0.1 is replaced with the IP the physics backend is running at. For example: ```sim_vehicle.py -v ArduCopter -f octaquad --model JSON:127.0.0.1 --map --console```

Connection to SITL is made via a UDP link. The physics backend should listen for incoming messages on port 9002 it should then reply to the IP and port the messages were received from. This removes the need to configure a target
IP and port for SITL in the physics backend. SITL will send a output message every 10 seconds allowing the physics backend to auto detect.

## SITL output

Data is output from SITL in a binary format:
```
    uint16 magic = 18458
    uint16 frame_rate
    uint32 frame_count
    uint16 pwm[16]
```

The magic value is a constant of 18458, this is used to confirm the packet is from ArduPilot and is used for protocol versioning.

The number of output channels may be increased to 32 by setting the parameter
SERVO_32_ENABLE = 1. The SITL output packet is then

```
    uint16 magic = 29569
    uint16 frame_rate
    uint32 frame_count
    uint16 pwm[32]
```

and uses a magic value 29569.  

The frame rate represents the time step the simulation should take, this can be changed with the SIM_RATE_HZ ArduPilot parameter. The physics backend is free to ignore this value, a maximum time step size would typically be set. The SIM_RATE_HZ should value be kept above the vehicle loop rate, by default this 400hz on copter and quadplanes and 50 hz on plane and rover.

The frame_count will increment for each output frame sent by ArduPilot, this count can be used to detect lost or duplicate frames. This count will be reset when SITL is re-started allowing the physics backend to reset the vehicle. If not input data is received after 10 seconds ArduPilot will re-send the output frame without incrementing the counter. This allows the physics model to be restarted and re-connect. Note that this may fill up the input buffer of the physics backend after some time. 

PWM is a array of 16 (or 32) servo values in micro seconds, typically in the 1000 to 2000 range as set by the servo output functions.

## SITL input

Data is received from the physics backend in a plain text JSON format. The data must contain the following fields:
```
    timestamp (s) physics time
    imu:
        gyro(roll, pitch, yaw) (radians/sec) body frame
        accel_body(x, y, z) (m/s^2) body frame
    position(north, east, down) (m) earth frame
    velocity(north, east, down) (m/s) earth frame
```

It is possible to send the attitude in a euler format using ```attitude``` or as a quaternion with ```quaternion```, one of these fields must be received. If both are received the quaternion attitude will be used.

```
    attitude(roll, pitch yaw) (radians)
    quaternion(q1, q2, q3, q4)
```

This is a example input frame, it should be preceded by and terminated with a carriage return ("\n") :
```
{"timestamp":2500,"imu":{"gyro":[0,0,0],"accel_body":[0,0,0]},"position":[0,0,0],"attitude":[0,0,0],"velocity":[0,0,0]}
```
The order of fields is not important.
Note that the timestamp is the absolute physics time, not the timestep.

It is possible to send optional fields to provide data for additional sensors, in most cases this will require setting the relevant sensor type param to the SITL driver.

rangefinder distances corresponding to driver instances:
```
    rng_1 (m)
    rng_2 (m)
    rng_3 (m)
    rng_4 (m)
    rng_5 (m)
    rng_6 (m)
```

## Apparent wind:
```
    windvane:
        direction (radians) clockwise relative to the front, i.e. 0 = head to wind
        speed (m/s)
```
for example:
```
 "windvane":{"direction":0,"speed":0}
```

## Wind Vector

3D wind can be provided in m/s NED frame, for example:
```
"velocity_wind":[3.2,0.0,-0.7]
```

## Airspeed:

```
    airspeed (m/s)
```

## RC Input

The controller can optionally provide R/C input data, up to 12
channels.
```
"rc":{"rc_1":1500,"rc_2":1500,"rc_3":1000,"rc_4":1500,"rc_5":1000,"rc_6":1000,"rc_7":1000,"rc_8":1500,rc_9":1500,"rc_10":1500}
```

## Battery

The controller can provide battery voltage and current. Voltage in
Volts, current in Amps:
```
"battery":{"voltage":50.39,"current":64.01}
```

## Debugging

When first connecting you will see a message reporting what fields were successfully received. If any of the mandatory fields are missing SITL will stop, however it will run without the optional fields. This message can be used to double check SITL is receiving everything being sent by the physics backend.

For example:
```
JSON received:
        timestamp
        gyro
        accel_body
        position
        attitude
        velocity
        rng_1
```

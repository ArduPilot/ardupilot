This is a tool to allow MATLAB to interface with the SITL JSON backend.

The SITL_connector function can be used to simplify the connection process. SITL_connector.m uses the TCP/UDP/IP Toolbox 2.0.6 by Peter Rydesäter and is much (10x) faster than the MATLAB functions available with the instrument control toolbox (and free!). However this may require compilation of a mex file, prebuilt mex files are included in most cases they will work without the need to re-compile. 

see Copter/SIM_multicopter.m for example usage.

The function is defined as:
```
    SITL_connector(state,init_function,physics_function,max_timestep)
```

- state: this is the persistent physics state of the vehicle its is a structure and must contain the following felids:
```
    state.gyro(roll, pitch, yaw) (radians/sec) body frame
    state.attitude(roll, pitch yaw) (radians)
    state.accel(x, y, z) (m/s^2) body frame
    state.velocity(north, east,down) (m/s) earth frame
    state.position(north, east, down) (m) earth frame 
```
the structure can have also any other felids required for the physics model

- init_function: function handle that will be called to init the physics model, this will be called on the first run and after a SITL reboot. It should take and return the state.
    function state init(state)
    init_function = @init;

- physics_function: function handle that will be called to update the physics model by a single time step. It should take in the state and array of 16 PWM inputs and return the state.
```
    function state = physics_step(pwm_in,state)
    physics_function = @physics_step;
```
- max_timestep: this is the maximum allowed time step size, the desired time step can be set with the SIM_RATE_HZ ArduPilot parameter.

The JSON SITL interface is lock-step scheduled. This allows matlab breakpoints to work as normal.

Using the connection it should be possible to achieve > 1500 fps, at this speed MATLAB code efficiency plays a important factor in the max frame rate. For a 400hz physics time step this gives a maximum speedup of 4 to 5 times. For planes and rovers it should be possible to use a much larger physics time step resulting in a larger maximum speed up. Note that large speedups risk the GCS getting left behind.

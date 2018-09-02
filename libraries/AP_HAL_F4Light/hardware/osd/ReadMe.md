Support for built-in MAX7456 for FCs. Uses files from 
MinimOSD-Extra project (https://github.com/night-ghost/minimosd-extra) which used as module
in AP_HAL_F4Light/support folder

Because Ardupolot is written on C++ and consists of objects with a hidden internal structure,
it is impossible to get the state of the flight controller from OSD driver. Therefore, 
the OSD works completely independently in its own process, and communicates with the controller
via emulated UART using the Mavlink protocol - as well as external OSD does. 

 So there can't be a cool config menu like Betaflight has, but can be support for parameters 
 changing, like in MP.

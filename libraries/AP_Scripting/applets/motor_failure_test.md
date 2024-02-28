# Motor Failure testing Lua script

This script allows testing failure of motors on copter and quadplane (VTOL only). Vehicles with eight or more motors should be able to fly easily with a single failed motor. Hexacopters can also cope with motor failure if they have sufficient thrust.

Motor failure is triggered by a RC switch configured to option 300 (Scripting1). Switch low all motors will run, switch high will stop motors.

Configure which motors stop with the param MOT_STOP_BITMASK, this is added by the script so will only show up once the script is loaded on the SD card. The parameters is a bitmask of motors to stop. A value of 1 will stop motor 1, value of 2 stop motor 2, a value of 3 stops both motors 1 and 2.

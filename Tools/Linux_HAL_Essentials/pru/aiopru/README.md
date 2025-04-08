# PRU firmware for 12 x PWM output and RC input

RC AllInOnePRU can be used with BeagleBone Black

* Written in [PRU Assembly](http://processors.wiki.ti.com/index.php/PRU_Assembly_Reference_Guide)
* 1 channel RCInput with 5ns accuracy
* 12 channel RCOutput with 1us accuracy 

## Build and install pasm (PRU Assembler) 
1. `git clone https://github.com/beagleboard/am335x_pru_package.git`
2. `cd am335x_pru_package`
3. `make`
4. `sudo make install`

## Rebuild RCAioPRU.p
1. `cd ardupilot/Tools/Linux_HAL_Essentials/pru/aiopru`
2. `make`


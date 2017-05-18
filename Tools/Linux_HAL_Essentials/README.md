PRU PWM
=======

###Updating kernel
* Check your kernel version using `uname -r`
* If you get 3.8.13bone56+ as output then you may skip Updating kernel.
* For other kernel versions write following cmds in your BBB:
```
wget https://rcn-ee.net/deb/wheezy-armhf/v3.8.13-bone57/install-me.sh
chmod +x install-me.sh
./install-me.sh
```
* After above steps reboot and check your kernel version using `uname -r`,it should be `3.8.13bone57`

----

NOTE: For Ubuntu different scripts shall be used. E.g., for precise:
    - http://rcn-ee.net/deb/precise-armhf/v3.8.13-bone57/install-me.sh

----

###Setting Up PRU Compiler
You should first check if there is the PRU Compiler available in your Linux-Distribution.
* `apt-get update`
* `sudo apt-get install ti-pru-cgt-installer`

If it is not available in your Linux-Distribution you have to download it from the TI website.
* Download [PRU C Compiler v2.0.0B2 installer](http://software-dl.ti.com/codegen/non-esd/downloads/beta.htm)
* Please check the path where you install PRU compiler.
* Setting environment variable
```
echo export PATH=/path/to/pru/compiler/bin:$PATH >> ~/.bashrc
echo export PRU_C_DIR="/path/to/pru/compiler/include;/path/to/pru/compiler/lib" >> ~/.bashrc
source ~/.bashrc
```

*note: semicolons in second command were intended*

###Compiling and loading the code
* just `make`
* copy generated executable `pwmpru1` to `ardupilot/Tools/Linux_HAL_Essentials/`.
* To load firmware use 
  - `./startup.sh load` : only once after reboot it will copy overlays and firmware to `/lib/firmware/` and loads them.
  - `./startup.sh reload` : it only copies firmware to `/lib/firmware/` and reloads pru firmware.
* Enable loading cape during BBB startup (no need to follow above step if you do this step once):
  - `cp BB-PXF-01-00A0.dtbo /lib/firmware`
  - add `cape_enable=capemgr.enable_partno=BB-PXF-01` to `/boot/uboot/uEnv.txt`
  - add `CAPE=BB-PXF-01` to `/etc/default/capemgr`


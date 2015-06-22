# BleeduPilot Project

BleeduPilot is a port of Ardupilot 3.1.5 to MultiWii flight controllers using the MPNG HAL code. Information 
about both projects is available on their respective pages - 
http://www.megapirateng.com/ and http://dev.ardupilot.com/

NOTE - This is a complete re-merge of the MPNG HAL code with ArduCopter 3.1.5. (i.e. I forked Arducopter 3.1.5 and reintegrated 
it with the HAL_MPNG of Megapriate... All features of ArduCopter 3.1.5 are available and the code plays nicely with the latest versions 
of Mission Planner. However, the AtMega 2560 processor is being pushed to it's limits. You can easily push it beyond by enabling too 
much in APMConfig.h. Be Careful!!! This is especially true on the Crius All-in-One v 2.0 which has dataflash logging (CPU hungry) and 
power issues. This is an experiment to see how far an 8-bit flight controller can be pushed without breaking. If that sounds like fun 
you are in the right place. If it sounds scary, you are not. I am willing to sacrifice my flying ROBOTs to find a better algorithm. 

Not everyone is.

###This software is NOT supported. You Have Been Warned.

This software works. But it is NOT supported.  

(Note - The above having been said, I've been flying this firmware for over a year now on Crius All-In-One v2 and MultiWii Pro boards [both Red HK and Witespy v2/3] without any flyaways, reboots, controller hangs or weird behavior. In addition SONAR, Power module, MAV/OSD, GPS, Autotune have ALL been tested and work normally. It has been tested with tricopter, quadcopter and traditional heli configurations (only limited trad heli testing so far). It has NOT been tested with Hex or Octo-copters yet. (CPU load may be an issue here as the board is generating twice as many PWM signals.) 

## Getting the source

You can either download the source using the "ZIP" button at the top
of the github page, or you can make a clone using git:

```
git clone git://github.com/alphacharlie/bleedupilot.git
```

## Building
The Following Instructions apply to building BleeduPilot on Windows using the Arduino IDE. (or also AtmelStudio)
### Step 1: Install MHV AVR Tools
Download the MHV AVR Utils from here http://firmware.diydrones.com/Tools/Arduino/MHV_AVR_Tools_20121007.exe and 
install it to the default location.
### Step 2: 
Download the hacked Arduino IDE from here - http://boffinry.org/BleeduPilot-Arduino-1.0.3-win.zip and extract it to wherever you want the 
program to live.(It includes the modified MPNG jar, the hardware definitions needed for Atmel Studio and GCC 4.7.2)
### Step 3: Configure IDE
Open Arduino IDE (arduino.exe) and select File->Preferences. Now change the 'Sketchbook Location' to wherever you cloned/extracted the 
source code to. Now uncheck 'Check for Updates on Startup'. Now Select Ardupilot->HAL Board and click 'MegaPirateNG'. Now go to 
Tools->Board and select 'Arduino Mega 2560 or Mega ADK'. Now close and re-open the Arduino IDE.
### Step 4: Configure APMConfig.h
Now open the ArduCopter sketch in Arduino IDE and navigate to APMConfig.h. Edit the options there to match your hardware. MPNG options are 
documented on the MegaPirate site, ArduCopter options are documented on the arducopter site. 
### Step 5: Build!!!

## Contributing
The end goal of this project is to improve the scheduler code to be able to more efficiently defer non-essential tasks on 8-bit without loop slow-downs. If you have fixes or code to contribute you may contact me at info@boffinry.org


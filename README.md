# BleeduPilot Project

BleeduPilot is a port of Ardupilot 3.1.5 to MultiWii flight controllers using the MPNG HAL code. Information 
about both projects is available on their respective pages - 
http://www.megapirateng.com/ and http://dev.ardupilot.com/

NOTE - This is a complete re-merge of the MPNG HAL code with ArduCopter 3.1.5. All features of ArduCopter 3.1.5 
are available and the code plays nicely with the latest versions of Mission Planner. However, the AtMega 2560 processor is being 
pushed to it's limits. You can easily push it beyond by enabling too much in APMConfig.h. Be Careful!!! This is especially true on the Crius All-in-One v 2.0 which has 
dataflash logging and power issues.
###You Have Been Warned.

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



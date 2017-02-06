ArduSub [![Build Status](https://travis-ci.org/bluerobotics/ardusub.svg?branch=master)](https://travis-ci.org/bluerobotics/ardusub) [![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/bluerobotics/ardusub?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
=======

An ArduPilot-based project for remotely operated underwater vehicles. **Please see the main ArduSub documentation at [ArduSub.com](http://ardusub.com).**

*****

## Intro ##

The ArduSub project is a subsea vehicle control system designed primarily for ROVs and AUVs. It is based on the *ArduCopter* vehicle-type of the [ArduPilot](https://github.com/diydrones/ardupilot) project. 

The majority of code changes required for the ArduSub are held in the [/ArduSub/](/ArduSub/) directory of this repository. Changes have also been made to a number of libraries:

- **AP_Baro:** Added support for MS58XX pressure sensors for water depth measurement
- **AP_Motors:** Added AP_Motors6DOF, AP_MotorsBlueROV6DOF, AP_MotorsVectoredROV classes 
- **AP_RCMapper:** Added "forward" and "strafe" input axes

## How To Get Involved ##

The ArduSub project is open source and we encourage participation and code contributions: [guidelines for contributors to the ardupilot codebase](http://dev.ardupilot.com/wiki/guidelines-for-contributors-to-the-apm-codebase).

Desired Enhancements and Bugs can be posted to the [issues list](https://github.com/bluerobotics/ardupilot/issues).

## License ##
[Overview of license](http://dev.ardupilot.com/wiki/license-gplv3)

[Full Text](https://github.com/bluerobotics/ardupilot/blob/master/COPYING.txt)

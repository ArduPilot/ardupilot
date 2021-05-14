#!/bin/bash
function format {
	DIR=$1
	find $DIR -regex ".*\.\(h\|cpp\|pde\)" -exec astyle {} \;
	find $DIR -regex ".*\.\(h\|cpp\|pde\)" -exec rm -f {}.orig \;
}

format apo
format ArduRover
format ArduBoat
format libraries/APO
format libraries/AP_Common
format libraries/AP_GPS

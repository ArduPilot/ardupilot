# Single range reading from an active ranger that emits energy and reports
# one range reading that is valid along an arc at the distance measured. 
# This message is not appropriate for fixed-range obstacle detectors, 
# such as the Sharp GP2D15. This message is also not appropriate for laser 
# scanners. See the LaserScan message if you are working with a laser scanner.

Header header    	# timestamp in the header is the time the ranger
		 	# returned the distance reading

# Radiation type enums
# If you want a value added to this list, send an email to the ros-users list
uint8 ULTRASOUND=0
uint8 INFRARED=1

uint8 radiation_type    # the type of radiation used by the sensor
		 	# (sound, IR, etc) [enum]

float32 field_of_view   # the size of the arc that the distance reading is
		 	# valid for [rad]
		 	# the object causing the range reading may have
		 	# been anywhere within -field_of_view/2 and
		 	# field_of_view/2 at the measured range. 
                        # 0 angle corresponds to the x-axis of the sensor.

float32 min_range       # minimum range value [m]
float32 max_range       # maximum range value [m]

float32 range           # range data [m]
		 	# (Note: values < range_min or > range_max
		 	# should be discarded)

include ../libraries/AP_Common/Arduino.mk

nologging:
	make -f Makefile EXTRAFLAGS="-DLOGGING_ENABLED=DISABLED"

nogps:
	make -f Makefile EXTRAFLAGS="-DGPS_PROTOCOL=GPS_PROTOCOL_NONE -DLOGGING_ENABLED=DISABLED"

hil:
	make -f Makefile EXTRAFLAGS="-DHIL_MODE=HIL_MODE_ATTITUDE -DCLI_SLIDER_ENABLED=DISABLED -DLOGGING_ENABLED=DISABLED"

hilsensors:
	make -f Makefile EXTRAFLAGS="-DHIL_MODE=HIL_MODE_SENSORS -DCLI_SLIDER_ENABLED=DISABLED -DLOGGING_ENABLED=DISABLED"

hilnocli:
	make -f Makefile EXTRAFLAGS="-DHIL_MODE=HIL_MODE_ATTITUDE -DCLI_ENABLED=DISABLED -DLOGGING_ENABLED=DISABLED"

nocli:
	make -f Makefile EXTRAFLAGS="-DCLI_ENABLED=DISABLED -DLOGGING_ENABLED=DISABLED"

ublox:
	make -f Makefile EXTRAFLAGS="-DGPS_PROTOCOL=GPS_PROTOCOL_UBLOX"

heli:
	make -f Makefile EXTRAFLAGS="-DFRAME_CONFIG=HELI_FRAME"

apm2:
	make -f Makefile EXTRAFLAGS="-DCONFIG_APM_HARDWARE=APM_HARDWARE_APM2"

apm2-uart2:
	make -f Makefile EXTRAFLAGS="-DCONFIG_APM_HARDWARE=APM_HARDWARE_APM2 -DTELEMETRY_UART2=ENABLED"

apm2beta:
	make -f Makefile EXTRAFLAGS="-DCONFIG_APM_HARDWARE=APM_HARDWARE_APM2 -DAPM2_BETA_HARDWARE"


apm2-mavlink10:
	make -f Makefile EXTRAFLAGS="-DCONFIG_APM_HARDWARE=APM_HARDWARE_APM2 -DMAVLINK10=1"

mavlink10:
	make -f Makefile EXTRAFLAGS="-DMAVLINK10=1"

sitl:
	make -f ../libraries/Desktop/Makefile.desktop

sitl-mavlink10:
	make -f ../libraries/Desktop/Makefile.desktop EXTRAFLAGS="-DMAVLINK10=1"

sitl-mount:
	make -f ../libraries/Desktop/Makefile.desktop EXTRAFLAGS="-DMOUNT=ENABLED"

sitl-quaternion:
	make -f ../libraries/Desktop/Makefile.desktop EXTRAFLAGS="-DQUATERNION_ENABLE=ENABLED"

etags:
	cd .. && etags -f ArduPlane/TAGS --langmap=C++:.pde.cpp.h $$(git ls-files ArduPlane libraries)


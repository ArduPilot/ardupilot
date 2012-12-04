include ../libraries/AP_Common/Arduino.mk

nologging:
	make -f Makefile EXTRAFLAGS="-DLOGGING_ENABLED=DISABLED"

nogps:
	make -f Makefile EXTRAFLAGS="-DGPS_PROTOCOL=GPS_PROTOCOL_NONE -DLOGGING_ENABLED=DISABLED"

hil:
	make -f Makefile EXTRAFLAGS="-DHIL_MODE=HIL_MODE_ATTITUDE"

hil-apm2:
	make -f Makefile EXTRAFLAGS="-DHIL_MODE=HIL_MODE_ATTITUDE -DCONFIG_APM_HARDWARE=APM_HARDWARE_APM2"

hilsensors:
	make -f Makefile EXTRAFLAGS="-DHIL_MODE=HIL_MODE_SENSORS"

hilsensors-apm2:
	make -f Makefile EXTRAFLAGS="-DHIL_MODE=HIL_MODE_SENSORS -DCONFIG_APM_HARDWARE=APM_HARDWARE_APM2"

hilnocli:
	make -f Makefile EXTRAFLAGS="-DHIL_MODE=HIL_MODE_ATTITUDE -DCLI_ENABLED=DISABLED"

nocli:
	make -f Makefile EXTRAFLAGS="-DCLI_ENABLED=DISABLED -DLOGGING_ENABLED=DISABLED"

ublox:
	make -f Makefile EXTRAFLAGS="-DGPS_PROTOCOL=GPS_PROTOCOL_UBLOX"

mtk:
	make -f Makefile EXTRAFLAGS="-DGPS_PROTOCOL=GPS_PROTOCOL_MTK"

mtk16:
	make -f Makefile EXTRAFLAGS="-DGPS_PROTOCOL=GPS_PROTOCOL_MTK16"

heli:
	make -f Makefile EXTRAFLAGS="-DFRAME_CONFIG=HELI_FRAME"

apm2:
	make -f Makefile EXTRAFLAGS="-DCONFIG_APM_HARDWARE=APM_HARDWARE_APM2"

apm2-uart2:
	make -f Makefile EXTRAFLAGS="-DCONFIG_APM_HARDWARE=APM_HARDWARE_APM2 -DTELEMETRY_UART2=ENABLED"

apm2beta:
	make -f Makefile EXTRAFLAGS="-DCONFIG_APM_HARDWARE=APM_HARDWARE_APM2 -DAPM2_BETA_HARDWARE"


sitl:
	make -f ../libraries/Desktop/Makefile.desktop

sitl-mount:
	make -f ../libraries/Desktop/Makefile.desktop EXTRAFLAGS="-DMOUNT=ENABLED"

etags:
	cd .. && etags -f ArduPlane/TAGS --langmap=C++:.pde.cpp.h $$(git ls-files ArduPlane libraries)

obc:
	make -f Makefile EXTRAFLAGS="-DCONFIG_APM_HARDWARE=APM_HARDWARE_APM2 -DOBC_FAILSAFE=ENABLED -DTELEMETRY_UART2=ENABLED -DSERIAL_BUFSIZE=512"

obc-sitl:
	make -f ../libraries/Desktop/Makefile.desktop EXTRAFLAGS="-DOBC_FAILSAFE=ENABLED -DSERIAL_BUFSIZE=512"

sitl-newcontrollers:
	make -f ../libraries/Desktop/Makefile.desktop EXTRAFLAGS="-DAPM_CONTROL=ENABLED"


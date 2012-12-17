include ../libraries/AP_Common/Arduino.mk

nogps: EXTRAFLAGS += "-DGPS_PROTOCOL=GPS_PROTOCOL_NONE "
nogps: all

hil-apm2: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
hil-apm2: apm2

hil-apm1: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
hil-apm1: apm1

hil-apm2-nologging: EXTRAFLAGS += "DLOGGING_ENABLED=DISABLED "
hil-apm2-nologging: hil-apm2

hilsensors: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "
hilsensors: all

hilsensors-apm2: HAL_BOARD = "HAL_BOARD_APM2"
hilsensors-apm2: hilsensors

hilsensors-apm1: HAL_BOARD = "HAL_BOARD_APM1"
hilsensors-apm1: hilsensors

hilsensors-apm2-nologging: EXTRAFLAGS += "DLOGGING_ENABLED=DISABLED "
hilsensors-apm2-nologging: hilsensors-apm2

hilnocli: EXTRAFLAGS += "-DCLI_ENABLED=DISABLED "
hilnocli: hil

nocli: EXTRAFLAGS +="-DCLI_ENABLED=DISABLED "
nocli: EXTRAFLAGS += "DLOGGING_ENABLED=DISABLED "
nocli: all

ublox: EXTRAFLAGS += "-DGPS_PROTOCOL=GPS_PROTOCOL_UBLOX "
ublox: all

mtk: EXTRAFLAGS += "-DGPS_PROTOCOL=GPS_PROTOCOL_MTK "
mtk: all

mtk16: EXTRAFLAGS += "-DGPS_PROTOCOL=GPS_PROTOCOL_MTK16 "
mtk16: all

heli: EXTRAFLAGS += "-DFRAME_CONFIG=HELI_FRAME "
heli: all

apm2-nologging: EXTRAFLAGS += "DLOGGING_ENABLED=DISABLED "
apm2-nologging: apm2

apm2-uart2: EXTRAFLAGS += "-DTELEMETRY_UART2=ENABLED "
apm2-uart2: apm2

apm2beta: EXTRAFLAGS += "-DAPM2_BETA_HARDWARE "
apm2beta: apm2

sitl-mount: EXTRAFLAGS += "-DMOUNT=ENABLED"
sitl-mount: sitl


obc: EXTRAFLAGS += "-DOBC_FAILSAFE=ENABLED "
obc: EXTRAFLAGS += "-DTELEMETRY_UART2=ENABLED "
obc: EXTRAFLAGS += "-DSERIAL_BUFSIZE=512 "
obc: apm2

obc-sitl: EXTRAFLAGS += "-DOBC_FAILSAFE=ENABLED "
obc-sitl: EXTRAFLAGS += "-DSERIAL_BUFSIZE=512 "
obc-sitl: sitl

sitl-newcontrollers: EXTRAFLAGS += "-DAPM_CONTROL=ENABLED "
sitl-newcontrollers: sitl

etags:
	cd .. && etags -f ArduPlane/TAGS --langmap=C++:.pde.cpp.h $$(git ls-files ArduPlane libraries)

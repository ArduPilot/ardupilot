include ../libraries/AP_Common/Arduino.mk

nologging: EXTRAFLAGS += "-DLOGGING_ENABLED=DISABLED"
nologging: all

nogps: EXTRAFLAGS += "-DGPS_PROTOCOL=GPS_PROTOCOL_NONE "
nogps: nologging

clidisabled-nologging: EXTRAFLAGS += "-DCLI_SLIDER_ENABLED=DISABLED "
clidisabled-nologging: nologging

hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
hil: nologging

apm1-hil: HAL_BOARD = "HAL_BOARD_APM1"
apm1-hil: hil

apm2-hil: HAL_BOARD="HAL_BOARD_APM2"
apm2-hil: hil

hilsensors: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "
hilsensors: clidisabled-nologging

apm1-hilsensors: HAL_BOARD = "HAL_BOARD_APM1"
apm1-hilsensors: hilsensors

apm2-hilsensors: HAL_BOARD = "HAL_BOARD_APM2"
apm2-hilsensors: hilsensors

heli: EXTRAFLAGS += "-DFRAME_CONFIG=HELI_FRAME "
heli: all

apm1: HAL_BOARD = "HAL_BOARD_APM1"
apm1: all

apm2: HAL_BOARD="HAL_BOARD_APM2"
apm2: all

dmp: EXTRAFLAGS += "-DDMP_ENABLED=ENABLED"
dmp: apm2

apm2hexa: EXTRAFLAGS += "-DFRAME_CONFIG=HEXA_FRAME "
apm2hexa: apm2

apm2beta: EXTRAFLAGS += "-DAPM2_BETA_HARDWARE "
apm2beta: apm2

sitl:
	make -f ../libraries/Desktop/Makefile.desktop

sitl-octa:
	make -f ../libraries/Desktop/Makefile.desktop octa

sitl-hexa:
	make -f ../libraries/Desktop/Makefile.desktop hexa

sitl-y6:
	make -f ../libraries/Desktop/Makefile.desktop y6

etags:
	cd .. && etags -f ArduCopter/TAGS --langmap=C++:.pde.cpp.h $$(git ls-files ArduCopter libraries)


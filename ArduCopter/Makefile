include ../libraries/AP_Common/Arduino.mk

nologging: EXTRAFLAGS += "-DLOGGING_ENABLED=DISABLED "
nologging: all

nogps: EXTRAFLAGS += "-DGPS_PROTOCOL=GPS_PROTOCOL_NONE "
nogps: nologging

clidisabled-nologging: EXTRAFLAGS += "-DCLI_SLIDER_ENABLED=DISABLED "
clidisabled-nologging: nologging

hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
hil: EXTRAFLAGS += "-DLOGGING_ENABLED=DISABLED "

apm1-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
apm1-hil: EXTRAFLAGS += "-DLOGGING_ENABLED=DISABLED "
apm1-hil: apm1

apm2-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
apm2-hil: EXTRAFLAGS += "-DLOGGING_ENABLED=DISABLED "
apm2-hil: apm2

apm1-hilsensors: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "
apm1-hilsensors: EXTRAFLAGS += "-DCLI_SLIDER_ENABLED=DISABLED "
apm1-hilsensors: EXTRAFLAGS += "-DLOGGING_ENABLED=DISABLED "
apm1-hilsensors: apm1

apm2-hilsensors: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "
apm2-hilsensors: EXTRAFLAGS += "-DCLI_SLIDER_ENABLED=DISABLED "
apm2-hilsensors: EXTRAFLAGS += "-DLOGGING_ENABLED=DISABLED "
apm2-hilsensors: apm2

heli: EXTRAFLAGS += "-DFRAME_CONFIG=HELI_FRAME "
heli: all

dmp: EXTRAFLAGS += "-DDMP_ENABLED=ENABLED"
dmp: apm2

apm2hexa: EXTRAFLAGS += "-DFRAME_CONFIG=HEXA_FRAME "
apm2hexa: apm2

apm2beta: EXTRAFLAGS += "-DAPM2_BETA_HARDWARE "
apm2beta: apm2

sitl: HAL_BOARD = "HAL_BOARD_AVR_SITL"
sitl: all

sitl-octa: EXTRAFLAGS += "-DFRAME_CONFIG=OCTA_FRAME "
sitl-octa: sitl

sitl-hexa: EXTRAFLAGS += "-DFRAME_CONFIG=HEXA_FRAME "
sitl-hexa: sitl

sitl-y6: EXTRAFLAGS += "-DFRAME_CONFIG=OCTA_FRAME "
sitl-y6: sitl

etags:
	cd .. && etags -f ArduCopter/TAGS --langmap=C++:.pde.cpp.h $$(git ls-files ArduCopter libraries)


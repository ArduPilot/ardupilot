default: all

# convenient targets for our supported boards
sitl: HAL_BOARD = HAL_BOARD_AVR_SITL
sitl: TOOLCHAIN = NATIVE
sitl: all

apm1: HAL_BOARD = HAL_BOARD_APM1
apm1: TOOLCHAIN = AVR
apm1: all

apm2: HAL_BOARD = HAL_BOARD_APM2
apm2: TOOLCHAIN = AVR
apm2: all

empty: HAL_BOARD = HAL_BOARD_EMPTY
empty: TOOLCHAIN = AVR
empty: all


nologging: EXTRAFLAGS += "-DLOGGING_ENABLED=DISABLED "
nologging: all

nogps: EXTRAFLAGS += "-DGPS_PROTOCOL=GPS_PROTOCOL_NONE "
nogps: nologging

clidisabled-nologging: EXTRAFLAGS += "-DCLI_ENABLED=DISABLED "
clidisabled-nologging: nologging

hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
hil: apm1

hilsensors: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "

apm1-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
apm1-hil: apm1

apm2-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
apm2-hil: apm2

apm1-hilsensors: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "
apm1-hilsensors: apm1

apm2-hilsensors: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "
apm2-hilsensors: apm2

apm2-nologging: EXTRAFLAGS += "-DLOGGING_ENABLED=DISABLED "
apm2-nologging: apm2

heli: EXTRAFLAGS += "-DFRAME_CONFIG=HELI_FRAME "
heli: all

dmp: EXTRAFLAGS += "-DDMP_ENABLED=ENABLED"
dmp: apm2

apm2hexa: EXTRAFLAGS += "-DFRAME_CONFIG=HEXA_FRAME "
apm2hexa: apm2

apm2beta: EXTRAFLAGS += "-DAPM2_BETA_HARDWARE "
apm2beta: apm2

sitl-octa: EXTRAFLAGS += "-DFRAME_CONFIG=OCTA_FRAME "
sitl-octa: sitl

sitl-hexa: EXTRAFLAGS += "-DFRAME_CONFIG=HEXA_FRAME "
sitl-hexa: sitl

sitl-y6: EXTRAFLAGS += "-DFRAME_CONFIG=OCTA_FRAME "
sitl-y6: sitl

obc-sitl: EXTRAFLAGS += "-DOBC_FAILSAFE=ENABLED "
obc-sitl: EXTRAFLAGS += "-DSERIAL_BUFSIZE=512 "
obc-sitl: sitl

obc: EXTRAFLAGS += "-DOBC_FAILSAFE=ENABLED "
obc: EXTRAFLAGS += "-DTELEMETRY_UART2=ENABLED "
obc: EXTRAFLAGS += "-DSERIAL_BUFSIZE=512 "
obc: apm2

sitl-mount: EXTRAFLAGS += "-DMOUNT=ENABLED"
sitl-mount: sitl

.PHONY: etags
etags:
	cd .. && etags -f ArduCopter/TAGS --lang=c++ $$(git ls-files ArduCopter libraries)
	cd .. && etags -f ArduPlane/TAGS --lang=c++ $$(git ls-files ArduPlane libraries)
	cd .. && etags -f APMrover2/TAGS --lang=c++ $$(git ls-files APMrover2 libraries)

clean:
ifneq ($(findstring CYGWIN, $(SYSTYPE)),)
	@del /S $(BUILDROOT)
else
	@rm -fr $(BUILDROOT)
endif


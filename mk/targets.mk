default: all

# convenient targets for our supported boards
sitl: HAL_BOARD = HAL_BOARD_AVR_SITL
sitl: TOOLCHAIN = NATIVE
sitl: all

apm1: HAL_BOARD = HAL_BOARD_APM1
apm1: TOOLCHAIN = AVR
apm1: all

apm1-1280: HAL_BOARD = HAL_BOARD_APM1
apm1-1280: TOOLCHAIN = AVR
apm1-1280: all

apm2: HAL_BOARD = HAL_BOARD_APM2
apm2: TOOLCHAIN = AVR
apm2: all

flymaple: HAL_BOARD = HAL_BOARD_FLYMAPLE
flymaple: TOOLCHAIN = ARM
flymaple: all

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


apm1-quad: EXTRAFLAGS += "-DFRAME_CONFIG=QUAD_FRAME "
apm1-quad: apm1

apm1-quad-hil: EXTRAFLAGS += "-DFRAME_CONFIG=QUAD_FRAME "
apm1-quad-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
apm1-quad-hil: apm1

apm1-tri: EXTRAFLAGS += "-DFRAME_CONFIG=TRI_FRAME "
apm1-tri: apm1

apm1-hexa: EXTRAFLAGS += "-DFRAME_CONFIG=HEXA_FRAME "
apm1-hexa: apm1

apm1-y6: EXTRAFLAGS += "-DFRAME_CONFIG=Y6_FRAME "
apm1-y6: apm1

apm1-octa: EXTRAFLAGS += "-DFRAME_CONFIG=OCTA_FRAME "
apm1-octa: apm1

apm1-octa-quad: EXTRAFLAGS += "-DFRAME_CONFIG=OCTA_QUAD_FRAME "
apm1-octa-quad: apm1

apm1-heli: EXTRAFLAGS += "-DFRAME_CONFIG=HELI_FRAME "
apm1-heli: apm1

apm1-heli-hil: EXTRAFLAGS += "-DFRAME_CONFIG=HELI_FRAME "
apm1-heli-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
apm1-heli-hil: apm1


apm2-quad: EXTRAFLAGS += "-DFRAME_CONFIG=QUAD_FRAME "
apm2-quad: apm2

apm2-quad-hil: EXTRAFLAGS += "-DFRAME_CONFIG=QUAD_FRAME "
apm2-quad-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
apm2-quad-hil: apm2

apm2-tri: EXTRAFLAGS += "-DFRAME_CONFIG=TRI_FRAME "
apm2-tri: apm2

apm2-hexa: EXTRAFLAGS += "-DFRAME_CONFIG=HEXA_FRAME "
apm2-hexa: apm2

apm2-y6: EXTRAFLAGS += "-DFRAME_CONFIG=Y6_FRAME "
apm2-y6: apm2

apm2-octa: EXTRAFLAGS += "-DFRAME_CONFIG=OCTA_FRAME "
apm2-octa: apm2

apm2-octa-quad: EXTRAFLAGS += "-DFRAME_CONFIG=OCTA_QUAD_FRAME "
apm2-octa-quad: apm2

apm2-heli: EXTRAFLAGS += "-DFRAME_CONFIG=HELI_FRAME "
apm2-heli: apm2

apm2-heli-hil: EXTRAFLAGS += "-DFRAME_CONFIG=HELI_FRAME "
apm2-heli-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
apm2-heli-hil: apm2


px4-quad: EXTRAFLAGS += "-DFRAME_CONFIG=QUAD_FRAME "
px4-quad: px4

px4-quad-hil: EXTRAFLAGS += "-DFRAME_CONFIG=QUAD_FRAME -DHIL_MODE=HIL_MODE_ATTITUDE "
px4-quad-hil: px4

px4-tri: EXTRAFLAGS += "-DFRAME_CONFIG=TRI_FRAME "
px4-tri: px4

px4-hexa: EXTRAFLAGS += "-DFRAME_CONFIG=HEXA_FRAME "
px4-hexa: px4

px4-y6: EXTRAFLAGS += "-DFRAME_CONFIG=Y6_FRAME "
px4-y6: px4

px4-octa: EXTRAFLAGS += "-DFRAME_CONFIG=OCTA_FRAME "
px4-octa: px4

px4-octa-quad: EXTRAFLAGS += "-DFRAME_CONFIG=OCTA_QUAD_FRAME "
px4-octa-quad: px4

px4-heli: EXTRAFLAGS += "-DFRAME_CONFIG=HELI_FRAME "
px4-heli: px4

px4-heli-hil: EXTRAFLAGS += "-DFRAME_CONFIG=HELI_FRAME -DHIL_MODE=HIL_MODE_ATTITUDE "
px4-heli-hil: px4

px4-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
px4-hil: px4

px4-hilsensors: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "
px4-hilsensors: px4

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
	@rm -fr $(BUILDROOT)


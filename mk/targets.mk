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

linux: HAL_BOARD = HAL_BOARD_LINUX
linux: TOOLCHAIN = NATIVE
linux: all

empty: HAL_BOARD = HAL_BOARD_EMPTY
empty: TOOLCHAIN = AVR
empty: all

# cope with HIL targets
%-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
%-hilsensors: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "

# cope with copter and hil targets
FRAMES = quad tri hexa y6 octa octa-quad heli single
BOARDS = apm1 apm2 apm2beta apm1-1280 px4 px4-v1 px4-v2 sitl flymaple linux

define frame_template
$(1)-$(2) : EXTRAFLAGS += "-DFRAME_CONFIG=$(shell echo $(2) | tr a-z A-Z | sed s/-/_/g)_FRAME "
$(1)-$(2) : $(1)
$(1)-$(2)-hil : $(1)-$(2)
$(1)-$(2)-hilsensors : $(1)-$(2)
$(1)-hil : $(1)
$(1)-hilsensors : $(1)
endef

# generate targets of the form BOARD-FRAME and BOARD-FRAME-HIL
$(foreach board,$(BOARDS),$(foreach frame,$(FRAMES),$(eval $(call frame_template,$(board),$(frame)))))

apm2beta: EXTRAFLAGS += "-DAPM2_BETA_HARDWARE "
apm2beta: apm2

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


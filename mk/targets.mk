default: help

# convenient targets for our supported boards
sitl: HAL_BOARD = HAL_BOARD_SITL
sitl: TOOLCHAIN = NATIVE
sitl: all

sitl-arm: HAL_BOARD = HAL_BOARD_SITL
sitl-arm: TOOLCHAIN = RPI
sitl-arm: all

apm1: HAL_BOARD = HAL_BOARD_APM1
apm1: TOOLCHAIN = AVR
apm1: all

apm1-1280: HAL_BOARD = HAL_BOARD_APM1
apm1-1280: TOOLCHAIN = AVR
apm1-1280: all
apm1-1280: BOARD = mega

apm2: HAL_BOARD = HAL_BOARD_APM2
apm2: TOOLCHAIN = AVR
apm2: all

flymaple: HAL_BOARD = HAL_BOARD_FLYMAPLE
flymaple: TOOLCHAIN = ARM
flymaple: all
flymaple-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_ATTITUDE "
flymaple-hil: flymaple

linux: HAL_BOARD = HAL_BOARD_LINUX
linux: TOOLCHAIN = NATIVE
linux: all

erle: HAL_BOARD = HAL_BOARD_LINUX
erle: TOOLCHAIN = BBONE
erle: all

zynq: HAL_BOARD = HAL_BOARD_LINUX
zynq: TOOLCHAIN = ZYNQ
zynq: all
zynq-hil: EXTRAFLAGS += "-DHILMODE=HIL_MODE_ATTITUDE -DHIL_MODE=HIL_MODE_SENSORS "
zynq-hil : zynq

pxf: HAL_BOARD = HAL_BOARD_LINUX
pxf: TOOLCHAIN = BBONE
pxf: all

bebop: HAL_BOARD = HAL_BOARD_LINUX
bebop: TOOLCHAIN = BBONE
bebop: all

navio: HAL_BOARD = HAL_BOARD_LINUX
navio: TOOLCHAIN = RPI
navio: all

bbbmini: HAL_BOARD = HAL_BOARD_LINUX
bbbmini: TOOLCHAIN = BBONE
bbbmini: all

empty: HAL_BOARD = HAL_BOARD_EMPTY
empty: TOOLCHAIN = AVR
empty: all

# cope with HIL targets
%-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "
%-hilsensors: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "

# cope with OBC targets
%-obc: EXTRAFLAGS += "-DOBC_FAILSAFE=ENABLED "

# support debug build
%-debug: OPTFLAGS = -g -O0

# cope with -nologging
%-nologging: EXTRAFLAGS += "-DLOGGING_ENABLED=DISABLED "

# cope with copter and hil targets
FRAMES = quad tri hexa y6 octa octa-quad heli single coax obc nologging
BOARDS = apm1 apm2 apm2beta apm1-1280 px4 px4-v1 px4-v2 sitl flymaple linux vrbrain vrbrain-v40 vrbrain-v45 vrbrainv-50 vrbrain-v51 vrbrain-v52 vrubrain-v51 vrubrain-v52 vrhero-v10 erle pxf navio bbbmini

define frame_template
$(1)-$(2) : EXTRAFLAGS += "-DFRAME_CONFIG=$(shell echo $(2) | tr a-z A-Z | sed s/-/_/g)_FRAME "
$(1)-$(2) : $(1)
$(1)-$(2)-hil : $(1)-$(2)
$(1)-$(2)-debug : $(1)-$(2)
$(1)-$(2)-hilsensors : $(1)-$(2)
$(1)-$(2)-upload : $(1)-$(2)
$(1)-$(2)-upload : $(1)-upload
endef

define board_template
$(1)-hil : $(1)
$(1)-debug : $(1)
$(1)-hilsensors : $(1)
endef

USED_BOARDS := $(foreach board,$(BOARDS), $(findstring $(board), $(MAKECMDGOALS)))
USED_FRAMES := $(foreach frame,$(FRAMES), $(findstring $(frame), $(MAKECMDGOALS)))
#$(warning $(USED_BOARDS))
#$(warning $(USED_FRAMES))
# generate targets of the form BOARD-FRAME and BOARD-FRAME-HIL
$(foreach board,$(USED_BOARDS),$(eval $(call board_template,$(board))))
$(foreach board,$(USED_BOARDS),$(foreach frame,$(USED_FRAMES),$(eval $(call frame_template,$(board),$(frame)))))

apm2beta: EXTRAFLAGS += "-DAPM2_BETA_HARDWARE "
apm2beta: apm2

sitl-mount: EXTRAFLAGS += "-DMOUNT=ENABLED"
sitl-mount: sitl

.PHONY: etags
etags:
	cd .. && etags -f ArduCopter/TAGS --lang=c++ $$(git ls-files ArduCopter libraries)
	cd .. && etags -f ArduPlane/TAGS --lang=c++ $$(git ls-files ArduPlane libraries)
	cd .. && etags -f APMrover2/TAGS --lang=c++ $$(git ls-files APMrover2 libraries)

clean:
	@rm -fr $(BUILDROOT)

include $(MK_DIR)/modules.mk

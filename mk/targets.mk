default: help

# convenient targets for our supported boards
sitl: HAL_BOARD = HAL_BOARD_SITL
sitl: TOOLCHAIN = NATIVE
sitl: all

sitl-arm: HAL_BOARD = HAL_BOARD_SITL
sitl-arm: TOOLCHAIN = RPI
sitl-arm: all

apm1 apm1-1280 apm2 apm2beta:
	$(error $@ is deprecated on master branch; use master-AVR)

flymaple flymaple-hil:
	$(error $@ is deprecated on master branch; use master-AVR)

linux: HAL_BOARD = HAL_BOARD_LINUX
linux: TOOLCHAIN = NATIVE
linux: BUILDSYS_DEPRECATED = 1
linux: all

erleboard: HAL_BOARD = HAL_BOARD_LINUX
erleboard: TOOLCHAIN = BBONE
erleboard: BUILDSYS_DEPRECATED = 1
erleboard: all

zynq: HAL_BOARD = HAL_BOARD_LINUX
zynq: TOOLCHAIN = ZYNQ
zynq: all
zynq-hil: EXTRAFLAGS += "-DHILMODE=HIL_MODE_ATTITUDE -DHIL_MODE=HIL_MODE_SENSORS "
zynq-hil : zynq

pxf: HAL_BOARD = HAL_BOARD_LINUX
pxf: TOOLCHAIN = BBONE
pxf: BUILDSYS_DEPRECATED = 1
pxf: all

bebop: HAL_BOARD = HAL_BOARD_LINUX
bebop: TOOLCHAIN = BBONE
bebop: LDFLAGS += "-static"
bebop: BUILDSYS_DEPRECATED = 1
bebop: all

minlure: HAL_BOARD = HAL_BOARD_LINUX
minlure: TOOLCHAIN = NATIVE
minlure: BUILDSYS_DEPRECATED = 1
minlure: all

navio: HAL_BOARD = HAL_BOARD_LINUX
navio: TOOLCHAIN = RPI
navio: BUILDSYS_DEPRECATED = 1
navio: all

navio2: HAL_BOARD = HAL_BOARD_LINUX
navio2: TOOLCHAIN = RPI
navio2: BUILDSYS_DEPRECATED = 1
navio2: all

raspilot: HAL_BOARD = HAL_BOARD_LINUX
raspilot: TOOLCHAIN = RPI
raspilot: BUILDSYS_DEPRECATED = 1
raspilot: all

erlebrain2: HAL_BOARD = HAL_BOARD_LINUX
erlebrain2: TOOLCHAIN = RPI
erlebrain2: BUILDSYS_DEPRECATED  = 1
erlebrain2: all

bbbmini: HAL_BOARD = HAL_BOARD_LINUX
bbbmini: TOOLCHAIN = BBONE
bbbmini: BUILDSYS_DEPRECATED  = 1
bbbmini: all

bhat: HAL_BOARD = HAL_BOARD_LINUX
bhat: TOOLCHAIN = RPI
bhat: BUILDSYS_DEPRECATED  = 1
bhat: all

pxfmini: HAL_BOARD = HAL_BOARD_LINUX
pxfmini: TOOLCHAIN = RPI
pxfmini: BUILDSYS_DEPRECATED  = 1
pxfmini: all

qflight: HAL_BOARD = HAL_BOARD_LINUX
qflight: TOOLCHAIN = QFLIGHT
qflight: all

empty: HAL_BOARD = HAL_BOARD_EMPTY
empty: TOOLCHAIN = AVR
empty: all

qurt: HAL_BOARD = HAL_BOARD_QURT
qurt: TOOLCHAIN = QURT
qurt: all

# cope with HIL targets
%-hil: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "
%-hilsensors: EXTRAFLAGS += "-DHIL_MODE=HIL_MODE_SENSORS "

# cope with OBC targets
%-obc: EXTRAFLAGS += "-DOBC_FAILSAFE=ENABLED "

# support debug build
%-debug: OPTFLAGS = -g -O0

# support address sanitiser
%-asan: OPTFLAGS = -g -O0 -fsanitize=address -fno-omit-frame-pointer
%-asan: LDFLAGS += -fsanitize=address

# cope with -nologging
%-nologging: EXTRAFLAGS += "-DLOGGING_ENABLED=DISABLED "

# cope with copter and hil targets
FRAMES = quad tri hexa y6 octa octa-quad heli single coax obc nologging
BOARDS = apm1 apm2 apm2beta apm1-1280 px4 px4-v1 px4-v2 px4-v4 sitl flymaple linux vrbrain vrbrain-v40 vrbrain-v45 vrbrainv-50 vrbrain-v51 vrbrain-v52 vrubrain-v51 vrubrain-v52 vrhero-v10 erle pxf navio navio2 raspilot bbbmini minlure erlebrain2 bhat qflight pxfmini

define frame_template
$(1)-$(2) : EXTRAFLAGS += "-DFRAME_CONFIG=$(shell echo $(2) | tr a-z A-Z | sed s/-/_/g)_FRAME "
$(1)-$(2) : $(1)
$(1)-$(2)-hil : $(1)-$(2)
$(1)-$(2)-debug : $(1)-$(2)
$(1)-$(2)-mavlink1 : $(1)-$(2)
$(1)-$(2)-debug-mavlink1 : $(1)-$(2)
$(1)-$(2)-hilsensors : $(1)-$(2)
$(1)-$(2)-upload : $(1)-$(2)
$(1)-$(2)-upload : $(1)-upload
endef

define board_template
$(1)-hil : $(1)
$(1)-debug : $(1)
$(1)-mavlink1 : $(1)
$(1)-debug-mavlink1 : $(1)-debug
$(1)-asan : $(1)
$(1)-hilsensors : $(1)
endef

USED_BOARDS := $(foreach board,$(BOARDS), $(findstring $(board), $(MAKECMDGOALS)))
USED_FRAMES := $(foreach frame,$(FRAMES), $(findstring $(frame), $(MAKECMDGOALS)))
#$(warning $(USED_BOARDS))
#$(warning $(USED_FRAMES))
# generate targets of the form BOARD-FRAME and BOARD-FRAME-HIL
$(foreach board,$(USED_BOARDS),$(eval $(call board_template,$(board))))
$(foreach board,$(USED_BOARDS),$(foreach frame,$(USED_FRAMES),$(eval $(call frame_template,$(board),$(frame)))))

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
include $(MK_DIR)/mavgen.mk

# Makefile wrapper to allow old make commands to use waf build system

SOURCEROOT = $(realpath $(dir $(lastword $(MAKEFILE_LIST)))/..)

SRCROOT	:=	$(realpath $(dir $(firstword $(MAKEFILE_LIST))))
BUILDDIR :=	$(lastword $(subst /, ,$(SRCROOT)))

# Workaround a $(lastword ) bug on cygwin
ifeq ($(BUILDDIR),)
  WORDLIST		:=	$(subst /, ,$(SRCROOT))
  BUILDDIR		:=	$(word $(words $(WORDLIST)),$(WORDLIST))
endif

TARGET_BOARD=sitl
TARGET_BUILD=$(BUILDDIR)
WAF_FLAGS=
FRAME=
BINARY_FRAME=

ifeq ($(BUILDDIR),ArduPlane)
TARGET_BUILD=plane
BINARY_NAME=arduplane
endif

ifeq ($(BUILDDIR),ArduCopter)
TARGET_BUILD=copter
BINARY_NAME=arducopter
FRAME="quad"
BINARY_FRAME="-quad"
endif

ifeq ($(BUILDDIR),APMrover2)
TARGET_BUILD=rover
BINARY_NAME=ardurover
endif

ifeq ($(BUILDDIR),AntennaTracker)
TARGET_BUILD=antennatracker
BINARY_NAME=antennatracker
endif

FRAMES = quad tri hexa y6 octa octa-quad heli single coax
BOARDS = px4-v1 px4-v2 px4-v4 sitl linux vrbrain vrbrain-v40 vrbrain-v45 vrbrainv-50 vrbrain-v51 vrbrain-v52 vrubrain-v51 vrubrain-v52 vrhero-v10 erle pxf navio navio2 raspilot bbbmini minlure erlebrain2 bhat qflight pxfmini

define frame_template
$(1)-$(2) : FRAME=$(2)
$(1)-$(2) : $(1)
$(1)-$(2) : BINARY_FRAME=-$(2)
$(1)-$(2)-debug : $(1)-$(2)
$(1)-$(2)-debug : WAF_FLAGS=--debug
$(1)-$(2)-upload : $(1)-$(2)
$(1)-$(2)-upload : WAF_FLAGS=--upload
endef

define board_template
$(1) : TARGET_BOARD=$(1)
$(1) : all
$(1)-debug : $(1)
$(1)-debug : WAF_FLAGS=--debug
$(1)-debug : BIN_DIR=$(1)-debug
$(1)-upload : $(1)
$(1)-upload : WAF_FLAGS=--upload
endef

USED_BOARDS := $(foreach board,$(BOARDS), $(findstring $(board), $(MAKECMDGOALS)))
USED_FRAMES := $(foreach frame,$(FRAMES), $(findstring $(frame), $(MAKECMDGOALS)))
#$(warning $(USED_BOARDS))
#$(warning $(USED_FRAMES))
# generate targets of the form BOARD-FRAME and BOARD-FRAME-HIL
$(foreach board,$(USED_BOARDS),$(eval $(call board_template,$(board))))
$(foreach board,$(USED_BOARDS),$(foreach frame,$(USED_FRAMES),$(eval $(call frame_template,$(board),$(frame)))))

ifeq ($(WAF_FLAGS),--debug)
BIN_DIR=$(TARGET_BOARD)-debug
else
BIN_DIR=$(TARGET_BOARD)
endif

ifneq ($(findstring px4, $(MAKECMDGOALS)),)
BINARY_EXTENSION=".px4"
BINARY_EXTENSION_DEST=".px4"
else
BINARY_EXTENSION=
BINARY_EXTENSION_DEST=".elf"
endif

all:
	@echo SOURCEROOT=$(SOURCEROOT)
	@echo BUILDDIR=$(BUILDDIR)
	@echo TARGET_BOARD=$(TARGET_BOARD)
	@echo TARGET_BUILD=$(TARGET_BUILD)
	@echo WAF_FLAGS=$(WAF_FLAGS)
	@echo FRAME=$(FRAME)
	@echo BIN_DIR=$(BIN_DIR)
	@echo BINARY_NAME=$(BINARY_NAME)$(BINARY_FRAME)
	@echo BINARY_EXTENSION=$(BINARY_EXTENSION)
	@echo BINARY_EXTENSION_DEST=$(BINARY_EXTENSION_DEST)
	mkdir -p $(SOURCEROOT)/build
	cd $(SOURCEROOT) && modules/waf/waf-light --board $(TARGET_BOARD) configure
	cd $(SOURCEROOT) && modules/waf/waf-light $(TARGET_BUILD) $(WAF_FLAGS)
	@cp $(SOURCEROOT)/build/$(BIN_DIR)/bin/$(BINARY_NAME)$(BINARY_FRAME)$(BINARY_EXTENSION) $(BUILDDIR)$(BINARY_EXTENSION_DEST)
	@ls -l $(BUILDDIR)$(BINARY_EXTENSION_DEST)
	@echo "Build done $(BUILDDIR)$(BINARY_EXTENSION_DEST)"

clean:
	@cd $(SOURCEROOT) && modules/waf/waf-light --board $(TARGET_BOARD) configure
	@cd $(SOURCEROOT) && modules/waf/waf-light clean

px4-clean:
	@echo Removing px4 build directories
	@rm -rf $(SOURCEROOT)/build/px4* $(SOURCEROOT)/build/c4che/px4*

px4-cleandep: px4-clean

cleandep:
	@rm -rf $(SOURCEROOT)/build

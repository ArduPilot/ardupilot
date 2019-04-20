# Main component makefile.
# It basically repack existing ardupilot libraries into one archive what esp-idf expect.
# We also rename functions sections in object files to place "hot" code to the IRAM
CDIR:=$(dir $(lastword $(MAKEFILE_LIST)))
BOARD:=$(shell cat $(abspath $(CDIR)../..)/board.txt)
A:=$(abspath $(CDIR)../../../../../build/$(BOARD)/lib)/libArduCopter_libs.a
B:=$(abspath $(CDIR)../../../../../build/$(BOARD)/lib/bin)/libarducopter.a
ARCH:=$(abspath $(CDIR)../..)/arch.py
FUNCS:=$(abspath $(CDIR))/functions.list

COMPONENT_OWNBUILDTARGET := true

build:$(COMPONENT_LIBRARY)

$(COMPONENT_LIBRARY):$(A) $(B) $(FUNCS)
	$(ARCH) $(AR) $(OBJCOPY) $(FUNCS) $(COMPONENT_LIBRARY) $(A) $(B)


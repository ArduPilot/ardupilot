# Main component makefile.
# It basically repack existing ardupilot libraries into one archive what esp-idf expect.
CDIR:=$(dir $(lastword $(MAKEFILE_LIST)))
BOARD = $(shell cat $(abspath $(CDIR)../..)/board.txt)
A=$(abspath $(CDIR)../../../../../build/$(BOARD)/lib)/libArduCopter_libs.a
B=$(abspath $(CDIR)../../../../../build/$(BOARD)/lib/bin)/libarducopter.a

COMPONENT_OWNBUILDTARGET := true

build:$(COMPONENT_LIBRARY)

$(COMPONENT_LIBRARY):$(A) $(B)
	rm -f $(COMPONENT_LIBRARY)
	echo "create $(COMPONENT_LIBRARY)\naddlib $(A)\naddlib $(B)\nsave\nend\n" | $(AR) -M


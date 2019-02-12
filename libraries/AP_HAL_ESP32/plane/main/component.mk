#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_ADD_LDFLAGS +=  -Wl,--whole-archive -L$(abspath $(dir $(lastword $(MAKEFILE_LIST)))../../../../build/esp32/lib) -lArduPlane_libs -L$(abspath $(dir $(lastword $(MAKEFILE_LIST)))../../../../build/esp32/lib/bin) -larduplane  -Wl,--no-whole-archive
COMPONENT_ADD_LINKER_DEPS += $(abspath $(dir $(lastword $(MAKEFILE_LIST)))../../../../build/esp32/lib)/libArduPlane_libs.a  $(abspath $(dir $(lastword $(MAKEFILE_LIST)))../../../../build/esp32/lib/bin)/libarduplane.a 
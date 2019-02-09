#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_ADD_LDFLAGS +=  -Wl,--whole-archive -L$(abspath $(dir $(lastword $(MAKEFILE_LIST)))../../../../build/esp32/lib) -lArduCopter_libs -L$(abspath $(dir $(lastword $(MAKEFILE_LIST)))../../../../build/esp32/lib/bin) -larducopter  -Wl,--no-whole-archive

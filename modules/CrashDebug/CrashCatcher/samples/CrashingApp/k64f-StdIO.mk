PROJECT         := CrashingApp_StdIO
DEVICES         := K64F
GCC4MBED_DIR    := ../../../gcc4mbed
NO_FLOAT_SCANF  := 1
NO_FLOAT_PRINTF := 1
MBED_OS_ENABLE  := 0
INCDIRS         := ../../include
LIBS_PREFIX     := ../../obj/armv7-m/Core/src/CrashCatcher_armv7m.o
LIBS_SUFFIX     := ../../lib/armv7-m/libCrashCatcher_StdIO_armv7m.a

include $(GCC4MBED_DIR)/build/gcc4mbed.mk

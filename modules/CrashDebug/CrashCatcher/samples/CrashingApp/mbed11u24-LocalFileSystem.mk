PROJECT         := CrashingApp_LocalFileSystem
DEVICES         := LPC11U24
GCC4MBED_DIR    := ../../../gcc4mbed
NO_FLOAT_SCANF  := 1
NO_FLOAT_PRINTF := 1
MBED_OS_ENABLE  := 0
INCDIRS         := ../../include
LIBS_PREFIX     := ../../obj/armv6-m/Core/src/CrashCatcher_armv6m.o
LIBS_SUFFIX     := ../../lib/armv6-m/libCrashCatcher_LocalFileSystem_armv6m.a

include $(GCC4MBED_DIR)/build/gcc4mbed.mk

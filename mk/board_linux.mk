TOOLCHAIN = NATIVE

include $(MK_DIR)/find_tools.mk

# Linux build is just the same as SITL for now
LIBS = -lm -lpthread -lrt
include $(MK_DIR)/board_native.mk

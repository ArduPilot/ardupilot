# PX4 specific build support
ifeq ($(APPDIR),)
include $(MK_DIR)/px4_targets.mk
else
include $(MK_DIR)/px4_core.mk
endif

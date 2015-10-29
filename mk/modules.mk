
# git submodule support

.PHONY: CHECK_MODULES

# PX4 build needs submodules
px4-%: CHECK_MODULES

CHECK_MODULES:
	$(v)$(MK_DIR)/check_modules.sh

module-update:
	git submodule update

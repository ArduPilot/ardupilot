
# git submodule support

.PHONY: CHECK_MODULES
all: CHECK_MODULES

CHECK_MODULES:
	$(v)$(MK_DIR)/check_modules.sh

module-update:
	git submodule update

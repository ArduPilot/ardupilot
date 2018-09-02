
# git submodule support

define echowarning
@echo "WARNING: $1" >&2
endef

define echoallwarnings
	$(call echowarning)
	$(call echowarning)
	$(call echowarning,make build system is deprecated for Linux boards)
	$(call echowarning,new features are not going to be added anymore)
	$(call echowarning,See README-WAF.md: https://github.com/ArduPilot/ardupilot/blob/master/README-WAF.md)
	$(call echowarning)
	$(call echowarning,The make build system will soon be removed)
	$(call echowarning)
	$(call echowarning)
endef

CHECK_MODULES:
	$(if $(BUILDSYS_DEPRECATED),$(call echoallwarnings))
	$(v)$(MK_DIR)/check_modules.sh

.PHONY: CHECK_MODULES
all: CHECK_MODULES

module-update:
	git submodule update

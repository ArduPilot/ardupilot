define LIBRARY_MODULE_template
dir := $(1)
LIBRARY_INCLUDES += -I$$(dir)
include $$(dir)/rules.mk
endef


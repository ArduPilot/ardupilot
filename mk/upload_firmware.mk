ifneq ($(BOARD_LINUX_HOST),)
upload: $(SKETCHELF).timestamp-upload

$(SKETCHELF).timestamp-upload: $(SKETCHELF)
	scp $(SKETCHELF) $(BOARD_LINUX_HOST):
	touch $@
else
upload:
	@echo Check your config.mk: BOARD_LINUX_HOST should be defined to upload firmware
	exit 1
endif

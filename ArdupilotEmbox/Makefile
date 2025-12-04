
ARDUPILOT_BOARD := $(call option_get,STRING,board)

$(CONFIGURE):
	cd ../ && ( \
		export EMBOX_GCC_LINK=full; \
		./waf distclean ; \
		./waf configure \
			--toolchain=$(TOOLCHAIN_DIR)/embox \
			--board $(ARDUPILOT_BOARD) \
			--disable-networking \
			--static \
			--debug \
	)
	touch $@

$(BUILD):
	cd ../ && ( \
		./waf copter \
	)
	touch $@

$(INSTALL):
	cd ../build/$(ARDUPILOT_BOARD) && ( \
		$(EMBOX_GCC) -fno-exceptions -Wl,--wrap,malloc \
			bin/arducopter \
			-o $(PKG_INSTALL_DIR)/arducopter.o \
			-static lib/libArduCopter_libs.a \
	)
	touch $@

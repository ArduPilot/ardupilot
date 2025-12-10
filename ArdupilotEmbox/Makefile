
ardupilot_board := $(call option_get,STRING,board)

ifeq (1,$(call option_get,BOOLEAN,debug))
ardupilot_debug := --debug
else
ardupilot_debug := --disable-Logging
endif

$(CONFIGURE):
	cd ../ && ( \
		export EMBOX_GCC_LINK=full; \
		./waf distclean ; \
		./waf configure \
			--toolchain=$(TOOLCHAIN_DIR)/embox \
			--board $(ardupilot_board) \
			--disable-networking \
			--static \
			$(ardupilot_debug) \
	)
	touch $@

$(BUILD):
	cd ../ && ( \
		./waf copter \
	)
	touch $@

$(INSTALL):
	cd ../build/$(ardupilot_board) && ( \
		$(EMBOX_GCC) -fno-exceptions -Wl,--wrap,malloc \
			bin/arducopter \
			-o $(PKG_INSTALL_DIR)/arducopter.o \
			-static lib/libArduCopter_libs.a \
	)
	touch $@


ardupilot_board := $(call option_get,STRING,board)
ardupilot_flags := 

ifeq (1,$(call option_get,BOOLEAN,debug))
ardupilot_flags += --debug
else
ardupilot_flags += --disable-Logging
endif

ifeq (0,$(call option_get,BOOLEAN,scripting))
ardupilot_flags += --disable-scripting
endif

ifeq (0,$(call option_get,BOOLEAN,networking))
ardupilot_flags += --disable-networking
endif

$(CONFIGURE):
	cd ../ && ( \
		export EMBOX_GCC_LINK=full; \
		./waf distclean ; \
		./waf configure \
			--toolchain=$(TOOLCHAIN_DIR)/embox \
			--board $(ardupilot_board) \
			$(ardupilot_flags) \
			--static \
	)
	touch $@

$(BUILD):
	cd ../ && ( \
		./waf copter \
	)
	touch $@

$(INSTALL):
	cd ../build/$(ardupilot_board) && ( \
		EMBOX_GCC_LINK=full $(EMBOX_GCC) -fno-exceptions -Wl,--wrap,malloc \
			bin/arducopter \
			-Wl,--cref -Wl,-Map $(PKG_INSTALL_DIR)/arducopter_full.map \
			-o $(PKG_INSTALL_DIR)/arducopter_full.o \
			-static lib/libArduCopter_libs.a \
	)
	cd ../build/$(ardupilot_board) && ( \
		$(EMBOX_GCC) -fno-exceptions -Wl,--wrap,malloc \
			bin/arducopter \
			-o $(PKG_INSTALL_DIR)/arducopter.o \
			-static lib/libArduCopter_libs.a \
	)
	touch $@

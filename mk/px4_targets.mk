# PX4 build is via external build system

ifneq ($(PX4_ROOT),)

# install rc.APM from the AP_HAL_PX4/scripts directory as /etc/init.d/rc.APM
HAL_PX4_DIR = $(realpath $(MK_DIR)/../libraries/AP_HAL_PX4)
PX4_EXTERNAL_SCRIPTS = $(HAL_PX4_DIR)/scripts/rc.APM~init.d/rc.APM

PX4_EXTERNAL = EXTERNAL_APPS=$(PWD) EXTERNAL_SCRIPTS=$(PX4_EXTERNAL_SCRIPTS)
PX4_MAKE = make -C $(PX4_ROOT) $(PX4_EXTERNAL) CONFIG_APM=y


px4:
	$(PX4_MAKE)

px4-clean: clean
	$(PX4_MAKE) clean
	$(PX4_MAKE) configure_px4fmu

px4-upload:
	$(PX4_MAKE) upload

else

px4:
	$(error ERROR: You need to add PX4_ROOT to your config.mk)

px4-clean: px4

px4-upload: px4

endif

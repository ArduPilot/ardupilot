# mr_vmu_rt1176 board-level CMake extension, auto-included by Zephyr.

if(CONFIG_NXP_IMXRT_BOOT_HEADER AND CONFIG_BOOT_FLEXSPI_NOR)
  zephyr_library()
  zephyr_library_sources(xip/mr_vmu_rt1176_flexspi_nor_config.c)
  zephyr_library_include_directories(xip)
endif()

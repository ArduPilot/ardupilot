cmake_minimum_required(VERSION 3.20.0)

# Register our out-of-tree boards directory so Zephyr can find mr_vmu_rt1176
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# waf passes ZEPHYR_BASE as a CMake cache variable. Prefer that and
# also support shell-exported ZEPHYR_BASE.
if(DEFINED ZEPHYR_BASE AND NOT "$ENV{ZEPHYR_BASE}" STREQUAL "${ZEPHYR_BASE}")
	set(ENV{ZEPHYR_BASE} "${ZEPHYR_BASE}")
endif()

find_package(Zephyr REQUIRED HINTS ${ZEPHYR_BASE} $ENV{ZEPHYR_BASE})
project(ap_hal_zephyr)

# The time base must survive idle. Every ARM board here stops its own clock in
# WFI - rt1176 gates the whole CM7 domain, CubeOrangeZephyr leaves TIM5LPEN clear
# - and the failure is quiet: only elapsed time is wrong, by 1/busy-fraction.
if(CONFIG_ARM AND NOT CONFIG_AP_NO_WFI_IDLE)
    message(FATAL_ERROR
        "CONFIG_AP_NO_WFI_IDLE is not set for this ARM target.\n"
        "\n"
        "It must be y on every ARM AP_HAL_Zephyr board. WFI stops the "
        "hardware timer that micros()/millis() are built on, so the vehicle "
        "loses exactly as much time as it spends idle. Nothing crashes - "
        "elapsed time simply runs slow, by a factor that changes with load.\n"
        "\n"
        "It defaults to y. Something has explicitly turned it off: look for "
        "CONFIG_AP_NO_WFI_IDLE=n in this board's prj.<board>.conf or in a "
        "layer it includes.\n"
        "\n"
        "Non-ARM targets (ESP32S3Zephyr, native_sim) must NOT set it: the "
        "veto works through ARM_ON_ENTER_CPU_IDLE_HOOK, which only ARM "
        "defines, and an ineffective Kconfig assignment breaks the configure.")
endif()

set(ARDUPILOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../..)

# Redirect malloc -> __wrap_malloc so bare malloc() returns zeroed memory, as
# ChibiOS and Linux do. Must target zephyr_final, not the intermediate 'app'.
# --allow-multiple-definition: MINIMAL_LIBCPP and c++.cpp both define new/delete.
if(TARGET zephyr_final)
    target_link_options(zephyr_final PRIVATE
        -Wl,--wrap,malloc
        -Wl,--allow-multiple-definition
        # Force k_busy_wait's object into the link: delay_microseconds() calls it,
        # but it is normally dragged in by some driver referencing it first. With
        # that driver disabled the link fails on z_impl_k_busy_wait.
        -Wl,--undefined=z_impl_k_busy_wait
    )
endif()

# Two include dirs serving different halves of the build; do not merge them.
# ../include is the C/C++ path, shared with AP so one header serves both sides.
# ./include is devicetree only, found via BOARD_ROOT and never seen by a compiler.
zephyr_include_directories(${ARDUPILOT_DIR}/libraries/AP_HAL_Zephyr/include)

# showinc: export Zephyr include dirs for WAF to inject into AP C++ compilation
add_custom_target(showinc
    COMMAND echo
        "$<TARGET_PROPERTY:zephyr_interface,INTERFACE_INCLUDE_DIRECTORIES>"
        > ${CMAKE_BINARY_DIR}/includes.list
    VERBATIM
    BYPRODUCTS ${CMAKE_BINARY_DIR}/includes.list
    COMMAND_EXPAND_LISTS
)
add_dependencies(showinc zephyr_generated_headers)

# showdefs: export Zephyr compile definitions for WAF
add_custom_target(showdefs
    COMMAND echo
        "$<TARGET_PROPERTY:zephyr_interface,INTERFACE_COMPILE_DEFINITIONS>"
        > ${CMAKE_BINARY_DIR}/compile_defs.list
    VERBATIM
    BYPRODUCTS ${CMAKE_BINARY_DIR}/compile_defs.list
    COMMAND_EXPAND_LISTS
)
add_dependencies(showdefs zephyr_generated_headers)

# FlexSPI NOR driver rodata relocation. Upstream relocates only .text for the
# FlexSPI XIP path, so the driver's const config stays in flash while its data is
# in RAM: init then reads config through the controller it is resetting, and HANGS.
if(CONFIG_FLASH_MCUX_FLEXSPI_XIP AND CONFIG_MEMC_MCUX_FLEXSPI)
    set(_flexspi_rodata_files
        ${ZEPHYR_BASE}/drivers/memc/memc_mcux_flexspi.c
        ${ZEPHYR_BASE}/drivers/flash/flash_mcux_flexspi_mx25um51345g.c
    )
    foreach(_f ${_flexspi_rodata_files})
        if(EXISTS ${_f})
            zephyr_code_relocate(FILES ${_f}
                LOCATION ${CONFIG_FLASH_MCUX_FLEXSPI_XIP_MEM}_RODATA)
            message(STATUS "AP: relocating rodata to "
                    "${CONFIG_FLASH_MCUX_FLEXSPI_XIP_MEM} for ${_f}")
        else()
            message(WARNING "AP: expected FlexSPI source not found: ${_f}")
        endif()
    endforeach()
endif()

target_sources(app PRIVATE src/main.cpp)
# Fault handling and the synchronous console it reports through. Needed on
# every board: Tools/scripts/zephyr_read_fatal.py reads its g_ap_fatal_*
# symbols, and it bridges faults to AP_HAL_Zephyr/Scheduler.cpp.
target_sources(app PRIVATE src/ap_fault_handler.c)
# Boot-stage checkpoint tracing - diagnostic only, for a board that will not boot
if(CONFIG_AP_BOOT_CHECKPOINTS)
    target_sources(app PRIVATE src/boot_checkpoints.c)
endif()
# RT1176: mask LPI2C1's irq before its driver enables it (floating SDA/SCL on an
# unconnected GPS1 hangs boot). Was unguarded, so it fired irq_disable(32) on
# every SoC.
if(CONFIG_SOC_SERIES_IMXRT11XX)
    target_sources(app PRIVATE src/rt1176_lpi2c1_irq_mask.c)
endif()
# self-gated on CONFIG_AP_SPI_PROBE_DIAG
target_sources(app PRIVATE src/spi_probe_diag.c)
# RT1176: route GPIO_MUX pads back to the standard GPIOn the DTS drives
target_sources(app PRIVATE src/rt1176_gpio_mux_fixup.c)
# RT1176: fix eDMA0 Group0-vs-Group1 fixed-priority arbitration at the root
# (CR.ERGA), rather than only via per-peripheral channel relocation
target_sources(app PRIVATE src/rt1176_edma_arbitration_fixup.c)
# self-gated on CONFIG_AP_I2C_PROBE_DIAG
target_sources(app PRIVATE src/i2c_probe_diag.c)
# RT1176 parameter storage: ROM-API flash writes. Enabling Zephyr's FlexSPI NOR
# driver instead breaks XIP and traps the core in BootROM, so ext_flash_ctrl
# stays disabled and writes go through the ROM API, as AP_Bootloader does.
if(CONFIG_AP_RT1176_ROMAPI_FLASH)
    target_sources(app PRIVATE src/rt1176_romapi_flash.c)
endif()

# Zephyr flash-driver facade over the ROM API: gives flash_map /
# fixed-partitions / img_mgmt a real flash device on a board where the
# FlexSPI driver cannot be enabled (breaks XIP). See the Kconfig help.
if(CONFIG_AP_ROMAPI_FLASH_DEV)
    target_sources(app PRIVATE src/rt1176_romapi_flash_dev.c)
endif()

# RT1176 SNVS SRTC: battery-backed wall clock behind Util::get/set_hw_rtc
# (ChibiOS RTC-backup-domain parity). Direct register backend; see the file
# header for why Zephyr's counter_mcux_snvs driver can't be used.
if(CONFIG_SOC_SERIES_IMXRT11XX)
    target_sources(app PRIVATE src/rt1176_snvs_rtc.c)
endif()

# RT1176 on-board crash dump: custom coredump backend, reusing the ROM-API
# flash primitives above. See rt1176_coredump_backend.c for why this can't
# be the stock coredump_backend_flash_partition.c on this board.
if(CONFIG_AP_RT1176_COREDUMP)
    target_sources(app PRIVATE src/rt1176_coredump_backend.c)
endif()

# FlexIO DShot: CogniPilot cerebri's driver, VENDORED in-tree. modules/cerebri is
# reference material, not a build dependency - registering it as a Zephyr module
# would drag in nanopb/synapse_pb/zros.
if(CONFIG_NXP_FLEXIO_DSHOT)
    target_sources(app PRIVATE src/nxp_flexio_dshot.c)
    # PWM<->DShot pad-ownership switching for RCOutput.cpp - a C file
    # because the NXP pinctrl_soc.h initializers won't compile as C++
    # (-Wnarrowing); see its own header comment.
    target_sources(app PRIVATE src/ap_rcout_pinmux.c)
endif()

# WFI gates the whole CM7 clock domain on this SoC (SysTick + DWT stop);
# veto it so the kernel clock can never stall. See AP_NO_WFI_IDLE help.
if(CONFIG_AP_NO_WFI_IDLE)
    target_sources(app PRIVATE src/no_wfi_idle.c)
endif()

# XBARA1 route for RC PPM hardware capture (pad -> XBAR IN12 -> QTMR1 ch0);
# see the file's header for the register citations.
if(CONFIG_AP_RCIN_PWM_CAPTURE AND CONFIG_SOC_SERIES_IMXRT11XX)
    target_sources(app PRIVATE src/rt1176_rcin_xbar_fixup.c)
endif()

# ardupilot_stub.c provides a weak ardupilot_entry for the initial cmake pass
# (before the AP static library exists).  When ARDUPILOT_LIB is set the stub
# is NOT added, so ardupilot_entry is undefined and the GNU linker will search
# the AP archive for the real strong definition.
if(NOT DEFINED ARDUPILOT_LIB OR "${ARDUPILOT_LIB}" STREQUAL "")
    target_sources(app PRIVATE src/ardupilot_stub.c)
endif()

# ── Link ArduPilot static libraries when built via WAF ──────────────────────
# WAF sets ARDUPILOT_LIB (lib dir) and ARDUPILOT_CMD (copter/plane/rover/etc.)
# before invoking cmake --build for the final link.
# On the first cmake configure pass (waf pre_build) ARDUPILOT_LIB is empty;
# guard so configure still succeeds.
if(DEFINED ARDUPILOT_LIB AND NOT "${ARDUPILOT_LIB}" STREQUAL "")
    if(NOT DEFINED ARDUPILOT_CMD)
        set(ARDUPILOT_CMD "copter")
    endif()

    message(STATUS "ARDUPILOT_CMD=${ARDUPILOT_CMD}")
    message(STATUS "ARDUPILOT_LIB=${ARDUPILOT_LIB}")
    message(STATUS "ARDUPILOT_BIN=${ARDUPILOT_BIN}")

    # AP archives use --start-group/--end-group for circular-reference safety.
    # ardupilot_stub.c is excluded from this build pass, so ardupilot_entry is
    # undefined and the linker will pull the strong T definition from the AP
    # archive. Do NOT add libkernel.a or other Zephyr archives here — Zephyr's
    # own CMake already links them; duplicating causes multiple-definition errors.
    if(${ARDUPILOT_CMD} STREQUAL "arducopter" OR ${ARDUPILOT_CMD} STREQUAL "copter")
        target_link_libraries(app PRIVATE
            -Wl,--start-group
            "${ARDUPILOT_BIN}/libarducopter.a"
            "${ARDUPILOT_LIB}/libArduCopter_libs.a"
            -Wl,--end-group)
    elseif(${ARDUPILOT_CMD} STREQUAL "arduplane" OR ${ARDUPILOT_CMD} STREQUAL "plane")
        target_link_libraries(app PRIVATE
            -Wl,--start-group
            "${ARDUPILOT_BIN}/libarduplane.a"
            "${ARDUPILOT_LIB}/libArduPlane_libs.a"
            -Wl,--end-group)
    elseif(${ARDUPILOT_CMD} STREQUAL "ardurover" OR ${ARDUPILOT_CMD} STREQUAL "rover")
        target_link_libraries(app PRIVATE
            -Wl,--start-group
            "${ARDUPILOT_BIN}/libardurover.a"
            "${ARDUPILOT_LIB}/libRover_libs.a"
            -Wl,--end-group)
    elseif(${ARDUPILOT_CMD} STREQUAL "AP_Bootloader")
        # Bootloader builds do not follow the tool pattern below: their wscript uses
        # use= not ap_libraries=, so no slim libap_<CMD>.a exists - link
        # AP_Bootloader_libs. The define lets shared sources tell the images apart.
        target_compile_definitions(app PRIVATE AP_ZEPHYR_BOOTLOADER_BUILD=1)
        target_link_libraries(app PRIVATE
            -Wl,--start-group
            "${ARDUPILOT_BIN}/libAP_Bootloader.a"
            "${ARDUPILOT_LIB}/libAP_Bootloader_libs.a"
            -Wl,--end-group)
    else()
        # tools / CPUInfo: waf puts program objects in ARDUPILOT_BIN. Which AP archive
        # holds the libraries depends on the wscript - ap_libraries= gives a slim
        # libap_<CMD>.a, use='ap' gives shared libap.a. Pick whichever exists.
        if(EXISTS "${ARDUPILOT_LIB}/libap_${ARDUPILOT_CMD}.a")
            set(AP_TOOL_ARCHIVE "${ARDUPILOT_LIB}/libap_${ARDUPILOT_CMD}.a")
        else()
            set(AP_TOOL_ARCHIVE "${ARDUPILOT_LIB}/libap.a")
        endif()
        message(STATUS "AP_TOOL_ARCHIVE=${AP_TOOL_ARCHIVE}")
        target_link_libraries(app PRIVATE
            -Wl,--start-group
            "${ARDUPILOT_BIN}/lib${ARDUPILOT_CMD}.a"
            "${AP_TOOL_ARCHIVE}"
            -Wl,--end-group)
    endif()

    target_include_directories(app PRIVATE
        ${ARDUPILOT_DIR}
        ${ARDUPILOT_DIR}/libraries
        ${ARDUPILOT_DIR}/libraries/AP_Common/missing
        # generated hwdef.h - USB identity strings, board name, device lookups
        ${ARDUPILOT_HWDEF_DIR}
    )

    target_compile_definitions(app PRIVATE
        CONFIG_HAL_BOARD=HAL_BOARD_ZEPHYR
        APM_BUILD_DIRECTORY=APM_BUILD_UNKNOWN
        __STDC_FORMAT_MACROS=1
        __AP_LINE__=__LINE__
    )

    set_target_properties(app PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

    # ── IRAM placement linker fragment ────────────────────────────────────────
    # ap_iram_sections.sh (called from zephyr.py _cmake_final_link) generates a
    # linker fragment mapping hot AP functions to .text sections that Zephyr
    # places in iram0_0_seg (ESP32-S3: 415 KB, currently ~11% used).
    # AP_IRAM_SECTIONS_LD is only set when the registry and script both exist.
    if(DEFINED AP_IRAM_SECTIONS_LD AND NOT "${AP_IRAM_SECTIONS_LD}" STREQUAL "")
        zephyr_linker_sources(SECTIONS "${AP_IRAM_SECTIONS_LD}")
        message(STATUS "AP IRAM placement: ${AP_IRAM_SECTIONS_LD}")
    endif()
endif()

# Hot ArduPilot code into ITCM (NXP i.MX RT only): zero-wait-state at core speed,
# versus XIP from external NOR. A linker fragment, not zephyr_code_relocate(),
# because AP arrives as a prebuilt archive path with no CMake target.
dt_has_chosen(HAS_ITCM PROPERTY "zephyr,itcm")
# SoC-pinned as well as chosen-pinned: another SoC declaring zephyr,itcm for
# its own on-chip SRAM would otherwise pick up the rt1176's 350 KB hot-code
# roster and overflow (measured 2026-08-15 on a 512 KB part, by 627 KB).
if(HAS_ITCM AND CONFIG_SOC_MIMXRT1176_CM7)
    set(AP_ITCM_LD "${CMAKE_CURRENT_SOURCE_DIR}/itcm_hot_code.ld")
    if(EXISTS "${AP_ITCM_LD}")
        zephyr_linker_sources(ITCM_SECTION "${AP_ITCM_LD}")
        message(STATUS "AP ITCM placement: ${AP_ITCM_LD}")
    endif()

    # No bulk-to-SRAM fragment here: tried and removed 2026-08-07. ld gives each
    # input section to the FIRST matching output, and only ITCM_SECTION precedes
    # the text rule - a RAMFUNC_SECTION fragment matches nothing. Grow ITCM instead.
endif()

*****************************************************************************
*** Files Organization                                                    ***
*****************************************************************************

--{root}                        - ChibiOS directory.
  +--readme.txt                 - This file.
  +--documentation.html         - Shortcut to the web documentation page.
  +--license.txt                - GPL license text.
  +--demos/                     - Demo projects, one directory per platform.
  +--docs/                      - Documentation.
  |  +--build/                  - Documentation builders.
  |  +--manual/                 - Manuals directory.
  |  +--quality/                - Reports and metrics.
  |  +--various/                - Various support files.
  +--ext/                       - External libraries, not part of ChibiOS.
  +--os/                        - ChibiOS components.
  |  +--common/                 - Shared OS modules.
  |  |  +--abstractions/        - API emulator wrappers.
  |  |  |  +--cmsis_os/         - CMSIS OS emulation layer for RT.
  |  |  |  +--nasa_osal/        - NASA Operating System Abstraction Layer.
  |  |  +--ext/                 - Vendor files used by the OS.
  |  |  +--ports/               - RTOS ports usable by both RT and NIL.
  |  |  +--startup/             - Startup support.
  |  +--ex/                     - EX component.
  |  |  +--dox/                 - EX documentation resources.
  |  |  +--include/             - EX header files.
  |  |  +--devices /            - EX complex drivers.
  |  +--hal/                    - HAL component.
  |  |  +--boards/              - HAL board support files.
  |  |  +--dox/                 - HAL documentation resources.
  |  |  +--include/             - HAL high level headers.
  |  |  +--lib/                 - HAL libraries.
  |  |  |  +--complex/          - HAL collection of complex drivers.
  |  |  |  |  +--mfs/           - HAL managed flash storage driver.
  |  |  |  |  +--serial_nor/    - HAL managed flash storage driver.
  |  |  |  +--fallback/         - HAL fall back software drivers.
  |  |  |  +--peripherals/      - HAL peripherals interfaces.
  |  |  |  +--streams/          - HAL streams.
  |  |  +--osal/                - HAL OSAL implementations.
  |  |  |  +--lib/              - HAL OSAL common modules.
  |  |  +--src/                 - HAL high level source.
  |  |  +--ports/               - HAL ports.
  |  |  +--templates/           - HAL driver template files.
  |  |     +--osal/             - HAL OSAL templates.
  |  +--oslib/                  - RTOS modules usable by both RT and NIL.
  |  |  +--include/             - OSLIB high level headers.
  |  |  +--src/                 - OSLIB high level source.
  |  |  +--templates/           - OSLIB configuration template files.
  |  +--nil/                    - NIL RTOS component.
  |  |  +--dox/                 - NIL documentation resources.
  |  |  +--include/             - NIL high level headers.
  |  |  +--src/                 - NIL high level source.
  |  |  +--templates/           - NIL configuration template files.
  |  +--rt/                     - RT RTOS component.
  |  |  +--dox/                 - RT documentation resources.
  |  |  +--include/             - RT high level headers.
  |  |  +--src/                 - RT high level source.
  |  |  +--templates/           - RT configuration template files.
  |  +--various/                - Various portable support files.
  +--test/                      - Kernel test suite source code.
  |  +--lib/                    - Portable test engine.
  |  +--hal/                    - HAL test suites.
  |  |  +--testbuild/           - HAL build test and MISRA check.
  |  +--nil/                    - NIL test suites.
  |  |  +--testbuild/           - NIL build test and MISRA check.
  |  +--rt/                     - RT test suites.
  |  |  +--testbuild/           - RT build test and MISRA check.
  |  |  +--coverage/            - RT code coverage project.
  +--testex/                    - EX integration test demos.
  +--testhal/                   - HAL integration test demos.

*****************************************************************************
*** Releases and Change Log                                               ***
*****************************************************************************

*** 21.11.5 ***
- NEW: STM32U0xx support.
- NEW: STM32U3xx support.
- FIX: Fixed OSLIB Jobs Queues: return JOB_NULL descriptors to the free pool
       (bug #1304).
- FIX: Fixed serialize ref duplication and harden dynamic object size
       calculations (bug #1303).
- FIX: Fixed harden memcore/heap boundary checks against arithmetic overflow
       (bug #1302).
- FIX: Fixed priority inheritance through prioritized message queues
       (bug #1301).
- FIX: Fixed  chMsgSend missing rdymsg assignment causes spurious NULL return
       from chMsgWaitTimeoutS (bug #1300).
- FIX: Fixed problem with queues counter in NIL (bug #1299).
- FIX: Fixed problem with CH_PORT_SUPPORTS_RECURSIVE_LOCKS in NIL (bug #1298).

*** 21.11.4 ***
- NEW: STM32C0xx support.
- NEW: STM32G0B0 support.
- NEW: STM32H5xx support.
- NEW: FDCAN support for STM32G4xx, STM32H5xx and STM32H7xx.
- NEW: XSNOR serial flash memories framework.
- NEW: I2C slave support in HAL and in ST low level drivers.
- NEW: Support for LittleFS flash file system.
- NEW: VFS subsystem.
- NEW: OOP framework.
- NEW: Added missing context switch hook in ARMv7-M-ALT port.
- NEW: FDCANv1 and FDCANv2 filter enabled.
- NEW: Recursive locks in RT and NIL made optional, only enabled if the
       underlying port supports the capability.
- NEW: Added STM32 FDCANv2 for STM32H7xx
- NEW: Improved DAC driver, updated STM32 DACv1.
- NEW: Removed oscillator stabilization delay on STM32 LSE and HSE when
       bypass mode is enabled.
- NEW: STM32 TIM1 and TIM8 support added to SYSTICKv1 driver.
- NEW: STM32 RTCv2 and RTCv3 modified to not use shadow registers.
- NEW: Enhanced STM32F7xx MPU configuration in mcuconf.h.
- NEW: I2C slave support in HAL high level driver.
- NEW: Added settings for STM32 OCTOSPIv1 and OCTOSPIv2 TCR bits SSHIFT and
       DHQC.
- NEW: Reworked STM32 SDMMCv1 and SDMMCv2 drivers, better timeout and clock
       handling, improved speed for aligned buffers.
- FIX: Set DAC_HAS_MCR FALSE in STM32F1xx registry.
- FIX: Fixed ADCv4 common registers reset at start (bug 1296).
- FIX: Fixed DMA2 not firing on STM32G431 (bug 1295).
- FIX: Fixed wrong STM32 ADCv2 stop method (bug 1294).
- FIX: Fixed STM32 OTGv1 driver does not re-enables endpoints on wakeup
       (bug 1293).
- FIX: Fixed missing assertion in OSLIB factory module (bug #1292).
- FIX: Fixed problem in FDCANv1 driver for G4 (bug #1291).
- FIX: Fixed problem in recursive locks functions (bug #1288).
- FIX: Fixed ARMv8-M-ML port compile fail when FPU is enabled (bug #1281).
- FIX: Fixed interrupts not enabled for STM32H735 TIM15, TIM16 and TIM17
       (bug #1280).
- FIX: Fixed wrong STM32 LSI activation check (bug #1279).
- FIX: Fixed STM32 HAL UART ISR flaw (bug #1278).
- FIX: Fixed race condition caused by chGuardedPoolAllocI() (bug #1277).
- FIX: Fixed avoid shadowing with build-in pow10 function in chprintf.c
       (bug #1274).
- FIX: Fixed enabling PWM on TIM1, 3, 4 causes compile errors in
       RT-STM32G0B1RE-NUCLEO64 (bug #1273).
- FIX: Fixed unnecessary code in SNOR device drivers (bug #1265).
- FIX: Fixed RP2040 HAL GPIO failed to compile (bug #1264).
- FIX: Fixed channel 0 corruption on STM32 BDMAv1 (bug #1263).
- FIX: Fixed wrong statistics in RT7 (bug #1262).
- FIX: Fixed missing cache management during Cortex-M RAM initializations
       (bug #1261).
- FIX: Fixed RTC & TAMP interrupts not functional (bug #1260).
- FIX: Fixed syntax errors in STM32H7xx/hal_lld_type2.h (bug #1259).
- FIX: Fixed unwanted reset of cache on STM32H7xx (bug #1258).
- FIX: Fixed invalid HSIDIV in STM32Ggxx clocks initialization (bug #1257).
- FIX: Fixed incorrect RTC initialization on STM32G4/L4/L4+ (bug #1256).
- FIX: Fixed syntax error in RP2040 GPIO driver (bug #1255).
- FIX: Fixed undefined STM32_SDMMC_MAXCLK value for STM32H7 type 1 and 2
       (bug #1254).
- FIX: Fixed invalid checks on PLLP/R/Q dividers on STM32H7 (bug #1253).
- FIX: Fixed remote wakeup failure in STM32 OTGv1 driver (bug #1252).
- FIX: Fixed wrong use of hooks in RT/NIL (bug #1251).
- FIX: Fixed SPI_MMC driver broken in 21.11.3 (bug #1249).

*** 21.11.3 ***
- NEW: STM32 DMA drivers now export an STM32_DMA_MAX_TRANSFER definition.
- NEW: PAL events for RP2040 added.
- NEW: Removed obsolete sandbox code from ARMv7-M port. Now ARMv7-M-ALT is
       the official port for use with sandboxes.
- NEW: Reworked HAL MAC driver, now with callback support.
- NEW: Fixed setting of SYSCLK when derived from divided HSI16
- NEW: Mass change: Source code convention changed from CRLF to just CR (Unix).
- NEW: Fixed some corner cases in ADC5, added ADC reset on start().
- NEW: Added a "BufferedSIODriver" class that implements the behavior of the
       legacy Serial driver on top of a SIO implementation (buffering, events
       and all).
- NEW; Now hal.h includes cc_portab.h by default making it mandatory.
- NEW: Moved HAL serial error flags into asynchronous channels interface
       definitions.
- NEW: Reworked HAL SIO driver.
- NEW: Non-proprietary LLVM build support.
- NEW: Added integration of LittleFS on top of our flash infrastructure.
- NEW: Added a centralized errors handler under /os/common/utils. It will
       replace those in HAL and SB and will be shared among multiple subsystems.
- NEW: Added a new OOP model under /os/common/utils. It will replace the
       one in HAL and will be shared among multiple subsystems.
- NEW: Added EFL driver support for STM32F401/411.
- FIX: Fixed broken support for STM32 UART9 and USART10 (bug #1248).
- FIX: Fixed wrong initialization in STM32L1xx ADC driver (bug #1247).
- FIX: Fixed wrong HSI48 support on STM32L0xx (bug #1246).
- FIX: Fixed wrong DMA definitions for STM32L0xx I2C3 peripheral (bug #1245).
- FIX: Fixed wrong path in STM32L053 ADC demo makefile (bug #1244).
- FIX: Fixed missing semicolon in STM32 OTGv1 driver (bug #1243).
- FIX: Fixed HSI48 not enabled for STM32L496/​4A6 (bug #1242).
- FIX: Fixed problem in STM32 gpt_lld_polled_delay() implementation (bug #1241).
- FIX: Fixed invalid delay loop in STM32G0/WL ADCv5 driver (bug #1240).
- FIX: Fixed STM32_MCOSEL setting problem (bug #1239).
- FIX: Fixed problems with cache in STM32 SDMMC drivers (bug #1238).
- FIX: Fixed missing clock enables for some GPIOS on some STM32L4s (bug #1237).
- FIX: Fixed old bugs in serial driver header (bug #1236).
- FIX: Fixed virtual timers lockup under rare conditions (bug #1235).
- FIX: Fixed STM32 RTCv2 locks for a second (bug #1234).

*** 21.11.2 ***
- NEW: Added dubby cycles support for SNOR using the normal SPI driver.
- NEW: Disabled priority check on STM32 EXTI interrupts when the default
       ISR is disabled. This allows for fast interrupts.
- NEW: Added support for UART9 and USART10 on STM32H7.
- NEW: Improved MFS to use explicitly non-cacheable buffers for potentially
       DMA-accessible I/O areas.
- NEW: FatFS now functional on STM32H7xx, added a target to the VFS demo.
- NEW: Improved cache settings in STM32H7xx mcuconf.h.
- NEW: Modified SDMMCv2 to allow for uncached buffers, tested on STM32H7xx.
- NEW: Added OCTOSPIv2 driver using MDMA for STM32H7xx.
- NEW: Added demos for STM32H723ZG Nucleo144 and STM32H735ZI Discovery boards.
- NEW: Added support for STM32H723/25/33/35/A3/B3/A3Q/B3Q.
- NEW: Updated ST Cube headers for STM32H7xx.
- NEW: Improved HAL flash interface with mutual exclusion methods, improved
       EFL and SNOR drivers to use it.
- NEW: Added EFL driver implementation for STM32G4xx.
- NEW: STM32G0B1 USBv2 driver.
- NEW: USBv1 driver optimization and removal of a potential race condition
       (not demonstrated).
- NEW: Added elfAcquireBus()/eflReleaseBus() functions to EFL driver.
- NEW: Added option to copy vectors in RAM on GCC startup for ARMv6-M,
       ARMv7-M and ARMv8-M-ML.
- NEW: On STM32WBxx added a check on STM32_LSI_ENABLE required by IWDG.
- NEW: Added SPIv2 support also to STM32WB and STM32WL.
- FIX: Fixed uninitialized return message in EX subsystem (bug #1267).
- FIX: Re-opened and fixed bug #1100.
- FIX: Fixed wrong buffers toggling in STM32 USBv1 isochronous mode (bug #1232).
- FIX: Fixed STM32 RTCv2 registers synchronization errata (bug #1231).
- FIX: Fixed STM32 ADCv1 and ADCv5 do not allow prescaler divide value of 1
       (bug #1230).
- FIX: Fixed missing check on STM32 SPIv2 DMA settings for SPI1 (bug #1229).
- FIX: Fixed ARMv6-M port Keil compiler fail (bug #1228).
- FIX: Fixed invalid handling of lwIP NETIF_FLAG_LINK_UP flag (bug #1227).
- FIX: Fixed missing TIM16/17 errata handling for STM32G0xx (bug #1226).
- FIX: Fixed missing ADC errata handling for STM32G0xx (bug #1225).
- FIX: Fixed problem in the HAL I2C fallback driver (bug #1224).
- FIX: Fixed GPIOH clock not enabled on STM32L432 (bug #1223).
- FIX: Fixed invalid cumulative time stat in RT (bug #1222).
- FIX: Fixed incorrect type cast in TIME_I2US() (bug #1221).
- FIX: Fixed missing clock disable for STM32 OCTOSPI2 (bug #1220).
- FIX: Fixed wrong condition in STM32 sio_lld_read() function (bug #1219).
- FIX: Fixed STM32 Ethernet driver causes system hang after 2^31 packets
       sent/received (bug #1218).
- FIX: Fixed clock re-initialization problem in STM32 USARTv2 and USARTv3
       drivers (bug #1217).
- FIX: Fixed assertion on initialization of STM32H7xx (bug #1216).
- FIX: Fixed Virtual Timers failure in a specific condition (bug #1215).
- FIX: Fixed invalid STM32_OTG_STEPPING for STM32F40_41xxx (bug #1214).
- FIX: Fixed SPIv2 driver compile fails when SPI_USE_SYNCHRONIZATION is FALSE
       (bug #1213).
- FIX: Fixed invalid state transition in SNOR flash driver (bug #1212).
- FIX: Fixed missing exit condition in sioSynchronizeRX() and
       sioSynchronizeTX() (bug #1211).
- FIX: Some MISRA-related fixes.
- FIX: Fixed missing check in chobjcaches.h (bug #1210).
- FIX: Fixed misspelled chTraceSuspendI() function name (bug #1209).
- FIX: Fixed RT testbuild application broken (bug #1208).

*** 21.11.1 ***
- NEW: Added EFL driver implementation for STM32G4xx.
- NEW: STM32G0B1 USBv2 driver.
- NEW: USBv1 driver optimization and removal of a potential race condition
       (not demonstrated).
- NEW: Added elfAcquireBus()/eflReleaseBus() functions to EFL driver.
- NEW: Added STM32L073RZ-Nucleo64 to USB-CDC "multi" demo. Removed old demo.
- NEW: Added an STM32 WDG "multi" demo. Removed all old WDG demos.
- NEW: Added option to copy vectors in RAM on GCC startup for ARMv6-M,
       ARMv7-M and ARMv8-M-ML.
- NEW: On STM32WBxx added a check on STM32_LSI_ENABLE required by IWDG.
- NEW: Added SPIv2 support also to STM32WB and STM32WL.
- FIX: Fixed PWR_CR2_USV not set in STM32L4+ mcuconf.h file (bug #1207).
- FIX: Fixed USB not enabled on STM32F103x6 (bug #1206).
- FIX: Fixed RT test suite build fails when CH_CFG_USE_TIMESTAMP is FALSE
       (bug #1205).
- FIX: Fixed wrong number of CAN filters for STM32L496/9A6 (bug #1204).
- FIX: Fixed DMA stream not disabled in STM32 QUADSPIv1 driver (bug #1203).
- FIX: Fixed I2C4 DMA streams for STM32L496 (bug #1202).
- FIX: Fixed STM32_SDMMC2_NUMBER on STM32H7 (bug #1201).
- FIX: Fixed STM32G0B1 demo application hangs debuggers (bug #1200).

*** 21.11.0 ***
- NEW: STM32 ADCv2 now supports return code on start function.
- NEW: Integrated FatFS with lwIP HTTPD, now it is possible to serve files
       using HTTP from a storage device.
- NEW: Updated FatFS to version 0.14b.
- NEW: SPIv2 driver has been implemented on: STM32F0, STM32F1, STM32F3,
       STM32F4, STM32F7, STM32G0, STM32G4, STM32L0, STM32L1, STM32L4,
       STM32L4+, STM32H7.
- NEW: New SPIv2 driver model, it is compatible with the previous SPI driver
       and introduces: better runtime errors handling, slave mode,
       data synchronization function, various other improvements.
- NEW: Added an alternate port for ARMv7-M, it uses less RAM and it is
       faster at interrupt processing, it is slightly slower at
       thread-to-thread context switch so it is not a full replacement.
- NEW: Now all xxxStart() functions in HAL are able to report a driver
       activation error.
- NEW: Support for STM32G031, STM32G041, STM32G0B1, STM32G0C1.
- NEW: Made STM32H7 non-cacheable memory option also shareable.
- NEW: EFL driver and demo for STM32F3xx.
- NEW: New unit test subsystem under /os/test. Now it is officially
       ChibiOS/TEST.

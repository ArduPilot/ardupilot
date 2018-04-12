* almost all code is fully rewritten
* added check for all input parameters - no more HardFaults if setted wrong pin numbers
* external I2C bus moved out from FlexiPort by Soft_I2C driver so we always has at least 3 UARTs
* added 1 full-functional UART (only for quads) and 1 RX-only UART for DSM satellite receiver on OpLink connector
* Unlike many other boards, fully implemented registerPeriodicCallback & Co calls
* implemented simple preemptive multitasking
* stack now in CCM memory
* PPM and PWM inputs works via Timer's driver handlers
* added DSM and SBUS parsing on PPM input
* all hardware description tables are now 'const' and locates in flash
* more reliable reset for I2C bus on hangups
* all drivers support set_retries()
* all delays - even microseconds - are very presize by using hardware clock counter (DWT_CYCCNT) in free-running mode
* separated USB and UART drivers
* new SoftwareSerial driver based on ST appnote
* now it uses MPU6000 DRDY output
* removed all compiler's warnings
* ported and slightly altered bootloader to support flashing and start firmware automatically at addresses 8010000 and 8020000 
  (2 low 16k flash pages are used to emulate EEPROM)
* EEPROM emulation altered to ensure the reliability of data storage at power failures
* optimized EEPROM usage by changing from 1-byte to 2-byte writes
* all internal calls use static private methods
* removed unused files
* micros() call uses 32-bit hardware timer instead of awful systick_micros()
* added parameters support for HAL
* OneShot supported
* added translation layer between system PWM modes and borad PWM modes
* added descriptors for all STM internal hardware
* added support to reboot to DFU mode (via "reboot to PX4 bootloader" in MP)
* after any Fault or Panic() automatically reboots to DFU mode
* diversity on RC_Input
* unified exception handling 
* added ability to bind Spectrum satellite without managed 3.3 DC-DC (requires short power off)
* added support for Arduino32 reset sequence - negative DTR edge on 1200 baud or '1eaf' packet with high DTR
* fixed hang on dataflash malfunction
* fixed USB characters loss *without* hangup if disconnected
* added failsafe on receiver's hangup - if no channel changes in 60 seconds
* changed to simplify support of slightly different boards - eg. AirbotF4
* full support for AirbotF4 (separate binaries)
* added support for servos on Input port unused pins
* Added handling of FLASH_SR error bits, including automatic clearing of write protection
* added Arduino-like support of relay on arbitrary pin
* simplified UART driver, buffering now used even in non-blocking mode
* EEPROM emulation locks flash after each write
* EEPROM error handling
* fixed Ardupilot's stealing of Servos even they marked as "Unused"
* added compilation date & time to log output
* added SBUS input via any USART with hardware inverter as on Airbot boards
* added per-board read_me.md files
* fixed Dataflash logs bug from mainstream - now logs are persists between reboots!
* DMA mode for lagre SPI transfers
* USB virtual com-port can be connected to any UART - eg. for OSD or 3DR modems setup
* any UART can be connected to ESC for 4-way interface
* support for logs on SD card for AirbotV2 board
* fixed 2nd Dataflash logs bug from mainstream - now logs are persists between reboots even on boards having chips with 64k sector
* I2C wait time limited to 30ms - no more forever hangs by external compass errors
* FlexiPort can be switched between UART and I2C by parameter
* The RCoutput module has been completely rewritten.
* For the PWM outputs, the error in setting the timer frequency has been compensated.
* added parameter to set PWM mode
* added used memory reporting
* added I2C error reporting
* realized low-power idle state with WFE, TIMER6 is used to generate events each 1uS
* added HAL_RC_INPUT parameter to allow to force RC input mode
* added used stack reporting
* added generation of .DFU file
* time-consuming operations moved out from ISR level to IO_Completion level with low priority
* added support for Clock Security System - if HSE fails in air system will use HSI instead
* added boardEmergencyHandler which will be called on any Fault or Panic() before halt - eg. to release parachute
* motor layout switched to per-board basis
* console assignment switched to per-board basis
* HAL switched to new DMA api with completion interrupts
* AirbotV2 is fully supported with SD card and OSD
* added support for reading from SD card via USB - HAL parameter allows to be USB MassStorege
* added check for stack overflow for all tasks
* all boards formats internal flash chip as FAT and allows access via USB
* added spi flash autodetection
* added support for TRIM command on FAT-formatted Dataflash
* rewritten SD library to support 'errno' and early distinguish between file and dir
* compass processing (4027uS) and baro processing(1271uS)  moved out from interrupt level to task level, because its
 execution time spoils loop time (500Hz = 2000uS for all)
* added reformatting of DataFlash in case of hard filesystem errors, which fixes FatFs bug when there is no free space 
 after file overflows and then deleted
* added autodetect for all known types of baro on external I2C bus
* added autodetect for all known types of compass on external I2C bus
* added check to I2C_Mgr for same device on same bus - to prevent autodetection like MS5611 (already initialized) as BMP_085
* added time offset HAL parameter
* added time syncronization between board's time and GPS time - so logs now will show real local date&time
* i2c driver is fully rewritten, added parsing of bus error flags - ARLO & BERR
* errors at STOP don't cause data loss or time errors - bus reset scheduled as once io_task
* added parsing of TIMEOUT bus flag
* added asyncronous bus reset in case when loockup occures after STOP generation
* full BusReset changed to SoftReset on 1st try
* new SoftI2C driver uses timer and works in interrupts
* added support for SUMD and non-inverted SBUS via PPM pins
* added support for SUMD via UARTs
* MPU not uses FIFO - data readed out via interrupts
* added parametr allowing to defer EEPROM save up to disarm
* optimized preemptive multitask, which does not context switch if next task is the same as current. Real context switch occures in ~4% of calls to task scheduler
* all work with task list moved out to ISR level so there is no race condition anymore
* all work with semaphores moved out to the same ISR level so serialized by hardware and don't requires disabling interrupts
* added parameter RC_FS to enable all RC failsafe
* added disabling of data cache on flash write, just for case (upstream has this update too)
* added a way to schedule context switch from ISR, eg. at data receive or IO_Complete
* added timeout to SPI flags waiting
* now task having started any IO (DMA or interrupts) goes to pause and resumes in IO_Completion ISR, not eating CPU time in wait loop
* optimized I2C wait times to work on noisy lines
* greatly reduced time of reformatting of DataFlash to FAT
* I2C driver fully rewritten again to work via interrupts - no DMA, no polling
* compass and baro gives bus semaphore ASAP to allow bus operations when calculations does
* full status on only 2 leds - GPS sats count, failsafe, compass calibration, autotune etc
* support for SBUS on any UART
* PWM_IN is rewritten to use HAL drivers, as result its size decreased four times (!)
* working PPM on AirbotV2/V3
* buzzer support
* added ability to connect buzzer to arbitrary pin (parameter BUZZ_PIN)
* added priority to SPI DMA transfers, to prevent such issues - https://github.com/betaflight/betaflight/issues/2631
* overclocking support
* OSD is working
* 'boards' folder moved from 'wirish' to HAL directory, to help to find them
* added translation of decoded serial data from PPMn input port to fake UARTs
* reduced to ~1.5uS time from interrupt to resuming task that was waiting that interrupt
* unified NVIC handling
* SoftSerial driver rewritten to not use PWM dependency. Now it can use any pin with timer for RX and any pin for TX, and there 
  can be any number of SoftSerial UARTs
* added per-task stack usage
* SPI driver rewritten: added ISR mode instead of polling, all transfers are monolitic (not divded to send and receive parts), setup for receive now in ISR
* all DataFlash reads and writes now in single SPI transfer
* removed usage of one-byte SPI functions from SD driver
* added support of criticalSections to Scheduler, which protect code from task switch without disabling interrupts
* added CS assert/release delays to SPI device descriptrion table
* added partial MPU support (only to protect from process stack overflow)
* removed -fpermissive from GCC options
* class SD is slightly redesigned, reducing the memory consumption by half (!)
* optimized dma_init_transfer() function: now it twice faster and requires 3 times less memory
* added SD size to bootlog
* SPI via interrupts now works
* added pin names to simplify porting of boards
* added DSM rssi as last channel
* IO tasks excluded from priority boost on yield()
* added awakening of main thread after receiving of data from MPU
* ArduCopter loop at 1KHz! fixed all issues, mean scheduling error is only 10uS - 10 times less than OS tick!
* narowed type for timer_channel
* added support for PWM outputs on N-channels of advanced timers
* added support for inverted buzzer
* added support for passive buzzer
* added support for Devo telemetry protocol
* fixed Soft_I2C timeout 
* all waits for SD answers moved to ISR as finite state machine
* a try to support Ardupilot parameters on builtin OSD (untested)
* removed unnecessary diagnostics which cause MAVlink corruption
* added gyro drift compensation, parameter HAL_CORRECT_GYRO is a integrator time in seconds (30 is a good starting point).
* added diagnosis of the cause of failsafe triggering
* Parameter HAL_RC_FS now sets a time of RC failsafe in seconds (60 is a good value for digital protocols)
* added MAVlink messages about SD card errors
* renamed board *_MP32V1F4 to *_Revolution to simplify things
* fixed bug in RC_Input that cause permanent Failsafe
* added motor clipping reporting and baro compensation by GPS from https://github.com/DuraCopter/ardupilot
* in case of any HardFault or Panic() in armed state, kill a current task and resume (or reboot a FC) instead of hang
* ...
* a lot of minor enhancements






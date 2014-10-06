#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "HAL_Linux_Class.h"
#include "AP_HAL_Linux_Private.h"

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <AP_HAL_Empty.h>
#include <AP_HAL_Empty_Private.h>

using namespace Linux;

// 3 serial ports on Linux for now
static LinuxUARTDriver uartADriver(true);
static LinuxUARTDriver uartBDriver(false);
static LinuxUARTDriver uartCDriver(false);

static LinuxSemaphore  i2cSemaphore;
static LinuxI2CDriver  i2cDriver(&i2cSemaphore, "/dev/i2c-1");
static LinuxSPIDeviceManager spiDeviceManager;
static LinuxAnalogIn analogIn;
static LinuxStorage storageDriver;
static LinuxGPIO gpioDriver;

/*
  use the PRU based RCInput driver on ERLE and PXF
 */
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
static LinuxRCInput_PRU rcinDriver;
#else
static LinuxRCInput rcinDriver;
#endif

/*
  use the PRU based RCOutput driver on ERLE and PXF
 */
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
static LinuxRCOutput_PRU rcoutDriver;
#else
static Empty::EmptyRCOutput rcoutDriver;
#endif
static LinuxScheduler schedulerInstance;
static LinuxUtil utilInstance;

HAL_Linux::HAL_Linux() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        NULL,            /* no uartD */
        NULL,            /* no uartE */
        &i2cDriver,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance)
{}

void _usage(void)
{
    printf("Usage: -A uartAPath -B uartBPath -C uartCPath\n");
    printf("Options:\n");
    printf("\t-serial:          -A /dev/ttyO4\n");
    printf("\t                  -B /dev/ttyS1\n");    
    printf("\t-tcp:             -C tcp:192.168.2.15:1243:wait\n");
    printf("\t                  -A tcp:11.0.0.2:5678\n");    
}

void HAL_Linux::init(int argc,char* const argv[]) const 
{
    int opt;
    /*
      parse command line options
     */
    while ((opt = getopt(argc, argv, "A:B:C:h")) != -1) {
        switch (opt) {
        case 'A':
            uartADriver.set_device_path(optarg);
            break;
        case 'B':
            uartBDriver.set_device_path(optarg);
            break;
        case 'C':
            uartCDriver.set_device_path(optarg);
            break;
        case 'h':
            _usage();
            exit(0);
        default:
            printf("Unknown option '%c'\n", (char)opt);
            exit(1);
        }
    }

    scheduler->init(NULL);
    gpio->init();
    rcout->init(NULL);
    rcin->init(NULL);
    uartA->begin(115200);
    i2c->begin();
    spi->init(NULL);
    utilInstance.init(argc, argv);
}

const HAL_Linux AP_HAL_Linux;

#endif

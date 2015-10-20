#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <assert.h>

#include "HAL_Linux_Class.h"
#include "AP_HAL_Linux_Private.h"

#include <AP_HAL/utility/getopt_cpp.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

using namespace Linux;

// 3 serial ports on Linux for now
static UARTDriver uartADriver(true);
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
static SPIUARTDriver uartBDriver;
#else
static UARTDriver uartBDriver(false);
#endif
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
static RPIOUARTDriver uartCDriver;
#else
static UARTDriver uartCDriver(false);
#endif
static UARTDriver uartEDriver(false);

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
static Semaphore  i2cSemaphore0;
static I2CDriver  i2cDriver0(&i2cSemaphore0, "/dev/i2c-0");
static Semaphore  i2cSemaphore1;
static I2CDriver  i2cDriver1(&i2cSemaphore1, "/dev/i2c-1");
static Semaphore  i2cSemaphore2;
static I2CDriver  i2cDriver2(&i2cSemaphore2, "/dev/i2c-2");
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
static Semaphore  i2cSemaphore0;
static I2CDriver  i2cDriver0(&i2cSemaphore0, "/dev/i2c-2");
#else
static Semaphore  i2cSemaphore0;
static I2CDriver  i2cDriver0(&i2cSemaphore0, "/dev/i2c-1");
#endif
static SPIDeviceManager spiDeviceManager;
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
static NavioAnalogIn analogIn;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
static RaspilotAnalogIn analogIn;
#else
static AnalogIn analogIn;
#endif

/*
  select between FRAM and FS
 */
#if LINUX_STORAGE_USE_FRAM == 1
static Storage_FRAM storageDriver;
#else
static Storage storageDriver;
#endif

/*
  use the BBB gpio driver on ERLE, PXF and BBBMINI
 */
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
static GPIO_BBB gpioDriver;
/*
  use the RPI gpio driver on Navio
 */
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
static GPIO_RPI gpioDriver;
#else
static Empty::EmptyGPIO gpioDriver;
#endif

/*
  use the PRU based RCInput driver on ERLE and PXF
 */
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD
static RCInput_PRU rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
static RCInput_AioPRU rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
static RCInput_Navio rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
static RCInput_Raspilot rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ZYNQ
static RCInput_ZYNQ rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
static RCInput_UDP  rcinDriver;
#else
static RCInput rcinDriver;
#endif

/*
  use the PRU based RCOutput driver on ERLE and PXF
 */
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD
static RCOutput_PRU rcoutDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
static RCOutput_AioPRU rcoutDriver;
/*
  use the PCA9685 based RCOutput driver on Navio
 */
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
static RCOutput_PCA9685 rcoutDriver(PCA9685_PRIMARY_ADDRESS, true, 3, RPI_GPIO_27);
/*
 use the STM32 based RCOutput driver on Raspilot
 */
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
static RCOutput_Raspilot rcoutDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ZYNQ
static RCOutput_ZYNQ rcoutDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
static RCOutput_Bebop rcoutDriver;
#else
static Empty::EmptyRCOutput rcoutDriver;
#endif

static Scheduler schedulerInstance;
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
static UtilRPI utilInstance;
#else
static Util utilInstance;
#endif

HAL_Linux::HAL_Linux() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        NULL,            /* no uartD */
        &uartEDriver,
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
        &i2cDriver0,
        &i2cDriver1,
        &i2cDriver2,
#else
        &i2cDriver0,
        NULL,
        NULL,
#endif
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
    printf("\t                  -A udp:11.0.0.2:5678\n");    
    printf("\t-custom log path:\n");        
    printf("\t                  --log-directory /var/APM/logs\n");
    printf("\t                  -l /var/APM/logs\n");
    printf("\t-custom terrain path:\n");
    printf("\t                   --terrain-directory /var/APM/terrain\n");
    printf("\t                   -t /var/APM/terrain\n");
}

void HAL_Linux::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    int opt;
    const struct GetOptLong::option options[] = {
        {"uartA",         true,  0, 'A'},
        {"uartB",         true,  0, 'B'},
        {"uartC",         true,  0, 'C'},
        {"uartE",         true,  0, 'E'},
        {"log-directory",       true,  0, 'l'},
        {"terrain-directory",   true,  0, 't'},
        {"help",                false,  0, 'h'},
        {0, false, 0, 0}
    };

    GetOptLong gopt(argc, argv, "A:B:C:E:l:t:h",
                    options);

    /*
      parse command line options
     */
    while ((opt = gopt.getoption()) != -1) {
        switch (opt) {
        case 'A':
            uartADriver.set_device_path(gopt.optarg);
            break;
        case 'B':
            uartBDriver.set_device_path(gopt.optarg);
            break;
        case 'C':
            uartCDriver.set_device_path(gopt.optarg);
            break;
        case 'E':
            uartEDriver.set_device_path(gopt.optarg);
            break;
        case 'l':            
            utilInstance.set_custom_log_directory(gopt.optarg);
            break;
        case 't':
            utilInstance.set_custom_terrain_directory(gopt.optarg);
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
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
    i2c->begin();
    i2c1->begin();
    i2c2->begin();
#else
    i2c->begin();
#endif
    spi->init(NULL);
    rcout->init(NULL);
    rcin->init(NULL);
    uartA->begin(115200);    
    uartE->begin(115200);    
    analogin->init(NULL);
    utilInstance.init(argc+gopt.optind-1, &argv[gopt.optind-1]);

    // NOTE: See commit 9f5b4ffca ("AP_HAL_Linux_Class: Correct
    // deadlock, and infinite loop in setup()") for details about the
    // order of scheduler initialize and setup on Linux.
    scheduler->system_initialized();
    callbacks->setup();

    for (;;) {
        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_Linux hal;
    return hal;
}

#endif

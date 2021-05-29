#include "HAL_Linux_Class.h"

#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RCOutput_Tap.h>
#include <AP_HAL/utility/getopt_cpp.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_Module/AP_Module.h>

#include "AnalogIn_ADS1115.h"
#include "AnalogIn_IIO.h"
#include "AnalogIn_Navio2.h"
#include "GPIO.h"
#include "I2CDevice.h"
#include "OpticalFlow_Onboard.h"
#include "RCInput.h"
#include "RCInput_AioPRU.h"
#include "RCInput_Navio2.h"
#include "RCInput_PRU.h"
#include "RCInput_RPI.h"
#include "RCInput_SoloLink.h"
#include "RCInput_UART.h"
#include "RCInput_UDP.h"
#include "RCInput_Multi.h"
#include "RCInput_ZYNQ.h"
#include "RCInput_RCProtocol.h"
#include "RCOutput_AioPRU.h"
#include "RCOutput_Bebop.h"
#include "RCOutput_Disco.h"
#include "RCOutput_PCA9685.h"
#include "RCOutput_PRU.h"
#include "RCOutput_Sysfs.h"
#include "RCOutput_ZYNQ.h"
#include "SPIDevice.h"
#include "SPIUARTDriver.h"
#include "Scheduler.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "Util.h"
#include "Util_RPI.h"
#include "CANSocketIface.h"

using namespace Linux;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR
static UtilRPI utilInstance;
#else
static Util utilInstance;
#endif

// 5 serial ports on Linux
static UARTDriver uartADriver(true);
static UARTDriver uartCDriver(false);
static UARTDriver uartDDriver(false);
static UARTDriver uartEDriver(false);
static UARTDriver uartFDriver(false);
static UARTDriver uartGDriver(false);
static UARTDriver uartHDriver(false);
static UARTDriver uartIDriver(false);

static I2CDeviceManager i2c_mgr_instance;
static SPIDeviceManager spi_mgr_instance;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
static SPIUARTDriver uartBDriver;
#else
static UARTDriver uartBDriver(false);
#endif

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR
static AnalogIn_ADS1115 analogIn;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
static AnalogIn_IIO analogIn;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2 || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE
static AnalogIn_Navio2 analogIn;
#else
static Empty::AnalogIn analogIn;
#endif

static Storage storageDriver;

/*
  use the BBB gpio driver on ERLE, PXF, BBBMINI, BLUE and PocketPilot
 */
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
static GPIO_BBB gpioDriver;
/*
  use the RPI gpio driver on Navio
 */
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR
static GPIO_RPI gpioDriver;
#elif  CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
       CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2 || \
       CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE
static GPIO_Sysfs gpioDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_AERO
static GPIO_Sysfs gpioDriver;
#else
static Empty::GPIO gpioDriver;
#endif

/*
  use the PRU based RCInput driver on ERLE and PXF
 */
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD
static RCInput_PRU rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
static RCInput_AioPRU rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
static RCInput_Multi rcinDriver{2, new RCInput_AioPRU, new RCInput_RCProtocol(NULL, "/dev/ttyO4")};
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI
static RCInput_RPI rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ZYNQ || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_OCPOC_ZYNQ
static RCInput_ZYNQ rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
static RCInput_UDP  rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
static RCInput_Multi rcinDriver{2, new RCInput_RCProtocol("/dev/uart-sbus", "/dev/uart-sumd"), new RCInput_UDP()};
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_AERO
static RCInput_SoloLink rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2 || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE
static RCInput_Navio2 rcinDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ
static RCInput_RCProtocol rcinDriver{"/dev/ttyPS0", NULL};
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_VNAV
// this is needed to allow for RC input using SERIALn_PROTOCOL=23. No fd is opened
// in the linux driver and instead user needs to provide a uart via SERIALn_PROTOCOL
static RCInput_RCProtocol rcinDriver{nullptr, nullptr};
#else
static RCInput rcinDriver;
#endif

/*
  use the PRU based RCOutput driver on ERLE and PXF
 */
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD
static RCOutput_PRU rcoutDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
static RCOutput_AioPRU rcoutDriver;
/*
  use the PCA9685 based RCOutput driver on Navio and Erle-Brain 2
 */
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2  || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI
static RCOutput_PCA9685 rcoutDriver(i2c_mgr_instance.get_device(1, PCA9685_PRIMARY_ADDRESS), 24576000, 3, RPI_GPIO_<27>());
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
static RCOutput_PCA9685 rcoutDriver(i2c_mgr_instance.get_device(1, PCA9685_PRIMARY_ADDRESS), 24576000, 3, NAVIO_GPIO_PCA_OE);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
static RCOutput_PCA9685 rcoutDriver(i2c_mgr_instance.get_device(1, PCA9685_QUATENARY_ADDRESS), 0, 0, RPI_GPIO_<4>());
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK
static RCOutput_PCA9685 rcoutDriver(i2c_mgr_instance.get_device(1, PCA9685_QUINARY_ADDRESS), 0, 0, RPI_GPIO_<27>());
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR
static RCOutput_PCA9685 rcoutDriver(i2c_mgr_instance.get_device(4, PCA9685_PRIMARY_ADDRESS), 24576000, 0, RPI_GPIO_<26>());
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ZYNQ || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_OCPOC_ZYNQ
static RCOutput_ZYNQ rcoutDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
static RCOutput_Bebop rcoutDriver(i2c_mgr_instance.get_device(HAL_RCOUT_BEBOP_BLDC_I2C_BUS, HAL_RCOUT_BEBOP_BLDC_I2C_ADDR));
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
static RCOutput_Disco rcoutDriver(i2c_mgr_instance.get_device(HAL_RCOUT_DISCO_BLDC_I2C_BUS, HAL_RCOUT_DISCO_BLDC_I2C_ADDR));
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2
static RCOutput_Sysfs rcoutDriver(0, 0, 14);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_AERO
static ap::RCOutput_Tap rcoutDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE
static RCOutput_Sysfs rcoutDriver(0, 0, 15);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ
static RCOutput_Sysfs rcoutDriver(0, 0, 8);
#else
static Empty::RCOutput rcoutDriver;
#endif

static Scheduler schedulerInstance;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
static OpticalFlow_Onboard opticalFlow;
#else
static Empty::OpticalFlow opticalFlow;
#endif

static Empty::DSP dspDriver;
static Empty::Flash flashDriver;

#if HAL_NUM_CAN_IFACES
static CANIface* canDrivers[HAL_NUM_CAN_IFACES];
#endif

HAL_Linux::HAL_Linux() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &uartEDriver,
        &uartFDriver,
        &uartGDriver,
        &uartHDriver,
        &uartIDriver,
        &i2c_mgr_instance,
        &spi_mgr_instance,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlow,
        &flashDriver,
        &dspDriver,
#if HAL_NUM_CAN_IFACES
        (AP_HAL::CANIface**)canDrivers
#else
        nullptr
#endif
        )
{}

void _usage(void)
{
    printf("Usage: -A uartAPath -B uartBPath -C uartCPath -D uartDPath -E uartEPath -F uartFPath -G uartGpath -H uartHpath -I uartIpath\n");
    printf("Options:\n");
    printf("\tserial:\n");
    printf("                    -A /dev/ttyO4\n");
    printf("\t                  -B /dev/ttyS1\n");
    printf("\tnetworking tcp:\n");
    printf("\t                  -C tcp:192.168.2.15:1243:wait\n");
    printf("\t                  -A tcp:11.0.0.2:5678\n");
    printf("\t                  -A udp:11.0.0.2:14550\n");
    printf("\tnetworking UDP:\n");
    printf("\t                  -A udp:11.0.0.255:14550:bcast\n");
    printf("\t                  -A udpin:0.0.0.0:14550\n");
    printf("\tcustom log path:\n");
    printf("\t                  --log-directory /var/APM/logs\n");
    printf("\t                  -l /var/APM/logs\n");
    printf("\tcustom terrain path:\n");
    printf("\t                   --terrain-directory /var/APM/terrain\n");
    printf("\t                   -t /var/APM/terrain\n");
    printf("\tcustom storage path:\n");
    printf("\t                   --storage-directory /var/APM/storage\n");
    printf("\t                   -s /var/APM/storage\n");
#if AP_MODULE_SUPPORTED
    printf("\tmodule support:\n");
    printf("\t                   --module-directory %s\n", AP_MODULE_DEFAULT_DIRECTORY);
    printf("\t                   -M %s\n", AP_MODULE_DEFAULT_DIRECTORY);
#endif
}

void HAL_Linux::run(int argc, char* const argv[], Callbacks* callbacks) const
{
#if AP_MODULE_SUPPORTED
    const char *module_path = AP_MODULE_DEFAULT_DIRECTORY;
#endif
    
    int opt;
    const struct GetOptLong::option options[] = {
        {"uartA",         true,  0, 'A'},
        {"uartB",         true,  0, 'B'},
        {"uartC",         true,  0, 'C'},
        {"uartD",         true,  0, 'D'},
        {"uartE",         true,  0, 'E'},
        {"uartF",         true,  0, 'F'},
        {"uartG",         true,  0, 'G'},
        {"uartH",         true,  0, 'H'},
        {"uartI",         true,  0, 'I'},
        {"log-directory",       true,  0, 'l'},
        {"terrain-directory",   true,  0, 't'},
        {"storage-directory",   true,  0, 's'},
        {"module-directory",    true,  0, 'M'},
        {"defaults",            true,  0, 'd'},
        {"help",                false,  0, 'h'},
        {0, false, 0, 0}
    };

    GetOptLong gopt(argc, argv, "A:B:C:D:E:F:G:H:l:t:s:he:SM:",
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
        case 'D':
            uartDDriver.set_device_path(gopt.optarg);
            break;
        case 'E':
            uartEDriver.set_device_path(gopt.optarg);
            break;
        case 'F':
            uartFDriver.set_device_path(gopt.optarg);
            break;
        case 'G':
            uartGDriver.set_device_path(gopt.optarg);
            break;
        case 'H':
            uartHDriver.set_device_path(gopt.optarg);
            break;
        case 'I':
            uartIDriver.set_device_path(gopt.optarg);
            break;
        case 'l':
            utilInstance.set_custom_log_directory(gopt.optarg);
            break;
        case 't':
            utilInstance.set_custom_terrain_directory(gopt.optarg);
            break;
        case 's':
            utilInstance.set_custom_storage_directory(gopt.optarg);
            break;
#if AP_MODULE_SUPPORTED
        case 'M':
            module_path = gopt.optarg;
            break;
#endif
        case 'd':
            utilInstance.set_custom_defaults_path(gopt.optarg);
            break;
        case 'h':
            _usage();
            exit(0);
        default:
            printf("Unknown option '%c'\n", (char)opt);
            exit(1);
        }
    }

    setup_signal_handlers();

    scheduler->init();
    gpio->init();
    rcout->init();
    rcin->init();
    serial(0)->begin(115200);
    analogin->init();
    utilInstance.init(argc+gopt.optind-1, &argv[gopt.optind-1]);

    // NOTE: See commit 9f5b4ffca ("AP_HAL_Linux_Class: Correct
    // deadlock, and infinite loop in setup()") for details about the
    // order of scheduler initialize and setup on Linux.
    scheduler->set_system_initialized();

    // possibly load external modules
#if AP_MODULE_SUPPORTED
    if (module_path != nullptr) {
        AP_Module::init(module_path);
    }
#endif

#if AP_MODULE_SUPPORTED
    AP_Module::call_hook_setup_start();
#endif
    callbacks->setup();
#if AP_MODULE_SUPPORTED
    AP_Module::call_hook_setup_complete();
#endif

    while (!_should_exit) {
        callbacks->loop();
    }

    rcin->teardown();
    I2CDeviceManager::from(i2c_mgr)->teardown();
    SPIDeviceManager::from(spi)->teardown();
    Scheduler::from(scheduler)->teardown();
}

void HAL_Linux::setup_signal_handlers() const
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = HAL_Linux::exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
}

HAL_Linux hal_linux;

void HAL_Linux::exit_signal_handler(int signum)
{
    hal_linux._should_exit = true;
}

const AP_HAL::HAL &AP_HAL::get_HAL()
{
    return hal_linux;
}

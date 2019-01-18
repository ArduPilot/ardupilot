/*
(c) 2017 night_ghost@ykoctpa.ru
 
*/

#pragma GCC optimize ("O2")

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT
#include <assert.h>
#include <AP_HAL_F4Light/AP_HAL_F4Light.h>
#include "AP_HAL_F4Light_Namespace.h"
#include "AP_HAL_F4Light_Private.h"
#include "HAL_F4Light_Class.h"
#include "RCInput.h"
#include "Util.h"

#include <AP_Param_Helper/AP_Param_Helper.h>

#include "UART_PPM.h"

#if defined(USE_SOFTSERIAL)
#include "UART_SoftDriver.h"
#endif



#if defined(BOARD_SDCARD_NAME) || defined(BOARD_DATAFLASH_FATFS)
#include "sd/SD.h"
#endif

#if defined(BOARD_OSD_CS_PIN)
#include "UART_OSD.h"
#endif

#if defined(USB_MASSSTORAGE) 
#include "massstorage/mass_storage.h"
#endif

using namespace F4Light;


static F4Light::I2CDeviceManager i2c_mgr_instance;

// XXX make sure these are assigned correctly
static USBDriver          USB_Driver(1);        // ACM
static UARTDriver uart1Driver(_USART1); // main port

#if defined(BOARD_USART6_RX_PIN) && defined(BOARD_USART6_TX_PIN)
static UARTDriver uart6Driver(_USART6); // pin 7&8(REVO)/5&6(RevoMini) of input port
#endif


#ifdef BOARD_HAS_UART3
 static UARTDriver uart3Driver(_USART3); // flexi port
// static SerialDriver IN_CCM softDriver(BOARD_SOFTSERIAL_TX, BOARD_SOFTSERIAL_RX, false);  // pin 7&8 of input port
#endif

#if defined(BOARD_SOFT_UART3) || defined(USE_SOFTSERIAL)
 static SerialDriver IN_CCM softDriver(BOARD_SOFTSERIAL_TX, BOARD_SOFTSERIAL_RX, false);
#endif


#ifdef BOARD_OSD_CS_PIN
 static UART_OSD uartOSDdriver;
#endif

#if defined(BOARD_USART4_RX_PIN) && defined( BOARD_USART4_TX_PIN)
 static UARTDriver uart4Driver(_UART4);  // pin 5&6 of servo port
#endif

#if defined(BOARD_USART5_RX_PIN) && defined( BOARD_USART5_TX_PIN)
 static UARTDriver uart5Driver(_UART5);  
#endif

static UART_PPM uartPPM2(1); // PPM2 input
static UART_PPM uartPPM1(0); // PPM1 input 



// only for DSM satellite, served in rc_input
//static UARTDriver uart5Driver(_UART5,0);  // D26/PD2  6 EXTI_RFM22B / UART5_RX  input-only UART for DSM satellite

/*
        input port pinout
        1    2    3    4    5    6    7   8
pin              b14  b15  c6   c7   c8   c9
       gnd  vcc  PPM  buzz 6_tx 6_rx Sscl Ssda
USE_SOFTSERIAL ->                    S_tx S_rx
servos ->        srv7 srv8 srv9 srv10 srv11 srv12 
*/


/* OPLINK AIR port pinout
1       2       3       4       5       6       7
                        
gnd    +5      26      103                     
               rx      pow
*/

static F4Light::SPIDeviceManager spiDeviceManager;
static AnalogIn  analogIn IN_CCM;
static Storage   storageDriver;
static GPIO      gpioDriver;
static RCInput   rcinDriver;
static RCOutput  rcoutDriver;
static Scheduler schedulerInstance;
static Util      utilInstance;

HAL_state HAL_F4Light::state;

//const AP_HAL::UARTDriver** HAL_F4Light::uarts[] = { &uartA, &uartB, &uartC, &uartD, &uartE, &uartF };

/*
        AP_HAL::UARTDriver* _uartA, // console
        AP_HAL::UARTDriver* _uartB, // 1st GPS
        AP_HAL::UARTDriver* _uartC, // telem1
        AP_HAL::UARTDriver* _uartD, // telem2
        AP_HAL::UARTDriver* _uartE, // 2nd GPS
        AP_HAL::UARTDriver* _uartF, // extra1


AP_SerialManager.cpp line 159
    // initialise pointers to serial ports
    state[1].uart = hal.uartC;  // serial1, uartC, normally telem1
    state[2].uart = hal.uartD;  // serial2, uartD, normally telem2
    state[3].uart = hal.uartB;  // serial3, uartB, normally 1st GPS
    state[4].uart = hal.uartE;  // serial4, uartE, normally 2nd GPS
    state[5].uart = hal.uartF;  // serial5



*/



HAL_F4Light::HAL_F4Light() :
    AP_HAL::HAL(
        &USB_Driver,            // uartA - USB console                                         - Serial0

#if   BOARD_UARTS_LAYOUT == 1 // Revolution

        &uart6Driver,           // uartB - pin 7&8(REVO)/5&6(RevoMini) of input port - for GPS - Serial3
        &uart1Driver,           // uartC - main port  - for telemetry                          - Serial1
        &uart3Driver,           // uartD - flexi port                                          - Serial2 
        &uart4Driver,           // uartE - PWM pins 5&6                                        - Serial4
        &softDriver,            // uartF - soft UART on pins 7&8                               - Serial5 

#elif BOARD_UARTS_LAYOUT == 2 // Airbot

        &uart6Driver,           // uartB - pin 7&8(REVO)/5&6(RevoMini) of input port - for GPS - Serial3
        &uart1Driver,           // uartC - main port  - for telemetry                          - Serial1
        &uart4Driver,           // uartD - PWM pins 5&6                                        - Serial2 
        &uartPPM2,              // uartE - input data from PPM2 pin                            - Serial4
        &uartPPM1,              // uartF - input data from PPM1 pin                            - Serial5 
        
#elif BOARD_UARTS_LAYOUT == 3 // AirbotV2

        &uart6Driver,           // uartB - pin 7&8(REVO)/5&6(RevoMini) of input port - for GPS - Serial3
        &uart1Driver,           // uartC - main port  - for telemetry                          - Serial1
        &uartOSDdriver,         // uartD - OSD emulated UART                                   - Serial2
        &uart4Driver,           // uartE - PWM pins 5&6                                        - Serial4 
        &uartPPM2,              // uartF - input data from PPM2 pin                            - Serial5

#elif BOARD_UARTS_LAYOUT == 4 // MiniOSD

        &uart6Driver,           // uartB - pin 7&8(REVO)/5&6(RevoMini) of input port - for GPS - Serial3
        &uart1Driver,           // uartC - main port  - for telemetry                          - Serial1
        NULL,
        NULL,
        NULL,
        
#elif BOARD_UARTS_LAYOUT == 5 // MicroOSD

        &uart6Driver,           // uartB - pin 7&8(REVO)/5&6(RevoMini) of input port - for GPS - Serial3
        &uart1Driver,           // uartC - main port  - for telemetry                          - Serial1
        NULL,
        NULL,
        NULL,
        
#elif BOARD_UARTS_LAYOUT == 6 // MatekF405_CTR

        &uart4Driver,           // uartB - UART4 for GPS                                       - Serial3
        &uart1Driver,           // uartC - UART1 for telemetry                                 - Serial1
        &uartOSDdriver,         // uartD - OSD emulated UART                                   - Serial2
        &uart3Driver,           // uartE - UART3                                               - Serial4 
        &uart5Driver,           // uartF - UART5                                               - Serial5
        
#elif BOARD_UARTS_LAYOUT == 7 // Cl_Racing F4

        &uart6Driver,           // uartB - pin 7&8(REVO)/5&6(RevoMini) of input port - for GPS - Serial3
        &uart1Driver,           // uartC - main port  - for telemetry                          - Serial1
        &uartOSDdriver,         // uartD - OSD emulated UART                                   - Serial2
        &uartPPM1,              // uartE - input data from PPM1 pin                            - Serial4
        NULL,                   // no uartF
        
#else
 #error no BOARD_UARTS_LAYOUT!
#endif
        nullptr,            // no uartG
        
        &i2c_mgr_instance,  // I2C
        &spiDeviceManager,  // spi 
        &analogIn,          // analogin 
        &storageDriver,     // storage 
        &HAL_CONSOLE,       // console via radio or USB on per-board basis 
        &gpioDriver,        // gpio 
        &rcinDriver,        // rcinput 
        &rcoutDriver,       // rcoutput 
        &schedulerInstance, // scheduler 
        &utilInstance,	    // util 
        nullptr,            // no onboard optical flow 
        nullptr             // no CAN 
    )
    
           //  0     1        2        3        4         5
           // USB    Main   Flexi/OSD  Uart4   UART6    Soft/Uart4
    , uarts{ &uartA, &uartC, &uartD,   &uartE, &uartB,   &uartF }

{

    uint32_t sig = board_get_rtc_register(RTC_CONSOLE_REG);
    if( (sig & ~CONSOLE_PORT_MASK) == CONSOLE_PORT_SIGNATURE) {
        AP_HAL::UARTDriver** up = uarts[sig & CONSOLE_PORT_MASK];
        if(up){
            console =  *up;
        }
    }
}

extern const AP_HAL::HAL& hal;


void HAL_F4Light::run(int argc,char* const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */

    scheduler->init();
    
    uint32_t start_t = AP_HAL::millis();

    gpio->init();

    rcout->init(); 
    
    usart_disable_all();

    {
#if defined(USB_MASSSTORAGE)
        uint32_t sig = board_get_rtc_register(RTC_MASS_STORAGE_REG);
        if( sig == MASS_STORAGE_SIGNATURE) {
            board_set_rtc_register(0, RTC_MASS_STORAGE_REG);

#if defined(BOARD_SDCARD_NAME) && defined(BOARD_SDCARD_CS_PIN)
            SD.begin(F4Light::SPIDeviceManager::_get_device(BOARD_SDCARD_NAME));
#elif defined(BOARD_DATAFLASH_FATFS)
            SD.begin(F4Light::SPIDeviceManager::_get_device(HAL_DATAFLASH_NAME));
#endif
            state.sd_busy=true;
            massstorage.setup();        //      init USB as mass-storage
        } 
        else 
#endif
        {
            extern void usb_init();     //      init as VCP
            usb_init(); // moved from boards.cpp

            uartA->begin(115200); // uartA is the USB serial port used for the console, so lets make sure it is initialized at boot 
        }
    }
    

    if(console != uartA) {
        console->begin(57600);  // init telemetry port as console
    }


    rcin->init();

    storage->init(); // Uses EEPROM.*, flash_stm* reworked
    analogin->init();

    printf("\nHAL startup  at %ldms\n", start_t);            

    if(!state.sd_busy) {


#if defined(BOARD_SDCARD_NAME) && defined(BOARD_SDCARD_CS_PIN)
        printf("\nEnabling SD at %ldms\n", AP_HAL::millis());            
        SD.begin(F4Light::SPIDeviceManager::_get_device(BOARD_SDCARD_NAME));
#elif defined(BOARD_DATAFLASH_FATFS)
        printf("\nEnabling AP_Logger as SD at %ldms\n", AP_HAL::millis());            
        SD.begin(F4Light::SPIDeviceManager::_get_device(HAL_DATAFLASH_NAME));
#endif

#if defined(BOARD_OSD_NAME)
        uartOSDdriver.begin(57600); // init OSD after SD but before call to lateInit(), but only if not in USB_STORAGE
#endif

    }


    printf("\nHAL init done at %ldms\n", AP_HAL::millis());

    callbacks->setup();

#if 0 //[ here is too late :( so we need a small hack and call lateInit from Scheduler::register_delay_callback 
//         which called when parameters already initialized
    
    lateInit();
#endif //]

    
    scheduler->system_initialized(); // clear bootloader flag

    Scheduler::start_stats_task(); 

    printf("\nLoop started at %ldms\n", AP_HAL::millis());            


// main application loop hosted here!
    for (;;) {
        callbacks->loop();
//        ((F4Light::Scheduler *)scheduler)->loop(); // to execute stats in main loop
//        ((F4Light::RCInput *)rcin)->loop(); // to execute debug in main loop
    }
}


static bool lateInitDone=false;

void HAL_F4Light::lateInit() {
    
    if(lateInitDone) return;
    
    lateInitDone=true;

    {
        uint32_t sig = board_get_rtc_register(RTC_CONSOLE_REG);
        uint8_t port = hal_param_helper->_console_uart;

        if(port < sizeof(uarts)/sizeof(AP_HAL::UARTDriver**) ){

            if( (sig & ~CONSOLE_PORT_MASK) == CONSOLE_PORT_SIGNATURE) {
                if(port != (sig & CONSOLE_PORT_MASK)) { // wrong console - reboot needed
                    board_set_rtc_register(CONSOLE_PORT_SIGNATURE | (port & CONSOLE_PORT_MASK), RTC_CONSOLE_REG);
                    board_set_rtc_register(FORCE_APP_RTC_SIGNATURE, RTC_SIGNATURE_REG); // force bootloader to not wait
                    Scheduler::_reboot(false);
                }
            } else { // no signature - set and check console
                board_set_rtc_register(CONSOLE_PORT_SIGNATURE | (port & CONSOLE_PORT_MASK), RTC_CONSOLE_REG);

                AP_HAL::UARTDriver** up = uarts[port];
                if(up && *up != console) {
                    board_set_rtc_register(FORCE_APP_RTC_SIGNATURE, RTC_SIGNATURE_REG); // force bootloader to not wait
                    Scheduler::_reboot(false);
                }
            }
        }
    }
    {
        uint32_t g   = board_get_rtc_register(RTC_OV_GUARD_REG);
        uint32_t sig = board_get_rtc_register(RTC_OVERCLOCK_REG);
        uint8_t  oc  = hal_param_helper->_overclock;
        
        if(g==OV_GUARD_FAIL_SIGNATURE) {
            if(oc==0) {
                board_set_rtc_register(OVERCLOCK_SIGNATURE | oc, RTC_OVERCLOCK_REG); // set default clock
                board_set_rtc_register(0, RTC_OV_GUARD_REG);                    // and reset failure
            } else printf("\noverclocking failed!\n");            
        } else {

            if((sig & ~OVERCLOCK_SIG_MASK ) == OVERCLOCK_SIGNATURE ) { // if correct signature
                board_set_rtc_register(OVERCLOCK_SIGNATURE | oc, RTC_OVERCLOCK_REG); // set required clock in any case
                if((sig & OVERCLOCK_SIG_MASK) != oc) {                 // if wrong clock
                    board_set_rtc_register(FORCE_APP_RTC_SIGNATURE, RTC_SIGNATURE_REG); // force bootloader to not wait
                    Scheduler::_reboot(false);                 //  then reboot required
                } 
            } else { // no signature, write only if needed
                if(oc) board_set_rtc_register(OVERCLOCK_SIGNATURE | oc, RTC_OVERCLOCK_REG); // set required clock
            }
        }
    }
    
    { // one-time connection to COM-port
        uint8_t conn = hal_param_helper->_connect_com;
        if(conn) {
            hal_param_helper->_connect_com = 0;
            hal_param_helper->_connect_com.save();
            
            if(conn < sizeof(uarts)/sizeof(AP_HAL::UARTDriver**) ){
                AP_HAL::UARTDriver** up = uarts[conn];
                if(up && *up){
                    connect_uart(uartA,*up, NULL);
                }
            } else printf("\nWrong HAL_CONNECT_COM selected!");
    
        }
    }

#if defined(USB_MASSSTORAGE)
    if(!state.sd_busy){    
        uint8_t conn=hal_param_helper->_usb_storage;
        
        if(conn){
            hal_param_helper->_usb_storage=0;
            hal_param_helper->_usb_storage.save();

            board_set_rtc_register(MASS_STORAGE_SIGNATURE, RTC_MASS_STORAGE_REG);
            board_set_rtc_register(FORCE_APP_RTC_SIGNATURE, RTC_SIGNATURE_REG); // force bootloader to not wait
            Scheduler::_reboot(false);
        }
    }
#endif

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
    { // one-time connection to ESC
        uint8_t conn = hal_param_helper->_connect_esc;

        if(conn){
            hal_param_helper->_connect_esc = 0;
            hal_param_helper->_connect_esc.save();

            conn-=1;

            if(conn < sizeof(uarts)/sizeof(AP_HAL::UARTDriver**) ){
                AP_HAL::UARTDriver** up = uarts[conn];
                if(up && *up){
                    RCOutput::do_4way_if(*up);
                }
            } else printf("\nWrong HAL_CONNECT_ESC selected!");
        }
    }
#endif

    RCOutput::lateInit(); // 2nd stage - now with loaded parameters

    // all another parameter-dependent inits

#ifdef BOARD_I2C_FLEXI
    if(hal_param_helper->_flexi_i2c) {
        uart3Driver.end();
        uart3Driver.disable(); // uses Flexi port occupied by I2C 
        printf("\nUART3 disabled by I2C2\n");            
    }
#endif
    I2CDevice::lateInit();

    Storage::late_init(hal_param_helper->_eeprom_deferred); 
    
    uint8_t flags=0;

    if(hal_param_helper->_rc_fs)      flags |= BOARD_RC_FAILSAFE;

    RCInput::late_init(flags);
}

// 57600 gives ~6500 chars per second or 6 chars per ms
void HAL_F4Light::connect_uart(AP_HAL::UARTDriver* uartL,AP_HAL::UARTDriver* uartR, AP_HAL::Proc proc){
    while(1){
        bool got=false;
        if(uartL->available() && uartR->txspace()) { uartR->write(uartL->read()); got=true; }
        if(uartR->available() && uartL->txspace()) { uartL->write(uartR->read()); got=true; }
        if(proc) proc();
        if(!got) Scheduler::yield(100); // give a chance to other threads
        if(state.disconnect) break; // if USB disconnected
    }
}

static const HAL_F4Light hal_revo;

const AP_HAL::HAL& AP_HAL::get_HAL() {
    return hal_revo;
}


extern "C" void usb_mass_mal_USBdisconnect();

void usb_mass_mal_USBdisconnect(){ 
    HAL_F4Light::state.sd_busy=false;
    HAL_F4Light::state.disconnect=true;
}

#endif

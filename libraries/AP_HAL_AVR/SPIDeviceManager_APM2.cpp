#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2

#include <avr/io.h>
#include <AP_HAL/AP_HAL.h>
#include "SPIDriver.h"
#include "SPIDevices.h"
#include "utility/pins_arduino_mega.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

void APM2SPIDeviceManager::init(void* machtnichts) {

    /* Note that the order of the init() of the MS5611 and MPU6k is
     * critical for the APM2. If you initialise in the wrong order
     * then the MS5611 doesn't initialise itself correctly. This
     * indicates an electrical fault in the APM2 which needs to be
     * investigated. Meanwhile, initialising the MPU6k CS pin before
     * the MS5611 CS pin works around the problem
     */

    #define SPI0_SPCR_8MHz   0
    #define SPI0_SPSR_8MHz   _BV(SPI2X)
    #define SPI0_SPCR_500kHz _BV(SPR1)
    #define SPI0_SPSR_500kHz _BV(SPI2X)

    /* mpu6k cs is on Arduino pin 53, PORTB0 */
    AVRDigitalSource* mpu6k_cs = new AVRDigitalSource(_BV(0), PB);
    /* mpu6k: run clock at 8MHz in high speed mode and 512kHz for low
     * speed */
    _mpu6k = new AVRSPI0DeviceDriver(mpu6k_cs, SPI0_SPCR_500kHz, SPI0_SPCR_8MHz, SPI0_SPSR_8MHz);
    _mpu6k->init();

    /* ms5611 cs is on Arduino pin 40, PORTG1 */
    AVRDigitalSource* ms5611_cs = new AVRDigitalSource(_BV(1), PG);
    /* ms5611: run clock at 8MHz */
    _ms5611 = new AVRSPI0DeviceDriver(ms5611_cs, SPI0_SPCR_500kHz, SPI0_SPCR_8MHz, SPI0_SPSR_8MHz);
    _ms5611->init();
   
    /* optflow cs is on Arduino pin A3, PORTF3 */
    AVRDigitalSource* optflow_cs = new AVRDigitalSource(_BV(3), PF);
    /* optflow: divide clock by 8 to 2Mhz
     * spcr gets bit SPR0, spsr gets bit SPI2X */
    _optflow_spi0 = new AVRSPI0DeviceDriver(optflow_cs, _BV(SPR0)|_BV(CPOL)|_BV(CPHA), _BV(SPR0)|_BV(CPOL)|_BV(CPHA), _BV(SPI2X));
    _optflow_spi0->init();

    /* Dataflash CS is on Arduino pin 28, PORTA6 */
    AVRDigitalSource* df_cs = new AVRDigitalSource(_BV(6), PA);
    /* dataflash uses mode 0 and a clock of 8mhz
     * ucsr3c = 0 
     * ubrr3 = 0 */
    _dataflash = new AVRSPI3DeviceDriver(df_cs, 0, 0);
    _dataflash->init();

    /* optflow uses mode 3 and a clock of 2mhz
     * ucsr3c = _BV(UCPHA3N)|_BV(UCPOL3) = 3
     * ubrr3 = 3 */
    _optflow_spi3 = new AVRSPI3DeviceDriver(optflow_cs, 3, 3);
    _optflow_spi3->init();
}

AP_HAL::SPIDeviceDriver* APM2SPIDeviceManager::device(enum AP_HAL::SPIDevice d) 
{
    switch (d) {
        case AP_HAL::SPIDevice_Dataflash:
            return _dataflash;
        case AP_HAL::SPIDevice_MS5611:
            return _ms5611;
        case AP_HAL::SPIDevice_MPU6000:
            return _mpu6k;
        case AP_HAL::SPIDevice_ADNS3080_SPI0:
            return _optflow_spi0;
        case AP_HAL::SPIDevice_ADNS3080_SPI3:
            return _optflow_spi3;
        default:
            return NULL;
    };
}

#endif

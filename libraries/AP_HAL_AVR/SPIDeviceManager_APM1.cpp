#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <avr/io.h>
#include <AP_HAL.h>
#include "SPIDriver.h"
#include "SPIDevices.h"
#include "GPIO.h"
#include "pins_arduino_mega.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

void APM1SPIDeviceManager::init(void* machtnichts) {
    /* dataflow cs is on arduino pin 53, PORTB0 */
    AVRDigitalSource* df_cs = new AVRDigitalSource(_BV(0), PB);
    /* dataflash: divide clock by 2 to 8Mhz, set SPI_MODE_3
     * spcr gets 0x0C to set SPI_MODE_3
     * spsr gets bit SPI2X for clock divider */
    _dataflash = new AVRSPI0DeviceDriver(df_cs, 0x0C, _BV(SPI2X));
    _dataflash->init();

    /* optflow cs is on Arduino pin 34, PORTC3 */
    AVRDigitalSource* opt_cs = new AVRDigitalSource(_BV(3), PC);
    /* optflow: divide clock by 8 to 2Mhz
     * spcr gets bit SPR0, spsr gets bit SPI2X */
    _optflow = new AVRSPI0DeviceDriver(opt_cs, _BV(SPR0), 0);
    _optflow->init();

    /* adc cs is on Arduino pin 33, PORTC4 */
    AVRDigitalSource* adc_cs = new AVRDigitalSource(_BV(4), PC);
    /* adc: ubbr2 gets value of 2 to run at 2.6Mhz
     * (config value cribbed from AP_ADC_ADS7844 driver pre-port) */
    _adc = new AVRSPI2DeviceDriver(adc_cs, 0, 2);
    _adc->init();
}

AP_HAL::SPIDeviceDriver* APM1SPIDeviceManager::device(enum AP_HAL::SPIDevice d) 
{
    switch (d) {
        case AP_HAL::SPIDevice_Dataflash:
            return _dataflash;
        case AP_HAL::SPIDevice_ADS7844:
            return _adc;
        case AP_HAL::SPIDevice_ADNS3080_SPI0:
            return _optflow;
        default:
            return NULL;
    };
}

#endif

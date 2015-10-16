
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
// Loopback test for SPI driver
// Connect MISO and MOSI pins together (12 and 13 on Flymaple)

#include <AP_Param/AP_Param.h>
#include <AP_Progmem/AP_Progmem.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_HAL::SPIDeviceDriver* spidev;

void setup (void) {
    hal.scheduler->delay(5000);
    hal.console->printf_P(PSTR("Starting AP_HAL_FLYMAPLE::SPIDriver test\r\n"));

    spidev = hal.spi->device(AP_HAL::SPIDevice_MPU6000); // Not really MPU6000, just a generic SPU driver
    if (!spidev)
       hal.scheduler->panic(PSTR("Starting AP_HAL_FLYMAPLE::SPIDriver failed to get spidev\r\n"));
}

void loop (void) { 
    uint8_t tx_data[] = { 'h', 'e', 'l', 'l', 'o', 0 };
    uint8_t rx_data[] = {  0,   0,   0,   0,   0,  0 };

    spidev->transaction(tx_data, rx_data, sizeof(tx_data));
    if (memcmp(tx_data, rx_data, sizeof(tx_data)))
	hal.console->println("Incorrect data read from SPI loopback. Do you have the loopback wire installed between pins 11 and 12?");
    else
	hal.console->println("Correct data read from SPI loopback");
    hal.scheduler->delay(500);
}

AP_HAL_MAIN();

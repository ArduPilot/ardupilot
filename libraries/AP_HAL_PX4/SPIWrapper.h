#include <AP_HAL/AP_HAL.h>

#include <arch/board/board.h>
#include "board_config.h"
#include <drivers/device/spi.h>
#include "AP_HAL_PX4.h"

extern const AP_HAL::HAL& hal;
/*
  wrapper class for SPI to expose protected functions from PX4NuttX
 */
class PX4::PX4_SPI {
public:
    PX4_SPI(uint8_t bus, enum spi_dev_e device, enum spi_mode_e mode, uint32_t frequency) :
        _bus(bus), _device(device), _mode(mode), _frequency(frequency) {
		_dev = up_spiinitialize(_bus);
        SPI_SELECT(_dev, _device, false);
    }
    
    bool do_transfer(uint8_t *send, uint8_t *recv, uint32_t len) {
        irqstate_t state = irqsave();
        SPI_SETFREQUENCY(_dev, _frequency);
        SPI_SETMODE(_dev, _mode);
        SPI_SETBITS(_dev, 8);
        SPI_SELECT(_dev, _device, true);
        SPI_EXCHANGE(_dev, send, recv, len);
        SPI_SELECT(_dev, _device, false);
        irqrestore(state);
        return true;
    }
    void set_speed(uint32_t speed_hz) {
        _frequency = speed_hz;
    }
private:
    struct spi_dev_s *_dev;
    uint8_t _bus;
    enum spi_dev_e _device;
    enum spi_mode_e _mode;
    uint32_t _frequency;
};


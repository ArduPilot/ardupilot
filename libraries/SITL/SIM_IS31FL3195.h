/*

  DataSheet: https://www.lumissil.com/assets/pdf/core/IS31FL3195_DS.pdf

  ./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb --debug --rgbled

  param set NTF_LED_TYPES 32772  # enable ToshibaLED and IS31FL3195
  reboot

  param set NTF_LED_OVERRIDE 1
  led 255 0 0   # red
  led 0 255 0   # green
  led 0 0 255   # blue

 */


#include "SIM_config.h"

#if AP_SIM_IS31FL3195_ENABLED

#include "SIM_I2CDevice.h"
#include "SIM_RGBLED.h"

namespace SITL {

class IS31FL3195DevReg : public I2CRegEnum {
public:
    static constexpr uint8_t PRODUCT_ID = 0x00;  // not really; actually i2c addr!
    static constexpr uint8_t SHUTDOWN_CONTROL = 0x01;
    // .
    // .
    static constexpr uint8_t P1_STATE = 0x0C;
    static constexpr uint8_t P2_STATE = 0x0D;
    static constexpr uint8_t P3_STATE = 0x0E;
    static constexpr uint8_t P4_STATE = 0x0F;
    static constexpr uint8_t OUT1 = 0x10;
    // .
    // .
    static constexpr uint8_t OUT2 = 0x21;
    // .
    // .
    static constexpr uint8_t OUT3 = 0x32;
    // .
    // .
    static constexpr uint8_t OUT4 = 0x40;
    // .
    // .
    static constexpr uint8_t COLOUR_UPDATE = 0x50;
    // .
    // .
    static constexpr uint8_t RESET_REGISTER = 0x5f;  // reset the registers value to default
};

class IS31FL3195 : public I2CDevice, protected I2CRegisters_8Bit
{
public:

    void init() override;
    void set_product_id(uint8_t product_id);

    void update(const class Aircraft &aircraft) override;

    void rdwr_store_register_value(uint8_t reg, uint8_t value) override;
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:

    void reset_registers();

    bool colour_update_register_poked;

    SIM_RGBLED rgbled{"IS31FL3195"};
};

} // namespace SITL

#endif  // AP_SIM_IS31FL3195_ENABLED

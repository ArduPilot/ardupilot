#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_MLX90614_ENABLED

/*
  Simulator for the MLX90614 IR thermometer

  DataSheet: https://www.melexis.com/en/documents/documentation/datasheets/datasheet-mlx90614
*/

#include "SIM_I2CDevice.h"

namespace SITL {

class MLX90614 : public I2CDevice
{
public:

    void update(const class Aircraft &aircraft) override;
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:

    float _temperature_degC;

};

} // namespace SITL

#endif  // AP_SIM_TEMPERATURE_MLX90614_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#define AUTO_INCREMENT          0xA0
#define REPEATED_BYTE           0x80

#define APDS9930_ENABLE         0x00
#define APDS9930_ATIME          0x01
#define APDS9930_PTIME          0x02
#define APDS9930_WTIME          0x03
#define APDS9930_AILTL          0x04
#define APDS9930_AILTH          0x05
#define APDS9930_AIHTL          0x06
#define APDS9930_AIHTH          0x07
#define APDS9930_PILTL          0x08
#define APDS9930_PILTH          0x09
#define APDS9930_PIHTL          0x0A
#define APDS9930_PIHTH          0x0B
#define APDS9930_PERS           0x0C
#define APDS9930_CONFIG         0x0D
#define APDS9930_PPULSE         0x0E
#define APDS9930_CONTROL        0x0F
#define APDS9930_ID             0x12
#define APDS9930_STATUS         0x13
#define APDS9930_Ch0DATAL       0x14
#define APDS9930_Ch0DATAH       0x15
#define APDS9930_Ch1DATAL       0x16
#define APDS9930_Ch1DATAH       0x17
#define APDS9930_PDATAL         0x18
#define APDS9930_PDATAH         0x19
#define APDS9930_POFFSET        0x1E

#define POWER                   0
#define AMBIENT_LIGHT           1
#define PROXIMITY               2
#define WAIT                    3
#define AMBIENT_LIGHT_INT       4
#define PROXIMITY_INT           5
#define SLEEP_AFTER_INT         6
#define ALL                     7

#define CLEAR_PROX_INT          0xE5

#define DEFAULT_ATIME           0xFF
#define DEFAULT_WTIME           0xFF
#define DEFAULT_PTIME           0xFF
#define DEFAULT_PPULSE          0x01
#define DEFAULT_POFFSET         0       // 0 offset
#define DEFAULT_CONFIG          0
#define DEFAULT_PDRIVE          0
#define DEFAULT_PDIODE          2
#define DEFAULT_PGAIN           1
#define DEFAULT_AGAIN           0
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            200      // High proximity threshold
#define DEFAULT_PPERS           0x9

class AP_LandingProximity
{
public:
    AP_LandingProximity();
    bool enabled() { return _enabled; }

    void init();
    bool proximity;
    
    static const struct AP_Param::GroupInfo var_info[];
private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    bool enableProximitySensor();
    bool setProximityGain(uint8_t drive);
    bool setLEDDrive(uint8_t drive);
    bool setProximityDiode(uint8_t drive);
    bool setProximityIntHighThreshold(uint16_t threshold);
    bool checkId();
    void timer();
    
    AP_Int8 _enabled;
    AP_Int16 _thd;
    AP_Int8 _pers;
    AP_Int8 _gain;
};

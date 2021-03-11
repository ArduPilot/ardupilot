#include <cstdint>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <unistd.h>

#include <AP_Common/AP_Common.h>
#include <AP_Logger/AP_Logger.h>



class AS5600_AOA {

public:

    AS5600_AOA(void);

    bool init();

    void update(void);

    uint16_t setMaxAngle(uint16_t newMaxAngle = -1);
    uint16_t getMaxAngle();
    uint16_t setStartPosition(uint16_t startAngle = -1);
    uint16_t getStartPosition();
    uint16_t setEndPosition(uint16_t endAngle = -1);
    uint16_t getEndPosition();
    uint16_t getScaledAngle();

    int  detectMagnet();
    int  getMagnetStrength();
    int  getAgc();
    uint16_t getMagnitude();

    int  getBurnCount();
    int  burnAngle();
    int  burnMaxAngleAndConfig();
    void setOutPut(uint8_t mode);

private:

    HAL_Semaphore sem;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    uint8_t  bus;
    uint8_t  address;

    uint16_t rawStartAngle;
    uint16_t zPosition;
    uint16_t rawEndAngle;
    uint16_t mPosition;
    uint16_t maxAngle;

    /* Registers */
//
//    int  _zmco;
//    int _zpos_hi;    /*zpos[11:8] high nibble  START POSITION */
//    int _zpos_lo;    /*zpos[7:0] */
//    int _mpos_hi;    /*mpos[11:8] high nibble  STOP POSITION */
//    int _mpos_lo;    /*mpos[7:0] */
//    int _mang_hi;    /*mang[11:8] high nibble  MAXIMUM ANGLE */
//    int _mang_lo;    /*mang[7:0] */
//    int _conf_hi;
//    int _conf_lo;
//    int _raw_ang_hi;
//    int _raw_ang_lo;
//    int _ang_hi;
//    int _ang_lo;
//    int _stat;
//    int _agc;
//    int _mag_hi;
//    int _mag_lo;
//    int _burn;

    static const uint8_t REG_ZMCO       = 0x00;
    static const uint8_t REG_ZPOS_HI    = 0x01;
    static const uint8_t REG_ZPOS_LO    = 0x02;
    static const uint8_t REG_MPOS_HI    = 0x03;
    static const uint8_t REG_MPOS_LO    = 0x04;
    static const uint8_t REG_MANG_HI    = 0x05;
    static const uint8_t REG_MANG_LO    = 0x06;
    static const uint8_t REG_CONF_HI    = 0x07;
    static const uint8_t REG_CONF_LO    = 0x08;
    static const uint8_t REG_RAW_ANG_HI = 0x0c;
    static const uint8_t REG_RAW_ANG_LO = 0x0d;
    static const uint8_t REG_ANG_HI     = 0x0e;
    static const uint8_t REG_ANG_LO     = 0x0f;
    static const uint8_t REG_STAT       = 0x0b;
    static const uint8_t REG_AGC        = 0x1a;
    static const uint8_t REG_MAG_HI     = 0x1b;
    static const uint8_t REG_MAG_LO     = 0x1c;
    static const uint8_t REG_BURN       = 0xff;

    int readOneByte(uint8_t in_adr);
    int readTwoBytes(uint8_t in_adr_hi, uint8_t in_adr_lo);
    int writeOneByte(uint8_t adr_in, uint8_t dat_in);

    uint8_t lowByte;
    uint8_t highByte;


    void _timer(void);
};

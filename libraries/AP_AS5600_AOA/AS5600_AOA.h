#include <cstdint>
#include <AP_HAL/I2CDevice.h>
#include <unistd.h>

class AS_5600 {

public:

    AS_5600(void);

    void init(void);

    uint32_t busMaskExt;
    uint32_t busMaskInt;

    void checkConnect(void);

    unsigned short setMaxAngle(unsigned short newMaxAngle = -1);
    unsigned short getMaxAngle();

    unsigned short setStartPosition(unsigned short startAngle = -1);
    unsigned short getStartPosition();

    unsigned short setEndPosition(unsigned short endAngle = -1);
    unsigned short getEndPosition();

    unsigned short getRawAngle(void);
    unsigned short getScaledAngle();

    int  detectMagnet();
    int  getMagnetStrength();
    int  getAgc();
    unsigned short getMagnitude();

    int  getBurnCount();
    int  burnAngle();
    int  burnMaxAngleAndConfig();
    void setOutPut(unsigned char mode);

private:

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    uint8_t  bus;
    uint8_t address;

    int as5600_address;

    unsigned short rawStartAngle;
    unsigned short zPosition;
    unsigned short rawEndAngle;
    unsigned short mPosition;
    unsigned short maxAngle;

    /* Registers */

    int  _zmco;
    int _zpos_hi;    /*zpos[11:8] high nibble  START POSITION */
    int _zpos_lo;    /*zpos[7:0] */
    int _mpos_hi;    /*mpos[11:8] high nibble  STOP POSITION */
    int _mpos_lo;    /*mpos[7:0] */
    int _mang_hi;    /*mang[11:8] high nibble  MAXIMUM ANGLE */
    int _mang_lo;    /*mang[7:0] */
    int _conf_hi;
    int _conf_lo;
    int _raw_ang_hi;
    int _raw_ang_lo;
    int _ang_hi;
    int _ang_lo;
    int _stat;
    int _agc;
    int _mag_hi;
    int _mag_lo;
    int _burn;

    int readOneByte(uint8_t in_adr);
    int readTwoBytes(uint8_t in_adr_hi, uint8_t in_adr_lo);
    int writeOneByte(uint8_t adr_in, uint8_t dat_in);

    unsigned char highByte(unsigned short short_in);
    unsigned char lowByte(unsigned short short_in);


};

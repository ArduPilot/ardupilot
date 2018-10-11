/* 
    File: AP_RoboClaw.h
    Author: Dr. -Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Created: 10 Oct 2018
    Last Modified: 11 Oct 2018
    Contributation: Dr. -Ing. Ahmad Kamal Nasir, 
    Description: 
    This library is used to communicate with motion control board such as 
    roboclaw through serial port communication. It uses pointer to the uart
    object, obtained through serial_manager class, to communicate with the
    roboclaw. This library contains the serial protocol implementation in
    order to send/recieve commands/data with connected roboclaw(s). 
 */
#include "AP_RoboClaw.h"

extern const AP_HAL::HAL &hal;

#define MAXRETRY 2
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg) >> 24), (uint8_t)(((uint32_t)arg) >> 16), (uint8_t)(((uint32_t)arg) >> 8), (uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg) >> 8), (uint8_t)arg

AP_RoboClaw::AP_RoboClaw()
{
    hserial = nullptr;
    _timeout = 0;
}

AP_RoboClaw::~AP_RoboClaw() = default;

void AP_RoboClaw::init(AP_HAL::UARTDriver *uart, uint32_t tout)
{
    if (uart != nullptr)
        hserial = uart;
    _timeout = tout;
}

size_t AP_RoboClaw::write(uint8_t byte)
{
    if (hserial)
    {
        return hserial->write(byte);
    }
    return 0;
}

int16_t AP_RoboClaw::read()
{
    if (hserial)
    {
        return hserial->read();
    }
    return 0;
}

uint32_t AP_RoboClaw::available()
{
    if (hserial)
    {
        return hserial->available();
    }
    return 0;
}

void AP_RoboClaw::flush()
{
    if (hserial)
    {
        hserial->flush();
    }
}

int16_t AP_RoboClaw::read(uint32_t timeout)
{
    // TODO change
    if (hserial)
    {
        uint32_t start = AP_HAL::micros();
        // Empty buffer?
        while (!hserial->available())
        {
            if ((AP_HAL::micros() - start) >= timeout)
                return -1;
        }
        return hserial->read();
    }
    return 0;
}

void AP_RoboClaw::clear()
{
    if (hserial)
    {
        while (hserial->available())
        {
            hserial->read();
        }
    }
}

void AP_RoboClaw::crc_clear()
{
    crc = 0;
}

void AP_RoboClaw::crc_update(uint8_t data)
{
    crc = crc ^ ((uint16_t)data << 8);
    for (uint8_t i = 0; i < 8; i++)
    {
        if (crc & 0x8000)
        {
            crc = (crc << 1) ^ 0x1021;
        }
        else
        {
            crc <<= 1;
        }
    }
}

uint16_t AP_RoboClaw::crc_get()
{
    return crc;
}

bool AP_RoboClaw::write_n(uint8_t cnt, ...)
{
    uint8_t trys = MAXRETRY;
    do
    {
        crc_clear();
        //send data with crc
        va_list marker;
        va_start(marker, cnt); /* Initialize variable arguments. */
        for (uint8_t index = 0; index < cnt; index++)
        {
            uint8_t data = va_arg(marker, int);
            crc_update(data);
            write(data);
        }
        va_end(marker); /* Reset variable arguments.      */
        uint16_t lcrc = crc_get();
        write(lcrc >> 8);
        write(lcrc);
        if (read(_timeout) == 0xFF)
        {
            return true;
        }
    } while (trys--);
    return false;
}

bool AP_RoboClaw::read_n(uint8_t cnt, uint8_t address, uint8_t cmd, ...)
{
    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        data = 0;
        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        //send data with crc
        va_list marker;
        va_start(marker, cmd); /* Initialize variable arguments. */
        for (uint8_t index = 0; index < cnt; index++)
        {
            uint32_t *ptr = va_arg(marker, uint32_t *);

            if (data != -1)
            {
                data = read(_timeout);
                crc_update(data);
                value = (uint32_t)data << 24;
            }
            else
            {
                break;
            }

            if (data != -1)
            {
                data = read(_timeout);
                crc_update(data);
                value |= (uint32_t)data << 16;
            }
            else
            {
                break;
            }

            if (data != -1)
            {
                data = read(_timeout);
                crc_update(data);
                value |= (uint32_t)data << 8;
            }
            else
            {
                break;
            }

            if (data != -1)
            {
                data = read(_timeout);
                crc_update(data);
                value |= (uint32_t)data;
            }
            else
            {
                break;
            }

            *ptr = value;
        }
        va_end(marker); /* Reset variable arguments.      */

        if (data != -1)
        {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1)
                {
                    ccrc |= data;
                    return crc_get() == ccrc;
                }
            }
        }
    } while (trys--);

    return false;
}

uint8_t AP_RoboClaw::Read1(uint8_t address, uint8_t cmd, bool *valid)
{
    if (valid)
        *valid = false;

    uint8_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read(_timeout);
        crc_update(data);
        value = data;

        if (data != -1)
        {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get() == ccrc)
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

uint16_t AP_RoboClaw::Read2(uint8_t address, uint8_t cmd, bool *valid)
{
    if (valid)
        *valid = false;

    uint16_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read(_timeout);
        crc_update(data);
        value = (uint16_t)data << 8;

        if (data != -1)
        {
            data = read(_timeout);
            crc_update(data);
            value |= (uint16_t)data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get() == ccrc)
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

uint32_t AP_RoboClaw::Read4(uint8_t address, uint8_t cmd, bool *valid)
{
    if (valid)
        *valid = false;

    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read(_timeout);
        crc_update(data);
        value = (uint32_t)data << 24;

        if (data != -1)
        {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t)data << 16;
        }

        if (data != -1)
        {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t)data << 8;
        }

        if (data != -1)
        {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t)data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get() == ccrc)
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

uint32_t AP_RoboClaw::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid)
{
    if (valid)
        *valid = false;

    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read(_timeout);
        crc_update(data);
        value = (uint32_t)data << 24;

        if (data != -1)
        {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t)data << 16;
        }

        if (data != -1)
        {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t)data << 8;
        }

        if (data != -1)
        {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t)data;
        }

        if (data != -1)
        {
            data = read(_timeout);
            crc_update(data);
            if (status)
                *status = data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get() == ccrc)
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

bool AP_RoboClaw::ForwardM1(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M1FORWARD, speed);
}

bool AP_RoboClaw::BackwardM1(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M1BACKWARD, speed);
}

bool AP_RoboClaw::SetMinVoltageMainBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3, address, SETMINMB, voltage);
}

bool AP_RoboClaw::SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3, address, SETMAXMB, voltage);
}

bool AP_RoboClaw::ForwardM2(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M2FORWARD, speed);
}

bool AP_RoboClaw::BackwardM2(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M2BACKWARD, speed);
}

bool AP_RoboClaw::ForwardBackwardM1(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M17BIT, speed);
}

bool AP_RoboClaw::ForwardBackwardM2(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M27BIT, speed);
}

bool AP_RoboClaw::ForwardMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDFORWARD, speed);
}

bool AP_RoboClaw::BackwardMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDBACKWARD, speed);
}

bool AP_RoboClaw::TurnRightMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDRIGHT, speed);
}

bool AP_RoboClaw::TurnLeftMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDLEFT, speed);
}

bool AP_RoboClaw::ForwardBackwardMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDFB, speed);
}

bool AP_RoboClaw::LeftRightMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDLR, speed);
}

uint32_t AP_RoboClaw::ReadEncM1(uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(address, GETM1ENC, status, valid);
}

uint32_t AP_RoboClaw::ReadEncM2(uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(address, GETM2ENC, status, valid);
}

uint32_t AP_RoboClaw::ReadSpeedM1(uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(address, GETM1SPEED, status, valid);
}

uint32_t AP_RoboClaw::ReadSpeedM2(uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(address, GETM2SPEED, status, valid);
}

bool AP_RoboClaw::ResetEncoders(uint8_t address)
{
    return write_n(2, address, RESETENC);
}

bool AP_RoboClaw::ReadVersion(uint8_t address, char *version)
{
    int16_t data;
    uint8_t trys = MAXRETRY;
    do
    {
        flush();

        data = 0;

        crc_clear();
        write(address);
        crc_update(address);
        write(GETVERSION);
        crc_update(GETVERSION);

        uint8_t i;
        for (i = 0; i < 48; i++)
        {
            data = read(_timeout);
            if (data != -1)
            {
                version[i] = data;
                crc_update(version[i]);
                if (version[i] == 0)
                {
                    uint16_t ccrc;
                    data = read(_timeout);
                    if (data != -1)
                    {
                        ccrc = data << 8;
                        data = read(_timeout);
                        if (data != -1)
                        {
                            ccrc |= data;
                            return crc_get() == ccrc;
                        }
                    }
                    break;
                }
            }
            else
            {
                break;
            }
        }
    } while (trys--);

    return false;
}

bool AP_RoboClaw::SetEncM1(uint8_t address, int32_t val)
{
    return write_n(6, address, SETM1ENCCOUNT, SetDWORDval(val));
}

bool AP_RoboClaw::SetEncM2(uint8_t address, int32_t val)
{
    return write_n(6, address, SETM2ENCCOUNT, SetDWORDval(val));
}

uint16_t AP_RoboClaw::ReadMainBatteryVoltage(uint8_t address, bool *valid)
{
    return Read2(address, GETMBATT, valid);
}

uint16_t AP_RoboClaw::ReadLogicBatteryVoltage(uint8_t address, bool *valid)
{
    return Read2(address, GETLBATT, valid);
}

bool AP_RoboClaw::SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3, address, SETMINLB, voltage);
}

bool AP_RoboClaw::SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3, address, SETMAXLB, voltage);
}

bool AP_RoboClaw::SetM1VelocityPID(uint8_t address,
                                   float kp_fp,
                                   float ki_fp,
                                   float kd_fp,
                                   uint32_t qpps)
{
    uint32_t kp = kp_fp * 65536;
    uint32_t ki = ki_fp * 65536;
    uint32_t kd = kd_fp * 65536;
    return write_n(18, address, SETM1PID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
}

bool AP_RoboClaw::SetM2VelocityPID(uint8_t address,
                                   float kp_fp,
                                   float ki_fp,
                                   float kd_fp,
                                   uint32_t qpps)
{
    uint32_t kp = kp_fp * 65536;
    uint32_t ki = ki_fp * 65536;
    uint32_t kd = kd_fp * 65536;
    return write_n(18, address, SETM2PID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
}

uint32_t AP_RoboClaw::ReadISpeedM1(uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(address, GETM1ISPEED, status, valid);
}

uint32_t AP_RoboClaw::ReadISpeedM2(uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(address, GETM2ISPEED, status, valid);
}

bool AP_RoboClaw::DutyM1(uint8_t address, uint16_t duty)
{
    return write_n(4, address, M1DUTY, SetWORDval(duty));
}

bool AP_RoboClaw::DutyM2(uint8_t address, uint16_t duty)
{
    return write_n(4, address, M2DUTY, SetWORDval(duty));
}

bool AP_RoboClaw::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2)
{
    return write_n(6, address, MIXEDDUTY, SetWORDval(duty1), SetWORDval(duty2));
}

bool AP_RoboClaw::SpeedM1(uint8_t address, uint32_t speed)
{
    return write_n(6, address, M1SPEED, SetDWORDval(speed));
}

bool AP_RoboClaw::SpeedM2(uint8_t address, uint32_t speed)
{
    return write_n(6, address, M2SPEED, SetDWORDval(speed));
}

bool AP_RoboClaw::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2)
{
    return write_n(10, address, MIXEDSPEED, SetDWORDval(speed1), SetDWORDval(speed2));
}

bool AP_RoboClaw::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed)
{
    return write_n(10, address, M1SPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed));
}

bool AP_RoboClaw::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed)
{
    return write_n(10, address, M2SPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed));
}
bool AP_RoboClaw::SpeedAccelM1M2(uint8_t address,
                                 uint32_t accel,
                                 uint32_t speed1,
                                 uint32_t speed2)
{
    return write_n(14, address, MIXEDSPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed1), SetDWORDval(speed2));
}

bool AP_RoboClaw::SpeedDistanceM1(uint8_t address,
                                  uint32_t speed,
                                  uint32_t distance,
                                  uint8_t flag)
{
    return write_n(11, address, M1SPEEDDIST, SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool AP_RoboClaw::SpeedDistanceM2(uint8_t address,
                                  uint32_t speed,
                                  uint32_t distance,
                                  uint8_t flag)
{
    return write_n(11, address, M2SPEEDDIST, SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool AP_RoboClaw::SpeedDistanceM1M2(uint8_t address,
                                    uint32_t speed1,
                                    uint32_t distance1,
                                    uint32_t speed2,
                                    uint32_t distance2,
                                    uint8_t flag)
{
    return write_n(19,
                   address,
                   MIXEDSPEEDDIST,
                   SetDWORDval(speed1),
                   SetDWORDval(distance1),
                   SetDWORDval(speed2),
                   SetDWORDval(distance2),
                   flag);
}

bool AP_RoboClaw::SpeedAccelDistanceM1(uint8_t address,
                                       uint32_t accel,
                                       uint32_t speed,
                                       uint32_t distance,
                                       uint8_t flag)
{
    return write_n(15, address, M1SPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool AP_RoboClaw::SpeedAccelDistanceM2(uint8_t address,
                                       uint32_t accel,
                                       uint32_t speed,
                                       uint32_t distance,
                                       uint8_t flag)
{
    return write_n(15, address, M2SPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool AP_RoboClaw::SpeedAccelDistanceM1M2(uint8_t address,
                                         uint32_t accel,
                                         uint32_t speed1,
                                         uint32_t distance1,
                                         uint32_t speed2,
                                         uint32_t distance2,
                                         uint8_t flag)
{
    return write_n(23,
                   address,
                   MIXEDSPEEDACCELDIST,
                   SetDWORDval(accel),
                   SetDWORDval(speed1),
                   SetDWORDval(distance1),
                   SetDWORDval(speed2),
                   SetDWORDval(distance2),
                   flag);
}

bool AP_RoboClaw::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2)
{
    bool valid;
    uint16_t value = Read2(address, GETBUFFERS, &valid);
    if (valid)
    {
        depth1 = value >> 8;
        depth2 = value;
    }
    return valid;
}

bool AP_RoboClaw::ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2)
{
    bool valid;
    uint32_t value = Read4(address, GETPWMS, &valid);
    if (valid)
    {
        pwm1 = value >> 16;
        pwm2 = value & 0xFFFF;
    }
    return valid;
}

bool AP_RoboClaw::ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2)
{
    bool valid;
    uint32_t value = Read4(address, GETCURRENTS, &valid);
    if (valid)
    {
        current1 = value >> 16;
        current2 = value & 0xFFFF;
    }
    return valid;
}

bool AP_RoboClaw::SpeedAccelM1M2_2(uint8_t address,
                                   uint32_t accel1,
                                   uint32_t speed1,
                                   uint32_t accel2,
                                   uint32_t speed2)
{
    return write_n(18,
                   address,
                   MIXEDSPEED2ACCEL,
                   SetDWORDval(accel1),
                   SetDWORDval(speed1),
                   SetDWORDval(accel2),
                   SetDWORDval(speed2));
}

bool AP_RoboClaw::SpeedAccelDistanceM1M2_2(uint8_t address,
                                           uint32_t accel1,
                                           uint32_t speed1,
                                           uint32_t distance1,
                                           uint32_t accel2,
                                           uint32_t speed2,
                                           uint32_t distance2,
                                           uint8_t flag)
{
    return write_n(27,
                   address,
                   MIXEDSPEED2ACCELDIST,
                   SetDWORDval(accel1),
                   SetDWORDval(speed1),
                   SetDWORDval(distance1),
                   SetDWORDval(accel2),
                   SetDWORDval(speed2),
                   SetDWORDval(distance2),
                   flag);
}

bool AP_RoboClaw::DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel)
{
    return write_n(8, address, M1DUTYACCEL, SetWORDval(duty), SetDWORDval(accel));
}

bool AP_RoboClaw::DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel)
{
    return write_n(8, address, M2DUTYACCEL, SetWORDval(duty), SetDWORDval(accel));
}

bool AP_RoboClaw::DutyAccelM1M2(uint8_t address,
                                uint16_t duty1,
                                uint32_t accel1,
                                uint16_t duty2,
                                uint32_t accel2)
{
    return write_n(14,
                   address,
                   MIXEDDUTYACCEL,
                   SetWORDval(duty1),
                   SetDWORDval(accel1),
                   SetWORDval(duty2),
                   SetDWORDval(accel2));
}

bool AP_RoboClaw::ReadM1VelocityPID(uint8_t address,
                                    float &Kp_fp,
                                    float &Ki_fp,
                                    float &Kd_fp,
                                    uint32_t &qpps)
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(4, address, READM1PID, &Kp, &Ki, &Kd, &qpps);
    Kp_fp = ((float)Kp) / 65536;
    Ki_fp = ((float)Ki) / 65536;
    Kd_fp = ((float)Kd) / 65536;
    return valid;
}

bool AP_RoboClaw::ReadM2VelocityPID(uint8_t address,
                                    float &Kp_fp,
                                    float &Ki_fp,
                                    float &Kd_fp,
                                    uint32_t &qpps)
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(4, address, READM2PID, &Kp, &Ki, &Kd, &qpps);
    Kp_fp = ((float)Kp) / 65536;
    Ki_fp = ((float)Ki) / 65536;
    Kd_fp = ((float)Kd) / 65536;
    return valid;
}

bool AP_RoboClaw::SetMainVoltages(uint8_t address, uint16_t min, uint16_t max)
{
    return write_n(6, address, SETMAINVOLTAGES, SetWORDval(min), SetWORDval(max));
}

bool AP_RoboClaw::SetLogicVoltages(uint8_t address, uint16_t min, uint16_t max)
{
    return write_n(6, address, SETLOGICVOLTAGES, SetWORDval(min), SetWORDval(max));
}

bool AP_RoboClaw::ReadMinMaxMainVoltages(uint8_t address, uint16_t &min, uint16_t &max)
{
    bool valid;
    uint32_t value = Read4(address, GETMINMAXMAINVOLTAGES, &valid);
    if (valid)
    {
        min = value >> 16;
        max = value & 0xFFFF;
    }
    return valid;
}

bool AP_RoboClaw::ReadMinMaxLogicVoltages(uint8_t address, uint16_t &min, uint16_t &max)
{
    bool valid;
    uint32_t value = Read4(address, GETMINMAXLOGICVOLTAGES, &valid);
    if (valid)
    {
        min = value >> 16;
        max = value & 0xFFFF;
    }
    return valid;
}

bool AP_RoboClaw::SetM1PositionPID(uint8_t address,
                                   float kp_fp,
                                   float ki_fp,
                                   float kd_fp,
                                   uint32_t kiMax,
                                   uint32_t deadzone,
                                   uint32_t min,
                                   uint32_t max)
{
    uint32_t kp = kp_fp * 1024;
    uint32_t ki = ki_fp * 1024;
    uint32_t kd = kd_fp * 1024;
    return write_n(30,
                   address,
                   SETM1POSPID,
                   SetDWORDval(kd),
                   SetDWORDval(kp),
                   SetDWORDval(ki),
                   SetDWORDval(kiMax),
                   SetDWORDval(deadzone),
                   SetDWORDval(min),
                   SetDWORDval(max));
}

bool AP_RoboClaw::SetM2PositionPID(uint8_t address,
                                   float kp_fp,
                                   float ki_fp,
                                   float kd_fp,
                                   uint32_t kiMax,
                                   uint32_t deadzone,
                                   uint32_t min,
                                   uint32_t max)
{
    uint32_t kp = kp_fp * 1024;
    uint32_t ki = ki_fp * 1024;
    uint32_t kd = kd_fp * 1024;
    return write_n(30,
                   address,
                   SETM2POSPID,
                   SetDWORDval(kd),
                   SetDWORDval(kp),
                   SetDWORDval(ki),
                   SetDWORDval(kiMax),
                   SetDWORDval(deadzone),
                   SetDWORDval(min),
                   SetDWORDval(max));
}

bool AP_RoboClaw::ReadM1PositionPID(uint8_t address,
                                    float &Kp_fp,
                                    float &Ki_fp,
                                    float &Kd_fp,
                                    uint32_t &KiMax,
                                    uint32_t &DeadZone,
                                    uint32_t &Min,
                                    uint32_t &Max)
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(7, address, READM1POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max);
    Kp_fp = ((float)Kp) / 1024;
    Ki_fp = ((float)Ki) / 1024;
    Kd_fp = ((float)Kd) / 1024;
    return valid;
}

bool AP_RoboClaw::ReadM2PositionPID(uint8_t address,
                                    float &Kp_fp,
                                    float &Ki_fp,
                                    float &Kd_fp,
                                    uint32_t &KiMax,
                                    uint32_t &DeadZone,
                                    uint32_t &Min,
                                    uint32_t &Max)
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(7, address, READM2POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max);
    Kp_fp = ((float)Kp) / 1024;
    Ki_fp = ((float)Ki) / 1024;
    Kd_fp = ((float)Kd) / 1024;
    return valid;
}

bool AP_RoboClaw::SpeedAccelDeccelPositionM1(uint8_t address,
                                             uint32_t accel,
                                             uint32_t speed,
                                             uint32_t deccel,
                                             uint32_t position,
                                             uint8_t flag)
{
    return write_n(19,
                   address,
                   M1SPEEDACCELDECCELPOS,
                   SetDWORDval(accel),
                   SetDWORDval(speed),
                   SetDWORDval(deccel),
                   SetDWORDval(position),
                   flag);
}

bool AP_RoboClaw::SpeedAccelDeccelPositionM2(uint8_t address,
                                             uint32_t accel,
                                             uint32_t speed,
                                             uint32_t deccel,
                                             uint32_t position,
                                             uint8_t flag)
{
    return write_n(19,
                   address,
                   M2SPEEDACCELDECCELPOS,
                   SetDWORDval(accel),
                   SetDWORDval(speed),
                   SetDWORDval(deccel),
                   SetDWORDval(position),
                   flag);
}

bool AP_RoboClaw::SpeedAccelDeccelPositionM1M2(uint8_t address,
                                               uint32_t accel1,
                                               uint32_t speed1,
                                               uint32_t deccel1,
                                               uint32_t position1,
                                               uint32_t accel2,
                                               uint32_t speed2,
                                               uint32_t deccel2,
                                               uint32_t position2,
                                               uint8_t flag)
{
    return write_n(35,
                   address,
                   MIXEDSPEEDACCELDECCELPOS,
                   SetDWORDval(accel1),
                   SetDWORDval(speed1),
                   SetDWORDval(deccel1),
                   SetDWORDval(position1),
                   SetDWORDval(accel2),
                   SetDWORDval(speed2),
                   SetDWORDval(deccel2),
                   SetDWORDval(position2),
                   flag);
}

bool AP_RoboClaw::SetM1DefaultAccel(uint8_t address, uint32_t accel)
{
    return write_n(6, address, SETM1DEFAULTACCEL, SetDWORDval(accel));
}

bool AP_RoboClaw::SetM2DefaultAccel(uint8_t address, uint32_t accel)
{
    return write_n(6, address, SETM2DEFAULTACCEL, SetDWORDval(accel));
}

bool AP_RoboClaw::SetPinFunctions(uint8_t address,
                                  uint8_t S3mode,
                                  uint8_t S4mode,
                                  uint8_t S5mode)
{
    return write_n(5, address, SETPINFUNCTIONS, S3mode, S4mode, S5mode);
}

bool AP_RoboClaw::GetPinFunctions(uint8_t address,
                                  uint8_t &S3mode,
                                  uint8_t &S4mode,
                                  uint8_t &S5mode)
{
    uint8_t val1, val2, val3;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(GETPINFUNCTIONS);
        crc_update(GETPINFUNCTIONS);

        data = read(_timeout);
        crc_update(data);
        val1 = data;

        if (data != -1)
        {
            data = read(_timeout);
            crc_update(data);
            val2 = data;
        }

        if (data != -1)
        {
            data = read(_timeout);
            crc_update(data);
            val3 = data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get() == ccrc)
                    {
                        S3mode = val1;
                        S4mode = val2;
                        S5mode = val3;
                        return true;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

bool AP_RoboClaw::SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max)
{
    return write_n(4, address, SETDEADBAND, Min, Max);
}

bool AP_RoboClaw::GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max)
{
    bool valid;
    uint16_t value = Read2(address, GETDEADBAND, &valid);
    if (valid)
    {
        Min = value >> 8;
        Max = value;
    }
    return valid;
}

bool AP_RoboClaw::ReadEncoders(uint8_t address, uint32_t &enc1, uint32_t &enc2)
{
    bool valid = read_n(2, address, GETENCODERS, &enc1, &enc2);
    return valid;
}

bool AP_RoboClaw::ReadISpeeds(uint8_t address, uint32_t &ispeed1, uint32_t &ispeed2)
{
    bool valid = read_n(2, address, GETISPEEDS, &ispeed1, &ispeed2);
    return valid;
}

bool AP_RoboClaw::RestoreDefaults(uint8_t address)
{
    return write_n(2, address, RESTOREDEFAULTS);
}

bool AP_RoboClaw::ReadTemp(uint8_t address, uint16_t &temp)
{
    bool valid;
    temp = Read2(address, GETTEMP, &valid);
    return valid;
}

bool AP_RoboClaw::ReadTemp2(uint8_t address, uint16_t &temp)
{
    bool valid;
    temp = Read2(address, GETTEMP2, &valid);
    return valid;
}

uint16_t AP_RoboClaw::ReadError(uint8_t address, bool *valid)
{
    return Read2(address, GETERROR, valid);
}

bool AP_RoboClaw::ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode)
{
    bool valid;
    uint16_t value = Read2(address, GETENCODERMODE, &valid);
    if (valid)
    {
        M1mode = value >> 8;
        M2mode = value;
    }
    return valid;
}

bool AP_RoboClaw::SetM1EncoderMode(uint8_t address, uint8_t mode)
{
    return write_n(3, address, SETM1ENCODERMODE, mode);
}

bool AP_RoboClaw::SetM2EncoderMode(uint8_t address, uint8_t mode)
{
    return write_n(3, address, SETM2ENCODERMODE, mode);
}

bool AP_RoboClaw::WriteNVM(uint8_t address)
{
    return write_n(6, address, WRITENVM, SetDWORDval(0xE22EAB7A));
}

bool AP_RoboClaw::ReadNVM(uint8_t address)
{
    return write_n(2, address, READNVM);
}

bool AP_RoboClaw::SetConfig(uint8_t address, uint16_t config)
{
    return write_n(4, address, SETCONFIG, SetWORDval(config));
}

bool AP_RoboClaw::GetConfig(uint8_t address, uint16_t &config)
{
    bool valid;
    uint16_t value = Read2(address, GETCONFIG, &valid);
    if (valid)
    {
        config = value;
    }
    return valid;
}

bool AP_RoboClaw::SetM1MaxCurrent(uint8_t address, uint32_t max)
{
    return write_n(10, address, SETM1MAXCURRENT, SetDWORDval(max), SetDWORDval(0));
}

bool AP_RoboClaw::SetM2MaxCurrent(uint8_t address, uint32_t max)
{
    return write_n(10, address, SETM2MAXCURRENT, SetDWORDval(max), SetDWORDval(0));
}

bool AP_RoboClaw::ReadM1MaxCurrent(uint8_t address, uint32_t &max)
{
    uint32_t tmax, dummy;
    bool valid = read_n(2, address, GETM1MAXCURRENT, &tmax, &dummy);
    if (valid)
        max = tmax;
    return valid;
}

bool AP_RoboClaw::ReadM2MaxCurrent(uint8_t address, uint32_t &max)
{
    uint32_t tmax, dummy;
    bool valid = read_n(2, address, GETM2MAXCURRENT, &tmax, &dummy);
    if (valid)
        max = tmax;
    return valid;
}

bool AP_RoboClaw::SetPWMMode(uint8_t address, uint8_t mode)
{
    return write_n(3, address, SETPWMMODE, mode);
}

bool AP_RoboClaw::GetPWMMode(uint8_t address, uint8_t &mode)
{
    bool valid;
    uint8_t value = Read1(address, GETPWMMODE, &valid);
    if (valid)
    {
        mode = value;
    }
    return valid;
}

#pragma once

#include "MsgHandler.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>

class LR_MsgHandler : public MsgHandler {
public:
    LR_MsgHandler(struct log_Format &f);
    virtual void process_message(uint8_t *msg) = 0;
    virtual void process_message(uint8_t *msg, uint8_t &core) {
        // base implementation just ignores the core parameter;
        // subclasses can override to fill the core in if they feel
        // like it.
        process_message(msg);
    }
};

class LR_MsgHandler_RFRH : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_EKF : public LR_MsgHandler
{
public:
    LR_MsgHandler_EKF(struct log_Format &_f, NavEKF2 &_ekf2, NavEKF3 &_ekf3) :
        LR_MsgHandler(_f),
        ekf2(_ekf2),
        ekf3(_ekf3) {}
    using LR_MsgHandler::LR_MsgHandler;
    virtual void process_message(uint8_t *msg) override = 0;
protected:
    NavEKF2 &ekf2;
    NavEKF3 &ekf3;
};

class LR_MsgHandler_RFRF : public LR_MsgHandler_EKF
{
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_ROFH : public LR_MsgHandler_EKF
{
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_REPH : public LR_MsgHandler_EKF
{
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RSLL : public LR_MsgHandler_EKF
{
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_REVH : public LR_MsgHandler_EKF
{
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RWOH : public LR_MsgHandler_EKF
{
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RBOH : public LR_MsgHandler_EKF
{
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RFRN : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_REV2 : public LR_MsgHandler_EKF
{
public:
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RSO2 : public LR_MsgHandler_EKF
{
public:
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RWA2 : public LR_MsgHandler_EKF
{
public:
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};


class LR_MsgHandler_REV3 : public LR_MsgHandler_EKF
{
public:
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RSO3 : public LR_MsgHandler_EKF
{
public:
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RWA3 : public LR_MsgHandler_EKF
{
public:
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_REY3 : public LR_MsgHandler_EKF
{
public:
    using LR_MsgHandler_EKF::LR_MsgHandler_EKF;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RISH : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};
class LR_MsgHandler_RISI : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};
class LR_MsgHandler_RASH : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};
class LR_MsgHandler_RASI : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RBRH : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};
class LR_MsgHandler_RBRI : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RRNH : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};
class LR_MsgHandler_RRNI : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RGPH : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};
class LR_MsgHandler_RGPI : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};
class LR_MsgHandler_RGPJ : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RMGH : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};
class LR_MsgHandler_RMGI : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};
class LR_MsgHandler_RBCH : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};
class LR_MsgHandler_RBCI : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_RVOH : public LR_MsgHandler
{
public:
    using LR_MsgHandler::LR_MsgHandler;
    void process_message(uint8_t *msg) override;
};

class LR_MsgHandler_PARM : public LR_MsgHandler
{
public:
    LR_MsgHandler_PARM(log_Format &_f) :
        LR_MsgHandler(_f)
        {};

    void process_message(uint8_t *msg) override;

private:
    bool set_parameter(const char *name, const float value);
};

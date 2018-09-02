#pragma once

/*
    imported from Betaflight/INAV

*/


#ifdef BOARD_NRF_CS_PIN

/*#pragma GCC push_options
#pragma GCC optimize ("O2")
#pragma GCC pop_options
*/
#include <AP_HAL/AP_HAL.h>


#include "AP_HAL_F4Light.h"

#include "RC_parser.h"

#define USE_WHITENING
#define USE_AUTO_ACKKNOWLEDGEMENT

    enum {      // instead of #define
        RX_TX_ADDR_LEN=5,
        NRF_MAX_PAYLOAD_SIZE=32,
        BIND_PAYLOAD_SIZE =      16,
        BIND_PAYLOAD0 =        0xad, // 10101101
        BIND_PAYLOAD1 =        0xc9, // 11001001
        BIND_ACK_PAYLOAD0 =    0x95, // 10010101
        BIND_ACK_PAYLOAD1 =    0xa9, // 10101001
        TELEMETRY_ACK_PAYLOAD0= 0x5a, // 01011010
        // TELEMETRY_ACK_PAYLOAD1 is sequence count
        DATA_PAYLOAD0 =        0x00,
        DATA_PAYLOAD1 =        0x00,
        INAV_PROTOCOL_PAYLOAD_SIZE_MIN = 8,
        INAV_PROTOCOL_PAYLOAD_SIZE_DEFAULT = 16,
        INAV_PROTOCOL_PAYLOAD_SIZE_MAX = 18,
        RX_TX_ADDR_4 = 0xD2, // rxTxAddr[4] always set to this value

        INAV_RF_CHANNEL_COUNT_MAX = 8,
        INAV_RF_CHANNEL_HOPPING_COUNT_DEFAULT = 4,
        INAV_RF_BIND_CHANNEL = 0x4c,

    };


// RC channels in AETR order
typedef enum {
    RC_SPI_ROLL = 0,
    RC_SPI_PITCH,
    RC_SPI_THROTTLE,
    RC_SPI_YAW,
    RC_SPI_AUX1,
    RC_SPI_AUX2,
    RC_SPI_AUX3,
    RC_SPI_AUX4,
    RC_SPI_AUX5,
    RC_SPI_AUX6,
    RC_SPI_AUX7,
    RC_SPI_AUX8,
    RC_SPI_AUX9,
    RC_SPI_AUX10,
    RC_SPI_AUX11,
    RC_SPI_AUX12,
    RC_SPI_AUX13,
    RC_SPI_AUX14
} rc_spi_aetr_e;

enum {
    RATE_LOW = 0,
    RATE_MID = 1,
    RATE_HIGH = 2,
};

enum {
    FLAG_FLIP     = 0x01,
    FLAG_PICTURE  = 0x02,
    FLAG_VIDEO    = 0x04,
    FLAG_RTH      = 0x08,
    FLAG_HEADLESS = 0x10,
};

#define PWM_RANGE_MIN 1100
#define PWM_RANGE_MAX 1900
#define PWM_RANGE_MIDDLE (PWM_RANGE_MIN + ((PWM_RANGE_MAX - PWM_RANGE_MIN) / 2))

class F4Light::NRF_parser : public F4Light::_parser {
public:
    NRF_parser() {}


    typedef enum {
        RX_SPI_RECEIVED_NONE = 0,
        RX_SPI_RECEIVED_BIND,
        RX_SPI_RECEIVED_DATA
    } rx_spi_received_e;

    typedef enum {
        STATE_BIND = 0,
        STATE_DATA
    } protocol_state_t;


    void init(uint8_t ch);

private:
    static AP_HAL::OwnPtr<AP_HAL::SPIDevice> nrf;

    void set_val(uint8_t ch, uint16_t val);
    
    uint32_t *fixedIdPtr; // TODO parameter

    void cs_assert(){ nrf->set_speed(AP_HAL::Device::SPEED_HIGH); GPIO::_write(BOARD_NRF_CS_PIN, 0); }
    void cs_release(){                                            GPIO::_write(BOARD_NRF_CS_PIN, 1); }

// high level functions

    void inavNrf24Setup(const uint32_t *fixedId);
    void inavNrf24SetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload);
    void inavSetHoppingChannels(void);
    void inavHopToNextChannel(void);
    void writeBindAckPayload(uint8_t *payload);
    void writeTelemetryAckPayload(void);
    void writeAckPayload(uint8_t *data, uint8_t length);
    void inavSetBound(void);
    void whitenPayload(uint8_t *payload, uint8_t len);
    bool checkBindPacket(const uint8_t *payload);
    

// low level
    void NRF24L01_Initialize(uint8_t baseConfig);
    uint8_t NRF24L01_WriteReg(uint8_t reg, uint8_t data);
    uint8_t NRF24L01_WriteRegisterMulti(uint8_t reg, const uint8_t *data, uint8_t length);
    uint8_t NRF24L01_WritePayload(const uint8_t *data, uint8_t length);
    uint8_t NRF24L01_WriteAckPayload(const uint8_t *data, uint8_t length, uint8_t pipe);
    uint8_t NRF24L01_ReadReg(uint8_t reg);
    uint8_t NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length);
    uint8_t NRF24L01_ReadPayload(uint8_t *data, uint8_t length);
    
    
    static rx_spi_received_e state;
    static protocol_state_t protocolState;
    static uint8_t rxTxAddr[RX_TX_ADDR_LEN];
    
// Utility functions
    
    void NRF24L01_FlushTx(void);
    void NRF24L01_FlushRx(void);
    uint8_t NRF24L01_Activate(uint8_t code);
    
    void NRF24L01_SetupBasic(void);
    void NRF24L01_SetStandbyMode(void);
    void NRF24L01_SetRxMode(void);
    void NRF24L01_SetTxMode(void);
    void NRF24L01_ClearAllInterrupts(void);
    void NRF24L01_SetChannel(uint8_t channel);
    bool NRF24L01_ReadPayloadIfAvailable(uint8_t *data, uint8_t length);
};

#endif // BOARD_NRF_CS_PIN


/*
    (c) 2017 night_ghost@ykoctpa.ru
 
based on: BetaFlight NRF driver
*/

#include <exti.h>
#include <timer.h>
#include "RCInput.h"
#include <pwm_in.h>
#include <AP_HAL/utility/dsm.h>
#include "sbus.h"
#include "GPIO.h"
#include "ring_buffer_pulse.h"
#include "RC_NRF_parser.h"


using namespace F4Light;


extern const AP_HAL::HAL& hal;

#if defined(BOARD_NRF_CS_PIN) && defined(BOARD_NRF_NAME)


static uint8_t NRF_Buffer[NRF_MAX_PAYLOAD_SIZE];
static uint8_t ackPayload[NRF24L01_MAX_PAYLOAD_SIZE];

NRF_parser::uint8_t rxTxAddr[RX_TX_ADDR_LEN] = {0x4b,0x5c,0x6d,0x7e,0x8f};

static  const uint8_t inavRfChannelHoppingCount = INAV_RF_CHANNEL_HOPPING_COUNT_DEFAULT;
static uint8_t inavRfChannelCount;
static uint8_t inavRfChannelIndex;
static uint8_t inavRfChannels[INAV_RF_CHANNEL_COUNT_MAX];

void NRF_parser::init(uint8_t ch){

    memset((void *)&val[0],    0, sizeof(val));
    
    _last_signal=0;
    _last_change =0;


    GPIO::_pinMode(BOARD_NRF24_CS_PIN, OUTPUT);
    GPIO::_write(BOARD_NRF24_CS_PIN, 1);

    nrf = hal.spi->get_device(BOARD_NRF_NAME);

    nrf->register_periodic_callback(FUNCTOR_BIND_MEMBER(&NRF_parser::_timer, bool), 100);

}

void NRF_parser::_timer() {
    
    uint32_t timeNowUs;
    switch (protocolState) {
    case STATE_BIND:
        if (NRF24L01_ReadPayloadIfAvailable(NRF_Buffer, NRF_MAX_PAYLOAD_SIZE)) {
            whitenPayload(NRF_Buffer, NRF_MAX_PAYLOAD_SIZE);
            const bool bindPacket = checkBindPacket(NRF_Buffer);
            if (bindPacket) {
                state = RX_SPI_RECEIVED_BIND;
                writeBindAckPayload(NRF_Buffer);
                // got a bind packet, so set the hopping channels and the rxTxAddr and start listening for data
                inavSetBound();
            }
        }
        break;

    case STATE_DATA:
        timeNowUs = micros();
        // read the payload, processing of payload is deferred
        if (NRF24L01_ReadPayloadIfAvailable(NRF_Buffer, NRF_MAX_PAYLOAD_SIZE)) {
            whitenPayload(NRF_Buffer, NRF_MAX_PAYLOAD_SIZE);
            receivedPowerSnapshot = NRF24L01_ReadReg(NRF24L01_09_RPD); // set to 1 if received power > -64dBm
            const bool bindPacket = checkBindPacket(NRF_Buffer);
            if (bindPacket) {
                // transmitter may still continue to transmit bind packets after we have switched to data mode
                state = RX_SPI_RECEIVED_BIND;
                writeBindAckPayload(NRF_Buffer);
            } else {
                state = RX_SPI_RECEIVED_DATA;
                writeTelemetryAckPayload();
            }
        }
        if ((state == RX_SPI_RECEIVED_DATA) || (timeNowUs > timeOfLastHop + hopTimeout)) {
            inavHopToNextChannel();
            timeOfLastHop = timeNowUs;
        }
        break;
    }
    

}

bool NRF_parser::checkBindPacket(const uint8_t *payload)
{
    bool bindPacket = false;
    if (payload[0] == BIND_PAYLOAD0  && payload[1] == BIND_PAYLOAD1) {
        bindPacket = true;
        if (protocolState ==STATE_BIND) {
            rxTxAddr[0] = payload[2];
            rxTxAddr[1] = payload[3];
            rxTxAddr[2] = payload[4];
            rxTxAddr[3] = payload[5];
            rxTxAddr[4] = payload[6];
            /*inavRfChannelHoppingCount = payload[7]; // !!TODO not yet implemented on transmitter
            if (inavRfChannelHoppingCount > INAV_RF_CHANNEL_COUNT_MAX) {
                inavRfChannelHoppingCount = INAV_RF_CHANNEL_COUNT_MAX;
            }*/
            if (fixedIdPtr != NULL && *fixedIdPtr == 0) {
                // copy the rxTxAddr so it can be saved
                memcpy(fixedIdPtr, rxTxAddr, sizeof(uint32_t));
            }
        }
    }
    return bindPacket;
}


void NRF_parser::whitenPayload(uint8_t *payload, uint8_t len)
{
#ifdef USE_WHITENING
    uint8_t whitenCoeff = 0x6b; // 01101011
    while (len--) {
        for (uint8_t m = 1; m; m <<= 1) {
            if (whitenCoeff & 0x80) {
                whitenCoeff ^= 0x11;
                (*payload) ^= m;
            }
            whitenCoeff <<= 1;
        }
        payload++;
    }
#else
    UNUSED(payload);
    UNUSED(len);
#endif
}

void NRF_parser::inavSetBound(void)
{
    protocolState = STATE_DATA;
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxTxAddr, RX_TX_ADDR_LEN);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,    rxTxAddr, RX_TX_ADDR_LEN);

    timeOfLastHop = micros();
    inavRfChannelIndex = 0;
    inavSetHoppingChannels();
    NRF24L01_SetChannel(inavRfChannels[0]);
#ifdef DEBUG_NRF24_INAV
    debug[0] = inavRfChannels[inavRfChannelIndex];
#endif
}

void NRF_parser::writeAckPayload(uint8_t *data, uint8_t length)
{
    whitenPayload(data, length);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BV(NRF24L01_07_STATUS_MAX_RT));
    NRF24L01_WriteAckPayload(data, length, NRF24L01_PIPE0);
}

void NRF_parser::writeTelemetryAckPayload(void)
{
#ifdef TELEMETRY_NRF24_LTM
    // set up telemetry data, send back telemetry data in the ACK packet
    static uint8_t sequenceNumber = 0;
    static ltm_frame_e ltmFrameType = LTM_FRAME_START;
    
    ackPayload[0] = TELEMETRY_ACK_PAYLOAD0;
    ackPayload[1] = sequenceNumber++;
    const int ackPayloadSize = getLtmFrame(&ackPayload[2], ltmFrameType) + 2;
    
    ++ltmFrameType;
    if (ltmFrameType > LTM_FRAME_COUNT) {
        ltmFrameType = LTM_FRAME_START;
    }
    writeAckPayload(ackPayload, ackPayloadSize);
#ifdef DEBUG_NRF24_INAV
    debug[1] = ackPayload[1]; // sequenceNumber
    debug[2] = ackPayload[2]; // frame type, 'A', 'S' etc
    debug[3] = ackPayload[3]; // pitch for AFrame
#endif
#endif
}


void NRF_parser::writeBindAckPayload(uint8_t *payload)
{
#ifdef USE_AUTO_ACKKNOWLEDGEMENT
    memcpy(ackPayload, payload, BIND_PAYLOAD_SIZE);
    // send back the payload with the first two bytes set to zero as the ack
    ackPayload[0] = BIND_ACK_PAYLOAD0;
    ackPayload[1] = BIND_ACK_PAYLOAD1;
    // respond to request for rfChannelCount;
    ackPayload[7] = inavRfChannelHoppingCount;
    // respond to request for payloadSize
    switch (payloadSize) {
    case INAV_PROTOCOL_PAYLOAD_SIZE_MIN:
    case INAV_PROTOCOL_PAYLOAD_SIZE_DEFAULT:
    case INAV_PROTOCOL_PAYLOAD_SIZE_MAX:
        ackPayload[8] = payloadSize;
        break;
    default:
        ackPayload[8] = INAV_PROTOCOL_PAYLOAD_SIZE_DEFAULT;
        break;
    }
    writeAckPayload(ackPayload, BIND_PAYLOAD_SIZE);
#else
    UNUSED(payload);
#endif
}


void NRF_parser::inavHopToNextChannel(void)
{
    ++inavRfChannelIndex;
    if (inavRfChannelIndex >= inavRfChannelCount) {
        inavRfChannelIndex = 0;
    }
    NRF24L01_SetChannel(inavRfChannels[inavRfChannelIndex]);
#ifdef DEBUG_NRF24_INAV
    debug[0] = inavRfChannels[inavRfChannelIndex];
#endif
}

// The hopping channels are determined by the low bits of rxTxAddr
void NRF_parser::inavSetHoppingChannels(void)
{
#ifdef NO_RF_CHANNEL_HOPPING
     // just stay on bind channel, useful for debugging
    inavRfChannelCount = 1;
    inavRfChannels[0] = INAV_RF_BIND_CHANNEL;
#else
    inavRfChannelCount = inavRfChannelHoppingCount;
    const uint8_t addr = rxTxAddr[0];
    uint8_t ch = 0x10 + (addr & 0x07);
    for (int ii = 0; ii < INAV_RF_CHANNEL_COUNT_MAX; ++ii) {
        inavRfChannels[ii] = ch;
        ch += 0x0c;
    }
#endif
}

void NRF_parser::set_val(uint8_t ch, uint16_t val){
    if(_val[ch] != val) {
        _val[ch] = val;
        _last_change = systick_uptime();
    }

}

void NRF_parser::inavNrf24SetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    memset(_val, 0, sizeof(_val));
    // payload[0] and payload[1] are zero in DATA state
    // the AETR channels have 10 bit resolution
    uint8_t lowBits = payload[6]; // least significant bits for AETR
    set_val(RC_SPI_ROLL,      PWM_RANGE_MIN + ((payload[2] << 2) | (lowBits & 0x03)) ); // Aileron
    lowBits >>= 2;
    set_val(RC_SPI_PITCH,     PWM_RANGE_MIN + ((payload[3] << 2) | (lowBits & 0x03)) ); // Elevator
    lowBits >>= 2;
    set_val(RC_SPI_THROTTLE,  PWM_RANGE_MIN + ((payload[4] << 2) | (lowBits & 0x03)) ); // Throttle
    lowBits >>= 2;
    set_val(RC_SPI_YAW,       PWM_RANGE_MIN + ((payload[5] << 2) | (lowBits & 0x03)) ); // Rudder

    if (payloadSize == INAV_PROTOCOL_PAYLOAD_SIZE_MIN) {
        // small payload variant of protocol, supports 6 channels
        set_val(RC_SPI_AUX1, PWM_RANGE_MIN + (payload[7] << 2) );
        set_val(RC_SPI_AUX2, PWM_RANGE_MIN + (payload[1] << 2) );
        _channels = RC_SPI_AUX2+1;
    } else {
        // channel AUX1 is used for rate, as per the deviation convention
        const uint8_t rate = payload[7];
        // AUX1
        if (rate == RATE_HIGH) {
            set_val(RC_CHANNEL_RATE, PWM_RANGE_MAX);
        } else if (rate == RATE_MID) {
            set_val(RC_CHANNEL_RATE, PWM_RANGE_MIDDLE);
        } else {
            set_val(RC_CHANNEL_RATE, PWM_RANGE_MIN);
        }

        // channels AUX2 to AUX7 use the deviation convention
        const uint8_t flags = payload[8];
        set_val(RC_CHANNEL_FLIP, (flags & FLAG_FLIP) ? PWM_RANGE_MAX : PWM_RANGE_MIN ); // AUX2
        set_val(RC_CHANNEL_PICTURE, (flags & FLAG_PICTURE) ? PWM_RANGE_MAX : PWM_RANGE_MIN ); // AUX3
        set_val(RC_CHANNEL_VIDEO, (flags & FLAG_VIDEO) ? PWM_RANGE_MAX : PWM_RANGE_MIN ); // AUX4
        set_val(RC_CHANNEL_HEADLESS, (flags & FLAG_HEADLESS) ? PWM_RANGE_MAX : PWM_RANGE_MIN ); //AUX5
        set_val(RC_CHANNEL_RTH, (flags & FLAG_RTH) ? PWM_RANGE_MAX : PWM_RANGE_MIN ); // AUX6

        // channels AUX7 to AUX10 have 10 bit resolution
        lowBits = payload[13]; // least significant bits for AUX7 to AUX10
        set_val(RC_SPI_AUX7,  PWM_RANGE_MIN + ((payload[9] << 2) | (lowBits & 0x03)) );
        lowBits >>= 2;
        set_val(RC_SPI_AUX8,  PWM_RANGE_MIN + ((payload[10] << 2) | (lowBits & 0x03)) );
        lowBits >>= 2;
        set_val(RC_SPI_AUX9,  PWM_RANGE_MIN + ((payload[11] << 2) | (lowBits & 0x03)) );
        lowBits >>= 2;
        set_val(RC_SPI_AUX10, PWM_RANGE_MIN + ((payload[12] << 2) | (lowBits & 0x03)) );
        lowBits >>= 2;

        // channels AUX11 and AUX12 have 8 bit resolution
        set_val(RC_SPI_AUX11, PWM_RANGE_MIN + (payload[14] << 2) );
        set_val(RC_SPI_AUX12, PWM_RANGE_MIN + (payload[15] << 2) );
        
        _channels = RC_SPI_AUX12+1;
    }
    if (payloadSize == INAV_PROTOCOL_PAYLOAD_SIZE_MAX) {
        // large payload variant of protocol
        // channels AUX13 to AUX16 have 8 bit resolution
        set_val(RC_SPI_AUX13, PWM_RANGE_MIN + (payload[16] << 2) );
        set_val(RC_SPI_AUX14, PWM_RANGE_MIN + (payload[17] << 2) );
        _channels = RC_SPI_AUX14+1;
    }
    
    _last_signal = systick_uptime();
}

void NRF_parser::inavNrf24Setup(const uint32_t *fixedId)
{

    // sets PWR_UP, EN_CRC, CRCO - 2 byte CRC, only get IRQ pin interrupt on RX_DR
    NRF24L01_Initialize(BV(NRF24L01_00_CONFIG_EN_CRC) | BV(NRF24L01_00_CONFIG_CRCO) | BV(NRF24L01_00_CONFIG_MASK_MAX_RT) | BV(NRF24L01_00_CONFIG_MASK_TX_DS));

#ifdef USE_AUTO_ACKKNOWLEDGEMENT
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, BV(NRF24L01_01_EN_AA_ENAA_P0)); // auto acknowledgment on P0
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, BV(NRF24L01_02_EN_RXADDR_ERX_P0));
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES); // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0);
    NRF24L01_Activate(0x73); // activate R_RX_PL_WID, W_ACK_PAYLOAD, and W_TX_PAYLOAD_NOACK registers
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, BV(NRF24L01_1D_FEATURE_EN_ACK_PAY) | BV(NRF24L01_1D_FEATURE_EN_DPL));
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, BV(NRF24L01_1C_DYNPD_DPL_P0)); // enable dynamic payload length on P0
    //NRF24L01_Activate(0x73); // deactivate R_RX_PL_WID, W_ACK_PAYLOAD, and W_TX_PAYLOAD_NOACK registers

    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rxTxAddr, RX_TX_ADDR_LEN);
#else
    NRF24L01_SetupBasic();
#endif

    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_250Kbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    // RX_ADDR for pipes P1-P5 are left at default values
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxTxAddr, RX_TX_ADDR_LEN);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, payloadSize);

#ifdef USE_BIND_ADDRESS_FOR_DATA_STATE
    inavSetBound();
    UNUSED(fixedId);
#else
    fixedId = NULL; // !!TODO remove this once  configurator supports setting rx_id
    if (fixedId == NULL || *fixedId == 0) {
        fixedIdPtr = NULL;
        protocolState = STATE_BIND;
        inavRfChannelCount = 1;
        inavRfChannelIndex = 0;
        NRF24L01_SetChannel(INAV_RF_BIND_CHANNEL);
    } else {
        fixedIdPtr = (uint32_t*)fixedId;
        // use the rxTxAddr provided and go straight into DATA_STATE
        memcpy(rxTxAddr, fixedId, sizeof(uint32_t));
        rxTxAddr[4] = RX_TX_ADDR_4;
        inavSetBound();
    }
#endif

    NRF24L01_SetRxMode(); // enter receive mode to start listening for packets
    // put a null packet in the transmit buffer to be sent as ACK on first receive
    writeAckPayload(ackPayload, payloadSize);
}




///////////////////////

#define NRF24_CE_HI()       cs_assert()
#define NRF24_CE_LO()       cs_release()

// Instruction Mnemonics
// nRF24L01:  Table 16. Command set for the nRF24L01 SPI. Product Specification, p46
// nRF24L01+: Table 20. Command set for the nRF24L01+ SPI. Product Specification, p51
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

uint8_t rxSpiTransferByte(uint8_t data){
    uint8_t bt;
    // const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len
    nrf->transfer(&data,1,&bt,1);
    return bt;
}

uint8_t rxSpiWriteByte(uint8_t data)
{   
    ENABLE_RX();
    const uint8_t ret = rxSpiTransferByte(data);
    DISABLE_RX();
    return ret;
}

void rxSpiWriteCommand(uint8_t reg, uint8_t data){
    uint8_t bt[2] = {reg, data  };

    nrf->transfer(&bt,2,NULL,0);
}


void rxSpiWriteCommandMulti(uint8_t reg, const uint8_t *data, uint8_t length){
    uint8_t bt[length+1];
    bt[0]=reg;
    for(uint8_t i=0;i<length;i++) {
        bt[i+1] = data[i];
    }
    nrf->transfer(&bt,length,NULL,0);
}

void rxSpiReadCommand(uint8_t reg, uint8_t bt){
    nrf->transfer(reg);
    return nrf->transfer(bt);
}


bool rxSpiReadCommandMulti(uint8_t reg, uint8_t op, uint8_t *data, uint8_t length)
    const uint8_t ret = rxSpiTransferByte(reg);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = rxSpiTransferByte(op);
    }
    return ret;
}


void NRF24L01_WriteReg(uint8_t reg, uint8_t data)
{
    rxSpiWriteCommand(W_REGISTER | (REGISTER_MASK & reg), data);
}

void NRF24L01_WriteRegisterMulti(uint8_t reg, const uint8_t *data, uint8_t length)
{
    rxSpiWriteCommandMulti(W_REGISTER | ( REGISTER_MASK & reg), data, length);
}

/*
 * Transfer the payload to the nRF24L01 TX FIFO
 * Packets in the TX FIFO are transmitted when the
 * nRF24L01 next enters TX mode
 */
uint8_t NRF24L01_WritePayload(const uint8_t *data, uint8_t length)
{
    return rxSpiWriteCommandMulti(W_TX_PAYLOAD, data, length);
}

uint8_t NRF24L01_WriteAckPayload(const uint8_t *data, uint8_t length, uint8_t pipe)
{
    return rxSpiWriteCommandMulti(W_ACK_PAYLOAD | (pipe & 0x07), data, length);
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    return rxSpiReadCommand(R_REGISTER | (REGISTER_MASK & reg), NOP);
}

uint8_t NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length)
{
    return rxSpiReadCommandMulti(R_REGISTER | (REGISTER_MASK & reg), NOP, data, length);
}

/*
 * Read a packet from the nRF24L01 RX FIFO.
 */
uint8_t NRF24L01_ReadPayload(uint8_t *data, uint8_t length)
{
    return rxSpiReadCommandMulti(R_RX_PAYLOAD, NOP, data, length);
}



/*
 * Empty the transmit FIFO buffer.
 */
void NRF24L01_FlushTx()
{
    rxSpiWriteByte(FLUSH_TX);
}

/*
 * Empty the receive FIFO buffer.
 */
void NRF24L01_FlushRx()
{
    rxSpiWriteByte(FLUSH_RX);
}

uint8_t NRF24L01_Activate(uint8_t code)
{
    return rxSpiWriteCommand(ACTIVATE, code);
}

// standby configuration, used to simplify switching between RX, TX, and Standby modes
static uint8_t standbyConfig;

void NRF24L01_Initialize(uint8_t baseConfig)
{
    standbyConfig = BV(NRF24L01_00_CONFIG_PWR_UP) | baseConfig;
    NRF24_CE_LO();
    // nRF24L01+ needs 100 milliseconds settling time from PowerOnReset to PowerDown mode
    static const timeUs_t settlingTimeUs = 100000;
    const timeUs_t currentTimeUs = micros();
    if (currentTimeUs < settlingTimeUs) {
        delayMicroseconds(settlingTimeUs - currentTimeUs);
    }
    // now in PowerDown mode
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, standbyConfig); // set PWR_UP to enter Standby mode
    // nRF24L01+ needs 4500 microseconds from PowerDown mode to Standby mode, for crystal oscillator startup
    delayMicroseconds(4500);
    // now in Standby mode
}

/*
 * Common setup of registers
 */
void NRF24L01_SetupBasic(void)
{
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00); // No auto acknowledgment
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, BV(NRF24L01_02_EN_RXADDR_ERX_P0));
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES); // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00); // Disable dynamic payload length on all pipes
}


/*
 * Enter standby mode
 */
void NRF24L01_SetStandbyMode(void)
{
    // set CE low and clear the PRIM_RX bit to enter standby mode
    NRF24_CE_LO();
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, standbyConfig);
}

/*
 * Enter receive mode
 */
void NRF24L01_SetRxMode(void)
{
    NRF24_CE_LO(); // drop into standby mode
    // set the PRIM_RX bit
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, standbyConfig | BV(NRF24L01_00_CONFIG_PRIM_RX));
    NRF24L01_ClearAllInterrupts();
    // finally set CE high to start enter RX mode
    NRF24_CE_HI();
    // nRF24L01+ will now transition from Standby mode to RX mode after 130 microseconds settling time
}

/*
 * Enter transmit mode. Anything in the transmit FIFO will be transmitted.
 */
void NRF24L01_SetTxMode(void)
{
    // Ensure in standby mode, since can only enter TX mode from standby mode
    NRF24L01_SetStandbyMode();
    NRF24L01_ClearAllInterrupts();
    // pulse CE for 10 microseconds to enter TX mode
    NRF24_CE_HI();
    delayMicroseconds(10);
    NRF24_CE_LO();
    // nRF24L01+ will now transition from Standby mode to TX mode after 130 microseconds settling time.
    // Transmission will then begin and continue until TX FIFO is empty.
}

void NRF24L01_ClearAllInterrupts(void)
{
    // Writing to the STATUS register clears the specified interrupt bits
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BV(NRF24L01_07_STATUS_RX_DR) | BV(NRF24L01_07_STATUS_TX_DS) | BV(NRF24L01_07_STATUS_MAX_RT));
}


bool NRF24L01_ReadPayloadIfAvailable(uint8_t *data, uint8_t length)
{
    if (NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS) & BV(NRF24L01_17_FIFO_STATUS_RX_EMPTY)) {
        return false;
    }
    NRF24L01_ReadPayload(data, length);
    return true;
}



#endif // BOARD_NRF24_CS_PIN


#include "AP_Radio_config.h"

#if AP_RADIO_BK2425_ENABLED

// --------------------------------------------------------------------
//  low level driver for the beken radio on SPI
// --------------------------------------------------------------------

#include "driver_bk2425.h"

#include <utility>
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS.h>
using namespace ChibiOS;

//#pragma GCC optimize("O0")

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#endif

#define BEKEN_SELECT()      (dev->set_chip_select(true))
#define BEKEN_DESELECT()    (dev->set_chip_select(false))
#define BEKEN_CE_HIGH()     (palSetLine(HAL_GPIO_PIN_RADIO_CE)) // (hal.gpio->write(HAL_CHIBIOS_GPIO_RADIO_CE, 1))
#define BEKEN_CE_LOW()      (palClearLine(HAL_GPIO_PIN_RADIO_CE)) // (hal.gpio->write(HAL_CHIBIOS_GPIO_RADIO_CE, 0))
#define BEKEN_PA_HIGH()     (palSetLine(HAL_GPIO_PIN_RADIO_PA_CTL)) // (hal.gpio->write(HAL_CHIBIOS_GPIO_RADIO_PA_CTL, 1))
#define BEKEN_PA_LOW()      (palClearLine(HAL_GPIO_PIN_RADIO_PA_CTL)) // (hal.gpio->write(HAL_CHIBIOS_GPIO_RADIO_PA_CTL, 0))

#if SUPPORT_BK_DEBUG_PINS
#define DEBUG1_HIGH()       (palSetLine(HAL_GPIO_PIN_DEBUG1))
#define DEBUG1_LOW()        (palClearLine(HAL_GPIO_PIN_DEBUG1))
#define DEBUG2_HIGH()       (palSetLine(HAL_GPIO_PIN_DEBUG2))
#define DEBUG2_LOW()        (palClearLine(HAL_GPIO_PIN_DEBUG2))
#else
#define DEBUG1_HIGH()       do {} while (0)
#define DEBUG1_LOW()        do {} while (0)
#define DEBUG2_HIGH()       do {} while (0)
#define DEBUG2_LOW()        do {} while (0)
#endif

// --------------------------------------------------------------------
// Radio initialisation tables
// --------------------------------------------------------------------

#if (TX_SPEED==250)
ITX_SPEED Radio_Beken::gTxSpeed = ITX_250;
#elif (TX_SPEED==100)
ITX_SPEED Radio_Beken::gTxSpeed = ITX_1000;
#elif (TX_SPEED==2000)
ITX_SPEED Radio_Beken::gTxSpeed = ITX_2000;
#endif


// --------------------------------------------------------------------
static const uint8_t Bank1_RegTable[ITX_MAX][IREG_MAX][5]= {
    // (TX_SPEED == 250u) // [ITX_250]
    {
        { BK2425_R1_4,  0xf9,0x96,0x8a,0xdb }, // 0xDB8A96f9ul, // [IREG1_4]  REG4 250kbps
        { BK2425_R1_5,  0x24,0x06,0x0f,0xb6 }, // 0xB60F0624ul, // [IREG1_5]  REG5 250kbps
        PLL_SPEED,                                              // [IREG1_12] REG12
        { BK2425_R1_13, 0x36,0xb4,0x80,0x00 }, // 0x36B48000ul, // [IREG1_13] REG13
        { BK2425_R1_4,  0xff,0x96,0x8a,0xdb }, // 0xDB8A96f9ul, // [IREG1_4A] REG4 250kbps
    },
    // (TX_SPEED == 1000u) [ITX_1000]
    {
        { BK2425_R1_4,  0xf9,0x96,0x82,0x1b }, // 0x1B8296f9ul, // [IREG1_4]  REG4 1Mbps
        { BK2425_R1_5,  0x24,0x06,0x0f,0xa6 }, // 0xA60F0624ul, // [IREG1_5]  REG5 1Mbps
        PLL_SPEED,                                              // [IREG1_12] REG12
        { BK2425_R1_13, 0x36,0xb4,0x80,0x00 }, // 0x36B48000ul, // [IREG1_13] REG13
        { BK2425_R1_4,  0xff,0x96,0x82,0x1b }, // 0x1B8296f9ul, // [IREG1_4A] REG4 1Mbps
    },
    // (TX_SPEED == 2000u) [ITX_2000]
    {
        { BK2425_R1_4,  0xf9,0x96,0x82,0xdb }, // 0xdb8296f9ul, // [IREG1_4]  REG4 2Mbps
        { BK2425_R1_5,  0x24,0x06,0x0f,0xb6 }, // 0xb60f0624ul, // [IREG1_5]  REG5 2Mbps
        PLL_SPEED,                                              // [IREG1_12] REG12
        { BK2425_R1_13, 0x36,0xb4,0x80,0x00 }, // 0x36B48000ul, // [IREG1_13] REG13
        { BK2425_R1_4,  0xff,0x96,0x82,0xdb }, // 0xdb8296f9ul, // [IREG1_4A] REG4 2Mbps
    },
    // (TX_SPEED == 0u) // [ITX_CARRIER]
    {
        { BK2425_R1_4,  0xf9,0x96,0x82,0x21 }, // 0xF9968221ul, // [IREG1_4] REG4 carrier
        { BK2425_R1_5,  0x24,0x06,0x0f,0xb6 }, // 0xB60F0624ul, // [IREG1_5] REG5 250kbps
        PLL_SPEED,                                              // [IREG1_12] REG12
        { BK2425_R1_13, 0x36,0xb4,0x80,0x00 }, // 0x36B48000ul, // [IREG1_13] REG13
        { BK2425_R1_4,  0xff,0x96,0x82,0x21 }, // 0xDB8A96f9ul, // [IREG1_4A] REG4 250kbps
    }
};


// --------------------------------------------------------------------
static const uint8_t Bank0_Reg6[ITX_MAX][2] = {
    {BK_RF_SETUP,   0x27}, //  250kbps (6) 0x27=250kbps
    {BK_RF_SETUP,   0x07}, // 1000kbps (6) 0x07=1Mbps, high gain, high txpower
    {BK_RF_SETUP,   0x2F}, // 2000kbps (6) 0x2F=2Mbps, high gain, high txpower
    {BK_RF_SETUP,   0x37}, //  250kbps (6) 0x10=carrier
};

// --------------------------------------------------------------------
static const uint8_t Bank1_Reg14[]= {
    0x41,0x20,0x08,0x04,0x81,0x20,0xcf,0xF7,0xfe,0xff,0xff
};

// --------------------------------------------------------------------
// Bank0 register initialization value
static const uint8_t Bank0_Reg[][2]= {

#if 0
    {BK_CONFIG,     BK_CONFIG_PWR_UP | BK_CONFIG_PRIM_RX }, // (0) 0x0F=Rx, PowerUp, no crc, all interrupts enabled
    {BK_EN_AA,      0x00}, // (1) 0x00=No auto acknowledge packets on all 6 data pipes (0..5)
    {BK_EN_RXADDR,  0x02}, // (2) 0x01=1 or 2 out of 6 data pipes enabled (pairing heartbeat and my tx)
    {BK_SETUP_AW,   0x03}, // (3) 0x01=3 byte address width
#else
    {BK_CONFIG,     BK_CONFIG_EN_CRC | BK_CONFIG_CRCO | BK_CONFIG_PWR_UP | BK_CONFIG_PRIM_RX }, // (0) 0x0F=Rx, PowerUp, crc16, all interrupts enabled
    {BK_EN_AA,      0x00}, // (1) 0x00=No auto acknowledge packets on all 6 data pipes (0..5)
    {BK_EN_RXADDR,  0x03}, // (2) 0x01=1 or 2 out of 6 data pipes enabled (pairing heartbeat and my tx)
    {BK_SETUP_AW,   0x03}, // (3) 0x03=5 byte address width
#endif
    {BK_SETUP_RETR, 0x00}, // (4) 0x00=No retransmissions
    {BK_RF_CH,      0x17}, // (5) 0x17=2423Mhz default frequency

    // Comment in Beken code says that 0x0F or 0x2F=2Mbps; 0x07=1Mbps; 0x27=250Kbps
#if (TX_SPEED == 2000)
    {BK_RF_SETUP,   0x2F},      // (6) 0x2F=2Mbps, high gain, high txpower
#elif (TX_SPEED == 1000)
    {BK_RF_SETUP,   0x07},      // (6) 0x07=1Mbps, high gain, high txpower
#elif (TX_SPEED == 250)
    {BK_RF_SETUP,   0x27},       // (6) 0x27=250kbps
    //{BK_RF_SETUP,   0x21},    // (6) 0x27=250kbps, lowest txpower
#endif

    {BK_STATUS,     0x07},          // (7) 7=no effect
    {BK_OBSERVE_TX, 0x00},          // (8) (no effect)
    {BK_CD,         0x00},          // (9) Carrier detect (no effect)
    // (10) = 5 byte register
    // (11) = 5 byte register
    {BK_RX_ADDR_P2, 0xc3},          // (12) rx address for data pipe 2
    {BK_RX_ADDR_P3, 0xc4},          // (13) rx address for data pipe 3
    {BK_RX_ADDR_P4, 0xc5},          // (14) rx address for data pipe 4
    {BK_RX_ADDR_P5, 0xc6},          // (15) rx address for data pipe 5
    // (16) = 5 byte register
    {BK_RX_PW_P0,   PACKET_LENGTH_RX_CTRL},          // (17) size of rx data pipe 0
    {BK_RX_PW_P1,   PACKET_LENGTH_RX_BIND},          // (18) size of rx data pipe 1
    {BK_RX_PW_P2,   0x20},          // (19) size of rx data pipe 2
    {BK_RX_PW_P3,   0x20},          // (20) size of rx data pipe 3
    {BK_RX_PW_P4,   0x20},          // (21) size of rx data pipe 4
    {BK_RX_PW_P5,   0x20},          // (22) size of rx data pipe 5
    {BK_FIFO_STATUS,0x00},          // (23) fifo status
    // (24,25,26,27)
    {BK_DYNPD,      0x3F},          // (28) 0x3f=enable dynamic payload length for all 6 data pipes
    {BK_FEATURE,    BK_FEATURE_EN_DPL | BK_FEATURE_EN_ACK_PAY | BK_FEATURE_EN_DYN_ACK }  // (29) 7=enable ack, no ack, dynamic payload length
};

// ----------------------------------------------------------------------------
const uint8_t RegPower[8][2] = {
    { OUTPUT_POWER_REG4_0, OUTPUT_POWER_REG6_0 },
    { OUTPUT_POWER_REG4_1, OUTPUT_POWER_REG6_1 },
    { OUTPUT_POWER_REG4_2, OUTPUT_POWER_REG6_2 },
    { OUTPUT_POWER_REG4_3, OUTPUT_POWER_REG6_3 },
    { OUTPUT_POWER_REG4_4, OUTPUT_POWER_REG6_4 },
    { OUTPUT_POWER_REG4_5, OUTPUT_POWER_REG6_5 },
    { OUTPUT_POWER_REG4_6, OUTPUT_POWER_REG6_6 },
    { OUTPUT_POWER_REG4_7, OUTPUT_POWER_REG6_7 },
};

// --------------------------------------------------------------------
// Generic functions
// --------------------------------------------------------------------

// --------------------------------------------------------------------
// constructor
Radio_Beken::Radio_Beken(AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev) :
    dev(std::move(_dev))
{
    ResetAddress();
}

// --------------------------------------------------------------------
// Use the default addresses
void Radio_Beken::ResetAddress(void)
{
    // Set the default address
    TX_Address[0] = 0x33;
    TX_Address[1] = RX0_Address[1] = 0x00;
    TX_Address[2] = RX0_Address[2] = 0x59;
    TX_Address[3] = RX0_Address[3] = 0x00;
    TX_Address[4] = RX0_Address[4] = 0x00;
    RX0_Address[0] = 0x31;
    RX1_Address[0] = 0x32;
    RX1_Address[1] = 0x99;
    RX1_Address[2] = 0x59;
    RX1_Address[3] = 0xC6;
    RX1_Address[4] = 0x2D;
}

// --------------------------------------------------------------------
// Raw SPI access functions
// --------------------------------------------------------------------

// --------------------------------------------------------------------
void Radio_Beken::ReadRegisterMulti(uint8_t address, uint8_t *data, uint8_t len)
{
    uint8_t tx[len+1];
    uint8_t rx[len+1];
    memset(tx, 0, len+1);
    memset(rx, 0, len+1);
    tx[0] = address;
    DEBUG2_HIGH();
    (void)dev->transfer_fullduplex(tx, rx, len+1);
    DEBUG2_LOW();
    memcpy(data, &rx[1], len);
}

// --------------------------------------------------------------------
void Radio_Beken::WriteRegisterMulti(uint8_t address, const uint8_t *data, uint8_t len)
{
    uint8_t tx[len+1];
    uint8_t rx[len+1];
    memset(rx, 0, len+1);
    tx[0] = address;
    memcpy(&tx[1], data, len);
    DEBUG2_HIGH();
    (void)dev->transfer_fullduplex(tx, rx, len+1);
    DEBUG2_LOW();
}

// --------------------------------------------------------------------
// Low-level Beken functions
// --------------------------------------------------------------------

// --------------------------------------------------------------------
uint8_t Radio_Beken::ReadStatus(void)
{
    uint8_t tx = BK_NOP;
    uint8_t rx = 0;
    DEBUG2_HIGH();
    (void)dev->transfer_fullduplex(&tx, &rx, 1);
    DEBUG2_LOW();
    return rx; // Status
}

// --------------------------------------------------------------------
uint8_t Radio_Beken::ReadReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];
    memset(tx, 0, 2);
    memset(rx, 0, 2);
    tx[0] = reg | BK_READ_REG;
    DEBUG2_HIGH();
    (void)dev->transfer_fullduplex(tx, rx, 2);
    DEBUG2_LOW();
    return rx[1];
}

// --------------------------------------------------------------------
uint8_t Radio_Beken::Strobe(uint8_t address)
{
    uint8_t tx = address;
    uint8_t rx = 0;
    DEBUG2_HIGH();
    (void)dev->transfer_fullduplex(&tx, &rx, 1);
    DEBUG2_LOW();
    return rx; // Status
}

// --------------------------------------------------------------------
// Set which register bank we are accessing
void Radio_Beken::SetRBank(uint8_t bank) // 1:Bank1 0:Bank0
{
    uint8_t lastbank = ReadStatus() & BK_STATUS_RBANK;
    if (!lastbank != !bank) {
        uint8_t tx[2];
        uint8_t rx[2];
        tx[0] = BK_ACTIVATE_CMD;
        tx[1] = 0x53;
        DEBUG2_HIGH();
        (void)dev->transfer_fullduplex(&tx[0], &rx[0], 2);
        DEBUG2_LOW();
    }
}

// --------------------------------------------------------------------
void Radio_Beken::WriteReg(uint8_t address, uint8_t data)
{
    uint8_t tx[2];
    uint8_t rx[2];
    memset(rx, 0, 2);
    tx[0] = address; // done by caller | BK_WRITE_REG;
    tx[1] = data;
    DEBUG2_HIGH();
    (void)dev->transfer_fullduplex(tx, rx, 2);
    DEBUG2_LOW();
}

// --------------------------------------------------------------------
void Radio_Beken::WriteRegisterMultiBank1(uint8_t address, const uint8_t *data, uint8_t length)
{
    SetRBank(1);
    WriteRegisterMulti(address, data, length);
    SetRBank(0);
}

// --------------------------------------------------------------------
// High-level Beken functions
// --------------------------------------------------------------------

// --------------------------------------------------------------------
// Set the radio transmission power of the beken
// Prerequisite: We should be in idle mode before calling this function
void Radio_Beken::SetPower(uint8_t power)
{
    if (power > 7) {
        power = 7;
    }
    uint8_t oldready = bkReady;
    bkReady = 0;
    hal.scheduler->delay(100); // delay more than 50ms.
    SetRBank(1);
    {
        const uint8_t* p = &Bank1_RegTable[fcc.CW_mode ? ITX_CARRIER : gTxSpeed][IREG1_4][0];
        uint8_t idx = *p++;
        uint8_t buf[4];
        buf[0] = *p++;
        buf[1] = *p++;
        buf[2] = *p++;
        buf[3] = *p++;
        buf[0] &= ~0x38;
        buf[0] |= (RegPower[power][0] << 3); // Bits 27..29
        WriteRegisterMulti((BK_WRITE_REG|idx), buf, 4);
    }
    hal.scheduler->delay(100); // delay more than 50ms.
    SetRBank(0);
    hal.scheduler->delay(100);

    uint8_t setup = ReadReg(BK_RF_SETUP);
    setup &= ~(3 << 1);
    setup |= (RegPower[power][1] << 1); // Bits 1..2
    if (fcc.CW_mode) {
        setup |= 0x10;
    }
    WriteReg(BK_WRITE_REG|BK_RF_SETUP, setup);
    bkReady = oldready;
    fcc.power = power;
}

// --------------------------------------------------------------------
// Set the physical radio transmission frequency of the beken
void Radio_Beken::SetChannel(uint8_t freq)
{
    lastTxChannel = freq;
    WriteReg(BK_WRITE_REG|BK_RF_CH, freq);
}

// --------------------------------------------------------------------
// Set the radio transmission mode of the beken
// Enable/disable the carrier sending mode
// Prerequisite: We should be in idle mode before calling this function
void Radio_Beken::SetCwMode(uint8_t cw)
{
    uint8_t oldready = bkReady;
    bkReady = 0;
    hal.scheduler->delay(100); // delay more than 50ms.
    SetRBank(1);
    {
        const uint8_t* p = &Bank1_RegTable[cw ? ITX_CARRIER : gTxSpeed][IREG1_4][0];
        uint8_t idx = *p++;
        uint8_t buf[4];
        buf[0] = *p++;
        buf[1] = *p++;
        buf[2] = *p++;
        buf[3] = *p++;
        buf[0] &= ~0x38;
        buf[0] |= (RegPower[fcc.power & 7][0] << 3); // Bits 27..29
        WriteRegisterMulti((BK_WRITE_REG|idx), buf, 4);
    }
    hal.scheduler->delay(100); // delay more than 50ms.
    SetRBank(0);
    hal.scheduler->delay(100); // delay more than 50ms.

    uint8_t setup = ReadReg(BK_RF_SETUP);
    setup &= ~(3 << 1);
    setup |= (RegPower[fcc.power & 7][1] << 1); // Bits 1..2
    if (cw) {
        setup |= 0x10;
    }
    WriteReg((BK_WRITE_REG|BK_RF_SETUP), setup);
    fcc.CW_mode = cw != 0;
    bkReady = oldready;
}

// --------------------------------------------------------------------
// Enable/disable the CRC receive  mode
// Prerequisite: We should be in idle mode before calling this function
void Radio_Beken::SetCrcMode(uint8_t disable_crc)
{
    uint8_t oldready = bkReady;
    bkReady = 0;

    uint8_t config = ReadReg(BK_CONFIG);
    if (disable_crc) {
        config &= ~(BK_CONFIG_EN_CRC | BK_CONFIG_CRCO);    // Disable CRC
    } else {
        config |= (BK_CONFIG_EN_CRC | BK_CONFIG_CRCO);    // Enable CRC
    }
    WriteReg((BK_WRITE_REG|BK_CONFIG), config);
    fcc.disable_crc = (disable_crc != 0);
    bkReady = oldready;
}

// ----------------------------------------------------------------------------
// Enable the carrier detect feature: Bank1 Reg5 Bit 18
void Radio_Beken::EnableCarrierDetect(bool bEnable)
{
    if (bEnable == fcc.enable_cd) {
        return;
    }
    uint8_t oldready = bkReady;
    bkReady = 0;
    SetRBank(1);
    {
        const uint8_t* p = &Bank1_RegTable[gTxSpeed][IREG1_5][0];
        uint8_t idx = *p++;
        uint8_t buf[4];
        buf[0] = *p++;
        buf[1] = *p++;
        buf[2] = *p++;
        buf[3] = *p++;
        if (bEnable) {
            buf[1] &= ~0x04;
        }
        WriteRegisterMulti((BK_WRITE_REG|idx), buf, 4);
    }
    SetRBank(0);
    bkReady = oldready;
    fcc.enable_cd = bEnable;
}

// ----------------------------------------------------------------------------
// Returns true if a carrier is detected
bool Radio_Beken::CarrierDetect(void)
{
    if (fcc.enable_cd) {
        if (ReadReg(BK_CD) & 0x01) {
            return true;
        }
    }
    return false;
}

// ----------------------------------------------------------------------------
void Radio_Beken::SetFactoryMode(uint8_t factory)
{
    uint8_t oldready = bkReady;
    bkReady = 0;

    // Set receive/transmit addresses
    if (factory) {
        // For factory modes, use fixed addresses
        TX_Address[0] = 0x35;
        TX_Address[1] = RX1_Address[1] = RX0_Address[1] = 0x99;
        TX_Address[2] = RX1_Address[2] = RX0_Address[2] = 0x59;
        TX_Address[3] = RX1_Address[3] = RX0_Address[3] = 0xC6;
        TX_Address[4] = RX1_Address[4] = RX0_Address[4] = factory;
        RX0_Address[0] = 0x34;
        RX1_Address[0] = 0x43;
    } else {
        // For normal modes, use the default addresses
        ResetAddress();
    }

    // Write the addresses to the registers
    WriteRegisterMulti((BK_WRITE_REG|BK_RX_ADDR_P0), RX0_Address, 5);
    WriteRegisterMulti((BK_WRITE_REG|BK_RX_ADDR_P1), RX1_Address, 5);
    WriteRegisterMulti((BK_WRITE_REG|BK_TX_ADDR), TX_Address, 5);
    WriteReg(BK_WRITE_REG|BK_EN_RXADDR, 0x03);

    // Frequency is set by the caller
    fcc.factory_mode = factory;
    bkReady = oldready;
}

// ----------------------------------------------------------------------------
bool Radio_Beken::Reset(void)
{
    //...
    hal.scheduler->delay_microseconds(1000);
    return 0;
}

// ----------------------------------------------------------------------------
// Delay after changing chip-enable
// This can be called from within the interrupt response thread
void Radio_Beken::DelayCE(void)
{
    DEBUG1_LOW();
    hal.scheduler->delay_microseconds(50);
    DEBUG1_HIGH();
}

// ----------------------------------------------------------------------------
bool Radio_Beken::WasTxMode(void)
{
    // Were we transmitting something?
    return bkMode == BKRADIO_TX;
}

// ----------------------------------------------------------------------------
bool Radio_Beken::WasRxMode(void)
{
    // Were we receiving something?
    return bkMode == BKRADIO_RX;
}

// ----------------------------------------------------------------------------
// Switch to Rx mode
void Radio_Beken::SwitchToRxMode(void)
{
    uint8_t value;

    Strobe(BK_FLUSH_RX); // flush Rx
    value = ReadStatus(); // read register STATUS's value
    WriteReg(BK_WRITE_REG|BK_STATUS, value); // clear RX_DR or TX_DS or MAX_RT interrupt flag

    BEKEN_CE_LOW();
    DelayCE();
    value = ReadReg(BK_CONFIG);	// read register CONFIG's value
    value |= BK_CONFIG_PRIM_RX; // set bit 0
    value |= BK_CONFIG_PWR_UP;
    WriteReg(BK_WRITE_REG | BK_CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..

    BEKEN_CE_HIGH();
    //BEKEN_PA_LOW(); // we dont have a PA on the RX side
    bkMode = BKRADIO_RX;
}

// ----------------------------------------------------------------------------
// switch to Tx mode
void Radio_Beken::SwitchToTxMode(void)
{
    uint8_t value;
    Strobe(BK_FLUSH_TX); // flush half-sent Tx
    Strobe(BK_FLUSH_RX); // flush half-received rx

    //  BEKEN_PA_HIGH();
    BEKEN_CE_LOW();
    DelayCE();
    value = ReadReg(BK_CONFIG); // read register CONFIG's value
    value &= ~BK_CONFIG_PRIM_RX; // Clear bit 0 (PTX)
    value |= BK_CONFIG_PWR_UP;
    WriteReg(BK_WRITE_REG | BK_CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled.
    //  BEKEN_CE_HIGH();
    bkMode = BKRADIO_TX;
}

// ----------------------------------------------------------------------------
// switch to Idle mode
void Radio_Beken::SwitchToIdleMode(void)
{
    Strobe(BK_FLUSH_TX); // flush Tx
    Strobe(BK_FLUSH_RX); // flush Rx

    BEKEN_PA_LOW();
    BEKEN_CE_LOW();
    DelayCE();
    bkMode = BKRADIO_IDLE;
}

// ----------------------------------------------------------------------------
// Switch to Sleep mode
void Radio_Beken::SwitchToSleepMode(void)
{
    uint8_t value;
    Strobe(BK_FLUSH_RX); // flush Rx
    Strobe(BK_FLUSH_TX); // flush Tx
    value = ReadStatus(); // read register STATUS's value
    WriteReg(BK_WRITE_REG|BK_STATUS, value); // clear RX_DR or TX_DS or MAX_RT interrupt flag

    BEKEN_PA_LOW();
    BEKEN_CE_LOW();
    DelayCE();
    value = ReadReg(BK_CONFIG);	// read register CONFIG's value
    value |= BK_CONFIG_PRIM_RX; // Receive mode
    value &= ~BK_CONFIG_PWR_UP; // Power down
    WriteReg(BK_WRITE_REG | BK_CONFIG, value); // Clear PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..
    // Stay low
    BEKEN_CE_LOW();
    bkMode = BKRADIO_SLEEP;
}

// ----------------------------------------------------------------------------
void Radio_Beken::InitBank0Registers(ITX_SPEED spd)
{
    int8_t i;

    //********************Write Bank0 register******************
    for (i=20; i >= 0; i--) { // From BK_FIFO_STATUS back to beginning of table
        uint8_t idx = Bank0_Reg[i][0];
        uint8_t value = Bank0_Reg[i][1];
        if (idx == BK_RF_SETUP) { // Adjust for speed
            value = Bank0_Reg6[spd][1];
        }
        WriteReg((BK_WRITE_REG|idx), value);
    }

    // Enable features
    i = ReadReg(BK_FEATURE);
    if (i == 0) { // i!=0 showed that chip has been actived. So do not active again (as that would toggle these features off again).
        WriteReg(BK_ACTIVATE_CMD,0x73);    // Activate the BK_FEATURE register. (This command must NOT have BK_WRITE_REG set)
    }
    for (i = 22; i >= 21; i--) {
        WriteReg((BK_WRITE_REG|Bank0_Reg[i][0]),Bank0_Reg[i][1]);
    }

    // Set the various 5 byte addresses
    WriteRegisterMulti((BK_WRITE_REG|BK_RX_ADDR_P0),RX0_Address,5); // reg 10 - Rx0 addr
    WriteRegisterMulti((BK_WRITE_REG|BK_RX_ADDR_P1),RX1_Address,5); // REG 11 - Rx1 addr
    WriteRegisterMulti((BK_WRITE_REG|BK_TX_ADDR),TX_Address,5); // REG 16 - TX addr
    WriteReg(BK_WRITE_REG|BK_EN_RXADDR, 0x03);

}

// ----------------------------------------------------------------------------
void Radio_Beken::InitBank1Registers(ITX_SPEED spd)
{
    int16_t i;

    for (i = IREG1_4; i <= IREG1_13; i++) {
        const uint8_t* p = &Bank1_RegTable[spd][i][0];
        uint8_t idx = *p++;
        WriteRegisterMulti((BK_WRITE_REG|idx), p, 4);
    }
    WriteRegisterMulti((BK_WRITE_REG|BK2425_R1_14),&(Bank1_Reg14[0]),11);

    //toggle REG4<25,26>
    {
        const uint8_t* p = &Bank1_RegTable[spd][IREG1_4A][0];
        uint8_t idx = *p++;
        WriteRegisterMulti((BK_WRITE_REG|idx), p, 4);
    }
    {
        const uint8_t* p = &Bank1_RegTable[spd][IREG1_4][0];
        uint8_t idx = *p++;
        WriteRegisterMulti((BK_WRITE_REG|idx), p, 4);
    }
}

// ----------------------------------------------------------------------------
// Set the rx and tx addresses
void Radio_Beken::SetAddresses(const uint8_t* txaddr)
{
    TX_Address[1] = RX0_Address[1] = txaddr[1];
    TX_Address[3] = RX0_Address[3] = txaddr[3];
    TX_Address[4] = RX0_Address[4] = txaddr[4];
    WriteRegisterMulti((BK_WRITE_REG|BK_RX_ADDR_P0), RX0_Address, 5);
    WriteRegisterMulti((BK_WRITE_REG|BK_TX_ADDR), TX_Address, 5);
    WriteReg(BK_WRITE_REG|BK_EN_RXADDR, 0x03);
}

// ----------------------------------------------------------------------------
bool Radio_Beken::ClearAckOverflow(void)
{
    uint8_t status = ReadStatus();
    if ((BK_STATUS_MAX_RT & status) == 0) {
        return false;
    } else {
        WriteReg((BK_WRITE_REG|BK_STATUS), BK_STATUS_MAX_RT);
        return true;
    }
}


// ----------------------------------------------------------------------------
// Write a data packet
bool Radio_Beken::SendPacket(uint8_t type, ///< WR_TX_PLOAD or W_TX_PAYLOAD_NOACK_CMD
                             const uint8_t* pbuf, ///< a buffer pointer
                             uint8_t len) ///< packet length in bytes
{
    uint8_t fifo_sta = ReadReg(BK_FIFO_STATUS);	// read register FIFO_STATUS's value
    bool returnValue = ClearAckOverflow();

    if (!(fifo_sta & BK_FIFO_STATUS_TX_FULL)) { // if not full, send data
        numTxPackets++;
        WriteRegisterMulti(type, pbuf, len); // Writes data to buffer A0,B0,A8
        BEKEN_CE_HIGH(); // Wait until FIFO has the data before sending it.
    }
    return returnValue;
}

// ----------------------------------------------------------------------------
// For debugging - tell us the current beken register values (from bank 0)
// This just prints it to the UART rather than to the console over WiFi
void Radio_Beken::DumpRegisters(void)
{
    uint8_t i;
    for (i = 0; i <= BK_FEATURE; ++i) {
        uint8_t len = 1;
        switch (i) {
        case 10: case 11: case 16: len = 5; break;
        case 24: case 25: case 26: case 27: len = 0; break;
        default: len = 1; break;
        };
        if (len == 1) {
            //printf("Bank0reg%d : %x\r\n", i, ReadReg(i));
        } else if (len == 5) {
            uint8_t data[5];
            ReadRegisterMulti(i, &data[0], len);
            //printf("Bank0reg%d : %x %x %x %x %x\r\n", i, data[0], data[1], data[2], data[3], data[4]);
        }
    }
    SetRBank(1);
    for (i = IREG1_4; i <= IREG1_13; ++i) {
        uint8_t len = 4;
        uint8_t data[4];
        ReadRegisterMulti(i, &data[0], len);
        //uint8_t idx = Bank1_RegTable[0][i][0];
        //printf("Bank1reg%d : %x %x %x %x\r\n", idx, data[0], data[1], data[2], data[3]);
    }
    SetRBank(0);
}

#endif // AP_RADIO_BK2425_ENABLED

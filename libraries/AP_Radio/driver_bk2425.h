//----------------------------------------------------------------------------------
// low level driver for the Beken BK2425 radio on SPI
// Note: Under ChiBios the knowledge of which pins are which is not in the driver.
// But ultimately comes from hwdef.dat
//----------------------------------------------------------------------------------

#pragma once

#include "AP_Radio_config.h"

#if AP_RADIO_BK2425_ENABLED

#include <AP_HAL/AP_HAL.h>

#define SUPPORT_BK_DEBUG_PINS 0 // 0=UART6 is for GPS, 1=UART6 is debug gpio
#define TX_SPEED 250u // Default transmit speed in kilobits per second.

/** Channel hopping parameters. Values are in MHz from 2400Mhz. */
enum CHANNEL_MHZ_e {
    CHANNEL_MIN_PHYSICAL = 0, ///< Minimum physical channel that is possible
    CHANNEL_MAX_PHYSICAL = 83, ///< Maximum physical channel that is possible
    CHANNEL_FCC_LOW = 10, ///< Minimum physical channel that will pass the FCC tests
    CHANNEL_FCC_HIGH = 72, ///< Maximum physical channel that will pass the FCC tests
    CHANNEL_FCC_MID = 41, ///< A representative physical channel
};

enum {
    CHANNEL_COUNT_LOGICAL = 16, ///< The maximum number of entries in each frequency table
    CHANNEL_BASE_TABLE = 0, ///< The table used for non wifi boards
    CHANNEL_SAFE_TABLE = 3, ///< A table that will receive packets even if wrong
    CHANNEL_NUM_TABLES = 6, ///< The number of tables
    CHANNEL_COUNT_TEST = 16, ///< The number of test mode tables
};


// ----------------------------------------------------------------------------
// Packet format definition
// ----------------------------------------------------------------------------

/** The type of packets being sent between controller and drone */
enum BK_PKT_TYPE_E {
    BK_PKT_TYPE_INVALID      = 0,    ///< Invalid packet from empty packets or bad CRC
    BK_PKT_TYPE_CTRL_FOUND   = 0x10, ///< (Tx->Drone) User control - known receiver
    BK_PKT_TYPE_CTRL_LOST    = 0x11, ///< (Tx->Drone) User control - unknown receiver
    BK_PKT_TYPE_BIND_AUTO    = 0x12, ///< (Tx->Drone) Tell drones this tx is broadcasting
    BK_PKT_TYPE_TELEMETRY    = 0x13, ///< (Drone->Tx) Send telemetry to tx
    BK_PKT_TYPE_DFU          = 0x14, ///< (Drone->Tx) Send new firmware to tx
    BK_PKT_TYPE_BIND_MANUAL  = 0x15, ///< (Tx->Drone) Tell drones this tx is broadcasting
    BK_PKT_TYPE_TUNE         = 0x16, ///< (Drone->Tx) Send musical tune to tx
};
typedef uint8_t BK_PKT_TYPE;

/** A bitset of the buttons on this controller. */
enum button_bits {
    BUTTON_NONE           = 0x00, ///< No buttons are held
    BUTTON_RIGHT          = 0x01, ///< SW1 = The right button (mode)
    BUTTON_LEFT           = 0x02, ///< SW2 = The left button (launch/land)
    BUTTON_MIDDLE         = 0x04, ///< SW3 = The middle button (GPS)
    BUTTON_LEFT_SHOULDER  = 0x08, ///< SW4 = The left shoulder button (stunt)
    BUTTON_RIGHT_SHOULDER = 0x10, ///< SW5 = The right shoulder button (video)
    BUTTON_POWER          = 0x20, ///< SW6 = The top button (POWER)
};

/** The type of info being sent in control packets */
enum BK_INFO_TYPE_E {
    BK_INFO_MIN = 1,
    BK_INFO_FW_VER = 1,
    BK_INFO_DFU_RX = 2,
    BK_INFO_FW_CRC_LO = 3,
    BK_INFO_FW_CRC_HI = 4,
    BK_INFO_FW_YM = 5,
    BK_INFO_FW_DAY = 6,
    BK_INFO_MODEL = 7,
    BK_INFO_PPS = 8,
    BK_INFO_BATTERY = 9,
    BK_INFO_COUNTDOWN = 10,
    BK_INFO_HOPPING0 = 11,
    BK_INFO_HOPPING1 = 12,
    BK_INFO_DRONEID0 = 13,
    BK_INFO_DRONEID1 = 14,
    BK_INFO_MAX
};
typedef uint8_t BK_INFO_TYPE;

/** Data for packets that are not droneid packets
	Onair order = little-endian */
typedef struct packetDataDeviceCtrl_s {
    uint8_t roll; ///< 2: Low 8 bits of the roll joystick
    uint8_t pitch; ///< 3: Low 8 bits of the pitch joystick
    uint8_t throttle; ///< 4: Low 8 bits of the throttle joystick
    uint8_t yaw; ///< 5: Low 8 bits of the yaw joystick
    uint8_t msb; ///< 6: High 2 bits of roll (7..6), pitch (5..4), throttle (3..2), yaw (1..0)
    uint8_t buttons_held; ///< 7: The buttons
    uint8_t buttons_toggled; ///< 8: The buttons
    uint8_t data_type; ///< 9: Type of extra data being sent
    uint8_t data_value_lo; ///< 10: Value of extra data being sent
    uint8_t data_value_hi; ///< 11: Value of extra data being sent
} packetDataDeviceCtrl;

enum { SZ_ADDRESS = 5 }; ///< Size of address for transmission packets (40 bits)
enum { SZ_CRC_GUID = 4 }; ///< Size of UUID for drone (32 bits)
enum { SZ_DFU = 16 }; ///< Size of DFU packets

/** Data for packets that are binding packets
	Onair order = little-endian */
typedef struct packetDataDeviceBind_s {
    uint8_t bind_address[SZ_ADDRESS]; ///< The address being used by control packets
    uint8_t hopping; ///< The hopping table in use for this connection
    uint8_t droneid[SZ_CRC_GUID]; ///<
} packetDataDeviceBind;

/** Data structure for data packet transmitted from device (controller) to host (drone) */
typedef struct packetDataDevice_s {
    BK_PKT_TYPE packetType; ///< 0: The packet type
    uint8_t channel; ///< 1: Next channel I will broadcast on
    union packetDataDevice_u { ///< The variant part of the packets
        packetDataDeviceCtrl ctrl; ///< Control packets
        packetDataDeviceBind bind; ///< Binding packets
    } u;
} packetFormatRx;

/** Data structure for data packet transmitted from host (drone) to device (controller) */
typedef struct packetDataDrone_s {
    BK_PKT_TYPE packetType; ///< 0: The packet type
    uint8_t channel; ///< 1: Next channel I will broadcast on
    uint8_t pps; ///< 2: Packets per second the drone received
    uint8_t flags; ///< 3: Flags
    uint8_t droneid[SZ_CRC_GUID]; ///< 4...7: CRC of the drone
    uint8_t flight_mode; ///< 8:
    uint8_t wifi; ///< 9: Wifi channel + 24 * tx power.
    uint8_t note_adjust; ///< 10: note adjust for the tx buzzer (should this be sent so often?)
    uint8_t hopping; ///< 11: The adapative hopping byte we want to use
} packetFormatTx;

typedef struct packetDataDfu_s {
    BK_PKT_TYPE packetType; ///< 0: The packet type
    uint8_t channel; ///< 1: Next channel I will broadcast on
    uint8_t address_lo; ///< 2:
    uint8_t address_hi; ///< 3:
    uint8_t data[SZ_DFU]; ///< 4...19:
} packetFormatDfu;


// ----------------------------------------------------------------------------
// BK2425 chip definition
// ----------------------------------------------------------------------------


//----------------------------------------------------------------------------------
/** SPI register commands for the BK2425 and nrf24L01+ chips */
typedef enum {
    // General commands
    BK_REG_MASK        = 0x1F,  // The range of registers that can be read and written
    BK_READ_REG        = 0x00,  // Define read command to register (0..1F)
    BK_WRITE_REG       = 0x20,  // Define write command to register (0..1F)
    BK_ACTIVATE_CMD	   = 0x50,
    BK_R_RX_PL_WID_CMD = 0x60,
    BK_RD_RX_PLOAD     = 0x61,  // Define RX payload register address
    BK_WR_TX_PLOAD     = 0xA0,  // Define TX payload register address
    BK_W_ACK_PAYLOAD_CMD = 0xA8, // (nrf: +pipe 0..7)
    BK_W_TX_PAYLOAD_NOACK_CMD = 0xB0,
    BK_FLUSH_TX        = 0xE1,  // Define flush TX register command
    BK_FLUSH_RX        = 0xE2,  // Define flush RX register command
    BK_REUSE_TX_PL     = 0xE3,  // Define reuse TX payload register command
    BK_NOP             = 0xFF,  // Define No Operation, might be used to read status register

    // BK2425 bank 0 register addresses
    BK_CONFIG          = 0x00,  // 'Config' register address
    BK_EN_AA           = 0x01,  // 'Enable Auto Acknowledgment' register address
    BK_EN_RXADDR       = 0x02,  // 'Enabled RX addresses' register address
    BK_SETUP_AW        = 0x03,  // 'Setup address width' register address
    BK_SETUP_RETR      = 0x04,  // 'Setup Auto. Retrans' register address
    BK_RF_CH           = 0x05,  // 'RF channel' register address
    BK_RF_SETUP        = 0x06,  // 'RF setup' register address
    BK_STATUS          = 0x07,  // 'Status' register address
    BK_OBSERVE_TX      = 0x08,  // 'Observe TX' register address (lost packets, retransmitted packets on this frequency)
    BK_CD              = 0x09,  // 'Carrier Detect' register address
    BK_RX_ADDR_P0      = 0x0A,  // 'RX address pipe0' register address (5 bytes)
    BK_RX_ADDR_P1      = 0x0B,  // 'RX address pipe1' register address (5 bytes)
    BK_RX_ADDR_P2      = 0x0C,  // 'RX address pipe2' register address (1 byte)
    BK_RX_ADDR_P3      = 0x0D,  // 'RX address pipe3' register address (1 byte)
    BK_RX_ADDR_P4      = 0x0E,  // 'RX address pipe4' register address (1 byte)
    BK_RX_ADDR_P5      = 0x0F,  // 'RX address pipe5' register address (1 byte)
    BK_TX_ADDR         = 0x10,  // 'TX address' register address (5 bytes)
    BK_RX_PW_P0        = 0x11,  // 'RX payload width, pipe0' register address
    BK_RX_PW_P1        = 0x12,  // 'RX payload width, pipe1' register address
    BK_RX_PW_P2        = 0x13,  // 'RX payload width, pipe2' register address
    BK_RX_PW_P3        = 0x14,  // 'RX payload width, pipe3' register address
    BK_RX_PW_P4        = 0x15,  // 'RX payload width, pipe4' register address
    BK_RX_PW_P5        = 0x16,  // 'RX payload width, pipe5' register address
    BK_FIFO_STATUS     = 0x17,  // 'FIFO Status Register' register address
    BK_DYNPD           = 0x1c,  // 'Enable dynamic payload length' register address
    BK_FEATURE         = 0x1d,  // 'Feature' register address
    BK_PAYLOAD_WIDTH   = 0x1f,  // 'payload length of 256 bytes modes register address

    // BK2425 bank 1 register addresses
    BK2425_R1_4      = 0x04,
    BK2425_R1_5      = 0x05,
    BK2425_R1_WHOAMI = 0x08, // Register to read that contains the chip id
    BK2425_R1_12     = 0x0C, // PLL speed 120 or 130us
    BK2425_R1_13     = 0x0D,
    BK2425_R1_14     = 0x0E,
} BK_SPI_CMD;

//----------------------------------------------------------------------------------
// Chip Status Byte
//----------------------------------------------------------------------------------

enum {
    BK_CHIP_ID_BK2425 = 0x63, // The expected value of reading BK2425_R1_WHOAMI
};

// Meanings of the BK_STATUS register
enum {
    BK_STATUS_RBANK = 0x80, // Register bank 1 is in use
    BK_STATUS_RX_DR = 0x40, // Data ready
    BK_STATUS_TX_DS = 0x20, // Data sent
    BK_STATUS_MAX_RT = 0x10, // Max retries failed
    BK_STATUS_RX_MASK = 0x0E, // Mask for the receptions bit
    BK_STATUS_RX_EMPTY = 0x0E,
    BK_STATUS_RX_P_5 = 0x0A, // Data pipe 5 has some data ready
    BK_STATUS_RX_P_4 = 0x08, // Data pipe 4 has some data ready
    BK_STATUS_RX_P_3 = 0x06, // Data pipe 3 has some data ready
    BK_STATUS_RX_P_2 = 0x04, // Data pipe 2 has some data ready
    BK_STATUS_RX_P_1 = 0x02, // Data pipe 1 has some data ready
    BK_STATUS_RX_P_0 = 0x00, // Data pipe 0 has some data ready
    BK_STATUS_TX_FULL = 0x01 // Tx buffer full
};

// Meanings of the FIFO_STATUS register
enum {
    BK_FIFO_STATUS_TX_REUSE = 0x40,
    BK_FIFO_STATUS_TX_FULL  = 0x20,
    BK_FIFO_STATUS_TX_EMPTY = 0x10,
    BK_FIFO_STATUS_RX_FULL  = 0x02,
    BK_FIFO_STATUS_RX_EMPTY = 0x01
};

// Meanings of the BK_CONFIG register
enum {
    BK_CONFIG_MASK_RX_DR = 0x40,  // Mask interrupt caused by RX_DR
    BK_CONFIG_MASK_TX_DS = 0x20,  // Mask interrupt caused by TX_DS
    BK_CONFIG_MASK_MAX_RT = 0x10, // Mask interrupt caused by MAX_RT
    BK_CONFIG_EN_CRC = 0x08,      // Enable CRC. Forced high if one of the bits in the EN_AA is high
    BK_CONFIG_CRCO = 0x04,        // CRC encoding scheme (0=8 bits, 1=16 bits)
    BK_CONFIG_PWR_UP = 0x02,      // POWER UP
    BK_CONFIG_PRIM_RX = 0x01,     // Receive/transmit
};

enum {
    BK_FEATURE_EN_DPL = 0x04,     //
    BK_FEATURE_EN_ACK_PAY = 0x02, //
    BK_FEATURE_EN_DYN_ACK = 0x01, //
};

// (Lets make it one radio interface for both projects)

/** The baud rate of the GFSK modulation */
typedef enum ITX_SPEED_e {
    ITX_250,  ///< 250kbps (slowest but furthest range)
    ITX_1000, ///< 1000kbps (balanced)
    ITX_2000, ///< 2000kbps (fastest hence least congested)
    ITX_CARRIER, ///< 0kbps (constant carrier wave)
    ITX_MAX
} ITX_SPEED;

enum {
    PACKET_LENGTH_RX_CTRL = 12,
    PACKET_LENGTH_RX_BIND = 12,
    PACKET_LENGTH_RX_MAX  = 12,
    PACKET_LENGTH_TX_TELEMETRY = 12,
    PACKET_LENGTH_TX_DFU = 20,
    PACKET_LENGTH_TX_MAX = 20,
};

// Note that bank 1 registers 0...8 are MSB first; others are LSB first

#define PLL_SPEED { BK2425_R1_12, 0x00,0x12,0x73,0x05 } // 0x00127305ul, // PLL locking time 130us compatible with nRF24L01;

// In the array Bank1_Reg0_13[],all the register values are the byte reversed!
enum {
    IREG1_4,
    IREG1_5,
    IREG1_12,
    IREG1_13,
    IREG1_4A,
    IREG_MAX
};

#define BK_MAX_PACKET_LEN 32 // max value is 32 bytes
#define BK_RCV_TIMEOUT 30

//----------------------------------------------------------------------------------
// Translate output power into a number
// must match up with the table RegPower[]
#define OUTPUT_POWER_REG6_0 0 // -25dB
#define OUTPUT_POWER_REG6_1 0 // -18dB
#define OUTPUT_POWER_REG6_2 1 // -18dB
#define OUTPUT_POWER_REG6_3 1 // -12dB
#define OUTPUT_POWER_REG6_4 1 // -12dB
#define OUTPUT_POWER_REG6_5 2 //  -7dB
#define OUTPUT_POWER_REG6_6 3 //  -1dB
#define OUTPUT_POWER_REG6_7 3 //  +4dB

// Register 4 in bank 1 only applies to Beken chip
#define OUTPUT_POWER_REG4_0 0 // -25dB
#define OUTPUT_POWER_REG4_1 3 // -18dB
#define OUTPUT_POWER_REG4_2 0 // -18dB
#define OUTPUT_POWER_REG4_3 3 // -12dB
#define OUTPUT_POWER_REG4_4 2 // -12dB
#define OUTPUT_POWER_REG4_5 0 //  -7dB
#define OUTPUT_POWER_REG4_6 0 //  -1dB
#define OUTPUT_POWER_REG4_7 7 //  +4dB

// Generic support
#define TOKENPASTE(x, y) x ## y
#define TOKENPASTE2(x, y) TOKENPASTE(x, y)
// The default register values that are for the default power setting
#define DEFAULT_OUTPUT_REG6 TOKENPASTE2(OUTPUT_POWER_REG6_,DEFAULT_OUTPUT_POWER)
#define DEFAULT_OUTPUT_REG4 TOKENPASTE2(OUTPUT_POWER_REG4_,DEFAULT_OUTPUT_POWER)

/** Parameters used by the fcc pretests */
typedef struct FccParams_s {
    uint8_t fcc_mode; ///< The value (0..6) last set by the user that we are using. Non-zero iff we are sending test signals
    bool scan_mode; ///< true for scanning, false for fixed frequencies
    bool CW_mode; ///< true for carrier wave, false for packets
    bool disable_crc_mode; ///< false for CRCs enabled, true for CRCs ignored on reception
    uint8_t scan_count; ///< In scan mode, packet count before incrementing scan
    uint8_t channel; ///< Current frequency 8..70
    uint8_t power; ///< Current power 1..8
    bool disable_crc; ///< true=crc is physically disabled
    uint8_t factory_mode; ///< factory test mode 0..8
    bool enable_cd; ///< enable carrier detect
    bool last_cd; ///< last carrier detect on a packet received
} FccParams;

typedef enum BkRadioMode_e {
    BKRADIO_SLEEP,
    BKRADIO_IDLE,
    BKRADIO_TX,
    BKRADIO_RX,
    BKRADIO_STANDBY1, // Not visible to the code yet
    BKRADIO_STANDBY2, // Not visible to the code yet
} BkRadioMode;

//----------------------------------------------------------------------------------
// BEKEN driver class
class Radio_Beken
{
public:
    // Generic functions
    Radio_Beken(AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev);
    bool lock_bus(void)
    {
        dev->get_semaphore()->take_blocking();
        return true;
    }
    void unlock_bus(void)
    {
        dev->get_semaphore()->give();
    }

    // Raw SPI access functions
    void ReadRegisterMulti(uint8_t address, uint8_t *data, uint8_t len);
    void WriteRegisterMulti(uint8_t address, const uint8_t *data, uint8_t len);

    // Low-level Beken functions
    uint8_t ReadStatus(void);
    uint8_t ReadReg(uint8_t reg);
    uint8_t Strobe(uint8_t address);
    void SetRBank(uint8_t bank);
    void WriteReg(uint8_t address, uint8_t data);
    void WriteRegisterMultiBank1(uint8_t address, const uint8_t *data, uint8_t len);

    // High-level Beken functions
    void SetPower(uint8_t power);
    void SetChannel(uint8_t channel); // Sets the physical channel
    void SetCwMode(uint8_t cw);
    void SetCrcMode(uint8_t disable_crc); // non-zero means crc is ignored
    void SetFactoryMode(uint8_t factory);
    bool Reset(void);
    void SwitchToRxMode(void);
    void SwitchToTxMode(void);
    void SwitchToIdleMode(void);
    void SwitchToSleepMode(void);
    void InitBank0Registers(ITX_SPEED spd);
    void InitBank1Registers(ITX_SPEED spd);
    void SetAddresses(const uint8_t* txaddr); // Set the rx and tx addresses
    bool ClearAckOverflow(void);
    bool SendPacket(uint8_t type, const uint8_t* pbuf, uint8_t len);
    void DelayCE(void);
    void DumpRegisters(void);
    bool WasTxMode(void);
    bool WasRxMode(void);
    void ResetAddress(void);
    void EnableCarrierDetect(bool bEnable);
    bool CarrierDetect(void);

    // Visible public variables (naughty)
    volatile uint8_t bkReady; // initialised in AP_Radio_bk2425.h radio_init() at the very end. Similar to a semaphore.
    static ITX_SPEED gTxSpeed;
    FccParams fcc;
    packetFormatTx pktDataTx; // Packet data to send (telemetry)
    packetFormatDfu pktDataDfu; // Packet data to send (DFU)
    uint8_t TX_Address[5]; // For sending telemetry and DFU

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    uint8_t bFreshData; // Have we received a packet since we last processed one
    uint32_t numTxPackets;
    packetFormatRx pktDataRx; // Last valid packet that has been received
    packetFormatRx pktDataRecv; // Packet data in process of being received
    uint8_t lastTxChannel; // 0..CHANNEL_COUNT_LOGICAL
    uint8_t RX0_Address[5]; // The data address
    uint8_t RX1_Address[5]; // The fixed binding address
    BkRadioMode bkMode;
};

#endif  // AP_RADIO_BK2425_ENABLED

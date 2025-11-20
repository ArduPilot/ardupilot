/*Add commentMore actions
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_ZeroOnex.h"
#include <GCS_MAVLink/GCS.h>
#if defined(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1)
#include <hal.h>
#endif

static constexpr uint32_t SPI_SPEED = 2 * 1000 * 1000;  // 2 MHz SPI serial interface
static constexpr uint16_t EOI = (1 << 1);               // End of Initialization
static constexpr uint16_t EN_SENSOR = (1 << 0);         // Enable RATE and ACC measurement
static constexpr uint16_t DRY_DRV_EN = (1 << 5);        // Enables Data ready function
static constexpr uint16_t FILTER_235HZ = (0b0000101101101);       // 235 Hz  filter
static constexpr uint16_t FILTER_68HZ = (0x0000);       // 68 Hz default filter
static constexpr uint16_t FILTER_30HZ = (0b0000001001001);       // 30 Hz  filter
static constexpr uint16_t FILTER_BYPASS = (0b0000000111111111);     // No filtering
static constexpr uint16_t RATE_300DPS_1518HZ = 0b0001001011011011; // Gyro XYZ range 300 deg/s @ 1518Hz
static constexpr uint16_t RATE_300DPS_3030HZ = 0b0001001010010010; // Gyro XYZ range 300 deg/s @ 3030Hz
static constexpr uint16_t RATE_300DPS_4419HZ = 0b0001001001001001; // Gyro XYZ range 300 deg/s @ 4419Hz
static constexpr uint16_t ACC12_8G_1518HZ = 0b0001001011011011;  // Acc XYZ range 8 G and 1518 update rate
static constexpr uint16_t ACC12_8G_3030HZ = 0b0001001010010010;     // Acc XYZ range 8 G and 3030 update rate
static constexpr uint16_t ACC12_8G_4419HZ = 0b0001001001001001;     // Acc XYZ range 8 G and 4419 update rate
static constexpr uint16_t ACC3_26G = (0b000 << 0);
static constexpr uint16_t SPI_SOFT_RESET = (0b1010);
static constexpr uint32_t POWER_ON_TIME = 250000UL;

// Data registers
#define RATE_X1         0x01 // 20 bit
#define RATE_Y1         0x02 // 20 bit
#define RATE_Z1         0x03 // 20 bit
#define ACC_X1          0x04 // 20 bit
#define ACC_Y1          0x05 // 20 bit
#define ACC_Z1          0x06 // 20 bit
#define ACC_X3          0x07 // 20 bit
#define ACC_Y3          0x08 // 20 bit
#define ACC_Z3          0x09 // 20 bit
#define RATE_X2         0x0A // 20 bit
#define RATE_Y2         0x0B // 20 bit
#define RATE_Z2         0x0C // 20 bit
#define ACC_X2          0x0D // 20 bit
#define ACC_Y2          0x0E // 20 bit
#define ACC_Z2          0x0F // 20 bit
#define TEMP            0x10 // 16 bit
// Status registers
#define STAT_SUM        0x14 // 16 bit
#define STAT_SUM_SAT    0x15 // 16 bit
#define STAT_COM        0x16 // 16 bit
#define STAT_RATE_COM   0x17 // 16 bit
#define STAT_RATE_X     0x18 // 16 bit
#define STAT_RATE_Y     0x19 // 16 bit
#define STAT_RATE_Z     0x1A // 16 bit
#define STAT_ACC_X      0x1B // 16 bit
#define STAT_ACC_Y      0x1C // 16 bit
#define STAT_ACC_Z      0x1D // 16 bit
// Control registers
#define CTRL_FILT_RATE  0x25 // 9 bit
#define CTRL_FILT_ACC12 0x26 // 9 bit
#define CTRL_FILT_ACC3  0x27 // 9 bit
#define CTRL_RATE       0x28 // 15 bit
#define CTRL_ACC12      0x29 // 15 bit
#define CTRL_ACC3       0x2A // 3 bit
#define CTRL_USER_IF    0x33 // 16 bit
#define CTRL_ST         0x34 // 13 bit
#define CTRL_MODE       0x35 // 4 bit
#define CTRL_RESET      0x36 // 4 bit
// Misc registers
#define ASIC_ID         0x3B // 12 bit
#define COMP_ID         0x3C // 16 bit
#define SN_ID1          0x3D // 16 bit
#define SN_ID2          0x3E // 16 bit
#define SN_ID3          0x3F // 16 bit

#define T_STALL_US   20U

#define SPI32BITCONVERT2_20BIT(x) (((int32_t)(((x)<<12)& 0xfffff000UL))>>12)

#define CtrlMode_Direct 0
#define CtrlMode_FpgaRead 1

#define CTRL_Shift_Mode (0)
#define CTRL_Shift_FifoRst (1)
#define CTRL_Shift_FifoEnable (2)
#define CTRL_Shift_CmdNumSub1 (8)
#define CTRL_Shift_BaudSub1 (16)

/*
Addr_R32_Verison
check the version of FPGA
*/
#define Addr_R32_Verison (0x0000)
/*
Addr_RW16_TestReg
Test register, used for read-write testing to determine whether the FPGA is normal
*/
#define Addr_RW16_TestReg (0x0001)
/*
Addr_W32_Sch16tCtrl
bit[0]: '0' - Direct Mode, '1' - FPGA Read-Write Mode
bit[1]: '1' - Reset FIFO
bit[2]: '1' - Enable FPGA to Read Sensor data
bit[15:8]: Number of FPGA Sensor Reading Instructions - 1
bit[23:16]: SPI Clock Cycle + 1, Unit: 16.667ns. Example: 7 - Clock Cycle = (7 + 1) × 16.667 ≈ 133ns
*/
#define Addr_W32_Sch16tCtrl (0x0010)
/*
Addr_RW8_DirectRam8
Used for SPI data reading and writing in direct mode
*/
#define Addr_RW8_DirectRam8 (0x0011)
/*
Addr_RW16_DirectSpiCtrl
Write Function:
bit[3:0]: Function bits. 1 - Reset, 2 - Start Transmission
bit[15:8]: Number of transmitted bytes minus 1
Read Function:
bit[0]: SPI direct transmission busy signal. 1 - Busy, 0 - Idle
*/
#define Addr_RW16_DirectSpiCtrl (0x0012)
/*
Addr_W8_SensorReadCmdRam8
Sensor Reading Instruction RAM
Each instruction is 64 bits, occupying 8 bytes, including a 48-bit instruction and a 16-bit instruction definition.
*/
#define Addr_W8_SensorReadCmdRam8 (0x0013)
/*
Addr_W8_SensorValueFifo8
Sensor data stroge in fifo
*/
#define Addr_W8_SensorValueFifo8 (0x0014)
/*
Addr_RW8_SensorFifoCtrl
Read:
bit[7:0]: Number of packets contained in the FIFO
bit[8]: RDY signal abnormal flag
Write:
bit[0]: After reading one packet, write '1' to notify the FPGA to load the next data packet
*/
#define Addr_RW8_SensorFifoCtrl (0x0015)

#define FuncBit_Write (0 << 7)
#define FuncBit_Read (1 << 7)
#define FuncBit_Reg (0 << 5)
#define FuncBit_Ram (1 << 5)
#define FuncBit_Fifo (2 << 5)
#define FuncBit_Bit8 (0 << 3)
#define FuncBit_Bit16 (1 << 3)
#define FuncBit_Bit32 (2 << 3)


uint8_t direct_mode = CtrlMode_Direct;
uint8_t fifo_enable = 0;
uint8_t fifo_cmd_num = 17;
uint8_t fifo_baudrate = 32;

extern const AP_HAL::HAL& hal;

AP_InertialSensor_ZeroOnex::AP_InertialSensor_ZeroOnex(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                         enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
{
    gyro_scale = radians(1.f / 1600.f);
    accel_scale = 1.f / 3200.f;

    _registers[0] = RegisterConfig(CTRL_FILT_RATE,  FILTER_235HZ);        // 68Hz -- default FILTER_235HZ
    _registers[1] = RegisterConfig(CTRL_FILT_ACC12, FILTER_235HZ);        // 68Hz -- default FILTER_235HZ
    _registers[2] = RegisterConfig(CTRL_FILT_ACC3,  FILTER_235HZ);        // 68Hz -- default FILTER_235HZ
    _registers[3] = RegisterConfig(CTRL_RATE,RATE_300DPS_1518HZ); // +/- 300 deg/s, 1600 LSB/(deg/s) -- default, Decimation 8, 1475Hz RATE_300DPS_1475HZ
    _registers[4] = RegisterConfig(CTRL_ACC12,ACC12_8G_1518HZ);    // +/- 80 m/s^2, 3200 LSB/(m/s^2) -- default, Decimation 8, 1475Hz ACC12_8G_1475HZ
    _registers[5] = RegisterConfig(CTRL_ACC3,ACC3_26G);           // +/- 260 m/s^2, 1600 LSB/(m/s^2) -- default ACC3_26G
}
AP_InertialSensor_Backend *
AP_InertialSensor_ZeroOnex::probe(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto sensor = new AP_InertialSensor_ZeroOnex(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_ZeroOnex::start()
{
    backend_rate_hz = 1518;
    if (!_imu.register_accel(accel_instance, backend_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_SCH16T)) ||
        !_imu.register_gyro(gyro_instance, backend_rate_hz,   dev->get_bus_id_devtype(DEVTYPE_INS_SCH16T))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    uint32_t period_us = 1000000UL / backend_rate_hz;
    periodic_handle = dev->register_periodic_callback(period_us, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ZeroOnex::run_state_machine, void));
}

void AP_InertialSensor_ZeroOnex::seneor_ctrl(uint8_t fifo_rst) 
{
    uint32_t reg;
    uint8_t wbuf[8];
    uint8_t ptr = 0;

    reg = (direct_mode & 1) << CTRL_Shift_Mode;
    reg |= (fifo_rst & 1) << CTRL_Shift_FifoRst;
    reg |= (fifo_enable & 1) << CTRL_Shift_FifoEnable;
    reg |= ((fifo_cmd_num - 1) & 0xff) << CTRL_Shift_CmdNumSub1;
    reg |= ((fifo_baudrate - 1) & 0xff) << CTRL_Shift_BaudSub1;

    wbuf[ptr++] = FuncBit_Write | FuncBit_Reg | FuncBit_Bit32;
    wbuf[ptr++] = Addr_W32_Sch16tCtrl & 0xff;
    wbuf[ptr++] = (Addr_W32_Sch16tCtrl >> 8) & 0xff;
    wbuf[ptr++] = reg & 0xff;
    wbuf[ptr++] = (reg >> 8) & 0xff;
    wbuf[ptr++] = (reg >> 16) & 0xff;
    wbuf[ptr++] = (reg >> 24) & 0xff;

    dev->transfer(wbuf, ptr, wbuf, ptr);
}

void AP_InertialSensor_ZeroOnex::init(void)
{
    /**< fpga read */
    fifo_enable = 0;
    seneor_ctrl(1);

    fpga_read_config();

    fifo_enable = 1;
    direct_mode = CtrlMode_FpgaRead;
    seneor_ctrl(1);
}

static uint8_t gen_crc8(uint8_t* data) {
    uint16_t crc = 0xff;
    uint8_t byte_value;
    for (uint8_t c = 0; c < 6; c++) {
        byte_value = (c == 5) ? 0x00 : data[c];
        for (uint8_t i = 0; i < 8; i++) {
            uint8_t data_bit = (byte_value >> (7 - i)) & 1;
            crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
        }
    }

    return crc;
}

static void _register_write(uint16_t addr, uint32_t value, uint8_t* buff) {
    buff[0] = (addr >> 2) & 0xff;
    buff[1] = ((addr & 3) << 6);
    buff[1] |= 1 << 5;
    buff[1] |= 1 << 3;
    buff[2] = (value >> 16) & 0xf;
    buff[3] = (value >> 8) & 0xff;
    buff[4] = value & 0xff;
    buff[5] = gen_crc8(buff);
}

static void _register_read(uint16_t addr, uint8_t* buff) {
    buff[0] = (addr >> 2) & 0xff;
    buff[1] = ((addr & 3) << 6);
    buff[1] |= 1 << 3;
    buff[2] = 0;
    buff[3] = 0;
    buff[4] = 0;
    buff[5] = gen_crc8(buff);
}



void AP_InertialSensor_ZeroOnex::fpga_read_config(void) {
    uint8_t ptr = 0;
    /**< reset fifo */
    fpga_write_cmd(RATE_X2, ptr, 0, 0, 0);
    ptr++;
    fpga_write_cmd(RATE_Y2, ptr, 1, 1, 0);
    ptr++;
    fpga_write_cmd(RATE_Z2, ptr, 1, 1, 1);
    ptr++;
    fpga_write_cmd(ACC_X2, ptr, 1, 1, 2);
    ptr++;
    fpga_write_cmd(ACC_Y2, ptr, 1, 1, 3);
    ptr++;
    fpga_write_cmd(ACC_Z2, ptr, 1, 1, 4);
    ptr++;
    fpga_write_cmd(TEMP, ptr, 1, 1, 5);
    ptr++;
    fpga_write_cmd(STAT_SUM, ptr, 1, 1, 6);
    ptr++;
    fpga_write_cmd(STAT_SUM_SAT, ptr, 1, 0, 0);
    ptr++;
    fpga_write_cmd(STAT_COM, ptr, 1, 0, 1);
    ptr++;
    fpga_write_cmd(STAT_RATE_COM, ptr, 1, 0, 2);
    ptr++;
    fpga_write_cmd(STAT_RATE_X, ptr, 1, 0, 3);
    ptr++;
    fpga_write_cmd(STAT_RATE_Y, ptr, 1, 0, 4);
    ptr++;
    fpga_write_cmd(STAT_RATE_Z, ptr, 1, 0, 5);
    ptr++;
    fpga_write_cmd(STAT_ACC_X, ptr, 1, 0, 6);
    ptr++;
    fpga_write_cmd(STAT_ACC_Y, ptr, 1, 0, 7);
    ptr++;
    fpga_write_cmd(STAT_ACC_Z, ptr, 1, 0, 8);
    ptr++;
    fpga_write_cmd(STAT_ACC_Z, ptr, 1, 0, 9);
    ptr++;
    fifo_cmd_num = ptr;
}

static void sch16t_gen_cmd(uint32_t addr, uint8_t* buff, uint8_t read_valid, uint8_t is_sensor, uint8_t offset) {
    buff[0] = read_valid ? 0x80 : 0x00;
    buff[0] |= is_sensor ? 0x40 : 0x00;
    buff[0] |= (offset & 0xf) << 2;
    buff[0] |= (offset >> 2) & 0x3;
    buff[1] = (offset & 3) << 6;
    _register_read(addr, buff + 2);
}

void AP_InertialSensor_ZeroOnex::fpga_write_cmd(uint32_t addr, uint8_t ptr, uint8_t read_valid, uint8_t is_sensor, uint8_t offset) 
{
    uint8_t buff[16];
    sch16t_gen_cmd(addr, buff, read_valid, is_sensor, offset);
    fpga_write_ram8(Addr_W8_SensorReadCmdRam8, ptr << 3, buff, 8);
}
void AP_InertialSensor_ZeroOnex::run_state_machine()
{
    switch (_state) {
    case State::PowerOn: {
            _state = State::Reset;
            dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);//250000UL  //250ms
            break;
        }

    case State::Reset: {
            failure_count = 0;
            reset_chip();
            _state = State::Configure;
            dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);
            break;
        }

    case State::Configure: {
            if (!read_product_id()) {
                _state = State::Reset;
                dev->adjust_periodic_callback(periodic_handle, 2000000); // 2s
                break;
            }
            configure_registers();
            _state = State::LockConfiguration;
            dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);
            break;
        }

    case State::LockConfiguration: {
            read_status_registers(); // Read all status registers once
            register_write(CTRL_MODE, EOI | EN_SENSOR);// Write EOI and EN_SENSOR
            _state = State::Validate;
            dev->adjust_periodic_callback(periodic_handle, 50000UL); // 50ms
            break;
        }

    case State::Validate: {
            read_status_registers(); // Read all status registers twice
            read_status_registers();
            // Check that registers are configured properly and that the sensor status is OK
            if (validate_sensor_status() && validate_register_configuration()) {
                _state = State::Read;
                init();             
                dev->adjust_periodic_callback(periodic_handle, 1000000UL / backend_rate_hz);

            } else {
                _state = State::Reset;
                dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);
            }
            break;
        }

    case State::Read: {  
            if (collect_and_publish()) {
                if (failure_count > 0) {
                    failure_count--;
                }
            } else {
                failure_count++;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "STATE:Read. failure_count is %d",failure_count); 
            }

            // Reset if successive failures
            if (failure_count > 10) {
                _state = State::Reset;
                return;
            }
            break;
        }

    default:
        break;
    } // end switch/case
}

bool AP_InertialSensor_ZeroOnex::collect_and_publish()
{
    bool success = read_data();
    return success;
}

bool AP_InertialSensor_ZeroOnex::read_data() {
    uint8_t pkt_num;
    SensorData data = {};
    uint8_t buff[32];

    pkt_num = fpga_read_reg8(Addr_RW8_SensorFifoCtrl);
    if(!pkt_num) return false;
    // adjust the periodic callback to be synchronous with the incoming data
    dev->adjust_periodic_callback(periodic_handle, 1000000UL / backend_rate_hz);//
    for(;pkt_num>1;pkt_num--)
    {
    fpga_read_fifo8(Addr_W8_SensorValueFifo8, buff, 24);
    fpga_write_reg8(Addr_RW8_SensorFifoCtrl, 0x01);
    if (buff[0] & 0x80) data.gyro_x = SPI32BITCONVERT2_20BIT(((buff[1] & 0xf) << 16) | (buff[2] << 8) | buff[3]);
    if (buff[0] & 0x40) data.gyro_y = -SPI32BITCONVERT2_20BIT(((buff[4] & 0xf) << 16) | (buff[5] << 8) | buff[6]);
    if (buff[0] & 0x20) data.gyro_z = -SPI32BITCONVERT2_20BIT(((buff[7] & 0xf) << 16) | (buff[8] << 8) | buff[9]);        
    if (buff[0] & 0x10) data.acc_x = SPI32BITCONVERT2_20BIT(((buff[10] & 0xf) << 16) | (buff[11] << 8) | buff[12]);  
    if (buff[0] & 0x08) data.acc_y = -SPI32BITCONVERT2_20BIT(((buff[13] & 0xf) << 16) | (buff[14] << 8) | buff[15]);
    if (buff[0] & 0x04) data.acc_z = -SPI32BITCONVERT2_20BIT(((buff[16] & 0xf) << 16) | (buff[17] << 8) | buff[18]);
    if (buff[0] & 0x02) data.temp = SPI32BITCONVERT2_20BIT(((buff[19] & 0xf) << 16) | (buff[20] << 8) | buff[21])>>4;
    Vector3f accel{accel_scale*data.acc_x, accel_scale*data.acc_y, accel_scale*data.acc_z};
    Vector3f gyro{gyro_scale*data.gyro_x, gyro_scale*data.gyro_y, gyro_scale*data.gyro_z};
    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    _publish_temperature(accel_instance, float(data.temp)/100.f);
    }
    return true;
}
void AP_InertialSensor_ZeroOnex::reset_chip()
{
#if defined(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1)
    palClearLine(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1);
    hal.scheduler->delay(2000);
    palSetLine(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1);
#else
    register_write(CTRL_RESET, SPI_SOFT_RESET);
#endif
}
bool AP_InertialSensor_ZeroOnex::read_product_id()
{
    uint32_t comp_id = 0, asic_id = 0;
    register_read(COMP_ID, 0);
    register_read(ASIC_ID, &comp_id);
    register_read(ASIC_ID, &asic_id);
    bool success = asic_id == 0x21 && comp_id == 0x23;

    return success;
}

void AP_InertialSensor_ZeroOnex::configure_registers()
{
    uint32_t reg_value;
    for (auto &r : _registers) {
        register_write(r.addr, r.value);
    }
    register_read(CTRL_USER_IF, 0);
    register_read(CTRL_USER_IF, &reg_value);
    reg_value |= DRY_DRV_EN;
    register_write(CTRL_USER_IF, reg_value);
    register_write(CTRL_MODE, EN_SENSOR);
}

bool AP_InertialSensor_ZeroOnex::validate_sensor_status()
{
    auto &s = _sensor_status;
    uint16_t values[] = { s.summary, s.saturation, s.common, s.rate_common, s.rate_x, s.rate_y, s.rate_z, s.acc_x, s.acc_y, s.acc_z };

    for (auto v : values) {
        if (v != 0xFFFF) {
            return false;
        }
    }
    return true;
}
bool AP_InertialSensor_ZeroOnex::validate_register_configuration()
{
    uint32_t value;
    bool success = true;

    for (auto &r : _registers) {
        register_read(r.addr,0); // double read, wasteful but makes the code cleaner, not high rate so doesn't matter anyway
        register_read(r.addr,&value); 

        if (value != r.value) {
            success = false;
        }
    }

    return success;
}



uint16_t AP_InertialSensor_ZeroOnex::fpga_read_reg16(uint16_t reg) {
    uint8_t wbuf[8];
    uint8_t ptr = 0;

    wbuf[ptr++] = FuncBit_Read | FuncBit_Reg | FuncBit_Bit16;
    wbuf[ptr++] = reg & 0xff;
    wbuf[ptr++] = (reg >> 8) & 0xff;
    wbuf[ptr++] = 0xff;
    wbuf[ptr++] = 0xff;
    wbuf[ptr++] = 0xff;
    dev->transfer(wbuf, ptr, wbuf, ptr);
    return wbuf[4] | (wbuf[5] << 8);
}

uint8_t AP_InertialSensor_ZeroOnex::wait_direct_busy(void) {
    uint16_t timeout = 0x200;
    while (timeout) {
        if (!(fpga_read_reg16(Addr_RW16_DirectSpiCtrl) & 1)) return 1;
        timeout--;
    }
    return 0;
}

uint16_t AP_InertialSensor_ZeroOnex::_sch16t_read_status(uint16_t addr) {
    uint32_t value;

    if (register_read(addr, &value) == 0) 
    {
        return 0;
    }
    return value & 0xffff;
}

void AP_InertialSensor_ZeroOnex::read_status_registers()
{
    _sch16t_read_status(STAT_SUM);
    _sensor_status.summary      = _sch16t_read_status(STAT_SUM);
    _sensor_status.summary      = _sch16t_read_status(STAT_SUM_SAT);
    _sensor_status.saturation   = _sch16t_read_status(STAT_COM);
    _sensor_status.common       = _sch16t_read_status(STAT_RATE_COM);
    _sensor_status.rate_common  = _sch16t_read_status(STAT_RATE_X);
    _sensor_status.rate_x       = _sch16t_read_status(STAT_RATE_Y);
    _sensor_status.rate_y       = _sch16t_read_status(STAT_RATE_Z);
    _sensor_status.rate_z       = _sch16t_read_status(STAT_ACC_X);
    _sensor_status.acc_x        = _sch16t_read_status(STAT_ACC_Y);
    _sensor_status.acc_y        = _sch16t_read_status(STAT_ACC_Z);
    _sensor_status.acc_z        = _sch16t_read_status(STAT_ACC_Z);
}

uint8_t AP_InertialSensor_ZeroOnex::register_read(uint16_t addr, uint32_t* value)
{
    uint8_t tbuf[8];
    uint8_t rbuf[8];

    direct_mode = CtrlMode_Direct;
    seneor_ctrl(0);

    _register_read(addr, tbuf);

    wait_direct_busy();
    fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x01);
    fpga_write_ram8(Addr_RW8_DirectRam8, 0, tbuf, 6);
    fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x02 | (5 << 8));
    wait_direct_busy();
    fpga_read_ram8(Addr_RW8_DirectRam8, 0, rbuf, 6);

    if (!value) {
        return 1;
    }

    if (gen_crc8(rbuf) != rbuf[5]) {
        return 0;
    }
    *value = ((rbuf[2] & 0xf) << 16) | (rbuf[3] << 8) | (rbuf[4]);
    return 1;
}

// Non-data registers are the only writable ones and are 16 bit or less
void AP_InertialSensor_ZeroOnex::register_write(uint16_t addr, uint32_t value)
{
    uint8_t tbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    direct_mode = CtrlMode_Direct;
    seneor_ctrl(0);

    _register_write(addr, value, tbuf);

    wait_direct_busy();
    fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x01);
    fpga_write_ram8(Addr_RW8_DirectRam8, 0, tbuf, 6);
    fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x02 | (5 << 8));
    wait_direct_busy();
}

// The SPI protocol (SafeSPI) is 48bit out-of-frame. This means read return frames will be received on the next transfer.
uint64_t AP_InertialSensor_ZeroOnex::transfer_spi_frame(uint64_t frame)
{
    uint16_t buf[3];
    for (int index = 0; index < 3; index++) {
        uint16_t lower_byte = (frame >> (index << 4)) & 0xFF;
        uint16_t upper_byte = (frame >> ((index << 4) + 8)) & 0xFF;
        buf[3 - index - 1] = (lower_byte << 8) | upper_byte;
    }

    dev->transfer((uint8_t*)buf, 6, (uint8_t*)buf, 6);

    uint64_t value = {};
    for (int index = 0; index < 3; index++) {
        uint16_t lower_byte = buf[index] & 0xFF;
        uint16_t upper_byte = (buf[index] >> 8) & 0xFF;
        value |= (uint64_t)(upper_byte | (lower_byte << 8)) << ((3 - index - 1) << 4);
    }

    return value;
}
uint8_t AP_InertialSensor_ZeroOnex::calculate_crc8(uint64_t frame)
{
    uint64_t data = frame & 0xFFFFFFFFFF00LL;
    uint8_t crc = 0xFF;

    for (int i = 47; i >= 0; i--) {
        uint8_t data_bit = data >> i & 0x01;
        crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
    }

    return crc;
}
void AP_InertialSensor_ZeroOnex::fpga_read_fifo8(uint16_t reg, uint8_t* value, uint8_t size) 
{
    uint8_t wbuf[32];
    uint8_t ptr = 0;
    uint8_t i;
    if (value ==nullptr) return;
    wbuf[ptr++] = FuncBit_Read | FuncBit_Fifo | FuncBit_Bit8;
    wbuf[ptr++] = reg & 0xff;
    wbuf[ptr++] = (reg >> 8) & 0xff;
    wbuf[ptr++] = size;
    wbuf[ptr++] = 0xff;
    for(i=0;i<size&&ptr<32;i++)
    {
        wbuf[ptr++] = *(value+i);
    }
        if(ptr == 32)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Buff ready to be overflow");
    }
    dev->transfer(wbuf, ptr, wbuf, ptr);

    for(i=0;i<size;i++)
    {
        *(value+i) = wbuf[5+i];
    }
}

void AP_InertialSensor_ZeroOnex::fpga_write_reg8(uint16_t reg, uint8_t value) 
{
    uint8_t wbuf[8];
    uint8_t ptr = 0;

    wbuf[ptr++] = FuncBit_Write | FuncBit_Reg | FuncBit_Bit8;
    wbuf[ptr++] = reg & 0xff;
    wbuf[ptr++] = (reg >> 8) & 0xff;
    wbuf[ptr++] = value;

    dev->transfer(wbuf, ptr, wbuf, ptr);
}
uint8_t AP_InertialSensor_ZeroOnex::fpga_read_reg8(uint16_t reg) 
{
    uint8_t wbuf[8];
    uint8_t ptr = 0;

    wbuf[ptr++] = FuncBit_Read | FuncBit_Reg | FuncBit_Bit8;
    wbuf[ptr++] = reg & 0xff;
    wbuf[ptr++] = (reg >> 8) & 0xff;
    wbuf[ptr++] = 0xff;
    wbuf[ptr++] = 0xff;
    dev->transfer(wbuf, ptr, wbuf, ptr);
    return wbuf[4];
}

void AP_InertialSensor_ZeroOnex::fpga_write_reg16(uint16_t reg, uint16_t value) 
{
    uint8_t wbuf[8];
    uint8_t ptr = 0;

    wbuf[ptr++] = FuncBit_Write | FuncBit_Reg | FuncBit_Bit16;
    wbuf[ptr++] = reg & 0xff;
    wbuf[ptr++] = (reg >> 8) & 0xff;
    wbuf[ptr++] = value & 0xff;
    wbuf[ptr++] = (value >> 8) & 0xff;
    

    dev->transfer(wbuf, ptr, wbuf, ptr);
}

void AP_InertialSensor_ZeroOnex::fpga_write_ram8(uint16_t reg, uint16_t ram_addr, uint8_t* value, uint16_t size) {
    uint8_t wbuf[32];
    uint8_t ptr = 0;
    uint8_t i;
    if (value ==nullptr) return;
    wbuf[ptr++] = FuncBit_Write | FuncBit_Ram | FuncBit_Bit8;
    wbuf[ptr++] = reg & 0xff;
    wbuf[ptr++] = (reg >> 8) & 0xff;
    wbuf[ptr++] = ram_addr & 0xff;
    wbuf[ptr++] = (ram_addr >> 8) & 0xff;
        for(i=0;i<size&&ptr<32;i++)
    {
        wbuf[ptr++] = *(value+i);
    }
        if(ptr == 32)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Buff ready to be overflow");
    }
    dev->transfer(wbuf, ptr, wbuf, ptr);
}

void AP_InertialSensor_ZeroOnex::fpga_read_ram8(uint16_t reg, uint16_t ram_addr, uint8_t* value, uint16_t size) {
    uint8_t wbuf[32];
    uint8_t ptr = 0;
    uint8_t i;  
    if (value ==nullptr) return;
    wbuf[ptr++] = FuncBit_Read | FuncBit_Ram | FuncBit_Bit8;
    wbuf[ptr++] = reg & 0xff;
    wbuf[ptr++] = (reg >> 8) & 0xff;
    wbuf[ptr++] = ram_addr & 0xff;
    wbuf[ptr++] = (ram_addr >> 8) & 0xff;
    wbuf[ptr++] = 0xff;
        for(i=0;i<size&&ptr<32;i++)
    {
        wbuf[ptr++] = *(value+i);
    }
    
    dev->transfer(wbuf, ptr, wbuf, ptr);

    for(i=0;i<size;i++)
    {
        *(value+i) = wbuf[6+i];
    }
}

uint8_t AP_InertialSensor_ZeroOnex::fpga_test(void) {
    uint16_t wreg, rreg = 0;
    wreg = rand() & 0xffff;
    fpga_write_reg16(Addr_RW16_TestReg, wreg);
    rreg = fpga_read_reg16(Addr_RW16_TestReg);
    if (wreg != rreg) 
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "failed,wreg is:%d,rreg is:%d",wreg,rreg);
        return 0;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "success,wreg is:%d,rreg is:%d",wreg,rreg);
    return 1;
}



bool AP_InertialSensor_ZeroOnex::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

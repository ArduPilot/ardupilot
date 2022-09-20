/*
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

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_SCHA63T.h"
#include <GCS_MAVLink/GCS.h>

#define S_SUM           0x0E
#define R_S1            0x10
#define A_S1            0x12
#define C_S1            0x14
#define C_S2            0x15
#define G_FILT_DYN      0x16
#define RESCTRL         0x18
#define MODE            0x19
#define A_FILT_DYN      0x1A
#define SEL_BANK        0x1F
#define SET_EOI         0x20

#define ACCEL_BACKEND_SAMPLE_RATE   2000
#define GYRO_BACKEND_SAMPLE_RATE    2000

extern const AP_HAL::HAL& hal;

#define CONSTANTS_ONE_G             (9.80665f)						// m/s^2
#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))

// CRC8-SAE J1850 (X8+X4+X3+X2+1) left move table
const unsigned char	bTblCrc[256] = {
	0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
	0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
	0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
	0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
	0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
	0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
	0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
	0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
	0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
	0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
	0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
	0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
	0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
	0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
	0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
	0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4,
};

AP_InertialSensor_SCHA63T::AP_InertialSensor_SCHA63T(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> _dev_accel,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> _dev_gyro,
                                                   enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev_accel(std::move(_dev_accel))
    , dev_gyro(std::move(_dev_gyro))
    , rotation(_rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_SCHA63T::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                enum Rotation rotation)
{
    if (!dev_accel || !dev_gyro) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_SCHA63T(imu, std::move(dev_accel), std::move(dev_gyro), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_SCHA63T::start()
{
    if (!_imu.register_accel(accel_instance, ACCEL_BACKEND_SAMPLE_RATE, dev_accel->get_bus_id_devtype(DEVTYPE_INS_SCHA63T)) ||
        !_imu.register_gyro(gyro_instance, GYRO_BACKEND_SAMPLE_RATE,   dev_gyro->get_bus_id_devtype(DEVTYPE_INS_SCHA63T))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);
    
    // setup callbacks
    dev_accel->register_periodic_callback(1000000UL / ACCEL_BACKEND_SAMPLE_RATE,
                                          FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SCHA63T::read_fifo_accel, void));
    dev_gyro->register_periodic_callback(1000000UL / GYRO_BACKEND_SAMPLE_RATE,
                                         FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SCHA63T::read_fifo_gyro, void));
}

/*
  probe and initialise accelerometer
 */
bool AP_InertialSensor_SCHA63T::init()
{
    dev_accel->get_semaphore()->take_blocking();
    dev_gyro->get_semaphore()->take_blocking();

    uint8_t v[4];

    hal.scheduler->delay(25);

	RegisterWrite(0, SEL_BANK, 0);
	RegisterWrite(0, RESCTRL, 0);
    hal.scheduler->delay(1);
	RegisterWrite(1, SEL_BANK, 0);
	RegisterWrite(1, RESCTRL, 0);

    hal.scheduler->delay(50);

	RegisterWrite(0, MODE, 0);
	RegisterWrite(0, G_FILT_DYN, 0);
 	RegisterWrite(0, A_FILT_DYN, 0);
    hal.scheduler->delay(1);
	RegisterWrite(1, MODE, 0);
	RegisterWrite(1, MODE, 0);
	RegisterWrite(1, G_FILT_DYN, 0);

    hal.scheduler->delay(500);

	RegisterWrite(0, SET_EOI, 0);
	RegisterWrite(1, SET_EOI, 0);

    hal.scheduler->delay(50);

	RegisterRead(0, S_SUM, v);
	RegisterRead(1, S_SUM, v);
    hal.scheduler->delay(50);
	RegisterRead(0, S_SUM, v);
	RegisterRead(1, S_SUM, v);
    hal.scheduler->delay(50);
	RegisterRead(0, S_SUM, v);
	RegisterRead(1, S_SUM, v);
    hal.scheduler->delay(50);
	int ret_uno = RegisterRead(0, S_SUM, v);
	int ret_due = RegisterRead(1, S_SUM, v);

    dev_accel->get_semaphore()->give();
    dev_gyro->get_semaphore()->give();

    return ret_uno && ret_due;
}

/*
  read accel fifo
 */
void AP_InertialSensor_SCHA63T::read_fifo_accel(void)
{
    static uint8_t cmd_accl_x[4] = { 0x10, 0x00, 0x00, 0xE9 }; // ACCL_X
    static uint8_t cmd_accl_y[4] = { 0x14, 0x00, 0x00, 0xEF }; // ACCL_Y
    static uint8_t cmd_accl_z[4] = { 0x18, 0x00, 0x00, 0xE5 }; // ACCL_Z
    static uint8_t cmd_rate_x[4] = { 0x04, 0x00, 0x00, 0xF7 }; // RATE_X
    static uint8_t cmd_temper[4] = { 0x1C, 0x00, 0x00, 0xE3 }; // TEMPER
 
    uint8_t rsp_accl_x[4];
    uint8_t rsp_accl_y[4];
    uint8_t rsp_accl_z[4];
    uint8_t rsp_rate_x[4];
    uint8_t rsp_temper[4];

    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

	// #### ACCL_X Cmd Send (This Response rsp_accl_x is Dust!!)
    dev_accel->read_registers_fullduplex(cmd_accl_x, rsp_accl_x, 4);

	// #### ACCL_Y Cmd Send + ACCL_X Response Receive
    dev_accel->read_registers_fullduplex(cmd_accl_y, rsp_accl_x, 4);

	// #### ACCL_Z Cmd Send + ACCL_Y Response Receive
    dev_accel->read_registers_fullduplex(cmd_accl_z, rsp_accl_y, 4);

	// ##### RATE_X Cmd Send + ACCL_Z Response Receive
    dev_accel->read_registers_fullduplex(cmd_rate_x, rsp_accl_z, 4);

	// ##### TEMPER Cmd Send + RATE_X Response Receive
    dev_accel->read_registers_fullduplex(cmd_temper, rsp_rate_x, 4);

	// ##### TEMPER Cmd Send + TEMPRE Response Receive
    dev_accel->read_registers_fullduplex(cmd_temper, rsp_temper, 4);

	accel_x = combine(rsp_accl_x[1], rsp_accl_x[2]);
	accel_y = combine(rsp_accl_y[1], rsp_accl_y[2]);
	accel_z = combine(rsp_accl_z[1], rsp_accl_z[2]);

	accel_z = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

    Vector3f accel(accel_x, accel_y, accel_z);
    accel *= (CONSTANTS_ONE_G / 4905.f); // 4905 LSB/g, 0.204mg/LSB
 
    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

	int16_t accl_temper = combine(rsp_temper[1], rsp_temper[2]);
	float temperature = 25.0f + ( accl_temper / 30 );
    float temp_degc = (0.5f * temperature) + 23.0f;
    _publish_temperature(accel_instance, temp_degc);

    AP_HAL::Device::checkreg reg;
    if (!dev_accel->check_next_register(reg)) {
        log_register_change(dev_accel->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
    }

	gyro_x = combine(rsp_rate_x[1], rsp_rate_x[2]);
}

/*
  read gyro fifo
 */
void AP_InertialSensor_SCHA63T::read_fifo_gyro(void)
{
	static uint8_t cmd_rate_y[4] = { 0x0C, 0x00, 0x00, 0xFB }; // RATE_Y
	static uint8_t cmd_rate_z[4] = { 0x04, 0x00, 0x00, 0xF7 }; // RATE_Z
	static uint8_t cmd_temper[4] = { 0x1C, 0x00, 0x00, 0xE3 }; // TEMPER

	uint8_t rsp_rate_y[4];
	uint8_t rsp_rate_z[4];
	uint8_t rsp_temper[4];

	int16_t gyro_y;
	int16_t gyro_z;

	// ####### RATE_Y Cmd Send (This Response rsp_rate_y is Dust!!)
    dev_gyro->read_registers_fullduplex(cmd_rate_y, rsp_rate_y, 4);
	
	// ####### RATE_Z Cmd Send + RATE_Y Response Receive
    dev_gyro->read_registers_fullduplex(cmd_rate_z, rsp_rate_y, 4);

	// ####### TEMPER Cmd Send + RATE_Z Response Receive
    dev_gyro->read_registers_fullduplex(cmd_temper, rsp_rate_z, 4);

	// ##### TEMPER Cmd Send + TEMPRE Response Receive
    dev_gyro->read_registers_fullduplex(cmd_temper, rsp_temper, 4);

	gyro_y = combine(rsp_rate_y[1], rsp_rate_y[2]);
	gyro_z = combine(rsp_rate_z[1], rsp_rate_z[2]);

	gyro_z = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

    Vector3f gyro(gyro_x, gyro_y, gyro_z);
    gyro *= radians(1.f / 80.f);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

	int16_t gyro_temper  = combine(rsp_temper[1], rsp_temper[2]);
	float temperature = 25.0f + ( gyro_temper / 30 );
    float temp_degc = (0.5f * temperature) + 23.0f;
    _publish_temperature(gyro_instance, temp_degc);

    AP_HAL::Device::checkreg reg;
    if (!dev_gyro->check_next_register(reg)) {
        log_register_change(dev_gyro->get_bus_id(), reg);
        _inc_gyro_error_count(gyro_instance);
    }
}

bool AP_InertialSensor_SCHA63T::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

bool AP_InertialSensor_SCHA63T::RegisterRead(int tp, uint8_t reg, uint8_t* val)
{
    bool ret = false;
    uint8_t *cmd;
    uint8_t acc0E[4] = { 0x38, 0x00, 0x00, 0xD5 };
    uint8_t acc10[4] = { 0x40, 0x00, 0x00, 0x91 };
    uint8_t acc12[4] = { 0x48, 0x00, 0x00, 0x9D };
    uint8_t acc14[4] = { 0x50, 0x00, 0x00, 0x89 };
    uint8_t acc15[4] = { 0x54, 0x00, 0x00, 0x8F };

    cmd = acc15;
    switch( reg )
	{
	case S_SUM:  /* 0x0E */
		cmd = acc0E;
		break;
	case R_S1:   /* 0x10 */
		cmd = acc10;
		break;
	case A_S1:   /* 0x12 */
		cmd = acc12;
		break;
	case C_S1:   /* 0x14 */
		cmd = acc14;
		break;
	case C_S2:   /* 0x15 */
		cmd = acc15;
		break;
	default:
		break;
	}

	//transfer(cmd, rrsp, 4);
	switch( tp )
	{
	case 0:
        ret = dev_accel->read_registers_fullduplex(cmd, val, 4);
		break;
	case 1:
        ret = dev_gyro->read_registers_fullduplex(cmd, val, 4);
		break;
	default:
		break;
	}

    if (ret == true)
    {
		unsigned char	bCrc = CalcTblCrc( val, 3);
		if( bCrc != val[3] ) {
	    	ret = false;
		}
    }

	return ret;
}

bool AP_InertialSensor_SCHA63T::RegisterWrite(int tp, uint8_t reg, uint8_t val)
{
    bool ret = false;
    uint8_t res[4];
	uint8_t *cmd;
	uint8_t acc16[4] = { 0xD8, 0x24, 0x24, 0xEE };
	uint8_t acc18[4] = { 0xE0, 0x00, 0x01, 0x7C };
	uint8_t acc19[4] = { 0xE4, 0x00, 0x00, 0x67 };
	uint8_t acc1A[4] = { 0xE8, 0x04, 0x44, 0x27 };
	uint8_t acc1F[4] = { 0xFC, 0x00, 0x00, 0x73 };
	uint8_t acc20[4] = { 0xE0, 0x00, 0x02, 0x5B };

	uint8_t set_d[4] = { 0x74, 0x00, 0x02, 0xB8 };
	uint8_t set_e[4] = { 0x78, 0x00, 0x02, 0x1E };
	uint8_t set_c[4] = { 0x70, 0x00, 0x02, 0x1C };

	cmd = acc16;
	switch( reg )
	{
	case G_FILT_DYN: /* 0x16 */
		cmd = acc16;
		break;
	case RESCTRL: /* 0x18 */
		cmd = acc18;
		break;
	case MODE: /* 0x19 */
		cmd = acc19;
		break;
	case A_FILT_DYN: /* 0x1A */
		cmd = acc1A;
		break;
	case SEL_BANK: /* 0x1F */
		cmd = acc1F;
		break;
	case SET_EOI: /* 0x20 */
		cmd = acc20;
		break;
	case 0x1d: /* 0x1d */
		cmd = set_d;
		break;
	case 0x1e: /* 0x1e */
		cmd = set_e;
		break;
	case 0x1c: /* 0x1c */
		cmd = set_c;
		break;
	default:
		break;
	}
    
	switch( tp )
	{
    case 0:
        ret = dev_accel->write_register_fullduplex(cmd, res, 4);
		break;
    case 1:
        ret = dev_gyro->write_register_fullduplex(cmd, res, 4);
		break;
	default:
		break;
	}
 
    return ret;
}

unsigned char AP_InertialSensor_SCHA63T::CalcTblCrc( unsigned char	*ptr, short	nLen)
{
	unsigned char	bCRC = 0xFF;

	while( nLen--) {
		bCRC = bTblCrc[ bCRC ^ ( *ptr++ & 0xFF ) ];
	}

	bCRC ^= 0xFF;

	return	bCRC;
}

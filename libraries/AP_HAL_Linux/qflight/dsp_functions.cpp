/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  This is an implementation of all of the code for the QFLIGHT board
  that runs on the DSPs. See qflight_dsp.idl for the interface
  definition for the RPC calls
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "qflight_dsp.h"
extern "C" {
#include "bmp280_api.h"
#include "mpu9x50.h"
}

#include <types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <sys/timespec.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <dspal_time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <stdlib.h>
#include <dev_fs_lib_serial.h>
#include "qflight_buffer.h"
#include <AP_HAL/utility/RingBuffer.h>

const float GRAVITY_MSS = 9.80665;
const float ACCEL_SCALE_1G = GRAVITY_MSS / 2048.0;
const float GYRO_SCALE = 0.0174532 / 16.4;
const float RAD_TO_DEG = 57.295779513082320876798154814105;

static ObjectBuffer<DSPBuffer::IMU::BUF> imu_buffer(30);
static ObjectBuffer<DSPBuffer::MAG::BUF> mag_buffer(10);
static ObjectBuffer<DSPBuffer::BARO::BUF> baro_buffer(10);
static bool mpu9250_started;
static uint32_t bmp280_handle;
static uint32_t baro_counter;

/*
  read buffering for UARTs
 */
static const uint8_t max_uarts = 8;
static uint8_t num_open_uarts;
static struct uartbuf {
    int fd;
    ByteBuffer *readbuffer;
} uarts[max_uarts];

extern "C" {
void HAP_debug(const char *msg, int level, const char *filename, int line);
}

void HAP_printf(const char *file, int line, const char *format, ...)
{
	va_list ap;
        char buf[300];
        
	va_start(ap, format);
	vsnprintf(buf, sizeof(buf), format, ap);
	va_end(ap);
        HAP_debug(buf, 0, file, line);
}

void HAP_printf(const char *file, int line, const char *format, ...);
#define HAP_PRINTF(...) HAP_printf(__FILE__, __LINE__, __VA_ARGS__)

static int init_barometer(void)
{
    int ret = bmp280_open("/dev/i2c-2", &bmp280_handle); 
    HAP_PRINTF("**** bmp280: ret=%d handle=0x%x\n", ret, (unsigned)bmp280_handle);
    return ret;
}
        
static int init_mpu9250(void)
{
    struct mpu9x50_config config;
    
    config.gyro_lpf = MPU9X50_GYRO_LPF_184HZ;
    config.acc_lpf  = MPU9X50_ACC_LPF_184HZ;
    config.gyro_fsr = MPU9X50_GYRO_FSR_2000DPS;
    config.acc_fsr  = MPU9X50_ACC_FSR_16G;
    config.gyro_sample_rate = MPU9x50_SAMPLE_RATE_1000HZ;
    config.compass_enabled = true;
    config.compass_sample_rate = MPU9x50_COMPASS_SAMPLE_RATE_100HZ;
    config.spi_dev_path = "/dev/spi-1";

    int ret;
    ret = mpu9x50_validate_configuration(&config);
    HAP_PRINTF("***** mpu9250 validate ret=%d\n", ret);
    if (ret != 0) {
        return ret;
    }
    ret = mpu9x50_initialize(&config);
    HAP_PRINTF("***** mpu9250 initialise ret=%d\n", ret);

    mpu9250_started = true;
    
    return ret;
}

/*
  thread gathering sensor data from mpu9250
 */
static void *mpu_data_ready(void *ctx)
{
    struct mpu9x50_data data;
    memset(&data, 0, sizeof(data));
           
    int ret = mpu9x50_get_data(&data);
    if (ret != 0) {
        return NULL;
    }
    DSPBuffer::IMU::BUF b;
    b.timestamp = data.timestamp;
    b.accel[0]  = data.accel_raw[0]*ACCEL_SCALE_1G;
    b.accel[1]  = data.accel_raw[1]*ACCEL_SCALE_1G;
    b.accel[2]  = data.accel_raw[2]*ACCEL_SCALE_1G;
    b.gyro[0]   = data.gyro_raw[0]*GYRO_SCALE;
    b.gyro[1]   = data.gyro_raw[1]*GYRO_SCALE;
    b.gyro[2]   = data.gyro_raw[2]*GYRO_SCALE;
    imu_buffer.push(b);

    if (data.mag_data_ready) {
        DSPBuffer::MAG::BUF m;
        m.mag_raw[0] = data.mag_raw[0];
        m.mag_raw[1] = data.mag_raw[1];
        m.mag_raw[2] = data.mag_raw[2];
        m.timestamp = data.timestamp;
        mag_buffer.push(m);
    }

    if (bmp280_handle != 0 && baro_counter++ % 10 == 0) {
        struct bmp280_sensor_data data;
        memset(&data, 0, sizeof(data));
        int ret = bmp280_get_sensor_data(bmp280_handle, &data, false);
        if (ret == 0) {
            DSPBuffer::BARO::BUF b;
            b.pressure_pa = data.pressure_in_pa;
            b.temperature_C = data.temperature_in_c;
            b.timestamp = data.last_read_time_in_usecs;
            baro_buffer.push(b);
        }
    }

    return NULL;
}

static void mpu9250_startup(void)
{
    if (!mpu9250_started) {
        if (init_mpu9250() != 0) {
            return;
        }
        mpu9x50_register_interrupt(65, mpu_data_ready, NULL);
    }
}

/*
  get any available IMU data
 */
int qflight_get_imu_data(uint8_t *buf, int len)
{
    DSPBuffer::IMU &imu = *(DSPBuffer::IMU *)buf;

    if (len != sizeof(imu)) {
        HAP_PRINTF("incorrect size for imu data %d should be %d\n",
                   len, sizeof(imu));
        return 1;
    }

    mpu9250_startup();

    imu.num_samples = 0;
    while (imu.num_samples < imu.max_samples &&
           imu_buffer.pop(imu.buf[imu.num_samples])) {
        imu.num_samples++;
    }
    
    return 0;
}

/*
  get any available mag data
 */
int qflight_get_mag_data(uint8_t *buf, int len)
{
    DSPBuffer::MAG &mag = *(DSPBuffer::MAG *)buf;

    if (len != sizeof(mag)) {
        HAP_PRINTF("incorrect size for mag data %d should be %d\n",
                   len, sizeof(mag));
        return 1;
    }

    mpu9250_startup();

    mag.num_samples = 0;
    while (mag.num_samples < mag.max_samples &&
           mag_buffer.pop(mag.buf[mag.num_samples])) {
        mag.num_samples++;
    }
    
    return 0;
}


/*
  get any available baro data
 */
int qflight_get_baro_data(uint8_t *buf, int len)
{
    DSPBuffer::BARO &baro = *(DSPBuffer::BARO *)buf;

    if (len != sizeof(baro)) {
        HAP_PRINTF("incorrect size for baro data %d should be %d\n",
                   len, sizeof(baro));
        return 1;
    }

    mpu9250_startup();

    if (bmp280_handle == 0) {
        if (init_barometer() != 0) {
            return 1;
        }
    }

    baro.num_samples = 0;
    while (baro.num_samples < baro.max_samples &&
           baro_buffer.pop(baro.buf[baro.num_samples])) {
        baro.num_samples++;
    }
    
    return 0;
}

extern "C" {
static void read_callback_trampoline(void *, char *, size_t );
}

static void read_callback_trampoline(void *ctx, char *buf, size_t size)
{
    if (size > 0) {
        ((ByteBuffer *)ctx)->write((const uint8_t *)buf, size);
    }
}

/*
  open a UART
 */
int qflight_UART_open(const char *device, int32_t *_fd)
{
    if (num_open_uarts == max_uarts) {
        return -1;
    }
    struct uartbuf &b = uarts[num_open_uarts];
    int fd = open(device, O_RDWR | O_NONBLOCK);
    if (fd == -1) {
        return -1;
    }
    b.fd = fd;
    b.readbuffer = new ByteBuffer(16384);

    struct dspal_serial_open_options options;
    options.bit_rate = DSPAL_SIO_BITRATE_57600;
    options.tx_flow = DSPAL_SIO_FCTL_OFF;
    options.rx_flow = DSPAL_SIO_FCTL_OFF;
    options.rx_data_callback = nullptr;
    options.tx_data_callback = nullptr;
    options.is_tx_data_synchronous = false;
    int ret = ioctl(fd, SERIAL_IOCTL_OPEN_OPTIONS, (void *)&options);
    if (ret != 0) {
        HAP_PRINTF("Failed to setup UART flow control options");
    }
    
    struct dspal_serial_ioctl_receive_data_callback callback {};
    callback.context = b.readbuffer;
    callback.rx_data_callback_func_ptr = read_callback_trampoline;
    ret = ioctl(fd, SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK, (void *)&callback);
    if (ret != 0) {
        HAP_PRINTF("Failed to setup UART read trampoline");
        delete b.readbuffer;
        close(fd);
        return -1;
    }

    HAP_PRINTF("UART open %s fd=%d num_open=%u",
               device, fd, num_open_uarts);
    num_open_uarts++;
    *_fd = fd;
    return 0;
}

/*
  close a UART
 */
int qflight_UART_close(int32_t fd)
{
    uint8_t i;
    for (i=0; i<num_open_uarts; i++) {
        if (fd == uarts[i].fd) break;
    }
    if (i == num_open_uarts) {
        return -1;
    }
    close(fd);
    delete uarts[i].readbuffer;
    if (i < num_open_uarts-1) {
        memmove(&uarts[i], &uarts[i+1], ((num_open_uarts-1)-i)*sizeof(uarts[0]));
    }
    num_open_uarts--;
    return 0;
}

/*
  read from a UART
 */
int qflight_UART_read(int32_t fd, uint8_t *buf, int size, int32_t *nread)
{
    uint8_t i;
    for (i=0; i<num_open_uarts; i++) {
        if (fd == uarts[i].fd) break;
    }
    if (i == num_open_uarts) {
        return -1;
    }
    *nread = uarts[i].readbuffer->read(buf, size);
    return 0;
}

/*
  write to a UART
 */
int qflight_UART_write(int32_t fd, const uint8_t *buf, int size, int32_t *nwritten)
{
    *nwritten = write(fd, buf, size);
    return 0;
}

static const struct {
    uint32_t baudrate;
    enum DSPAL_SERIAL_BITRATES arg;
} baudrate_table[] = {
    {    9600, DSPAL_SIO_BITRATE_9600 },
    {   14400, DSPAL_SIO_BITRATE_14400 },
    {   19200, DSPAL_SIO_BITRATE_19200 },
    {   38400, DSPAL_SIO_BITRATE_38400 },
    {   57600, DSPAL_SIO_BITRATE_57600 },
    {   76800, DSPAL_SIO_BITRATE_76800 },
    {  115200, DSPAL_SIO_BITRATE_115200 },
    {  230400, DSPAL_SIO_BITRATE_230400 },
    {  250000, DSPAL_SIO_BITRATE_250000 },
    {  460800, DSPAL_SIO_BITRATE_460800 },
    {  921600, DSPAL_SIO_BITRATE_921600 },
    { 2000000, DSPAL_SIO_BITRATE_2000000 },
};

/*
  set UART baudrate
 */
int qflight_UART_set_baudrate(int32_t fd, uint32_t baudrate)
{
    for (uint8_t i=0; i<sizeof(baudrate_table)/sizeof(baudrate_table[0]); i++) {
        if (baudrate <= baudrate_table[i].baudrate) {
            struct dspal_serial_ioctl_data_rate rate {};
            rate.bit_rate = baudrate_table[i].arg;
            int ret = ioctl(fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&rate);
            HAP_PRINTF("set_rate -> %d\n", ret);
            return 0;
        }
    }
    return -1;
}

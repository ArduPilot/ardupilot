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
 *   AP_BoardConfig - px4 driver loading and setup
 */


#include <AP_HAL/AP_HAL.h>
#include "AP_BoardConfig.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <GCS_MAVLink/GCS.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>
#include <nuttx/arch.h>
#include <spawn.h>

extern const AP_HAL::HAL& hal;

AP_BoardConfig::px4_board_type AP_BoardConfig::px4_configured_board;

/* 
   declare driver main entry points
 */
extern "C" {
    int mpu6000_main(int , char **);
    int mpu9250_main(int , char **);
    int ms5611_main(int , char **);
    int l3gd20_main(int , char **);
    int lsm303d_main(int , char **);
    int hmc5883_main(int , char **);
    int uavcan_main(int, char **);
    int fmu_main(int, char **);
    int px4io_main(int, char **);
    int adc_main(int, char **);
    int tone_alarm_main(int, char **);
};


#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && !defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
/*
  this is needed for the code to wait for CAN startup
 */
#define _UAVCAN_IOCBASE             (0x4000)                        // IOCTL base for module UAVCAN
#define _UAVCAN_IOC(_n)             (_IOC(_UAVCAN_IOCBASE, _n))

#define UAVCAN_IOCG_NODEID_INPROGRESS  _UAVCAN_IOC(1)               // query if node identification is in progress
#endif


/*
  setup PWM pins
 */
void AP_BoardConfig::px4_setup_pwm()
{
    /* configure the FMU driver for the right number of PWMs */
    static const struct {
        uint8_t mode_parm;
        uint8_t mode_value;
        uint8_t num_gpios;
    } mode_table[] = {
        /* table mapping BRD_PWM_COUNT to ioctl arguments */
        { 0, PWM_SERVO_MODE_NONE, 6 },
        { 2, PWM_SERVO_MODE_2PWM, 4 },
        { 4, PWM_SERVO_MODE_4PWM, 2 },
        { 6, PWM_SERVO_MODE_6PWM, 0 },
        { 7, PWM_SERVO_MODE_3PWM1CAP, 2 },
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        { 8, PWM_SERVO_MODE_12PWM, 0 },
#endif
    };
    uint8_t mode_parm = (uint8_t)px4.pwm_count.get();
    uint8_t i;
    for (i=0; i<ARRAY_SIZE(mode_table); i++) {
        if (mode_table[i].mode_parm == mode_parm) {
            break;
        }
    }
    if (i == ARRAY_SIZE(mode_table)) {
        hal.console->printf("RCOutput: invalid BRD_PWM_COUNT %u\n", mode_parm); 
    } else {
        int fd = open("/dev/px4fmu", 0);
        if (fd == -1) {
            AP_HAL::panic("Unable to open /dev/px4fmu");
        }
        if (ioctl(fd, PWM_SERVO_SET_MODE, mode_table[i].mode_value) != 0) {
            hal.console->printf("RCOutput: unable to setup AUX PWM with BRD_PWM_COUNT %u\n", mode_parm);
        }   
        close(fd);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        if (mode_table[i].num_gpios < 2) {
            // reduce change of config mistake where relay and PWM interfere
            AP_Param::set_default_by_name("RELAY_PIN", -1);
            AP_Param::set_default_by_name("RELAY_PIN2", -1);
        }
#endif
    }
}


/*
  setup flow control on UARTs
 */
void AP_BoardConfig::px4_setup_uart()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    hal.uartC->set_flow_control((AP_HAL::UARTDriver::flow_control)px4.ser1_rtscts.get());
    if (hal.uartD != nullptr) {
        hal.uartD->set_flow_control((AP_HAL::UARTDriver::flow_control)px4.ser2_rtscts.get());
    }
#endif
}

/*
  setup safety switch
 */
void AP_BoardConfig::px4_setup_safety_mask()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // setup channels to ignore the armed state
    int px4io_fd = open("/dev/px4io", 0);
    if (px4io_fd != -1) {
        if (ioctl(px4io_fd, PWM_SERVO_IGNORE_SAFETY, (uint16_t)px4.ignore_safety_channels) != 0) {
            hal.console->printf("IGNORE_SAFETY failed\n");
        }
    }
    int px4fmu_fd = open("/dev/px4fmu", 0);
    if (px4fmu_fd != -1) {
        uint16_t mask = px4.ignore_safety_channels;
        if (px4io_fd != -1) {
            mask >>= 8;
        }
        if (ioctl(px4fmu_fd, PWM_SERVO_IGNORE_SAFETY, (uint16_t)mask) != 0) {
            hal.console->printf("IGNORE_SAFETY failed\n");
        }
        close(px4fmu_fd);
    }
    if (px4io_fd != -1) {
        close(px4io_fd);
    }
#endif
}

/*
  setup safety switch
 */
void AP_BoardConfig::px4_setup_safety()
{
    px4_setup_safety_mask();

    if (px4.safety_enable.get() == 0) {
        hal.rcout->force_safety_off();
        hal.rcout->force_safety_no_wait();
        // wait until safety has been turned off
        uint8_t count = 20;
        while (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_ARMED && count--) {
            hal.scheduler->delay(20);
        }
    }
}


/*
  setup SBUS
 */
void AP_BoardConfig::px4_setup_sbus(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    if (px4.sbus_out_rate.get() >= 1) {
        static const struct {
            uint8_t value;
            uint16_t rate;
        } rates[] = {
            { 1, 50 },
            { 2, 75 },
            { 3, 100 },
            { 4, 150 },
            { 5, 200 },
            { 6, 250 },
            { 7, 300 }
        };
        uint16_t rate = 300;
        for (uint8_t i=0; i<ARRAY_SIZE(rates); i++) {
            if (rates[i].value == px4.sbus_out_rate) {
                rate = rates[i].rate;
            }
        }
        if (!hal.rcout->enable_sbus_out(rate)) {
            hal.console->printf("Failed to enable SBUS out\n");
        }
    }
#endif
}


/*
  setup CANBUS drivers
 */
void AP_BoardConfig::px4_setup_canbus(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && !defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
    if (px4.can_enable >= 1) {
        // give time for other drivers to fully start before we start
        // canbus. This prevents a race where a canbus mag comes up
        // before the hmc5883
        hal.scheduler->delay(500);
        if (px4_start_driver(uavcan_main, "uavcan", "start")) {
            hal.console->printf("UAVCAN: started\n");            
            // give some time for CAN bus initialisation
            hal.scheduler->delay(2000);
        } else {
            hal.console->printf("UAVCAN: failed to start\n");
        }
        // give time for canbus drivers to register themselves
        hal.scheduler->delay(2000);
    }
    if (px4.can_enable >= 2) {
        if (px4_start_driver(uavcan_main, "uavcan", "start fw")) {
            uint32_t start_wait_ms = AP_HAL::millis();
            int fd = open("/dev/uavcan/esc", 0); // design flaw of uavcan driver, this should be /dev/uavcan/node one day
            if (fd == -1) {
                AP_HAL::panic("Configuration invalid - unable to open /dev/uavcan/esc");
            }

            // delay startup, UAVCAN still discovering nodes
            while (ioctl(fd, UAVCAN_IOCG_NODEID_INPROGRESS,0) == OK &&
                   AP_HAL::millis() - start_wait_ms < 7000) {
                hal.scheduler->delay(500);
            }
            hal.console->printf("UAVCAN: node discovery complete\n");
            close(fd);
        }
    }
#endif // CONFIG_HAL_BOARD && !CONFIG_ARCH_BOARD_PX4FMU_V1
}

extern "C" int waitpid(pid_t, int *, int);

/*
  start one px4 driver
 */
bool AP_BoardConfig::px4_start_driver(main_fn_t main_function, const char *name, const char *arguments)
{
    char *s = strdup(arguments);
    char *args[10];
    uint8_t nargs = 0;
    char *saveptr = nullptr;

    // parse into separate arguments
    for (char *tok=strtok_r(s, " ", &saveptr); tok; tok=strtok_r(nullptr, " ", &saveptr)) {
        args[nargs++] = tok;
        if (nargs == ARRAY_SIZE(args)-1) {
            break;
        }
    }
    args[nargs++] = nullptr;

    printf("Starting driver %s %s\n", name, arguments);
    pid_t pid;
    
    if (task_spawn(&pid, name, main_function, nullptr, nullptr,
                   args, nullptr) != 0) {
        free(s);
        printf("Failed to spawn %s\n", name);
        return false;
    }

    // wait for task to exit and gather status
    int status = -1;
    if (waitpid(pid, &status, 0) != pid) {
        printf("waitpid failed for %s\n", name);
        free(s);
        return false;
    }
    free(s);
    return (status >> 8) == 0;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
/*
  setup sensors for PX4v2
 */
void AP_BoardConfig::px4_start_fmuv2_sensors(void)
{
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
    bool have_FMUV3 = false;
    
    printf("Starting FMUv2 sensors\n");
    if (px4_start_driver(hmc5883_main, "hmc5883", "-C -T -I -R 4 start")) {
        printf("Have internal hmc5883\n");
    } else {
        printf("No internal hmc5883\n");
    }

    // external MPU6000 is rotated YAW_180 from standard
    if (px4_start_driver(mpu6000_main, "mpu6000", "-X -R 4 start")) {
        printf("Found MPU6000 external\n");
        have_FMUV3 = true;
    } else {
        if (px4_start_driver(mpu9250_main, "mpu9250", "-X -R 4 start")) {
            printf("Found MPU9250 external\n");
            have_FMUV3 = true;
        } else {
            printf("No MPU6000 or MPU9250 external\n");
        }
    }
    if (have_FMUV3) {
        // external L3GD20 is rotated YAW_180 from standard
        if (px4_start_driver(l3gd20_main, "l3gd20", "-X -R 4 start")) {
            printf("l3gd20 external started OK\n");
        } else {
            px4_sensor_error("No l3gd20");
        }
        // external LSM303D is rotated YAW_270 from standard
        if (px4_start_driver(lsm303d_main, "lsm303d", "-a 16 -X -R 6 start")) {
            printf("lsm303d external started OK\n");
        } else {
            px4_sensor_error("No lsm303d");
        }
        // internal MPU6000 is rotated ROLL_180_YAW_270 from standard
        if (px4_start_driver(mpu6000_main, "mpu6000", "-R 14 start")) {
            printf("Found MPU6000 internal\n");
        } else {
            if (px4_start_driver(mpu9250_main, "mpu9250", "-R 14 start")) {
                printf("Found MPU9250 internal\n");
            } else {
                px4_sensor_error("No MPU6000 or MPU9250");
            }
        }
        if (px4_start_driver(hmc5883_main, "hmc5883", "-C -T -S -R 8 start")) {
            printf("Found SPI hmc5883\n");
        }
    } else {
        // not FMUV3 (ie. not a pixhawk2)
        if (px4_start_driver(mpu6000_main, "mpu6000", "start")) {
            printf("Found MPU6000\n");
        } else {
            if (px4_start_driver(mpu9250_main, "mpu9250", "start")) {
                printf("Found MPU9250\n");
            } else {
                printf("No MPU6000 or MPU9250\n");
            }
        }
        if (px4_start_driver(l3gd20_main, "l3gd20", "start")) {
            printf("l3gd20 started OK\n");
        } else {
            px4_sensor_error("no l3gd20 found");
        }
        if (px4_start_driver(lsm303d_main, "lsm303d", "-a 16 start")) {
            printf("lsm303d started OK\n");
        } else {
            px4_sensor_error("no lsm303d found");
        }
    }

    if (have_FMUV3) {
        // on Pixhawk2 default IMU temperature to 60
        _imu_target_temperature.set_default(60);
    }
    
    printf("FMUv2 sensors started\n");
#endif // CONFIG_ARCH_BOARD_PX4FMU_V2
}


/*
  setup sensors for PX4v1
 */
void AP_BoardConfig::px4_start_fmuv1_sensors(void)
{
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
    printf("Starting FMUv1 sensors\n");
    if (px4_start_driver(hmc5883_main, "hmc5883", "-C -T -I start")) {
        printf("Have internal hmc5883\n");
    } else {
        printf("No internal hmc5883\n");
    }
    if (px4_start_driver(mpu6000_main, "mpu6000", "start")) {
        printf("mpu6000 started OK\n");
    } else {
        px4_sensor_error("mpu6000");
    }
#endif // CONFIG_ARCH_BOARD_PX4FMU_V1
}

/*
  setup common sensors
 */
void AP_BoardConfig::px4_start_common_sensors(void)
{
#ifndef CONFIG_ARCH_BOARD_PX4FMU_V4
    if (px4_start_driver(ms5611_main, "ms5611", "start")) {
        printf("ms5611 started OK\n");
    } else {
        px4_sensor_error("no ms5611 found");
    }
    if (px4_start_driver(hmc5883_main, "hmc5883", "-C -T -X start")) {
        printf("Have external hmc5883\n");
    } else {
        printf("No external hmc5883\n");
    }
#endif
}

#endif // CONFIG_HAL_BOARD

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
void AP_BoardConfig::vrx_start_brain51_sensors(void)
{
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
    if (px4_start_driver(hmc5883_main, "hmc5883", "-C -R 12 -I start")) {
        printf("HMC5883 Internal GPS started OK\n");
    } else {
        printf("HMC5883 Internal GPS start failed\n");
    }

    if (px4_start_driver(mpu6000_main, "mpu6000", "-R 12 start")) {
        printf("MPU6000 Internal started OK\n");
    } else {
        px4_sensor_error("MPU6000 Internal start failed");
    }
#endif
}

void AP_BoardConfig::vrx_start_brain52_sensors(void)
{
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V52)
    if (px4_start_driver(hmc5883_main, "hmc5883", "-C -R 12 -I start")) {
        printf("HMC5883 Internal GPS started OK\n");
    } else {
        printf("HMC5883 Internal GPS start failed\n");
    }

    if (px4_start_driver(mpu6000_main, "mpu6000", "-R 12 start")) {
        printf("MPU6000 Internal started OK\n");
    } else {
        px4_sensor_error("MPU6000 Internal start failed");
    }
#endif
}

void AP_BoardConfig::vrx_start_ubrain51_sensors(void)
{
#if defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
    if (px4_start_driver(mpu6000_main, "mpu6000", "-R 12 start")) {
        printf("MPU6000 Internal started OK\n");
    } else {
        px4_sensor_error("MPU6000 Internal start failed");
    }
#endif
}

void AP_BoardConfig::vrx_start_ubrain52_sensors(void)
{
#if defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)
    if (px4_start_driver(mpu6000_main, "mpu6000", "-R 12 start")) {
        printf("MPU6000 Internal started OK\n");
    } else {
        px4_sensor_error("MPU6000 Internal start failed");
    }
#endif
}

void AP_BoardConfig::vrx_start_core10_sensors(void)
{
#if defined(CONFIG_ARCH_BOARD_VRCORE_V10)
    if (px4_start_driver(mpu9250_main, "mpu9250", "-R 4 start")) {
        printf("MPU9250 Internal started OK\n");
    } else {
        px4_sensor_error("MPU9250 Internal start failed");
    }
#endif
}

void AP_BoardConfig::vrx_start_brain54_sensors(void)
{
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V54)
    if (px4_start_driver(hmc5883_main, "hmc5883", "-C -R 12 -I start")) {
        printf("HMC5883 Internal GPS started OK\n");
    } else {
        printf("HMC5883 Internal GPS start failed\n");
    }

    if (px4_start_driver(mpu6000_main, "mpu6000", "-R 12 start")) {
        printf("MPU6000 Internal started OK\n");
    } else {
        px4_sensor_error("MPU6000 Internal start failed");
    }
#endif
}

/*
  setup common sensors
 */
void AP_BoardConfig::vrx_start_common_sensors(void)
{
    if (px4_start_driver(ms5611_main, "ms5611", "-s start")) {
        printf("MS5611 Internal started OK\n");
    } else {
        px4_sensor_error("MS5611 Internal start failed");
    }

    if (px4_start_driver(hmc5883_main, "hmc5883", "-C -X start")) {
        printf("HMC5883 External GPS started OK\n");
    } else {
        printf("HMC5883 External GPS start failed\n");
    }
}

#endif // CONFIG_HAL_BOARD

void AP_BoardConfig::px4_setup_drivers(void)
{
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
    /*
      this works around an issue with some FMUv4 hardware (eg. copies
      of the Pixracer) which have incorrect components leading to
      sensor brownout on boot
     */
    if (px4_start_driver(fmu_main, "fmu", "sensor_reset 20")) {
        printf("FMUv4 sensor reset complete\n");        
    }
#endif

    // run board auto-detection
    px4_autodetect();

    if (px4.board_type == PX4_BOARD_PH2SLIM ||
        px4.board_type == PX4_BOARD_PIXHAWK2) {
        _imu_target_temperature.set_default(60);
    }
    
    if (px4.board_type == PX4_BOARD_PX4V1 ||
        px4.board_type == PX4_BOARD_PHMINI ||
        px4.board_type == PX4_BOARD_PH2SLIM ||
        px4.board_type == PX4_BOARD_PIXRACER ||
        px4.board_type == PX4_BOARD_PIXHAWK ||
        px4.board_type == PX4_BOARD_PIXHAWK2) {
        // use in-tree drivers
        printf("Using in-tree drivers\n");
        px4_configured_board = (enum px4_board_type)px4.board_type.get();
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    px4_start_common_sensors();
    switch ((px4_board_type)px4.board_type.get()) {
    case PX4_BOARD_AUTO:
    default:
        px4_start_fmuv1_sensors();
        px4_start_fmuv2_sensors();
        break;
    }
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    vrx_start_common_sensors();
    switch ((px4_board_type)px4.board_type.get()) {
    case VRX_BOARD_BRAIN51:
        vrx_start_brain51_sensors();
        break;

    case VRX_BOARD_BRAIN52:
        vrx_start_brain52_sensors();
        break;

    case VRX_BOARD_UBRAIN51:
        vrx_start_ubrain51_sensors();
        break;

    case VRX_BOARD_UBRAIN52:
        vrx_start_ubrain52_sensors();
        break;

    case VRX_BOARD_CORE10:
        vrx_start_core10_sensors();
        break;

    case VRX_BOARD_BRAIN54:
        vrx_start_brain54_sensors();
        break;

    default:
        break;
    }
#endif // HAL_BOARD_PX4

    px4_configured_board = (enum px4_board_type)px4.board_type.get();
    
    // delay for 1 second to give time for drivers to initialise
    hal.scheduler->delay(1000);
}

/*
  play a tune
 */
void AP_BoardConfig::px4_tone_alarm(const char *tone_string)
{
    px4_start_driver(tone_alarm_main, "tone_alarm", tone_string);
}

/*
  setup px4io, possibly updating firmware
 */
void AP_BoardConfig::px4_setup_px4io(void)
{
    if (px4_start_driver(px4io_main, "px4io", "start norc")) {
        printf("px4io started OK\n");
    } else {
        // might be in bootloader mode if user held down safety switch
        // at power on
        printf("Loading /etc/px4io/px4io.bin\n");
        px4_tone_alarm("MBABGP");
        if (px4_start_driver(px4io_main, "px4io", "update /etc/px4io/px4io.bin")) {
            printf("upgraded PX4IO firmware OK\n");
            px4_tone_alarm("MSPAA");
        } else {
            printf("Failed to upgrade PX4IO firmware\n");
            px4_tone_alarm("MNGGG");
        }
        hal.scheduler->delay(1000);
        if (px4_start_driver(px4io_main, "px4io", "start norc")) {
            printf("px4io started OK\n");
        } else {
            px4_sensor_error("px4io start failed");
        }
    }

    /*
      see if we need to update px4io firmware
     */
    if (px4_start_driver(px4io_main, "px4io", "checkcrc /etc/px4io/px4io.bin")) {
        printf("PX4IO CRC OK\n");
    } else {
        printf("PX4IO CRC failure\n");
        px4_tone_alarm("MBABGP");
        if (px4_start_driver(px4io_main, "px4io", "safety_on")) {
            printf("PX4IO disarm OK\n");
        } else {
            printf("PX4IO disarm failed\n");
        }
        hal.scheduler->delay(1000);

        if (px4_start_driver(px4io_main, "px4io", "forceupdate 14662 /etc/px4io/px4io.bin")) {
            hal.scheduler->delay(1000);
            if (px4_start_driver(px4io_main, "px4io", "start norc")) {
                printf("px4io restart OK\n");
                px4_tone_alarm("MSPAA");
            } else {
                px4_tone_alarm("MNGGG");
                px4_sensor_error("PX4IO restart failed");
            }
        } else {
            printf("PX4IO update failed\n");
            px4_tone_alarm("MNGGG");
        }
    }
}

/*
  setup required peripherals like adc, rcinput and rcoutput
 */
void AP_BoardConfig::px4_setup_peripherals(void)
{
    // always start adc
    if (px4_start_driver(adc_main, "adc", "start")) {
        hal.analogin->init();
        printf("ADC started OK\n");
    } else {
        px4_sensor_error("no ADC found");
    }

#ifndef CONFIG_ARCH_BOARD_PX4FMU_V4
    px4_setup_px4io();
#endif

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
    const char *fmu_mode = "mode_serial";
#else
    const char *fmu_mode = "mode_pwm4";
#endif
    if (px4_start_driver(fmu_main, "fmu", fmu_mode)) {
        printf("fmu %s started OK\n", fmu_mode);
    } else {
        px4_sensor_error("fmu start failed");
    }

    hal.gpio->init();
    hal.rcin->init();
    hal.rcout->init();
}

/*
  check a SPI device for a register value
 */
bool AP_BoardConfig::spi_check_register(const char *devname, uint8_t regnum, uint8_t value, uint8_t read_flag)
{
    auto dev = hal.spi->get_device(devname);
    if (!dev) {
        printf("%s: no device\n", devname);
        return false;
    }
    dev->set_read_flag(read_flag);
    uint8_t v;
    if (!dev->read_registers(regnum, &v, 1)) {
        printf("%s: reg %02x read fail\n", devname, (unsigned)regnum);
        return false;
    }
    printf("%s: reg %02x %02x %02x\n", devname, (unsigned)regnum, (unsigned)value, (unsigned)v);
    return v == value;
}

#define MPUREG_WHOAMI 0x75
#define MPU_WHOAMI_MPU60X0  0x68
#define MPU_WHOAMI_MPU9250  0x71
#define MPU_WHOAMI_ICM20608 0xaf

#define LSMREG_WHOAMI 0x0f
#define LSM_WHOAMI_LSM303D 0x49

/*
  auto-detect board type
 */
void AP_BoardConfig::px4_autodetect(void)
{
    if (px4.board_type != PX4_BOARD_AUTO) {
        // user has chosen a board type
        return;
    }
    
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
    // only one choice
    px4.board_type.set(PX4_BOARD_PX4V1);
    hal.console->printf("Detected PX4v1\n");

#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
    if ((spi_check_register(HAL_INS_MPU60x0_EXT_NAME, MPUREG_WHOAMI, MPU_WHOAMI_MPU60X0) ||
         spi_check_register(HAL_INS_MPU9250_EXT_NAME, MPUREG_WHOAMI, MPU_WHOAMI_MPU9250) ||
         spi_check_register(HAL_INS_ICM20608_EXT_NAME, MPUREG_WHOAMI, MPU_WHOAMI_ICM20608)) &&
        spi_check_register(HAL_INS_LSM9DS0_EXT_A_NAME, LSMREG_WHOAMI, LSM_WHOAMI_LSM303D)) {
        // Pixhawk2 has LSM303D and MPUxxxx on external bus
        px4.board_type.set(PX4_BOARD_PIXHAWK2);
        hal.console->printf("Detected PIXHAWK2\n");
    } else if (spi_check_register(HAL_INS_ICM20608_AM_NAME, MPUREG_WHOAMI, MPU_WHOAMI_ICM20608) &&
               spi_check_register(HAL_INS_MPU9250_NAME, MPUREG_WHOAMI, MPU_WHOAMI_MPU9250)) {
        // PHMINI has an ICM20608 and MPU9250 on sensor bus
        px4.board_type.set(PX4_BOARD_PHMINI);
        hal.console->printf("Detected PixhawkMini\n");
    } else if (spi_check_register(HAL_INS_LSM9DS0_A_NAME, LSMREG_WHOAMI, LSM_WHOAMI_LSM303D) &&
               (spi_check_register(HAL_INS_MPU60x0_NAME, MPUREG_WHOAMI, MPU_WHOAMI_MPU60X0) ||
                spi_check_register(HAL_INS_ICM20608_NAME, MPUREG_WHOAMI, MPU_WHOAMI_ICM20608) ||
                spi_check_register(HAL_INS_MPU9250_NAME, MPUREG_WHOAMI, MPU_WHOAMI_MPU9250))) {
        // classic or upgraded Pixhawk1
        px4.board_type.set(PX4_BOARD_PIXHAWK);
        hal.console->printf("Detected Pixhawk\n");
    } else {
        px4_sensor_error("Unable to detect board type");
    }
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
    // only one choice
    px4.board_type.set_and_notify(PX4_BOARD_PIXRACER);    
    hal.console->printf("Detected Pixracer\n");
#endif
    
}

/*
  fail startup of a required sensor
 */
void AP_BoardConfig::px4_sensor_error(const char *reason)
{
    /*
      to give the user the opportunity to connect to USB we keep
      repeating the error.  The mavlink delay callback is initialised
      before this, so the user can change parameters (and in
      particular BRD_TYPE if needed)
    */
    while (true) {
        printf("Sensor failure: %s\n", reason);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "Check BRD_TYPE: %s", reason);
        hal.scheduler->delay(3000);
    }
}

/*
  setup px4 peripherals and drivers
 */
void AP_BoardConfig::px4_setup()
{
    px4_setup_peripherals();
    px4_setup_pwm();
    px4_setup_safety();
    px4_setup_uart();
    px4_setup_sbus();
    px4_setup_drivers();
    px4_setup_canbus();
}

#endif // HAL_BOARD_PX4

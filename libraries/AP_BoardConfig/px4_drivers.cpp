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
 *   AP_BoardConfig - px4 driver loading and setup
 */


#include <AP_HAL/AP_HAL.h>
#include "AP_BoardConfig.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <GCS_MAVLink/GCS.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>
#include <nuttx/arch.h>
#include <spawn.h>

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
#define BOARD_PWM_COUNT_DEFAULT 2
#define BOARD_SER1_RTSCTS_DEFAULT 0 // no flow control on UART5 on FMUv1
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
#define BOARD_PWM_COUNT_DEFAULT 6
#define BOARD_SER1_RTSCTS_DEFAULT 2
#else // V2
#define BOARD_PWM_COUNT_DEFAULT 4
#define BOARD_SER1_RTSCTS_DEFAULT 2
#endif

extern const AP_HAL::HAL& hal;

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
    int ets_airspeed_main(int, char **);
    int meas_airspeed_main(int, char **);
    int ll40ls_main(int, char **);
    int trone_main(int, char **);
    int mb12xx_main(int, char **);
    int pwm_input_main(int, char **);
    int uavcan_main(int, char **);
};


#if !defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
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
        if (mode_table[i].num_gpios < 2) {
            // reduce change of config mistake where relay and PWM interfere
            AP_Param::set_default_by_name("RELAY_PIN", -1);
            AP_Param::set_default_by_name("RELAY_PIN2", -1);
        }
    }
}


/*
  setup flow control on UARTs
 */
void AP_BoardConfig::px4_setup_uart()
{
    hal.uartC->set_flow_control((AP_HAL::UARTDriver::flow_control)px4.ser1_rtscts.get());
    if (hal.uartD != NULL) {
        hal.uartD->set_flow_control((AP_HAL::UARTDriver::flow_control)px4.ser2_rtscts.get());
    }
}

/*
  setup safety switch
 */
void AP_BoardConfig::px4_setup_safety()
{
    // setup channels to ignore the armed state
    int px4io_fd = open("/dev/px4io", 0);
    if (px4io_fd != -1) {
        if (ioctl(px4io_fd, PWM_SERVO_IGNORE_SAFETY, (uint16_t)(0x0000FFFF & px4.ignore_safety_channels)) != 0) {
            hal.console->printf("IGNORE_SAFETY failed\n");
        }
        close(px4io_fd);
    }

    if (px4.safety_enable.get() == 0) {
        hal.rcout->force_safety_off();
    }
}


/*
  setup SBUS
 */
void AP_BoardConfig::px4_setup_sbus(void)
{
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
}


/*
  setup CANBUS drivers
 */
void AP_BoardConfig::px4_setup_canbus(void)
{
#if !defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
    if (px4.can_enable >= 1) {
        if (px4_start_driver(uavcan_main, "uavcan", "start")) {
            hal.console->printf("UAVCAN: started\n");            
            // give some time for CAN bus initialisation
            hal.scheduler->delay(2000);
        } else {
            hal.console->printf("UAVCAN: failed to start\n");
        }
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
#endif // CONFIG_ARCH_BOARD_PX4FMU_V1
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
        px4.board_type.set_and_notify(PX4_BOARD_PIXHAWK2);
    } else {
        px4.board_type.set_and_notify(PX4_BOARD_PIXHAWK);
    }
    
    printf("FMUv2 sensors started\n");
#endif // CONFIG_ARCH_BOARD_PX4FMU_V2
}


/*
  setup sensors for Pixhawk2-slim
 */
void AP_BoardConfig::px4_start_pixhawk2slim_sensors(void)
{
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
    printf("Starting PH2SLIM sensors\n");
    if (px4_start_driver(hmc5883_main, "hmc5883", "-C -T -I -R 4 start")) {
        printf("Have internal hmc5883\n");
    } else {
        printf("No internal hmc5883\n");
    }

    // external MPU6000 is rotated YAW_180 from standard
    if (px4_start_driver(mpu6000_main, "mpu6000", "-X -R 4 start")) {
        printf("Found MPU6000 external\n");
    } else {
        if (px4_start_driver(mpu9250_main, "mpu9250", "-X -R 4 start")) {
            printf("Found MPU9250 external\n");
        } else {
            printf("No MPU6000 or MPU9250 external\n");
        }
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

    // on Pixhawk2 default IMU temperature to 60
    _imu_target_temperature.set_default(60);
    
    printf("PH2SLIM sensors started\n");
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
    if (px4_start_driver(l3gd20_main, "l3gd20", "start")) {
        printf("l3gd20 started OK\n");
    } else {
        printf("No l3gd20\n");
    }
    px4.board_type.set_and_notify(PX4_BOARD_PX4V1);
#endif // CONFIG_ARCH_BOARD_PX4FMU_V1
}

/*
  setup sensors for FMUv4
 */
void AP_BoardConfig::px4_start_fmuv4_sensors(void)
{
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
    printf("Starting FMUv4 sensors\n");
    if (px4_start_driver(hmc5883_main, "hmc5883", "-C -T -S -R 2 start")) {
        printf("Have SPI hmc5883\n");
    } else {
        printf("No SPI hmc5883\n");
    }

    if (px4_start_driver(mpu6000_main, "mpu6000", "-R 2 -T 20608 start")) {
        printf("Found ICM-20608 internal\n");
    }

    if (px4_start_driver(mpu9250_main, "mpu9250", "-R 2 start")) {
        printf("Found mpu9250 internal\n");
    }
    px4.board_type.set_and_notify(PX4_BOARD_PIXRACER);
#endif // CONFIG_ARCH_BOARD_PX4FMU_V4
}

/*
  setup common sensors
 */
void AP_BoardConfig::px4_start_common_sensors(void)
{
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
}


/*
  setup optional sensors
 */
void AP_BoardConfig::px4_start_optional_sensors(void)
{
    if (px4_start_driver(ets_airspeed_main, "ets_airspeed", "start")) {
        printf("Found ETS airspeed sensor\n");
    }

    if (px4_start_driver(meas_airspeed_main, "meas_airspeed", "start")) {
        printf("Found MEAS airspeed sensor\n");
    } else if (px4_start_driver(meas_airspeed_main, "meas_airspeed", "start -b 2")) {
        printf("Found MEAS airspeed sensor (bus2)\n");
    }

    if (px4_start_driver(ll40ls_main, "ll40ls", "-X start")) {
        printf("Found external ll40ls sensor\n");
    }
    if (px4_start_driver(ll40ls_main, "ll40ls", "-I start")) {
        printf("Found internal ll40ls sensor\n");
    }
    if (px4_start_driver(trone_main, "trone", "start")) {
        printf("Found trone sensor\n");
    }
    if (px4_start_driver(mb12xx_main, "mb12xx", "start")) {
        printf("Found mb12xx sensor\n");
    }

    if (px4_start_driver(pwm_input_main, "pwm_input", "start")) {
        printf("started pwm_input driver\n");
    }
}


void AP_BoardConfig::px4_setup_drivers(void)
{
    px4_start_common_sensors();
    switch ((px4_board_type)px4.board_type.get()) {
    case PX4_BOARD_PH2SLIM:
        px4_start_pixhawk2slim_sensors();
        break;
    case PX4_BOARD_AUTO:
    default:
        px4_start_fmuv1_sensors();
        px4_start_fmuv2_sensors();
        px4_start_fmuv4_sensors();
        break;
    }
    px4_start_optional_sensors();
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
        hal.console->printf("Sensor failure: %s\n", reason);
        hal.scheduler->delay(3000);
    }
}

/*
  setup px4 peripherals and drivers
 */
void AP_BoardConfig::px4_setup()
{
    px4_setup_pwm();
    px4_setup_safety();
    px4_setup_uart();
    px4_setup_sbus();
    px4_setup_canbus();
    px4_setup_drivers();

    // delay for 1 second to give time for drivers to initialise
    hal.scheduler->delay(1000);
}

#endif // HAL_BOARD_PX4

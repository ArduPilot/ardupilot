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

#if HAL_WITH_UAVCAN
#include <AP_HAL_PX4/CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#endif

extern const AP_HAL::HAL& hal;

AP_BoardConfig::px4_board_type AP_BoardConfig::px4_configured_board;

/* 
   declare driver main entry points
 */
extern "C" {
#if HAL_WITH_UAVCAN
    int uavcan_main(int, char **);
#endif
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
#if HAL_WITH_UAVCAN
    if (_var_info_can._can_enable >= 1) {
        if(hal.can_mgr == nullptr)
        {
            const_cast <AP_HAL::HAL&> (hal).can_mgr = new PX4::PX4CANManager;
        }

        if(hal.can_mgr != nullptr)
        {
            if(_var_info_can._uavcan_enable > 0)
            {
                _var_info_can._uavcan = new AP_UAVCAN;
                if(_var_info_can._uavcan != nullptr)
                {
                    AP_Param::load_object_from_eeprom(_var_info_can._uavcan, AP_UAVCAN::var_info);

                    hal.can_mgr->set_UAVCAN(_var_info_can._uavcan);

                    bool initret = hal.can_mgr->begin(_var_info_can._can_bitrate, _var_info_can._can_enable);
                    if (!initret) {
                        hal.console->printf("Failed to initialize can_mgr\n\r");
                    } else {
                        hal.console->printf("can_mgr initialized well\n\r");

                        // start UAVCAN working thread
                        hal.scheduler->create_uavcan_thread();
                    }
                } else
                {
                    _var_info_can._uavcan_enable.set(0);
                    hal.console->printf("AP_UAVCAN failed to allocate\n\r");
                }
            }
        }
    }
#endif
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

    if (px4.board_type == PX4_BOARD_OLDDRIVERS) {
        printf("Old drivers no longer supported\n");
        px4.board_type = PX4_BOARD_AUTO;
    }

    // run board auto-detection
    px4_autodetect();

    if (px4.board_type == PX4_BOARD_PH2SLIM ||
        px4.board_type == PX4_BOARD_PIXHAWK2) {
        _imu_target_temperature.set_default(60);
    }

    px4_configured_board = (enum px4_board_type)px4.board_type.get();

    switch (px4_configured_board) {
    case PX4_BOARD_PX4V1:
    case PX4_BOARD_PIXHAWK:
    case PX4_BOARD_PIXHAWK2:
    case PX4_BOARD_PIXRACER:
    case PX4_BOARD_PHMINI:
    case PX4_BOARD_AUAV21:
    case PX4_BOARD_PH2SLIM:
    case PX4_BOARD_AEROFC:
        break;
    default:
        px4_sensor_error("Unknown board type");
        break;
    }
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

#if HAL_PX4_HAVE_PX4IO
    if (px4.io_enable.get() != 0) {
        px4_setup_px4io();
    }
#endif

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
    const char *fmu_mode = "mode_serial";
#elif defined(CONFIG_ARCH_BOARD_AEROFC_V1)
    const char *fmu_mode = "mode_rcin";
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
#define MPU_WHOAMI_ICM20602 0x12

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
         spi_check_register(HAL_INS_ICM20608_EXT_NAME, MPUREG_WHOAMI, MPU_WHOAMI_ICM20608) ||
         spi_check_register(HAL_INS_ICM20608_EXT_NAME, MPUREG_WHOAMI, MPU_WHOAMI_ICM20602)) &&
        spi_check_register(HAL_INS_LSM9DS0_EXT_A_NAME, LSMREG_WHOAMI, LSM_WHOAMI_LSM303D)) {
        // Pixhawk2 has LSM303D and MPUxxxx on external bus
        px4.board_type.set(PX4_BOARD_PIXHAWK2);
        hal.console->printf("Detected PIXHAWK2\n");
    } else if ((spi_check_register(HAL_INS_ICM20608_AM_NAME, MPUREG_WHOAMI, MPU_WHOAMI_ICM20608) ||
                spi_check_register(HAL_INS_ICM20608_AM_NAME, MPUREG_WHOAMI, MPU_WHOAMI_ICM20602)) &&
               spi_check_register(HAL_INS_MPU9250_NAME, MPUREG_WHOAMI, MPU_WHOAMI_MPU9250)) {
        // PHMINI has an ICM20608 and MPU9250 on sensor bus
        px4.board_type.set(PX4_BOARD_PHMINI);
        hal.console->printf("Detected PixhawkMini\n");
    } else if (spi_check_register(HAL_INS_LSM9DS0_A_NAME, LSMREG_WHOAMI, LSM_WHOAMI_LSM303D) &&
               (spi_check_register(HAL_INS_MPU60x0_NAME, MPUREG_WHOAMI, MPU_WHOAMI_MPU60X0) ||
                spi_check_register(HAL_INS_ICM20608_NAME, MPUREG_WHOAMI, MPU_WHOAMI_ICM20608) ||
                spi_check_register(HAL_INS_ICM20608_NAME, MPUREG_WHOAMI, MPU_WHOAMI_ICM20602) ||
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
#elif defined(CONFIG_ARCH_BOARD_AEROFC_V1)
    px4.board_type.set_and_notify(PX4_BOARD_AEROFC);
    hal.console->printf("Detected Aero FC\n");
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

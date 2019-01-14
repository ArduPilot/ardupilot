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
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
  init safety state
 */
void AP_BoardConfig::board_init_safety()
{
#if HAL_HAVE_SAFETY_SWITCH
    if (state.safety_enable.get() == 0) {
        hal.rcout->force_safety_off();
        hal.rcout->force_safety_no_wait();
        // wait until safety has been turned off
        uint8_t count = 20;
        while (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_ARMED && count--) {
            hal.scheduler->delay(20);
        }
    }
#endif
}


#if AP_FEATURE_BOARD_DETECT

AP_BoardConfig::px4_board_type AP_BoardConfig::px4_configured_board;

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
extern "C" {
    int fmu_main(int, char **);
};
#endif

void AP_BoardConfig::board_setup_drivers(void)
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

    if (state.board_type == PX4_BOARD_OLDDRIVERS) {
        printf("Old drivers no longer supported\n");
        state.board_type = PX4_BOARD_AUTO;
    }

    // run board auto-detection
    board_autodetect();

    if (state.board_type == PX4_BOARD_PH2SLIM ||
        state.board_type == PX4_BOARD_PIXHAWK2) {
        _imu_target_temperature.set_default(45);
        if (_imu_target_temperature.get() < 0) {
            // don't allow a value of -1 on the cube, or it could cook
            // the IMU
            _imu_target_temperature.set(45);
        }
    }

    px4_configured_board = (enum px4_board_type)state.board_type.get();

    switch (px4_configured_board) {
    case PX4_BOARD_PX4V1:
    case PX4_BOARD_PIXHAWK:
    case PX4_BOARD_PIXHAWK2:
    case PX4_BOARD_FMUV5:
    case PX4_BOARD_SP01:
    case PX4_BOARD_PIXRACER:
    case PX4_BOARD_PHMINI:
    case PX4_BOARD_AUAV21:
    case PX4_BOARD_PH2SLIM:
    case VRX_BOARD_BRAIN51:
    case VRX_BOARD_BRAIN52:
    case VRX_BOARD_BRAIN52E:
    case VRX_BOARD_UBRAIN51:
    case VRX_BOARD_UBRAIN52:
    case VRX_BOARD_CORE10:
    case VRX_BOARD_BRAIN54:
    case PX4_BOARD_AEROFC:
    case PX4_BOARD_PIXHAWK_PRO:
    case PX4_BOARD_PCNC1:
    case PX4_BOARD_MINDPXV2:
        break;
    default:
        sensor_config_error("Unknown board type");
        break;
    }
}

#define SPI_PROBE_DEBUG 0

/*
  check a SPI device for a register value
 */
bool AP_BoardConfig::spi_check_register(const char *devname, uint8_t regnum, uint8_t value, uint8_t read_flag)
{
    auto dev = hal.spi->get_device(devname);
    if (!dev) {
#if SPI_PROBE_DEBUG
        hal.console->printf("%s: no device\n", devname);
#endif
        return false;
    }
    dev->set_read_flag(read_flag);
    if (!dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    uint8_t v;
    if (!dev->read_registers(regnum, &v, 1)) {
#if SPI_PROBE_DEBUG
        hal.console->printf("%s: reg %02x read fail\n", devname, (unsigned)regnum);
#endif
        dev->get_semaphore()->give();
        return false;
    }
    dev->get_semaphore()->give();
#if SPI_PROBE_DEBUG
    hal.console->printf("%s: reg %02x expected:%02x got:%02x\n", devname, (unsigned)regnum, (unsigned)value, (unsigned)v);
#endif
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
  validation of the board type
 */
void AP_BoardConfig::validate_board_type(void)
{
    /* some boards can be damaged by the user setting the wrong board
       type.  The key one is the cube which has a heater which can
       cook the IMUs if the user uses an old paramater file. We
       override the board type for that specific case
     */
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V2) || defined(HAL_CHIBIOS_ARCH_FMUV3)
    if (state.board_type == PX4_BOARD_PIXHAWK &&
        (spi_check_register("mpu6000_ext", MPUREG_WHOAMI, MPU_WHOAMI_MPU60X0) ||
         spi_check_register("mpu9250_ext", MPUREG_WHOAMI, MPU_WHOAMI_MPU9250) ||
         spi_check_register("icm20608", MPUREG_WHOAMI, MPU_WHOAMI_ICM20608) ||
         spi_check_register("icm20608_ext", MPUREG_WHOAMI, MPU_WHOAMI_ICM20602)) &&
        spi_check_register("lsm9ds0_ext_am", LSMREG_WHOAMI, LSM_WHOAMI_LSM303D)) {
        // Pixhawk2 has LSM303D and MPUxxxx on external bus. If we
        // detect those, then force PIXHAWK2, even if the user has
        // configured for PIXHAWK1
#if !defined(CONFIG_ARCH_BOARD_PX4FMU_V3) && !defined(HAL_CHIBIOS_ARCH_FMUV3)
        // force user to load the right firmware
        sensor_config_error("Pixhawk2 requires FMUv3 firmware");        
#endif
        state.board_type.set(PX4_BOARD_PIXHAWK2);
        hal.console->printf("Forced PIXHAWK2\n");
    }
#endif

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V4PRO)
	// Nothing to do for the moment
#endif
}

/*
  auto-detect board type
 */
void AP_BoardConfig::board_autodetect(void)
{
    if (state.board_type != PX4_BOARD_AUTO) {
        validate_board_type();
        // user has chosen a board type
        return;
    }

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
    // only one choice
    state.board_type.set(PX4_BOARD_PX4V1);
    hal.console->printf("Detected PX4v1\n");

#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V2) || defined(HAL_CHIBIOS_ARCH_FMUV3)
    if ((spi_check_register("mpu6000_ext", MPUREG_WHOAMI, MPU_WHOAMI_MPU60X0) ||
         spi_check_register("mpu6000_ext", MPUREG_WHOAMI, MPU_WHOAMI_MPU60X0) ||
         spi_check_register("mpu9250_ext", MPUREG_WHOAMI, MPU_WHOAMI_MPU60X0) ||
         spi_check_register("mpu9250_ext", MPUREG_WHOAMI, MPU_WHOAMI_MPU9250) ||
         spi_check_register("icm20608_ext", MPUREG_WHOAMI, MPU_WHOAMI_ICM20608) ||
         spi_check_register("icm20608_ext", MPUREG_WHOAMI, MPU_WHOAMI_ICM20602)) &&
        spi_check_register("lsm9ds0_ext_am", LSMREG_WHOAMI, LSM_WHOAMI_LSM303D)) {
        // Pixhawk2 has LSM303D and MPUxxxx on external bus
        state.board_type.set(PX4_BOARD_PIXHAWK2);
        hal.console->printf("Detected PIXHAWK2\n");
    } else if ((spi_check_register("icm20608-am", MPUREG_WHOAMI, MPU_WHOAMI_ICM20608) ||
                spi_check_register("icm20608-am", MPUREG_WHOAMI, MPU_WHOAMI_ICM20602)) &&
               spi_check_register("mpu9250", MPUREG_WHOAMI, MPU_WHOAMI_MPU9250)) {
        // PHMINI has an ICM20608 and MPU9250 on sensor bus
        state.board_type.set(PX4_BOARD_PHMINI);
        hal.console->printf("Detected PixhawkMini\n");
    } else if (spi_check_register("lsm9ds0_am", LSMREG_WHOAMI, LSM_WHOAMI_LSM303D) &&
               (spi_check_register("mpu6000", MPUREG_WHOAMI, MPU_WHOAMI_MPU60X0) ||
                spi_check_register("icm20608", MPUREG_WHOAMI, MPU_WHOAMI_ICM20608) ||
                spi_check_register("icm20608", MPUREG_WHOAMI, MPU_WHOAMI_ICM20602) ||
                spi_check_register("mpu9250", MPUREG_WHOAMI, MPU_WHOAMI_MPU9250))) {

        // classic or upgraded Pixhawk1
        state.board_type.set(PX4_BOARD_PIXHAWK);
        hal.console->printf("Detected Pixhawk\n");
    } else {
        sensor_config_error("Unable to detect board type");
    }
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V4) || defined(HAL_CHIBIOS_ARCH_FMUV4)
    // only one choice
    state.board_type.set_and_notify(PX4_BOARD_PIXRACER);
    hal.console->printf("Detected Pixracer\n");
#elif defined(HAL_CHIBIOS_ARCH_MINDPXV2)
    // only one choice
    state.board_type.set_and_notify(PX4_BOARD_MINDPXV2);
    hal.console->printf("Detected MindPX-V2\n");
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V4PRO) || defined(HAL_CHIBIOS_ARCH_FMUV4PRO)
    // only one choice
    state.board_type.set_and_notify(PX4_BOARD_PIXHAWK_PRO);
    hal.console->printf("Detected Pixhawk Pro\n");	
#elif defined(CONFIG_ARCH_BOARD_AEROFC_V1)
    state.board_type.set_and_notify(PX4_BOARD_AEROFC);
    hal.console->printf("Detected Aero FC\n");
#elif defined(HAL_CHIBIOS_ARCH_FMUV5)
    state.board_type.set_and_notify(PX4_BOARD_FMUV5);
    hal.console->printf("Detected FMUv5\n");
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
    state.board_type.set_and_notify(VRX_BOARD_BRAIN51);
    hal.console->printf("Detected VR Brain 5.1\n");
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V52) || defined(HAL_CHIBIOS_ARCH_BRAINV52)
    state.board_type.set_and_notify(VRX_BOARD_BRAIN52);
    hal.console->printf("Detected VR Brain 5.2\n");
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V52E)
    state.board_type.set_and_notify(VRX_BOARD_BRAIN52E);
    hal.console->printf("Detected VR Brain 5.2E\n");
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51) || defined(HAL_CHIBIOS_ARCH_UBRAINV51)
    state.board_type.set_and_notify(VRX_BOARD_UBRAIN51);
    hal.console->printf("Detected VR Micro Brain 5.1\n");
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)
    state.board_type.set_and_notify(VRX_BOARD_UBRAIN52);
    hal.console->printf("Detected VR Micro Brain 5.2\n");
#elif defined(CONFIG_ARCH_BOARD_VRCORE_V10)
    state.board_type.set_and_notify(VRX_BOARD_CORE10);
    hal.console->printf("Detected VR Core 1.0\n");
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V54)
    state.board_type.set_and_notify(VRX_BOARD_BRAIN54);
    hal.console->printf("Detected VR Brain 5.4\n");
#endif

}

#endif // AP_FEATURE_BOARD_DETECT

/*
  setup flow control on UARTs
 */
void AP_BoardConfig::board_setup_uart()
{
#if AP_FEATURE_RTSCTS
    hal.uartC->set_flow_control((AP_HAL::UARTDriver::flow_control)state.ser1_rtscts.get());
    if (hal.uartD != nullptr) {
        hal.uartD->set_flow_control((AP_HAL::UARTDriver::flow_control)state.ser2_rtscts.get());
    }
#endif
}

/*
  setup SBUS
 */
void AP_BoardConfig::board_setup_sbus(void)
{
#if AP_FEATURE_SBUS_OUT
    if (state.sbus_out_rate.get() >= 1) {
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
            if (rates[i].value == state.sbus_out_rate) {
                rate = rates[i].rate;
            }
        }
        if (!hal.rcout->enable_px4io_sbus_out(rate)) {
            hal.console->printf("Failed to enable SBUS out\n");
        }
    }
#endif
}


/*
  setup peripherals and drivers
 */
void AP_BoardConfig::board_setup()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    px4_setup_peripherals();
    px4_setup_pwm();
    px4_setup_safety_mask();
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // init needs to be done after boardconfig is read so parameters are set
    hal.gpio->init();
    hal.rcin->init();
    hal.rcout->init();
#endif
    board_setup_uart();
    board_setup_sbus();
#if AP_FEATURE_BOARD_DETECT
    board_setup_drivers();
#endif
}


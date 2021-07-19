#pragma once

#define HAL_BOARD_NAME "Linux"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000
#define HAL_OS_POSIX_IO 1
#define HAL_OS_SOCKETS 1
#define HAL_STORAGE_SIZE            16384
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_DSHOT_ALARM 0

// make sensor selection clearer
#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_IMU_I2C2(driver, bus, addr1, addr2, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.i2c_mgr->get_device(bus, addr1),hal.i2c_mgr->get_device(bus, addr2),##args))
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define PROBE_IMU_SPI2(driver, devname1, devname2, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname1),hal.spi->get_device(devname2),##args))

#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_BARO_SPI(driver, devname, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(hal.spi->get_device(devname)),##args))

#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))
#define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NONE
    #define HAL_BOARD_LOG_DIRECTORY "logs"
    #define HAL_BOARD_TERRAIN_DIRECTORY "terrain"
    #define HAL_BOARD_STORAGE_DIRECTORY "."
    #define HAL_INS_DEFAULT HAL_INS_NONE
    #define HAL_BARO_DEFAULT HAL_BARO_NONE
    #define HAL_COMPASS_DEFAULT HAL_COMPASS_NONE
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD
    #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
      #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_ROLL_180_YAW_270)
    #else
      #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_ROLL_180_YAW_90)
    #endif
    #define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(MS56XX, "ms5611")
    #define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
    #define HAL_GPIO_A_LED_PIN        61
    #define HAL_GPIO_B_LED_PIN        48
    #define HAL_GPIO_C_LED_PIN        117
    #define HAL_GPIO_LED_ON           0
    #define HAL_GPIO_LED_OFF          1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
    #define HAL_BOARD_LOG_DIRECTORY "/data/ftp/internal_000/ardupilot/logs"
    #define HAL_BOARD_TERRAIN_DIRECTORY "/data/ftp/internal_000/ardupilot/terrain"
    #define HAL_BOARD_STORAGE_DIRECTORY "/data/ftp/internal_000/ardupilot"
    #define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensense, 2, 0x68, ROTATION_YAW_270)
    #define HAL_MAG_PROBE_LIST PROBE_MAG_I2C(AK8963, 1, 0x0d, ROTATION_NONE)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(MS56XX, 1, 0x77, AP_Baro_MS56XX::BARO_MS5607)
    #define HAL_HAVE_IMU_HEATER 1
    #define HAL_IMU_TEMP_DEFAULT 55
    #define HAL_UTILS_HEAT HAL_LINUX_HEAT_PWM
    #define HAL_LINUX_HEAT_PWM_NUM  6
    #define HAL_LINUX_HEAT_KP 20000
    #define HAL_LINUX_HEAT_KI 6
    #define HAL_LINUX_HEAT_PERIOD_NS 125000
    #define HAL_LINUX_HEAT_TARGET_TEMP 50
    #define BEBOP_CAMV_PWM  9
    #define BEBOP_CAMV_PWM_FREQ 43333333
    #define HAL_OPTFLOW_ONBOARD_VDEV_PATH "/dev/video0"
    #define HAL_OPTFLOW_ONBOARD_SUBDEV_PATH "/dev/v4l-subdev0"
    #define HAL_OPTFLOW_ONBOARD_SENSOR_WIDTH 320
    #define HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT 240
    #define HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH 64
    #define HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT 64
    #define HAL_OPTFLOW_ONBOARD_CROP_WIDTH 240
    #define HAL_OPTFLOW_ONBOARD_CROP_HEIGHT 240
    #define HAL_OPTFLOW_ONBOARD_NBUFS 8
    #define HAL_FLOW_PX4_MAX_FLOW_PIXEL 4
    #define HAL_FLOW_PX4_BOTTOM_FLOW_FEATURE_THRESHOLD 30
    #define HAL_FLOW_PX4_BOTTOM_FLOW_VALUE_THRESHOLD 5000
    #define HAL_PARAM_DEFAULTS_PATH "/data/ftp/internal_000/ardupilot/bebop.parm"
    #define HAL_RCOUT_BEBOP_BLDC_I2C_BUS 1
    #define HAL_RCOUT_BEBOP_BLDC_I2C_ADDR 0x08
    /* focal length 2.21mm pixel size 3.6 um, 2x binning in each direction
    * 240x240 crop rescaled to 64x64 */
    #define HAL_FLOW_PX4_FOCAL_LENGTH_MILLIPX (2.21 / (3.6 * 2.0 * 240 / 64))
    #define HAL_RANGEFINDER_LIGHTWARE_I2C_BUS 0
    #define HAL_BATT_MONITOR_DEFAULT AP_BattMonitor::Type::BEBOP
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_VNAV
    // linux SBC with VectorNav AHRS
    #define HAL_EXTERNAL_AHRS_DEFAULT 1
    #define HAL_SERIAL3_PROTOCOL 36
    #define HAL_COMPASS_DEFAULT HAL_COMPASS_NONE
    #define HAL_AIRSPEED_TYPE_DEFAULT 0
    #define HAL_GPS_TYPE_DEFAULT 21
    #define HAL_AHRS_EKF_TYPE_DEFAULT 11
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
    #define HAL_BOARD_LOG_DIRECTORY "/data/ftp/internal_000/ardupilot/logs"
    #define HAL_BOARD_TERRAIN_DIRECTORY "/data/ftp/internal_000/ardupilot/terrain"
    #define HAL_BOARD_STORAGE_DIRECTORY "/data/ftp/internal_000/ardupilot"
    #define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensense, 2, 0x68, ROTATION_PITCH_180_YAW_90)
    #define HAL_MAG_PROBE_LIST PROBE_MAG_I2C(AK8963, 1, 0x0d, ROTATION_NONE)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(MS56XX, 1, 0x77, AP_Baro_MS56XX::BARO_MS5607)
    #define HAL_UTILS_HEAT HAL_LINUX_HEAT_PWM
    #define HAL_LINUX_HEAT_PWM_NUM  10
    #define HAL_LINUX_HEAT_KP 20000
    #define HAL_LINUX_HEAT_KI 6
    #define HAL_LINUX_HEAT_PERIOD_NS 125000
    #define HAL_LINUX_HEAT_TARGET_TEMP 50
    #define BEBOP_CAMV_PWM  9
    #define BEBOP_CAMV_PWM_FREQ 43333333
    #define HAL_OPTFLOW_ONBOARD_VDEV_PATH "/dev/video0"
    #define HAL_OPTFLOW_ONBOARD_SUBDEV_PATH "/dev/v4l-subdev0"
    #define HAL_OPTFLOW_ONBOARD_SENSOR_WIDTH 320
    #define HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT 240
    #define HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH 64
    #define HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT 64
    #define HAL_OPTFLOW_ONBOARD_CROP_WIDTH 240
    #define HAL_OPTFLOW_ONBOARD_CROP_HEIGHT 240
    #define HAL_OPTFLOW_ONBOARD_NBUFS 8
    #define HAL_FLOW_PX4_MAX_FLOW_PIXEL 4
    #define HAL_FLOW_PX4_BOTTOM_FLOW_FEATURE_THRESHOLD 30
    #define HAL_FLOW_PX4_BOTTOM_FLOW_VALUE_THRESHOLD 5000
    #define HAL_RCOUT_DISCO_BLDC_I2C_BUS 1
    #define HAL_RCOUT_DISCO_BLDC_I2C_ADDR 0x08
    #define HAL_PARAM_DEFAULTS_PATH "/data/ftp/internal_000/ardupilot/disco.parm"
    /* focal length 2.21mm pixel size 3.6 um, 2x binning in each direction
    * 240x240 crop rescaled to 64x64 */
    #define HAL_FLOW_PX4_FOCAL_LENGTH_MILLIPX (2.21 / (3.6 * 2.0 * 240 / 64))
    #define HAL_RANGEFINDER_LIGHTWARE_I2C_BUS 0
    // the disco has challenges with its magnetic setup
    #define AP_COMPASS_OFFSETS_MAX_DEFAULT 2200
    #define HAL_BATT_MONITOR_DEFAULT AP_BattMonitor::Type::BEBOP
    #define HAL_GPIO_SCRIPT "/data/ftp/internal_000/ardupilot/gpio.sh"
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
    #define HAL_GPIO_A_LED_PIN 0
    #define HAL_GPIO_B_LED_PIN 1
    #define HAL_GPIO_C_LED_PIN 2
    #define HAL_GPIO_LED_ON 1
    #define HAL_GPIO_LED_OFF 0
    #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_NONE)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(MS56XX, 1, 0x77)
    #define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2
    #define HAL_HAVE_SERVO_VOLTAGE 1
    #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_NONE)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(MS56XX, 1, 0x77)
    #define HAL_MAG_PROBE1 PROBE_MAG_SPI(LSM9DS1, "lsm9ds1_m",  ROTATION_ROLL_180)
    #define HAL_MAG_PROBE2 PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
    #define HAL_MAG_PROBE_LIST HAL_MAG_PROBE1; HAL_MAG_PROBE2
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2
    #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_YAW_270)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(MS56XX, "ms5611")
    #define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
    #define HAL_GPIO_A_LED_PIN        24
    #define HAL_GPIO_B_LED_PIN        25
    #define HAL_GPIO_C_LED_PIN        16
    #define HAL_GPIO_LED_ON           0
    #define HAL_GPIO_LED_OFF          1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ZYNQ
    // Stub the sensors out for now, at least we can build and run
    #define HAL_INS_DEFAULT HAL_INS_NONE
    #define HAL_BARO_DEFAULT HAL_BARO_NONE
    #define HAL_COMPASS_DEFAULT HAL_COMPASS_NONE
    // only external compasses
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
    #define HAL_COMPASS_DEFAULT HAL_COMPASS_NONE
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_OCPOC_ZYNQ
    #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_NONE)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(MS56XX, "ms5611")
    #define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    #define HAL_GPIO_A_LED_PIN 69
    #define HAL_GPIO_B_LED_PIN 68
    #define HAL_GPIO_C_LED_PIN 45
    #define HAL_GPIO_LED_ON 0
    #define HAL_GPIO_LED_OFF 1
    #define HAL_BUZZER_PIN 11
    #define HAL_INS_PROBE1 PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_NONE)
    #define HAL_INS_PROBE2 PROBE_IMU_SPI(Invensense, "mpu9250ext", ROTATION_NONE)
    #define HAL_INS_PROBE_LIST HAL_INS_PROBE1; HAL_INS_PROBE2
    #define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(MS56XX, "ms5611")
    #define HAL_MAG_PROBE1 PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
    #define HAL_MAG_PROBE2 PROBE_MAG_IMU(AK8963, mpu9250, 1, ROTATION_NONE)
    #define HAL_MAG_PROBE_LIST HAL_MAG_PROBE1; HAL_MAG_PROBE2
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
    #define HAL_OPTFLOW_PX4FLOW_I2C_BUS 2
    #define HAL_RANGEFINDER_LIGHTWARE_I2C_BUS 2
    #define HAL_NUM_CAN_IFACES 1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR
    #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "icm20602", ROTATION_ROLL_180_YAW_270)
    #define HAL_MAG_PROBE_LIST PROBE_MAG_I2C(MMC5XX3, 1, 0x30, false, ROTATION_YAW_90)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(BMP280, 4, 0x76)
    #define HAL_BATT_CURR_PIN    4
    #define HAL_BATT_CURR_SCALE  1
    #define HAL_BATT_VOLT_PIN    5
    #define HAL_BATT_VOLT_SCALE  1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
    #define HAL_GPIO_A_LED_PIN 66
    #define HAL_GPIO_B_LED_PIN 67
    #define HAL_GPIO_LED_ON    1
    #define HAL_GPIO_LED_OFF   0
    #define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensense, 2, 0x68, ROTATION_NONE)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(BMP280, 2, 0x76)
    #define HAL_MAG_PROBE_LIST PROBE_MAG_IMU_I2C(AK8963, mpu9250, 2, 0x0c, ROTATION_NONE)
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
    #define HAL_OPTFLOW_PX4FLOW_I2C_BUS 1
    #define HAL_RANGEFINDER_LIGHTWARE_I2C_BUS 1
    #define HAL_NUM_CAN_IFACES 1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
    #define HAL_GPIO_A_LED_PIN 59
    #define HAL_GPIO_B_LED_PIN 58
    #define HAL_GPIO_C_LED_PIN 57
    #define HAL_GPIO_LED_ON    1
    #define HAL_GPIO_LED_OFF   0
    #define HAL_BUZZER_PIN 28
    #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_NONE)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(BMP280, "bmp280")
    #define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
    #define HAL_OPTFLOW_PX4FLOW_I2C_BUS 2
    #define HAL_RANGEFINDER_LIGHTWARE_I2C_BUS 2
    #define HAL_NUM_CAN_IFACES 1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
    #define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(MS56XX, 1, 0x77)
    #define HAL_INS_PROBE1 PROBE_IMU_I2C(Invensense, 1, 0x69, ROTATION_NONE)
    #define HAL_INS_PROBE2 PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_NONE)
    #define HAL_INS_PROBE_LIST HAL_INS_PROBE1; HAL_INS_PROBE2
    #define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
    #define HAL_GPIO_A_LED_PIN        17
    #define HAL_GPIO_B_LED_PIN        18
    #define HAL_GPIO_C_LED_PIN        22
    #define HAL_GPIO_LED_ON           0
    #define HAL_GPIO_LED_OFF          1
    #define HAL_RCOUT_RGBLED_RED      13
    #define HAL_RCOUT_RGBLED_GREEN    14
    #define HAL_RCOUT_RGBLED_BLUE     15
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI
    #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_YAW_270)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(MS56XX, "ms5611")
    #define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
    #define HAL_GPIO_A_LED_PIN        24
    #define HAL_GPIO_B_LED_PIN        25
    #define HAL_GPIO_C_LED_PIN        16
    #define HAL_GPIO_LED_ON           0
    #define HAL_GPIO_LED_OFF          1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_AERO
    #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(BMI160, "bmi160")
    #define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(MS56XX, 2, 0x76)
    #define HAL_MAG_PROBE1 PROBE_MAG_I2C(BMM150, 2, 0x12, ROTATION_NONE)
    #define HAL_MAG_PROBE2 PROBE_MAG_I2C(HMC5843, 4, 0x1e, true, ROTATION_NONE)
    #define HAL_MAG_PROBE3 PROBE_MAG_I2C(IST8310, 4, 0x0e, true, ROTATION_PITCH_180_YAW_90)
    #define HAL_MAG_PROBE_LIST HAL_MAG_PROBE1; HAL_MAG_PROBE2; HAL_MAG_PROBE3
    #define HAL_RCOUTPUT_TAP_DEVICE "/dev/ttyS1"
    #define HAL_NUM_CAN_IFACES 1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK
    #define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_NONE)
    #define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(MS56XX, 1, 0x77)
    #define HAL_GPIO_A_LED_PIN        24
    #define HAL_GPIO_B_LED_PIN        25
    #define HAL_GPIO_C_LED_PIN        16
    #define HAL_GPIO_LED_ON           0
    #define HAL_GPIO_LED_OFF          1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE
    #define HAL_BOARD_LOG_DIRECTORY "/edge/ardupilot/logs"
    #define HAL_BOARD_TERRAIN_DIRECTORY "/edge/ardupilot/terrain"
    #define HAL_BOARD_STORAGE_DIRECTORY "/edge/ardupilot"
    #define HAL_INS_PROBE1 PROBE_IMU_SPI(Invensense, "mpu60x0", ROTATION_YAW_90)
    #define HAL_INS_PROBE2 PROBE_IMU_SPI(Invensense, "mpu60x0ext", ROTATION_YAW_90)
    #define HAL_INS_PROBE_LIST HAL_INS_PROBE1; HAL_INS_PROBE2
    #define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(MS56XX, "ms5611")
    // only external compasses
    #define HAL_PROBE_EXTERNAL_I2C_COMPASSES
    #define HAL_COMPASS_DEFAULT HAL_COMPASS_NONE
    #define HAL_NUM_CAN_IFACES 1
    #define HAL_IMU_TEMP_DEFAULT 55
    #define HAL_HAVE_IMU_HEATER 1
    #define HAL_UTILS_HEAT HAL_LINUX_HEAT_PWM
    #define HAL_LINUX_HEAT_PWM_NUM  15
    #define HAL_LINUX_HEAT_KP 20000
    #define HAL_LINUX_HEAT_KI 6
    #define HAL_LINUX_HEAT_PERIOD_NS 2040816
    #define HAL_GPS_TYPE_DEFAULT 9
    #define HAL_CAN_DRIVER_DEFAULT 1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ
    #define HAL_INS_PROBE_LIST PROBE_IMU_SPI2(Invensense, "rst_g", "rst_a", ROTATION_ROLL_180_YAW_90, ROTATION_ROLL_180_YAW_90)
    #define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(MS56XX, "ms5611")
    #define HAL_MAG_PROBE_LIST PROBE_MAG_SPI(LIS3MDL, lis3mdl, false, ROTATION_ROLL_180_YAW_90)
    #define HAL_OPTFLOW_PX4FLOW_I2C_BUS 0

    #define HAL_HAVE_GETTIME_SETTIME 1

#else
    #error "no Linux board subtype set"
#endif

#ifndef HAL_COMPASS_DEFAULT
    #define HAL_COMPASS_DEFAULT -1
#endif

#ifndef HAL_OPTFLOW_PX4FLOW_I2C_ADDRESS
    #define HAL_OPTFLOW_PX4FLOW_I2C_ADDRESS 0x42
#endif

#ifndef HAL_OPTFLOW_PX4FLOW_I2C_BUS
    #define HAL_OPTFLOW_PX4FLOW_I2C_BUS 1
#endif

#define HAL_HAVE_BOARD_VOLTAGE 1
#define HAL_HAVE_SAFETY_SWITCH 0


#ifndef HAL_HAVE_SERVO_VOLTAGE
    #define HAL_HAVE_SERVO_VOLTAGE 0
#endif

#ifndef AP_STATEDIR
    #define HAL_BOARD_STATE_DIRECTORY "/var/lib/ardupilot"
#else
    #define HAL_BOARD_STATE_DIRECTORY AP_STATEDIR
#endif

#ifndef HAL_BOARD_LOG_DIRECTORY
    #define HAL_BOARD_LOG_DIRECTORY HAL_BOARD_STATE_DIRECTORY "/logs"
#endif

#ifndef HAL_BOARD_TERRAIN_DIRECTORY
    #define HAL_BOARD_TERRAIN_DIRECTORY HAL_BOARD_STATE_DIRECTORY "/terrain"
#endif

#ifndef HAL_BOARD_STORAGE_DIRECTORY
    #define HAL_BOARD_STORAGE_DIRECTORY HAL_BOARD_STATE_DIRECTORY
#endif

#ifndef HAL_BOARD_CAN_IFACE_NAME
    #define HAL_BOARD_CAN_IFACE_NAME "can0"
#endif

// if bus masks are not setup above then use these defaults
#ifndef HAL_LINUX_I2C_BUS_MASK
    #define HAL_LINUX_I2C_BUS_MASK 0xFFFF
#endif

#ifndef HAL_LINUX_I2C_INTERNAL_BUS_MASK
    #define HAL_LINUX_I2C_INTERNAL_BUS_MASK 0xFFFF
#endif

#ifndef HAL_LINUX_I2C_EXTERNAL_BUS_MASK
    #define HAL_LINUX_I2C_EXTERNAL_BUS_MASK 0xFFFF
#endif

#include <AP_HAL_Linux/Semaphores.h>
#define HAL_Semaphore Linux::Semaphore
#include <AP_HAL/EventHandle.h>
#define HAL_EventHandle AP_HAL::EventHandle

#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 1
#endif

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

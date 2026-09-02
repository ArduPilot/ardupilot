#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Authoritative catalog of peripherals attachable to Renode boards."""

import re

COVERAGE_DETECTION = 'detection'
COVERAGE_DYNAMIC = 'dynamic'


# Keep this catalog declarative.  The launcher filters it by bus type and the
# board generator consumes the model-specific fields.  Coverage metadata is
# also used by driver_inventory.py and does not affect generated platforms.
ATTACHABLE_DEVICES = {
    'ublox-gps': {
        'name': 'u-blox GPS',
        'category': 'GNSS',
        'bus': 'uart',
        'model': 'Sensors.AP_UBlox',
        'driver': 'AP_GPS/AP_GPS_UBLOX.cpp',
        'feature': 'AP_GPS_UBLOX_ENABLED',
        'coverage': COVERAGE_DYNAMIC,
        'hotplug': 'runtime',
        'parameters': (
            ('SERIAL{serial}_PROTOCOL', 5),
            ('SERIAL{serial}_BAUD', 230),
            ('GPS{instance}_TYPE', 2),
        ),
    },
    'benewake-rangefinder': {
        'name': 'Benewake rangefinder',
        'category': 'Rangefinder',
        'bus': 'uart',
        'model': 'Sensors.AP_Benewake',
        'driver': 'AP_RangeFinder/AP_RangeFinder_Benewake.cpp',
        'feature': 'AP_RANGEFINDER_BENEWAKE_ENABLED',
        'coverage': COVERAGE_DYNAMIC,
        'hotplug': 'runtime',
        'physics': {
            'source': 'rangefinder',
            'property': 'RangefinderIndex',
            'count': 10,
        },
        'parameters': (
            ('SERIAL{serial}_PROTOCOL', 9),
            ('SERIAL{serial}_BAUD', 115),
            ('RNGFND{instance}_TYPE', 19),
        ),
    },
    'lightware-rangefinder': {
        'name': 'LightWare rangefinder',
        'category': 'Rangefinder',
        'bus': 'uart',
        'model': 'Sensors.AP_LightWare',
        'driver': 'AP_RangeFinder/AP_RangeFinder_LightWareSerial.cpp',
        'feature': 'AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED',
        'coverage': COVERAGE_DYNAMIC,
        'hotplug': 'runtime',
        'physics': {
            'source': 'rangefinder',
            'property': 'RangefinderIndex',
            'count': 10,
        },
        'parameters': (
            ('SERIAL{serial}_PROTOCOL', 9),
            ('SERIAL{serial}_BAUD', 115),
            ('RNGFND{instance}_TYPE', 8),
        ),
    },
    'ist8310-compass': {
        'name': 'IST8310 compass',
        'category': 'Compass',
        'bus': 'i2c',
        'model': 'Sensors.AP_IST8310',
        'address': 0x0E,
        'driver': 'AP_Compass/AP_Compass_IST8310.cpp',
        'feature': 'AP_COMPASS_IST8310_ENABLED',
        'coverage': COVERAGE_DYNAMIC,
        'hotplug': 'boot-probe',
        'selection': 'auto-probe',
    },
    'ms4525-airspeed': {
        'name': 'MS4525 airspeed',
        'category': 'Airspeed',
        'bus': 'i2c',
        'model': 'Sensors.AP_Airspeed',
        'address': 0x28,
        'driver': 'AP_Airspeed/AP_Airspeed_MS4525.cpp',
        'feature': 'AP_AIRSPEED_MS4525_ENABLED',
        'coverage': COVERAGE_DYNAMIC,
        'hotplug': 'parameter-reinit',
        'parameters': (
            ('{airspeed_prefix}_TYPE', 1),
            ('{airspeed_prefix}_BUS', '{i2c_index}'),
        ),
    },
    'auav-airspeed': {
        'name': 'AUAV airspeed',
        'category': 'Airspeed',
        'bus': 'i2c',
        'model': 'Sensors.AP_Airspeed_AUAV',
        'address': 0x26,
        'driver': 'AP_Airspeed/AP_Airspeed_AUAV.cpp',
        'feature': 'AP_AIRSPEED_AUAV_ENABLED',
        'coverage': COVERAGE_DYNAMIC,
        'hotplug': 'parameter-reinit',
        'parameters': (
            ('{airspeed_prefix}_TYPE', 17),
            ('{airspeed_prefix}_BUS', '{i2c_index}'),
        ),
    },
    'asp5033-airspeed': {
        'name': 'ASP5033 airspeed',
        'category': 'Airspeed',
        'bus': 'i2c',
        'model': 'Sensors.AP_Airspeed_ASP5033',
        'address': 0x6C,
        'driver': 'AP_Airspeed/AP_Airspeed_ASP5033.cpp',
        'feature': 'AP_AIRSPEED_ASP5033_ENABLED',
        'coverage': COVERAGE_DYNAMIC,
        'hotplug': 'parameter-reinit',
        'parameters': (
            ('{airspeed_prefix}_TYPE', 15),
            ('{airspeed_prefix}_BUS', '{i2c_index}'),
        ),
    },
    'bmp280-barometer': {
        'name': 'BMP280 barometer',
        'category': 'Barometer',
        'bus': 'i2c',
        'model': 'Sensors.AP_BMP280',
        'address': 0x76,
        'driver': 'AP_Baro/AP_Baro_BMP280.cpp',
        'feature': 'AP_BARO_BMP280_ENABLED',
        'coverage': COVERAGE_DYNAMIC,
        'hotplug': 'parameter-reinit',
        'parameters': (
            ('BARO_PROBE_EXT', 2),
            ('BARO_EXT_BUS', '{i2c_index}'),
        ),
    },
    'dronecan-airspeed': {
        'name': 'DroneCAN airspeed',
        'category': 'Airspeed',
        'bus': 'can',
        'sidecar': 'dronecan-airspeed',
        'driver': 'AP_Airspeed/AP_Airspeed_DroneCAN.cpp',
        'feature': 'AP_AIRSPEED_DRONECAN_ENABLED',
        'coverage': COVERAGE_DETECTION,
        'hotplug': 'runtime',
        'selection': 'auto-probe',
    },
}


DRIVER_PROBE_PROFILES = {
    'matekh743-navigation': {
        'board': 'MatekH743',
        'vehicle': 'arduplane',
        'build_target': 'plane',
        'defaults': 'Tools/renode/tests/MatekH743-plane.parm',
        'devices': (
            {
                'device': 'ublox-gps',
                'port': 'SERIAL3',
                'instance': 1,
                'assertions': (
                    'detection', 'stable-values', 'stepped-values',
                    'output-corruption-recovery',
                    'output-suppression-recovery',
                ),
            },
            {
                'device': 'ist8310-compass',
                'port': 'I2C1',
                'instance': 1,
                'assertions': (
                    'device-id', 'stable-values', 'stepped-values',
                    'corrupt-read-recovery',
                ),
            },
        ),
    },
    'matekh743-rangefinders': {
        'board': 'MatekH743',
        'vehicle': 'arduplane',
        'build_target': 'plane',
        'defaults': 'Tools/renode/tests/MatekH743-rangefinders.parm',
        'devices': (
            {
                'device': 'benewake-rangefinder',
                'port': 'SERIAL4',
                'instance': 1,
                'assertions': (
                    'stable-values', 'stepped-values',
                    'output-suppression-recovery',
                ),
            },
            {
                'device': 'lightware-rangefinder',
                'port': 'SERIAL5',
                'instance': 2,
                'assertions': (
                    'stable-values', 'stepped-values',
                    'output-suppression-recovery',
                ),
            },
        ),
    },
    'matekh743-airspeed': {
        'board': 'MatekH743',
        'vehicle': 'arduplane',
        'build_target': 'plane',
        'defaults': 'Tools/renode/tests/MatekH743-airspeed.parm',
        'devices': (
            {
                'device': 'ms4525-airspeed',
                'port': 'I2C1',
                'instance': 1,
                'assertions': (
                    'device-id', 'stable-values', 'stepped-values',
                    'corrupt-read-recovery',
                ),
            },
        ),
    },
    'matekh743-airspeeds': {
        'board': 'MatekH743',
        'vehicle': 'arduplane',
        'build_target': 'plane',
        'defaults': 'Tools/renode/tests/MatekH743-airspeeds.parm',
        'devices': (
            {
                'device': 'auav-airspeed',
                'port': 'I2C1',
                'instance': 1,
                'assertions': (
                    'device-id', 'stable-values', 'stepped-values',
                    'corrupt-read-recovery',
                ),
            },
            {
                'device': 'asp5033-airspeed',
                'port': 'I2C1',
                'instance': 2,
                'assertions': (
                    'device-id', 'stable-values', 'stepped-values',
                    'corrupt-read-recovery',
                ),
            },
        ),
    },
    'matekh743-bmp280': {
        'board': 'MatekH743',
        'vehicle': 'arduplane',
        'build_target': 'plane',
        'defaults': 'Tools/renode/tests/MatekH743-bmp280.parm',
        'devices': (
            {
                'device': 'bmp280-barometer',
                'port': 'I2C1',
                'instance': 1,
                'assertions': (
                    'device-id', 'stable-values', 'stepped-values',
                    'stuck-sample-recovery',
                ),
            },
        ),
    },
}


# AP_SerialManager protocols describe more than physical peripherals.  Keeping
# every enum name classified prevents a new protocol silently disappearing
# from the Renode coverage plan.  "partial" means at least one device using
# that protocol is modelled, not that every backend is covered.
SERIAL_PROTOCOL_CLASSIFICATIONS = {
    'None': ('disabled', 'not-applicable'),
    'Console': ('internal', 'not-applicable'),
    'MAVLink': ('link', 'supported'),
    'MAVLink2': ('alias', 'not-applicable'),
    'FrSky_D': ('output', 'planned'),
    'FrSky_SPort': ('output', 'planned'),
    'GPS': ('device', 'partial'),
    'GPS2': ('alias', 'not-applicable'),
    'AlexMos': ('device', 'planned'),
    'Gimbal': ('device', 'planned'),
    'Rangefinder': ('device', 'partial'),
    'FrSky_SPort_Passthrough': ('output', 'planned'),
    'Lidar360': ('device', 'planned'),
    'Aerotenna_USD1': ('alias', 'planned'),
    'Beacon': ('device', 'planned'),
    'Volz': ('device', 'planned'),
    'Sbus1': ('output', 'planned'),
    'ESCTelemetry': ('device', 'planned'),
    'Devo_Telem': ('output', 'planned'),
    'OpticalFlow': ('device', 'planned'),
    'Robotis': ('device', 'planned'),
    'NMEAOutput': ('output', 'planned'),
    'WindVane': ('device', 'planned'),
    'SLCAN': ('link', 'planned'),
    'RCIN': ('device', 'planned'),
    'EFI': ('device', 'planned'),
    'LTM_Telem': ('output', 'planned'),
    'RunCam': ('device', 'planned'),
    'Hott': ('output', 'planned'),
    'Scripting': ('link', 'supported'),
    'CRSF': ('device', 'planned'),
    'Generator': ('device', 'planned'),
    'Winch': ('device', 'planned'),
    'MSP': ('device', 'planned'),
    'DJI_FPV': ('output', 'planned'),
    'AirSpeed': ('device', 'planned'),
    'ADSB': ('device', 'planned'),
    'AHRS': ('device', 'planned'),
    'SmartAudio': ('device', 'planned'),
    'FETtecOneWire': ('device', 'planned'),
    'Torqeedo': ('device', 'planned'),
    'AIS': ('device', 'planned'),
    'CoDevESC': ('device', 'planned'),
    'MSP_DisplayPort': ('output', 'planned'),
    'MAVLinkHL': ('link', 'planned'),
    'Tramp': ('device', 'planned'),
    'DDS_XRCE': ('link', 'planned'),
    'IMUOUT': ('output', 'planned'),
    'PPP': ('link', 'planned'),
    'IBUS_Telem': ('output', 'planned'),
    'IOMCU': ('internal', 'partial'),
}


# These are probe family names actually used with I2C in ChibiOS hwdefs.  A
# modelled entry means a Renode class exists; production-driver and dynamic
# tests are tracked independently as the matrix is built out.
I2C_HWDEF_CLASSIFICATIONS = {
    'barometer': {
        'AUAV': 'modelled',
        'BMP085': 'modelled',
        'BMP280': 'modelled',
        'BMP388': 'modelled',
        'BMP581': 'modelled',
        'DPS280': 'modelled',
        'DPS310': 'modelled',
        'ICM20789': 'planned',
        'ICP101XX': 'planned',
        'ICP201XX': 'modelled',
        'LPS2XH': 'modelled',
        'MS5611': 'modelled',
        'SPL06': 'modelled',
    },
    'compass': {
        'AK09916': 'planned',
        'AK8963': 'planned',
        'BMM150': 'planned',
        'BMM350': 'planned',
        'HMC5843': 'planned',
        'IIS2MDC': 'planned',
        'IST8308': 'planned',
        'IST8310': 'modelled',
        'LIS2MDL': 'planned',
        'LIS3MDL': 'planned',
        'MMC3416': 'planned',
        'QMC5883L': 'planned',
        'QMC5883P': 'planned',
        'RM3100': 'planned',
    },
    'imu': {
        'BMI088': 'modelled',
        'Invensense': 'modelled',
    },
}


# Direct I2C users found by driver_inventory.py.  This separates concrete
# peripherals from frontends, reusable bases and bus-access services.  Header-
# only backends and backends receiving a generic AP_HAL::Device are added to
# the device-family manifest independently.
I2C_SOURCE_CLASSIFICATIONS = {
    'AP_ADC/AP_ADC_ADS1115.cpp': 'device',
    'AP_Airspeed/AP_Airspeed.cpp': 'frontend',
    'AP_Airspeed/AP_Airspeed_ASP5033.cpp': 'device',
    'AP_Airspeed/AP_Airspeed_AUAV.cpp': 'device',
    'AP_Airspeed/AP_Airspeed_DLVR.cpp': 'device',
    'AP_Airspeed/AP_Airspeed_MS4525.cpp': 'device',
    'AP_Airspeed/AP_Airspeed_MS5525.cpp': 'device',
    'AP_Airspeed/AP_Airspeed_SDP3X.cpp': 'device',
    'AP_Baro/AP_Baro.cpp': 'frontend',
    'AP_Baro/AP_Baro_KellerLD.cpp': 'device',
    'AP_BattMonitor/AP_BattMonitor_AD7091R5.cpp': 'device',
    'AP_BattMonitor/AP_BattMonitor_INA2xx.cpp': 'device',
    'AP_BattMonitor/AP_BattMonitor_INA3221.cpp': 'device',
    'AP_BattMonitor/AP_BattMonitor_LTC2946.cpp': 'device',
    'AP_BattMonitor/AP_BattMonitor_SMBus.cpp': 'base',
    'AP_BattMonitor/AP_BattMonitor_TIBQ76952.cpp': 'device',
    'AP_Compass/AP_Compass.cpp': 'frontend',
    'AP_DAC/AP_DAC_MCP40D1x.cpp': 'device',
    'AP_DAC/AP_DAC_TIx3204.cpp': 'device',
    'AP_IRLock/AP_IRLock_I2C.cpp': 'device',
    'AP_InertialSensor/AP_InertialSensor.cpp': 'frontend',
    'AP_InertialSensor/AP_InertialSensor_BMI160.cpp': 'device',
    'AP_InertialSensor/AP_InertialSensor_BMI270.cpp': 'device',
    'AP_InertialSensor/AP_InertialSensor_Invensense.cpp': 'device',
    'AP_InertialSensor/AP_InertialSensor_Invensensev2.cpp': 'device',
    'AP_InertialSensor/AP_InertialSensor_L3G4200D.cpp': 'device',
    'AP_Notify/Display.cpp': 'frontend',
    'AP_Notify/IS31FL3195.cpp': 'device',
    'AP_Notify/LP5562.cpp': 'device',
    'AP_Notify/NCP5623.cpp': 'device',
    'AP_Notify/OreoLED_I2C.cpp': 'device',
    'AP_Notify/PCA9685LED_I2C.cpp': 'device',
    'AP_Notify/ToshibaLED_I2C.cpp': 'device',
    'AP_OpticalFlow/AP_OpticalFlow_PX4Flow.cpp': 'device',
    'AP_RangeFinder/AP_RangeFinder.cpp': 'frontend',
    'AP_RangeFinder/AP_RangeFinder_Benewake_TFS20L.cpp': 'device',
    'AP_RangeFinder/AP_RangeFinder_LightWare_GRF_I2C.cpp': 'device',
    'AP_RangeFinder/AP_RangeFinder_PulsedLightLRF.cpp': 'device',
    'AP_RangeFinder/AP_RangeFinder_TeraRangerI2C.cpp': 'device',
    'AP_RangeFinder/AP_RangeFinder_VL53L0X.cpp': 'device',
    'AP_RangeFinder/AP_RangeFinder_VL53L1X.cpp': 'device',
    'AP_Scripting/lua_bindings.cpp': 'service',
    'AP_TemperatureSensor/AP_TemperatureSensor_MCP9600.cpp': 'device',
    'AP_TemperatureSensor/AP_TemperatureSensor_MLX90614.cpp': 'device',
    'AP_TemperatureSensor/AP_TemperatureSensor_SHT3x.cpp': 'device',
    'AP_TemperatureSensor/AP_TemperatureSensor_TMP119.cpp': 'device',
    'AP_TemperatureSensor/AP_TemperatureSensor_TSYS01.cpp': 'device',
    'AP_TemperatureSensor/AP_TemperatureSensor_TSYS03.cpp': 'device',
    'GCS_MAVLink/GCS_DeviceOp.cpp': 'service',
}


# Direct UART users found by driver_inventory.py.  The role separates concrete
# peripherals from framework code while the family groups sources which can be
# covered by one configurable emulator model or protocol implementation.
SERIAL_SOURCE_CLASSIFICATIONS = {
    'AP_ADSB/AP_ADSB_Sagetech.cpp': ('device', 'adsb-sagetech'),
    'AP_ADSB/AP_ADSB_Sagetech_MXS.cpp': ('device', 'adsb-sagetech'),
    'AP_ADSB/AP_ADSB_uAvionix_UCP.cpp': ('device', 'adsb-uavionix'),
    'AP_AIS/AP_AIS.cpp': ('device', 'ais'),
    'AP_Airspeed/AP_Airspeed_NMEA.cpp': ('device', 'nmea-airspeed'),
    'AP_BLHeli/AP_BLHeli.cpp': ('device', 'esc-telemetry'),
    'AP_Beacon/AP_Beacon_Backend.cpp': ('base', 'beacon'),
    'AP_BoardConfig/board_drivers.cpp': ('service', 'uart-configuration'),
    'AP_CANManager/AP_SLCANIface.cpp': ('link', 'slcan'),
    'AP_Camera/AP_Camera.cpp': ('frontend', 'camera'),
    'AP_Camera/AP_RunCam.cpp': ('device', 'runcam'),
    'AP_Common/NMEA.cpp': ('base', 'nmea'),
    'AP_DDS/AP_DDS_Serial.cpp': ('link', 'dds-xrce'),
    'AP_Devo_Telem/AP_Devo_Telem.cpp': ('output', 'devo-telemetry'),
    'AP_EFI/AP_EFI_Serial_Hirth.cpp': ('device', 'efi-hirth'),
    'AP_EFI/AP_EFI_Serial_Lutan.cpp': ('device', 'efi-lutan'),
    'AP_EFI/AP_EFI_Serial_MS.cpp': ('device', 'efi-megasquirt'),
    'AP_ExternalAHRS/AP_ExternalAHRS_Aeron_plx.cpp': (
        'device', 'ahrs-aeron'),
    'AP_ExternalAHRS/AP_ExternalAHRS_GSOF.cpp': ('device', 'ahrs-gsof'),
    'AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp': (
        'device', 'ahrs-inertiallabs'),
    'AP_ExternalAHRS/AP_ExternalAHRS_MicroStrain5.cpp': (
        'device', 'ahrs-microstrain'),
    'AP_ExternalAHRS/AP_ExternalAHRS_MicroStrain7.cpp': (
        'device', 'ahrs-microstrain'),
    'AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp': ('device', 'ahrs-sbg'),
    'AP_ExternalAHRS/AP_ExternalAHRS_SensAItion.cpp': (
        'device', 'ahrs-sensaition'),
    'AP_ExternalAHRS/AP_ExternalAHRS_VectorNav.cpp': (
        'device', 'ahrs-vectornav'),
    'AP_FETtecOneWire/AP_FETtecOneWire.cpp': ('device', 'esc-fettec'),
    'AP_Frsky_Telem/AP_Frsky_Backend.cpp': ('base', 'frsky-telemetry'),
    'AP_Frsky_Telem/AP_Frsky_Telem.cpp': ('output', 'frsky-telemetry'),
    'AP_GPS/AP_GPS.cpp': ('frontend', 'gnss'),
    'AP_GPS/AP_GPS_GSOF.cpp': ('device', 'gnss-gsof'),
    'AP_GPS/AP_GPS_NOVA.cpp': ('device', 'gnss-nova'),
    'AP_GPS/AP_GPS_SBF.cpp': ('device', 'gnss-sbf'),
    'AP_GPS/AP_GPS_SBP.cpp': ('device', 'gnss-sbp'),
    'AP_GPS/AP_GPS_SBP2.cpp': ('device', 'gnss-sbp'),
    'AP_GPS/AP_GPS_SIRF.cpp': ('device', 'gnss-sirf'),
    'AP_GPS/AP_GPS_UBLOX.cpp': ('device', 'gnss-ublox'),
    'AP_GPS/GPS_Backend.cpp': ('base', 'gnss'),
    'AP_Generator/AP_Generator_IE_FuelCell.cpp': (
        'device', 'generator-fuelcell'),
    'AP_Generator/AP_Generator_RichenPower.cpp': (
        'device', 'generator-richenpower'),
    'AP_Hott_Telem/AP_Hott_Telem.cpp': ('output', 'hott-telemetry'),
    'AP_IBus_Telem/AP_IBus_Telem.cpp': ('output', 'ibus-telemetry'),
    'AP_IOMCU/AP_IOMCU.cpp': ('internal', 'iomcu'),
    'AP_InertialSensor/AP_InertialSensor.cpp': ('output', 'imu-output'),
    'AP_LTM_Telem/AP_LTM_Telem.cpp': ('output', 'ltm-telemetry'),
    'AP_MSP/AP_MSP.cpp': ('frontend', 'msp'),
    'AP_MSP/AP_MSP_Telem_Backend.cpp': ('base', 'msp-telemetry'),
    'AP_MSP/AP_MSP_Telem_DJI.cpp': ('output', 'msp-dji'),
    'AP_Mount/AP_Mount_Alexmos.cpp': ('device', 'gimbal-alexmos'),
    'AP_Mount/AP_Mount_Backend_Serial.cpp': ('base', 'gimbal-serial'),
    'AP_NMEA_Output/AP_NMEA_Output.cpp': ('output', 'nmea-output'),
    'AP_Networking/AP_Networking_PPP.cpp': ('link', 'ppp'),
    'AP_Networking/AP_Networking_port.cpp': ('base', 'network-port'),
    'AP_OpticalFlow/AP_OpticalFlow_CXOF.cpp': ('device', 'flow-cxof'),
    'AP_OpticalFlow/AP_OpticalFlow_UPFLOW.cpp': ('device', 'flow-upflow'),
    'AP_Proximity/AP_Proximity_Backend_Serial.cpp': (
        'base', 'serial-proximity'),
    'AP_RCProtocol/AP_RCProtocol.cpp': ('frontend', 'rc-input'),
    'AP_RCProtocol/AP_RCProtocol_CRSF.cpp': ('device', 'rc-crsf'),
    'AP_RCProtocol/AP_RCProtocol_FPort.cpp': ('device', 'rc-fport'),
    'AP_RCProtocol/AP_RCProtocol_FPort2.cpp': ('device', 'rc-fport'),
    'AP_RCProtocol/AP_RCProtocol_GHST.cpp': ('device', 'rc-ghst'),
    'AP_RCProtocol/AP_RCProtocol_SRXL2.cpp': ('device', 'rc-srxl2'),
    'AP_RCTelemetry/AP_CRSF_Telem.cpp': ('output', 'rc-crsf'),
    'AP_RCTelemetry/AP_GHST_Telem.cpp': ('output', 'rc-ghst'),
    'AP_RangeFinder/AP_RangeFinder_BLPing.cpp': (
        'device', 'rangefinder-blping'),
    'AP_RangeFinder/AP_RangeFinder_Backend_Serial.cpp': (
        'base', 'serial-rangefinder'),
    'AP_RobotisServo/AP_RobotisServo.cpp': ('device', 'servo-robotis'),
    'AP_SBusOut/AP_SBusOut.cpp': ('output', 'sbus-output'),
    'AP_Scripting/AP_Scripting_SerialAccess.cpp': ('service', 'scripting'),
    'AP_Scripting/lua_bindings.cpp': ('service', 'scripting'),
    'AP_SerialManager/AP_SerialManager.cpp': ('service', 'serial-manager'),
    'AP_Torqeedo/AP_Torqeedo_TQBus.cpp': ('device', 'motor-torqeedo'),
    'AP_VideoTX/AP_SmartAudio.cpp': ('device', 'vtx-smartaudio'),
    'AP_VideoTX/AP_Tramp.cpp': ('device', 'vtx-tramp'),
    'AP_Volz_Protocol/AP_Volz_Protocol.cpp': ('device', 'servo-volz'),
    'AP_Winch/AP_Winch_Daiwa.cpp': ('device', 'winch-daiwa'),
    'AP_WindVane/AP_WindVane_NMEA.cpp': ('device', 'nmea-wind'),
    'GCS_MAVLink/GCS_Common.cpp': ('link', 'mavlink'),
    'GCS_MAVLink/GCS_FTP.cpp': ('service', 'mavlink'),
    'GCS_MAVLink/GCS_MAVLink.cpp': ('link', 'mavlink'),
    'GCS_MAVLink/GCS_serial_control.cpp': ('service', 'serial-control'),
}


def resolve_parameter_recipe(device_id, port, instance=1):
    """Resolve a catalog recipe for a concrete hwdef-derived port."""
    device = ATTACHABLE_DEVICES.get(device_id)
    if device is None:
        raise ValueError('unknown emulated device %s' % device_id)
    if device['bus'] != port.get('bus'):
        raise ValueError('%s cannot attach to %s' %
                         (device['name'], port.get('name', port.get('id'))))
    if (not isinstance(instance, int) or isinstance(instance, bool) or
            instance < 1):
        raise ValueError('device instance must be a positive integer')

    context = {
        'instance': instance,
        'i2c_index': port.get('index'),
        'airspeed_prefix': 'ARSPD' if instance == 1 else 'ARSPD%u' % instance,
    }
    if device['bus'] == 'uart':
        match = re.fullmatch(r'SERIAL([0-9]+)', port.get('id', ''))
        if match is None:
            raise ValueError('UART port lacks a SERIAL index')
        context['serial'] = int(match.group(1))
    resolved = []
    for name, value in device.get('parameters', ()):
        try:
            resolved_name = name.format(**context)
            resolved_value = (value.format(**context)
                              if isinstance(value, str) else value)
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError('%s has an invalid parameter recipe' % device_id) from error
        resolved.append((resolved_name, resolved_value))
    return tuple(resolved)

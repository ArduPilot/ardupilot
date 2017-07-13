#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_SensorHub.h"

#if HAL_SENSORHUB_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/Device.h>

extern const AP_HAL::HAL& hal;


bool GyroMessageHandler::isValid(GyroMessage::data_t *data)
{
    return true;
}

bool AccelMessageHandler::isValid(AccelMessage::data_t *data)
{
    return true;
}

void GyroMessageHandler::handle(GyroMessage::data_t *data)
{
    if (_backend->_sem_gyro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (isValid(data)) {
            uint8_t ins = data->instance;
            // NOTE: We currently assume that the sensor instances will match
            // between Sink & Source.
            auto gyro_info = _backend->_gyro_instance[ins];

            if (gyro_info.registered) {
                if (gyro_info.devtype != data->devtype) {
                    // The sensor has changed. Keep same instance & update the id.
                    auto devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SENSORHUB, 0, 0, data->devtype);
                    auto result = _backend->_imu.set_gyro_instance(AP_SensorHub::UPDATE_RATE_HZ, devid, ins);
                    if (!result) {
                        AP_HAL::panic("ERROR: Could not update gyro instance.\n");
                    }

                    _backend->_gyro_instance[ins].devtype = data->devtype;

                    hal.console->printf("GYRO: Sensor Changed. Inst: %d DevType: %d\n", ins, data->devtype);
                }
            } else {
                // A new sensor is detected. Register it.
                auto devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SENSORHUB, 0, 0, data->devtype);
                _backend->_imu.register_gyro(AP_SensorHub::UPDATE_RATE_HZ, devid);
                _backend->_gyro_instance[ins].registered = true;
                _backend->_gyro_instance[ins].devtype = data->devtype;

                hal.console->printf("GYRO: Register New. Inst: %d Devtype: %d\n", ins, data->devtype);
            }

            _backend->_gyro[ins].x = GyroMessage::scaleFromPacket(data->gyrox);
            _backend->_gyro[ins].y = GyroMessage::scaleFromPacket(data->gyroy);
            _backend->_gyro[ins].z = GyroMessage::scaleFromPacket(data->gyroz);
            _backend->_last_timestamp[ins] = AP_HAL::micros64();

            _backend->_notify_new_gyro_raw_sample(ins, _backend->_gyro[ins], AP_HAL::micros64(), data->dt);
            _count++;
        } else {
            hal.console->printf("ERROR: GYRO data invalid!\n");
            _error++;
        }

        _backend->_sem_gyro->give();
    }
}

void AccelMessageHandler::handle(AccelMessage::data_t *data)
{
    if (_backend->_sem_accel->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (isValid(data)) {
            uint8_t ins = data->instance;
            auto accel_info = _backend->_accel_instance[ins];

            if (accel_info.registered) {
                if (accel_info.devtype != data->devtype) {
                    // The sensor has changed. Keep same instance & update the id.
                    auto devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SENSORHUB, 0, 0, data->devtype);
                    auto result = _backend->_imu.set_accel_instance(AP_SensorHub::UPDATE_RATE_HZ, devid, ins);
                    if (!result) {
                        AP_HAL::panic("ERROR: Could not update accel instance.\n");
                    }

                    _backend->_accel_instance[ins].devtype = data->devtype;
                    hal.console->printf("ACCEL: Sensor Changed. Inst: %d Devtype: %d\n", ins, data->devtype);
                }
            } else {
                // A new sensor is detected. Register it.
                auto devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SENSORHUB, 0, 0, data->devtype);
                _backend->_imu.register_accel(AP_SensorHub::UPDATE_RATE_HZ, devid);
                _backend->_accel_instance[ins].registered = true;
                _backend->_accel_instance[ins].devtype = data->devtype;
                hal.console->printf("ACCEL: Register New. Inst: %d Devtype: %d\n", ins, data->devtype);
            }

            _backend->_accel[ins].x = AccelMessage::scaleFromPacket(data->accelx);
            _backend->_accel[ins].y = AccelMessage::scaleFromPacket(data->accely);
            _backend->_accel[ins].z = AccelMessage::scaleFromPacket(data->accelz);
            _backend->_last_timestamp[ins] = AP_HAL::micros64();

            _backend->_notify_new_accel_raw_sample(ins, _backend->_accel[ins], AP_HAL::micros64(), false, data->dt);
            _count++;
        } else {
            hal.console->printf("ERROR: ACCEL data invalid!\n");
            _error++;
        }

        _backend->_sem_accel->give();
    }
}

AP_InertialSensor_SensorHub::AP_InertialSensor_SensorHub(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{

    // NOTE: We begin by registering gyro & accel the system needs one to
    // startup. We then update this instance & register new sensors as we
    // receive messages.
    auto gyro_ins = _imu.register_gyro(AP_SensorHub::UPDATE_RATE_HZ, 0);
    auto accel_ins = _imu.register_accel(AP_SensorHub::UPDATE_RATE_HZ, 0);

    _gyro_instance[gyro_ins].registered = true;
    _gyro_instance[gyro_ins].devtype = 0;

    _accel_instance[accel_ins].registered = true;
    _accel_instance[accel_ins].devtype = 0;

    _sem_gyro = hal.util->new_semaphore();
    _sem_accel = hal.util->new_semaphore();
    _shub->registerHandler(&_gyro_handler);
    _shub->registerHandler(&_accel_handler);
}

AP_InertialSensor_Backend *AP_InertialSensor_SensorHub::detect(AP_InertialSensor &imu)
{
    AP_InertialSensor_SensorHub *sensor = new AP_InertialSensor_SensorHub(imu);
    if (sensor == nullptr) {
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_SensorHub::update()
{
    if (_sem_gyro->take(HAL_SEMAPHORE_BLOCK_FOREVER) && _sem_accel->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {

        for (int i = 0; i < INS_MAX_INSTANCES; i++) {
            if (_gyro_instance[i].registered && _accel_instance[i].registered) {
                update_gyro(i);
                update_accel(i);
            }
        }

        _sem_gyro->give();
        _sem_accel->give();
        return true;
    }

    return false;
}
#endif
#include <AP_HAL/AP_HAL.h>
#include "AP_Compass_SensorHub.h"

#if HAL_SENSORHUB_ENABLED

extern const AP_HAL::HAL& hal;

bool CompassMessageHandler::isValid(CompassMessage::data_t *data)
{
    return true;
}

void CompassMessageHandler::handle(CompassMessage::data_t *data)
{
    if(_backend->_sem_mag->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (isValid(data)) {
            auto ins = data->instance;
            auto instance_registered = _backend->_instance[ins];

            if (!instance_registered) {
                // There are messages being received by more than one compass.
                _backend->register_compass();
                _backend->_instance[ins] = true;
                hal.console->printf("COMPASS: Register New. Inst: %u\n", ins);
            }

            // NOTE: Fields are already rotated.
            _backend->_field[ins].x = data->magx;
            _backend->_field[ins].y = data->magy;
            _backend->_field[ins].z = data->magz;
            _backend->_last_timestamp[ins] = AP_HAL::micros();

            _backend->publish_raw_field(_backend->_field[ins], AP_HAL::micros(), ins);
            _backend->correct_field(_backend->_field[ins], ins);
            _count++;

            _backend->_mag_accum[ins] += _backend->_field[ins];
            _backend->_accum_count[ins]++;
            if (_backend->_accum_count[ins] == 10) {
                _backend->_mag_accum[ins] /= 2;
                _backend->_accum_count[ins] = 5;
                _backend->_has_sample[ins] = true;
            }

        } else {
            // A packet is successfully decoded, however its data is not valid.
            // This most likely due to CRC collision.
            // TODO: Appropriately log this error.
            hal.console->printf("ERROR:Compass data invalid!\n");
            _error++;
        }

        _backend->_sem_mag->give();
    }
}

AP_Compass_SensorHub::AP_Compass_SensorHub(Compass &compass) :
    AP_Compass_Backend(compass)
{
    _sem_mag = hal.util->new_semaphore();

    auto ins = register_compass();
    _instance[ins] = true;

    _shub->registerHandler(&_handler);
}

void AP_Compass_SensorHub::read()
{
    if (_sem_mag->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_sem->take_nonblocking()) {
            for (int i = 0; i < COMPASS_MAX_INSTANCES; i++) {
                if (_instance[i] && _has_sample[i]) {
                    Vector3f field(_mag_accum[i]);
                    field /= _accum_count[i];
                    _mag_accum[i].zero();
                    _accum_count[i] = 0;
                    publish_filtered_field(field, i);
                    _has_sample[i] = false;
                }
            }
            _sem->give();
        }
        _sem_mag->give();
    }
}
#endif
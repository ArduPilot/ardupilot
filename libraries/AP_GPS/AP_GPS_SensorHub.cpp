#include "AP_GPS_SensorHub.h"
#include <AP_Math/AP_Math.h>

#if HAL_SENSORHUB_ENABLED

extern const AP_HAL::HAL& hal;

bool GPSMessageHandler::isValid(GPSMessage::data_t *data)
{
    return !isnan(data->ground_speed)
        && !isinf(data->ground_speed)
        && !isnan(data->ground_course)
        && !isinf(data->ground_course)
        && !isnan(data->velocityx) && !isnan(data->velocityy) && !isnan(data->velocityz)
        && !isinf(data->velocityx) && !isinf(data->velocityy) && !isinf(data->velocityz)
        && !isnan(data->speed_accuracy)
        && !isinf(data->speed_accuracy)
        && !isnan(data->horizontal_accuracy)
        && !isinf(data->horizontal_accuracy)
        && !isnan(data->vertical_accuracy)
        && !isinf(data->vertical_accuracy);
}

void GPSMessageHandler::handle(GPSMessage::data_t *data)
{
    if (_backend->_sem_gnss->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (isValid(data)) {
            _backend->_interm_state.status                   = data->status;
            _backend->_interm_state.time_week_ms             = data->time_week_ms;
            _backend->_interm_state.time_week                = data->time_week;
            _backend->_interm_state.location                 = data->location;
            _backend->_interm_state.ground_speed             = data->ground_speed;
            _backend->_interm_state.ground_course            = data->ground_course;
            _backend->_interm_state.hdop                     = data->hdop;
            _backend->_interm_state.vdop                     = data->vdop;
            _backend->_interm_state.num_sats                 = data->num_sats;
            _backend->_interm_state.velocity                 = Vector3f(data->velocityx, data->velocityy, data->velocityz);
            _backend->_interm_state.speed_accuracy           = data->speed_accuracy;
            _backend->_interm_state.horizontal_accuracy      = data->horizontal_accuracy;
            _backend->_interm_state.vertical_accuracy        = data->vertical_accuracy;
            _backend->_interm_state.have_vertical_velocity   = data->have_vertical_velocity;
            _backend->_interm_state.have_speed_accuracy      = data->have_speed_accuracy;
            _backend->_interm_state.have_horizontal_accuracy = data->have_horizontal_accuracy;
            _backend->_interm_state.have_vertical_accuracy   = data->have_vertical_accuracy;
            _backend->_interm_state.last_gps_time_ms         = AP_HAL::millis();
            _backend->_new_data = true;
            _count++;
        } else {
            hal.console->printf("ERROR: GPS data invalid!\n");
            _error++;
        }

        _backend->_sem_gnss->give();
    }

}

AP_GPS_SensorHub::AP_GPS_SensorHub(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    _new_data = false;
    _sem_gnss = hal.util->new_semaphore();
    _shub->registerHandler(&_handler);
}

bool AP_GPS_SensorHub::read(void)
{
    if (_sem_gnss->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_new_data) {
            _new_data = false;

            state = _interm_state;
            _sem_gnss->give();

            return true;
        }

        _sem_gnss->give();
    }
    return false;
}
#endif
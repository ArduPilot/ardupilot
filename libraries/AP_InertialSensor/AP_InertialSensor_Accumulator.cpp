#include "AP_InertialSensor.h"

AP_InertialSensor::DeltaAccumulator::IMUDeltas AP_InertialSensor::DeltaAccumulator::pop_imu_deltas() {
    WITH_SEMAPHORE(_sem);
    IMUDeltas ret = _deltas;

    _deltas.del_ang_dt = 0;
    _deltas.del_vel_dt = 0;
    _deltas.del_quat = Quaternion();
    _deltas.del_vel = Vector3f();

    return ret;
}

void AP_InertialSensor::accumulate_del_ang(uint8_t instance) {
    auto accumulator = _accumulator_list_head[instance];

    for (; accumulator != nullptr; accumulator = accumulator->_next) {
        Vector3f del_ang;
        if (get_delta_angle(instance, del_ang)) {
            WITH_SEMAPHORE(accumulator->_sem);
            accumulator->_deltas.del_quat.rotate(del_ang);
            accumulator->_deltas.del_ang_dt += get_delta_angle_dt(instance);
        }
    }
}

void AP_InertialSensor::accumulate_del_vel(uint8_t instance) {
    auto accumulator = _accumulator_list_head[instance];

    for (; accumulator != nullptr; accumulator = accumulator->_next) {
        Vector3f del_vel;
        if (get_delta_velocity(instance, del_vel)) {
            Matrix3f deltaRotMat;
            WITH_SEMAPHORE(accumulator->_sem);
            accumulator->_deltas.del_quat.rotation_matrix(deltaRotMat);
            accumulator->_deltas.del_vel += deltaRotMat * del_vel;
            accumulator->_deltas.del_vel_dt += get_delta_velocity_dt(instance);
        }
    }
}

void AP_InertialSensor::register_delta_accumulator(uint8_t instance, DeltaAccumulator& accumulator) {
    accumulator._next = _accumulator_list_head[instance];
    _accumulator_list_head[instance] = &accumulator;
}

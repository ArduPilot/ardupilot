#include <AP_HAL/AP_HAL.h>
#include "AP_PM_SNGCJA5.h"
#include <stdio.h>
#include <utility>
#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/Location.h>
#include <AP_Vehicle/AP_Vehicle.h>      
#include <AP_Logger/AP_Logger.h>  

extern const AP_HAL::HAL& hal;

#define SNGCJA5_I2C_ADDRESS 0x33

#define SNGCJA5_STATUS 0x26

#define SNGCJA5_PM1_0_LL 0x00
#define SNGCJA5_PM1_0_LH 0x01
#define SNGCJA5_PM1_0_HL 0x02
#define SNGCJA5_PM1_0_HH 0x03

#define SNGCJA5_PM2_5_LL 0x04
#define SNGCJA5_PM2_5_LH 0x05
#define SNGCJA5_PM2_5_HL 0x06
#define SNGCJA5_PM2_5_HH 0x07

#define SNGCJA5_PM10_0_LL 0x08
#define SNGCJA5_PM10_0_LH 0x09
#define SNGCJA5_PM10_0_HL 0x0A
#define SNGCJA5_PM10_0_HH 0x0B

#define SNGCJA5_PC_0_5_L 0x0C
#define SNGCJA5_PC_0_5_H 0x0D

#define SNGCJA5_PC_1_0_L 0x0E
#define SNGCJA5_PC_1_0_H 0x0F

#define SNGCJA5_PC_2_5_L 0x10
#define SNGCJA5_PC_2_5_H 0x11

#define SNGCJA5_PC_5_0_L 0x14
#define SNGCJA5_PC_5_0_H 0x15

#define SNGCJA5_PC_7_5_L 0x16
#define SNGCJA5_PC_7_5_H 0x17

#define SNGCJA5_PC_10_0_L 0x18
#define SNGCJA5_PC_10_0_H 0x19

void AP_PM_SNGCJA5::init()
{
    FOREACH_I2C(i) {
        if (init(i)) {
            return;
        }
    }
    gcs().send_text(MAV_SEVERITY_INFO, "No SNGCJA5 found");
}

bool AP_PM_SNGCJA5::init(int8_t bus)
{
    dev = std::move(hal.i2c_mgr->get_device(bus, SNGCJA5_I2C_ADDRESS));
    if (!dev) {
        return false;
    }

    // read at 1Hz
    printf("Starting Particle Matter Sensor on I2C\n");

    dev->register_periodic_callback(1000000, FUNCTOR_BIND_MEMBER(&AP_PM_SNGCJA5::read_frames, void));
    return true;
}

void AP_PM_SNGCJA5::read_frames(void)
{
    WITH_SEMAPHORE(dev->get_semaphore());

    uint8_t val[1];
    if (!dev->read_registers(SNGCJA5_STATUS, val, sizeof(val))) {
        return;
    }

    uint8_t ALL_REGS[24];
    if (!dev->read_registers(SNGCJA5_PM1_0_LL, ALL_REGS, sizeof(ALL_REGS))) {
        return;
    }
    double  PM1_0  = (ALL_REGS[0] | ALL_REGS[1] << 8 | ALL_REGS[2] << 16 | ALL_REGS[3] << 24);
    double  PM2_5  = (ALL_REGS[4] | ALL_REGS[5] << 8 | ALL_REGS[6] << 16 | ALL_REGS[7] << 24);
    double  PM10_0 = (ALL_REGS[8] | ALL_REGS[9] << 8 | ALL_REGS[10] << 16 | ALL_REGS[11] << 24);
    int     PC0_5  = (ALL_REGS[12] | ALL_REGS[13] << 8);
    int     PC1_0  = (ALL_REGS[14] | ALL_REGS[15] << 8);
    int     PC2_5  = (ALL_REGS[16] | ALL_REGS[17] << 8);
    int     PC5_0  = (ALL_REGS[18] | ALL_REGS[19] << 8);
    int     PC7_5  = (ALL_REGS[20] | ALL_REGS[21] << 8);
    int     PC10_0 = (ALL_REGS[22] | ALL_REGS[23] << 8);

    // get current location from EKF
    Location current_loc;
    AP::ahrs_navekf().get_location(current_loc);

    AP::logger().Write("PM1", "TimeUS,lat,lon,alt,Status,PM1_0,PM2_5,PM10_0", "QLLfIfff",
                                            AP_HAL::micros64(),
                                            current_loc.lat,
                                            current_loc.lng,
                                            (double)current_loc.alt*1.0e-2f,
                                            val[0],
                                            (double)PM1_0*1.0e-3f,
                                            (double)PM2_5*1.0e-3f,
                                            (double)PM10_0*1.0e-3f
                                            );
                                        
    AP::logger().Write("PM2", "TimeUS,lat,lon,alt,PC0_5,PC1_0,PC2_5,PC5_0,PC7_5,PC10_0", "QLLfIIIIII",
                                            AP_HAL::micros64(),
                                            current_loc.lat,
                                            current_loc.lng,
                                            (double)current_loc.alt*1.0e-2f,
                                            PC0_5,
                                            PC1_0,
                                            PC2_5,
                                            PC5_0,
                                            PC7_5,
                                            PC10_0
                                            );
                                            

    // gcs().send_text(MAV_SEVERITY_INFO,"PS Status: %u", (unsigned)val[0]);
    // gcs().send_text(MAV_SEVERITY_INFO,"PS Lat: %d, Lon: %d, PC0.5: %d, PC1.0: %d, PC2.5: %d, PC5.0: %d, PC7.5: %d, PC10.0: %d", current_loc.lat, current_loc.lng, PC0_5, PC1_0, PC2_5, PC5_0, PC7_5, PC10_0);
    // gcs().send_text(MAV_SEVERITY_INFO,"PS PM1.0: %0.0lf, PM2.5: %0.0lf, PM10.0: %0.0lf", PM1_0, PM2_5, PM10_0);
}

// periodically called from vehicle code
void AP_PM_SNGCJA5::update()
{
    read_frames();
}

#include "Rover.h"
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_GPS/AP_GPS.h>
#include <stdio.h>

// setup UART at 57600 kkouer add
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL)
    {
        // that UART doesn't exist on this platform
        return;
    }
    /// begin(baudrate,Rx,Tx)
    uart->begin(9600);
    //set the protocol to SmartAudio for the customer
    //设置4串口为smartaudio 波特率9600
    AP::serialmanager().set_protocol_and_baud(2,AP_SerialManager::SerialProtocol_SmartAudio,9);
    
}

#ifdef USERHOOK_INIT
void Rover::userhook_init()
{
    //防止参数错误导致初始化失败
    // put your initialisation code here
    // this will be called once at start-up
    gcs().send_text(MAV_SEVERITY_INFO, "rover user hook init!");
    setup_uart(hal.serial(2), "uartD");
    
}
#endif

#ifdef USERHOOK_FASTLOOP
void Rover::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Rover::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Rover::userhook_MediumLoop()
{
    const RangeFinder *rngfnd = AP::rangefinder();
  
    float distance = 0;

    for (uint8_t i=0; i<rngfnd->num_sensors(); i++) {

        AP_RangeFinder_Backend *sensor = rngfnd->get_backend(i);
        if (sensor == nullptr) {
            continue;
        }
         if (sensor->has_data())
         {
            if(sensor->orientation() == 25)
            {
                distance = sensor->distance();
            }
        }

    }

    // AP::logger().Write("WTLD", "TimeUS,Lat,Lng,HAE,DPTH,Roll,Pitch,Yaw",
    //             "QLLifccC", // format: uint64_t, float
    //             AP_HAL::micros64(),
    //             current_loc.lat,
    //             current_loc.lng,
    //             gps.height_elipsoid_mm,
    //             (double)distance,
    //             ahrs.roll_sensor,
    //             ahrs.pitch_sensor,
    //             ahrs.yaw_sensor);
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Rover::userhook_SlowLoop()
{
    // put your 3.3Hz code here
     // gcs().send_text(MAV_SEVERITY_INFO, "user hook 1 hz!");
    uint8_t portNum = 2;

    // rover.runcount ++;

    // if(rover.runcount == 1)
    // {
    //     gcs().send_text(MAV_SEVERITY_INFO, "rover user hook init!");
    //     setup_uart(hal.serial(portNum), "uartD");
    // }

    //char incoming[32];
    uint8_t data;
    //uint8_t index = 0;
    int16_t numc;

    numc = hal.serial(portNum)->available();

    if(numc > 0)
        gcs().send_text(MAV_SEVERITY_INFO, "data32 length %u ", numc);


    if (numc > 0)
    {
        mavlink_data32_t p1;
        
        memset(&p1, 0, sizeof(p1));

        //防止溢出错误
        if (numc > 32)
            numc = 32;
        for (int16_t i = 0; i < numc; i++)
        {
            data = hal.serial(portNum)->read();
            if (i < numc)
            {
                p1.data[i] = (uint8_t)data;
            }
            if (i == numc - 1)
            {
                p1.type = 1;
                p1.len = numc;

                printf("sending data 32");
                mavlink_msg_data32_send(MAVLINK_COMM_1, p1.type, p1.len, p1.data);
                mavlink_msg_data32_send(MAVLINK_COMM_0, p1.type, p1.len, p1.data);
            }
        }
    }
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Rover::userhook_SuperSlowLoop()
{
    gcs().send_text(MAV_SEVERITY_INFO, "rouserhook_SuperSlowLoop!");

    // put your 1Hz code here
   


}
#endif
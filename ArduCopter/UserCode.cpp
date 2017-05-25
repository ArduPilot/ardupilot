#include "Copter.h"
#include <time.h>

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    //  put your 1Hz code here
#ifdef MQTT_ENABLED
    Location loc;
    if (ahrs.get_position(loc)) {
        char buf[100];
        char timebuf[100];
        time_t now_time;
        struct tm *t_st;
        time(&now_time);
        t_st = localtime(&now_time);
        ::sprintf(timebuf, "%04d%02d%02d%02d%02d%02d",
                  t_st->tm_year + 1900,
                  t_st->tm_mon + 1,
                  t_st->tm_mday,
                  t_st->tm_hour,
                  t_st->tm_min,
                  t_st->tm_sec);

        ::sprintf(buf,"id:\"%04d\",time:\"%s\",lat:%ld,lon:%ld,alt:%ld\n",
                  mavlink_system.sysid,
                  timebuf,
                  (long)loc.lat,
                  (long)loc.lng,
                  (long)loc.alt);
        telemetry.send_text(buf);
    } else {
        telemetry.send_text("Not ahrs found.");
    }

    mavlink_message_t msg;
    if (telemetry.recv_mavlink_message(&msg) != 0)
        {
            telemetry.send_text("Message arrived.");
            gcs_chan[0].handleMessage(&msg);
        }
#endif
}
#endif


/*
 *
 *  This test assumes you are at the LOWl demo Airport
 *
 */

#include "Waypoints.h"
#include "Navigation.h"
#include "AP_GPS_IMU.h"
#include "AP_RC.h"


AP_GPS_IMU gps;
Navigation      nav((GPS *) &gps);
AP_RC rc;

#define CH_ROLL 0
#define CH_PITCH 1
#define CH_THROTTLE 2
#define CH_RUDDER 3

#define CH3_MIN 1100
#define CH3_MAX 1900

#define REVERSE_RUDDER 1
#define REVERSE_ROLL 1
#define REVERSE_PITCH 1

int8_t did_init_home;
int16_t servo_out[4];
int16_t radio_trim[4] = {1500,1500,1100,1500};
int16_t radio_in[4];

void setup()
{
    Serial.begin(38400);
    Serial.println("Navigation test");
    Waypoints::WP location_B = {0, 0, 74260, 472586364, 113366012};
    nav.set_next_wp(location_B);
    rc.init(radio_trim);
}

void loop()
{
    delay(20);
    gps.update();
    rc.read_pwm();
    for(int y=0; y<4; y++) {
        //rc.set_ch_pwm(y, rc.input[y]); // send to Servos
        radio_in[y] = rc.input[y];
    }

    servo_out[CH_ROLL]  = ((radio_in[CH_ROLL]  - radio_trim[CH_ROLL]) * 45  * REVERSE_ROLL) / 500;
    servo_out[CH_PITCH] = ((radio_in[CH_PITCH] - radio_trim[CH_PITCH]) * 45 * REVERSE_PITCH) / 500;
    servo_out[CH_RUDDER] = ((radio_in[CH_RUDDER] - radio_trim[CH_RUDDER]) * 45 * REVERSE_RUDDER) / 500;
    servo_out[CH_THROTTLE] = (float)(radio_in[CH_THROTTLE] - CH3_MIN) / (float)(CH3_MAX - CH3_MIN) *100;
    servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], 0, 100);

    output_Xplane();
    if(gps.new_data /* && gps.fix */ ) {
        if(did_init_home == 0) {
            did_init_home = 1;
            Waypoints::WP wp = {0, 0, gps.altitude, gps.lattitude, gps.longitude};
            nav.set_home(wp);
            Serial.println("MSG init home");
        }else{
            nav.update_gps();
        }
        //print_loc();
    }
}

void print_loc()
{
    Serial.print("MSG GPS: ");
    Serial.print(nav.location.lat, DEC);
    Serial.print(", ");
    Serial.print(nav.location.lng, DEC);
    Serial.print(", ");
    Serial.print(nav.location.alt, DEC);
    Serial.print("\tDistance = ");
    Serial.print(nav.distance, DEC);
    Serial.print(" Bearing = ");
    Serial.print(nav.bearing/100, DEC);
    Serial.print(" Bearing err = ");
    Serial.print(nav.bearing_error/100, DEC);
    Serial.print(" alt err = ");
    Serial.print(nav.altitude_error/100, DEC);
    Serial.print(" Alt above home = ");
    Serial.println(nav.altitude_above_home/100, DEC);
}

void print_pwm()
{
    Serial.print("ch1 ");
    Serial.print(rc.input[CH_ROLL],DEC);
    Serial.print("\tch2: ");
    Serial.print(rc.input[CH_PITCH],DEC);
    Serial.print("\tch3 :");
    Serial.print(rc.input[CH_THROTTLE],DEC);
    Serial.print("\tch4 :");
    Serial.println(rc.input[CH_RUDDER],DEC);
}




byte buf_len = 0;
byte out_buffer[32];

void output_Xplane(void)
{
    // output real-time sensor data
    Serial.print("AAA");                                                                //      Message preamble
    output_int((int)(servo_out[CH_ROLL]*100));                          //  0	bytes 0,1
    output_int((int)(servo_out[CH_PITCH]*100));                         //  1	bytes 2,3
    output_int((int)(servo_out[CH_THROTTLE]));                          //  2	bytes 4,5
    output_int((int)(servo_out[CH_RUDDER]*100));                //  3	bytes 6,7
    output_int((int)nav.distance);                                              //  4	bytes 8,9
    output_int((int)nav.bearing_error);                                         //  5	bytes 10,11
    output_int((int)nav.altitude_error);                                //  6	bytes 12,13
    output_int(0);                                                                              //  7	bytes 14,15
    output_byte(1);                                                                             //  8	bytes 16
    output_byte(1);                                                                             //  9	bytes 17

    // print out the buffer and checksum
    // ---------------------------------
    print_buffer();
}

void output_int(int value)
{
    //buf_len += 2;
    out_buffer[buf_len++]   = value & 0xff;
    out_buffer[buf_len++]   = (value >> 8) & 0xff;
}
void output_byte(byte value)
{
    out_buffer[buf_len++]   = value;
}

void print_buffer()
{
    byte ck_a = 0;
    byte ck_b = 0;
    for (byte i = 0; i < buf_len; i++) {
        Serial.print (out_buffer[i]);
    }
    Serial.print('\r');
    Serial.print('\n');
    buf_len = 0;
}

#include <Waypoints.h>
#include <Navigation.h>
#include <AP_GPS_IMU.h>


AP_GPS_IMU gps;
Navigation      nav((GPS *) & gps);

void setup()
{
    Serial.begin(38400);
    Serial.println("Navigation test");

    Waypoints::WP location_A = {0, 0, 38.579633 * T7, -122.566265 * T7, 10000};
    Waypoints::WP location_B = {0, 0, 38.578743 * T7, -122.572782 * T7, 5000};

    long distance   = nav.get_distance(&location_A, &location_B);
    long bearing    = nav.get_bearing(&location_A, &location_B);

    Serial.print("\tDistance = ");
    Serial.print(distance, DEC);
    Serial.print(" Bearing = ");
    Serial.println(bearing, DEC);
}

void loop()
{
}

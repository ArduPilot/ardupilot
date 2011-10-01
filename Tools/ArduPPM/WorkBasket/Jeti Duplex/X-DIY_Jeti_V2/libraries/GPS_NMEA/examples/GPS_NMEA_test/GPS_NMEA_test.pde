/*
  Example of GPS NMEA library.
  Code by Jordi Muñoz and Jose Julio. DIYDrones.com

  Works with Ardupilot Mega Hardware (GPS on Serial Port1)
  and with standard ATMega168 and ATMega328 on Serial Port 0
*/

#include <GPS_NMEA.h> // NMEA GPS Library

void setup()
{
  Serial.begin(57600);
  Serial.println("GPS NMEA library test");
  GPS.Init();   // GPS Initialization
  delay(1000);
}
void loop()
{
  GPS.Read();
  if (GPS.NewData)  // New GPS data?
    {
    Serial.print("GPS:");
    Serial.print(" Time:");
    Serial.print(GPS.Time);
    Serial.print(" Fix:");
    Serial.print((int)GPS.Fix);
    Serial.print(" Lat:");
    Serial.print(GPS.Lattitude);
    Serial.print(" Lon:");
    Serial.print(GPS.Longitude);
    Serial.print(" Alt:");
    Serial.print(GPS.Altitude/1000.0);
    Serial.print(" Speed:");
    Serial.print(GPS.Ground_Speed/100.0);
    Serial.print(" Course:");
    Serial.print(GPS.Ground_Course/100.0);
    Serial.println();
    GPS.NewData = 0; // We have readed the data
    }
  delay(20);
}
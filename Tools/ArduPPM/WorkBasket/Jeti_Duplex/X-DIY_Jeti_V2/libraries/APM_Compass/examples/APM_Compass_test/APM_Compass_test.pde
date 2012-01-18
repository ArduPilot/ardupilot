/*
  Example of APM_Compass library (HMC5843 sensor).
  Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
  
  offsets displayed are calculated from the min/max values from each axis.
  rotate the compass so as to produce the maximum and minimum values
  after the offsets stop changing, edit the code to pass these offsets into
  APM_Compass.SetOffsets.
*/

#include <Wire.h>
#include <APM_Compass.h> // Compass Library
#include <AP_Math.h> // Math library

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

unsigned long timer;

void setup()
{  
  Serial.begin(38400);
  Serial.println("Compass library test (HMC5843)");
  
  APM_Compass.Init();   // Initialization
  APM_Compass.SetOrientation(APM_COMPASS_COMPONENTS_UP_PINS_FORWARD);  // set compass's orientation on aircraft
  APM_Compass.SetOffsets(0,0,0);  // set offsets to account for surrounding interference
  APM_Compass.SetDeclination(ToRad(0.0));  // set local difference between magnetic north and true north
  
  delay(1000);
  timer = millis();
}

void loop()
{
  static float min[3], max[3], offset[3];
  
  if((millis()- timer) > 100)
  {
    timer = millis();
    APM_Compass.Read();
    APM_Compass.Calculate(0,0);  // roll = 0, pitch = 0 for this example
    
    // capture min
    if( APM_Compass.Mag_X < min[0] )
        min[0] = APM_Compass.Mag_X;
    if( APM_Compass.Mag_Y < min[1] )
        min[1] = APM_Compass.Mag_Y;
    if( APM_Compass.Mag_Z < min[2] )
        min[2] = APM_Compass.Mag_Z;
        
    // capture max
    if( APM_Compass.Mag_X > max[0] )
        max[0] = APM_Compass.Mag_X;
    if( APM_Compass.Mag_Y > max[1] )
        max[1] = APM_Compass.Mag_Y;
    if( APM_Compass.Mag_Z > max[2] )
        max[2] = APM_Compass.Mag_Z;
        
    // calculate offsets
    offset[0] = -(max[0]+min[0])/2;
    offset[1] = -(max[1]+min[1])/2;
    offset[2] = -(max[2]+min[2])/2;
    
    // display all to user
    Serial.print("Heading:");
    Serial.print(ToDeg(APM_Compass.Heading));
    Serial.print("  (");
    Serial.print(APM_Compass.Mag_X);
    Serial.print(",");
    Serial.print(APM_Compass.Mag_Y);
    Serial.print(",");    
    Serial.print(APM_Compass.Mag_Z);
    Serial.print(")");

    // display offsets
    Serial.print("\t offsets(");
    Serial.print(offset[0]);
    Serial.print(",");
    Serial.print(offset[1]);
    Serial.print(",");
    Serial.print(offset[2]);
    Serial.print(")");

    Serial.println();
  }
}

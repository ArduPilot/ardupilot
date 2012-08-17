/*******************************************
*   Sample sketch that configures an HMC5883L 3 axis
*   magnetometer to continuous mode and reads back
*   the three axis of data.
*   Code compiles to a size of 1500 bytes
*   Equivalent Wire Library code compiles to 2032 bytes
*******************************************/

#include <I2C.h>

#define HMC5883L  0x1E

int x = 0;
int y = 0;
int z = 0;


void setup()
{
    I2c.begin();
    I2c.write(HMC5883L,0x02,0x00); //configure device for continuous mode
}

void loop()
{
    I2c.read(HMC5883L,0x03,6); //read 6 bytes (x,y,z) from the device
    x = I2c.receive() << 8;
    x |= I2c.receive();
    y = I2c.receive() << 8;
    y |= I2c.receive();
    z = I2c.receive() << 8;
    z |= I2c.receive();
}


/* Wire library equivalent would be this
 *
 *  //#include <Wire.h>
 *
 * #define HMC5883L  0x1E
 *
 *  int x = 0;
 *  int y = 0;
 *  int z = 0;
 *
 *
 *  void setup()
 *  {
 *  Wire.begin();
 *  Wire.beginTransmission(HMC5883L);
 *  Wire.send(0x02);
 *  Wire.send(0x00);
 *  Wire.endTransmission();
 *  }
 *
 *  void loop()
 *  {
 *  Wire.beginTransmission(HMC5883L);
 *  Wire.send(0x03);
 *  Wire.endTransmission();
 *  Wire.requestFrom(HMC5883L,6);
 *  x = Wire.receive() << 8;
 *  x |= Wire.receive();
 *  y = Wire.receive() << 8;
 *  y |= Wire.receive();
 *  z = Wire.receive() << 8;
 *  z |= Wire.receive();
 *  }
 *
 ********************************************/

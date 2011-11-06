
#include "AP_Baro_MS5611.h"

#include "WProgram.h"
#include <SPI.h>

/* For bringup: chip select tied to PJ0; arduino pin 63 */
#define CS_PORT PORTJ
#define CS_MASK 0x01;
#define CS_ASSERT  do { CS_PORT |=  CS_MASK; } while (0)
#define CS_RELEASE do { CS_PORT &= ~CS_MASK; } while (0)

AP_Baro_MS5611::AP_Baro_MS5611() {}

void AP_Baro_MS5611::init()
{
  _send_reset();
  _start_conversion_D1();
}

/* Send reset transaction. */
void AP_Baro_MS5611::_send_reset()
{
  CS_ASSERT;

  byte resetcode = 0x1E;
  byte garbage = SPI.transfer( resetcode );

  delay(3);

  CS_RELEASE;
}

void AP_Baro_MS5611::_start_conversion_D1()
{
  CS_ASSERT;

  byte conversioncode = 0x48; // D1 OSR = 4096
  byte garbage = SPI.transfer( conversioncode );
 
  CS_RELEASE;
}

void AP_Baro_MS5611::_start_conversion_D2()
{
  CS_ASSERT;

  byte conversioncode = 0x58; // D2 OSR = 4096
  byte garbage = SPI.transfer( conversioncode );

  CS_RELEASE;
}

bool AP_Baro_MS5611::_adc_read( int32_t * raw )
{
  CS_ASSERT;

  byte readcode = 0x00; // Just write 0 to read adc.
  byte garbage = SPI.transfer( readcode );
  byte b1      = SPI.transfer( 0 );
  byte b2      = SPI.transfer( 0 );
  byte b3      = SPI.transfer( 0 );

  CS_RELEASE;

  int32_t result = (((int32_t) b1 ) << 16) |
                   (((int32_t) b2 ) << 8 ) |
                   ((int32_t) b3 );

  // Result = 0 if we read before the conversion was complete
  if (result != 0) {
    *raw = result;
    return true; 
  }
  return false;
}

uint8_t AP_Baro_MS5611::update()
{
  

}

int32_t AP_Baro_MS5611::get_pressure()
{


}

float AP_Baro_MS5611::get_temp()
{


}



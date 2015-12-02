/*
  simple test of Storage API
 */

#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_HAL::Storage *st;

void setup(void) 
{
    /*
      init Storage API
     */
    hal.console->printf("Starting AP_HAL::Storage test\r\n");
    st->init();

    /*
      Calculate XOR of the full conent of memory
      Do it by block of 8 bytes
    */
    unsigned int i, j;
    unsigned char buff[8], XOR_res = 0;

    for(i = 0; i < HAL_STORAGE_SIZE; i += 8)
    {
        st->read_block((void *) buff, i, 8);
        for(j = 0; j < 8; j++)
            XOR_res ^= buff[j];
    }

    /*
      print XORed result
     */
    hal.console->printf("XORed ememory: %u\r\n", (unsigned) XOR_res);
}

// In main loop do nothing
void loop(void) 
{	
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();

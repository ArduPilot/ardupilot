#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif

uint16_t ppm_center[ 8 ];
uint16_t ppm_min[ 8 ];
uint16_t ppm_max[ 8 ];
uint16_t ppm_delta[ 8 ];
uint16_t ppm_mark[ 8 ];


void setup (void) {
    hal.console->printf_P(PSTR("RC input jitter test - Do not move sticks:\n"));
 
    hal.scheduler->delay(2000);
    
    for( uint8_t rep = 0; rep < 10; rep++ )
    {
        for( uint8_t i = 0; i < 8; i++ )
        {
            uint16_t ppm_tmp = hal.rcin->read( i ); 
            ppm_center[ i ] += ppm_tmp;
        }
        hal.scheduler->delay( 100 );        
    }

    for( uint8_t i = 0; i < 8; i++ )
    {
        uint16_t ppm_tmp = ppm_center[ i ] / 10;
        ppm_center[ i ] = ppm_tmp;
        ppm_min[ i ] = ppm_tmp;
        ppm_max[ i ] = ppm_tmp;
        ppm_delta[ i ] = 0;
    }    

}

void loop (void) {
    
    
    static bool update = true;
    static uint32_t time_trigger;

    for( uint8_t i = 0; i < 8; i++ )
    {
        uint16_t ppm_tmp = hal.rcin->read( i );
        
        // min?
        if( ppm_tmp < ppm_min[ i ] )
        {
            uint16_t delta_tmp = ppm_center[ i ] - ppm_tmp;
            if( delta_tmp > ppm_delta[ i ] ) ppm_delta[ i ] = delta_tmp;
            
            ppm_min[ i ] = ppm_tmp;
            ppm_mark[ i ] = ppm_tmp;
            
            update = true;
        }
        
        // max?
        if( ppm_tmp > ppm_max[ i ] )
        {
            uint16_t delta_tmp = ppm_tmp - ppm_center[ i ];
            if( delta_tmp > ppm_delta[ i ] ) ppm_delta[ i ] = delta_tmp;

            ppm_max[ i ] = ppm_tmp;
            ppm_mark[ i ] = ppm_tmp;
            
            update = true;
        }

    }
    
    if( update && hal.scheduler->millis() > time_trigger )
    {
        update = false;
        time_trigger = hal.scheduler->millis() + 5000;

        uint32_t time = hal.scheduler->millis() / 1000;
       
        uint16_t hour = time / 3600;
        time -= hour * 3600;

        uint16_t minutte = time / 60;
        time -= minutte * 60;
        
        uint16_t second = time;
        
        //hal.console->printf_P( PSTR("%c%c%c\r\n"), 0x1B, 0x5B, 0x48 ); // Terminal home position
        hal.console->printf_P( PSTR("\n< %.2u:%.2u:%.2u >------------------------------------- \n"), hour, minutte, second );
        for( uint8_t i = 0; i < 8; i++ )    
        {
            hal.console->printf_P( PSTR("ch%d: center:%d min:%d max:%d delta:%d"),
                                   i + 1, ppm_center[ i ], ppm_min[ i ],ppm_max[ i ], ppm_delta[ i ] );     

            if( ppm_mark[ i ] > 0 )
            {
                hal.console->printf_P( PSTR(" [%d]"), ppm_mark[ i ] );
                ppm_mark[ i ] = 0;
            }

            hal.console->printf_P( PSTR("\n") );
        }
    }

    hal.scheduler->delay( 10 );
}

AP_HAL_MAIN();


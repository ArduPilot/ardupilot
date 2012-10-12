// -------------------------------------------------------------
// PPM ENCODER V3.0.0 (12-10-2012)
// -------------------------------------------------------------
// By: John Arne Birkeland - 2012
// By Olivier ADLER : PPM redundancy mode - APM v1.x adaptation and "difficult" receiver testing - 2012
// 
// -------------------------------------------------------------
// See changelog_v3 for a complete descrition of changes
// -------------------------------------------------------------
//
// 12-10-2012
// V3.0.0 - Added dual input PPM redundancy mode with auto switchover. This is mainly for dual PPM receivers setup.
//			This mode Can be used as well if a PPM conversion is needed (Futaba 16 channels 760us mode to APM mode)
// -------------------------------------------------------------

#ifndef _PPM_ENCODER_H_
#define _PPM_ENCODER_H_

#include <avr/io.h>

// -------------------------------------------------------------

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>


#ifndef F_CPU
#define F_CPU               16000000UL
#endif

#ifndef true
#define true                1
#endif

#ifndef false
#define false               0
#endif

#ifndef bool
#define bool                _Bool
#endif

// -------------------------------------------------------------
// INPUT MODE (by jumper selection)
// -------------------------------------------------------------

#define JUMPER_SELECT_MODE    0    // Default - PPM passtrough mode selected if channel 2&3 shorted. Normal servo input (pwm) if not shorted.
#define SERVO_PWM_MODE        1    // Normal 8 channel servo (pwm) input
#define PPM_PASSTROUGH_MODE   2    // PPM signal passtrough on channel 1
#define PPM_REDUNDANCY_MODE   3    // PPM redundancy on channels 1 and 2
#define SPEKTRUM_MODE         4    // Spektrum satelitte on channel 1 (reserved but not implemented yet)

volatile uint8_t servo_input_mode = JUMPER_SELECT_MODE;

// -------------------------------------------------------------
// PPM REDUNDANCY MODE SETTINGS
// -------------------------------------------------------------

#define _TEACHER_STUDENT_MODE_			// Enable teacher student mode. See manual.
#define TEACHER_STUDENT_CHANNEL		8	// PPM Channel to force receiver 2

#define RECEIVER_1_FRAME_FORMAT		0	// 0 for standard PPM, 1 for PPMv2 (Futaba 760 us 16 Channels), 2 for PPMv3 (Jeti 1050 us 16 channels), 3 for Hitec 9 channels
#define RECEIVER_2_FRAME_FORMAT		0	// See manual for details

#define SWITCHOVER_1_to_2_DELAY_MS		50	// Delay for switching to receiver 2
#define SWITCHOVER_2_to_1_DELAY_MS		200	// Delay for switching back to receiver 1

// -------------------------------------------------------------
// PWM INPUT FILTERS
// -------------------------------------------------------------
// Using both filters is not recommended and may reduce servo input resolution

// #define _AVERAGE_FILTER_         // Average filter to smooth servo input capture jitter
// #define _JITTER_FILTER_             // Cut filter to remove 0,5us servo input capture jitter

// -------------------------------------------------------------
// GLOBAL SETTINGS
// -------------------------------------------------------------

// Number of Timer1 ticks for 1 microsecond
#define ONE_US                F_CPU / 8 / 1000 / 1000

// -------------------------------------------------------------
// PWM inputs used channels, limits and center positions
// -------------------------------------------------------------

// Number of input PWM channels
#define SERVO_CHANNELS        8

// Servo minimum position
#define PWM_SERVO_MIN         ONE_US * 900 - PPM_PRE_PULSE

// Servo center position
#define PPM_SERVO_CENTER      ONE_US * 1500 - PPM_PRE_PULSE

// Servo maximum position
#define PWM_SERVO_MAX         ONE_US * 2100 - PPM_PRE_PULSE


// -------------------------------------------------------------
// PPM output default values
// -------------------------------------------------------------

// Throttle default at power on
#define PPM_THROTTLE_DEFAULT  ONE_US * 1100 - PPM_PRE_PULSE

// Throttle during failsafe
#define PPM_THROTTLE_FAILSAFE ONE_US * 900 - PPM_PRE_PULSE

// CH5 power on values (mode selection channel)
#define PPM_CH5_MODE_4        ONE_US * 1555 - PPM_PRE_PULSE

// -------------------------------------------------------------
// PPM output frame format
// -------------------------------------------------------------

// Number of output PPM channels
#define PPM_CHANNELS        8

// 400us PPM pre pulse
#define PPM_PRE_PULSE         ONE_US * 400

// PPM period 18.5ms - 26.5ms (54hz - 37Hz) 
#define PPM_PERIOD            ONE_US * ( 22500 - ( PPM_CHANNELS * 1500 ) )

// Size of ppm[..] data array
#define PPM_ARRAY_MAX         PPM_CHANNELS * 2 + 2

// -------------------------------------------------------------
// PPM output frame variables
// -------------------------------------------------------------

// Data array for storing ppm (8 channels) pulse widths.

volatile uint16_t ppm[ PPM_ARRAY_MAX ] =                                
{
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 1 
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 2
    PPM_PRE_PULSE,
    PPM_THROTTLE_DEFAULT,     // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CH5_MODE_4,           // Channel 5
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 6
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 7
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_PERIOD
};

// SERVO FAILSAFE VALUES

const uint16_t failsafe_ppm[ PPM_ARRAY_MAX ] =                               
{
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 1
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 2
    PPM_PRE_PULSE,
    PPM_THROTTLE_FAILSAFE,    // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CH5_MODE_4,           // Channel 5
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 6
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 7
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_PERIOD
};
// -------------------------------------------------------------

// AVR parameters for PhoneDrone and APM2 boards using ATmega32u2
#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)

#define SERVO_DDR             DDRB
#define SERVO_PORT            PORTB
#define SERVO_INPUT           PINB
#define SERVO_INT_VECTOR      PCINT0_vect
#define SERVO_INT_MASK        PCMSK0
#define SERVO_INT_CLEAR_FLAG  PCIF0
#define SERVO_INT_ENABLE      PCIE0
#define SERVO_TIMER_CNT       TCNT1

#define PPM_DDR               DDRC
#define PPM_PORT              PORTC
#define PPM_OUTPUT_PIN        PC6
#define PPM_INT_VECTOR        TIMER1_COMPA_vect
#define PPM_COMPARE           OCR1A
#define PPM_COMPARE_FLAG      COM1A0
#define PPM_COMPARE_ENABLE    OCIE1A

#define    USB_DDR            DDRC
#define USB_PORT              PORTC
#define    USB_PIN            PC2

// true if we have received a USB device connect event
static bool usb_connected;

// USB connected event
void EVENT_USB_Device_Connect(void)
{
    // Toggle USB pin high if USB is connected
    USB_PORT |= (1 << USB_PIN);

    usb_connected = true;

    // this unsets the pin connected to PA1 on the 2560
    // when the bit is clear, USB is connected
    PORTD &= ~1;
}

// USB disconnect event
void EVENT_USB_Device_Disconnect(void)
{
    // toggle USB pin low if USB is disconnected
    USB_PORT &= ~(1 << USB_PIN);

    usb_connected = false;

    // this sets the pin connected to PA1 on the 2560
    // when the bit is clear, USB is connected
    PORTD |= 1;
}

// AVR parameters for ArduPilot MEGA v1.4 PPM Encoder (ATmega328P)
#elif defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)

#define SERVO_DDR             DDRD
#define SERVO_PORT            PORTD
#define SERVO_INPUT           PIND
#define SERVO_INT_VECTOR      PCINT2_vect
#define SERVO_INT_MASK        PCMSK2
#define SERVO_INT_CLEAR_FLAG  PCIF2
#define SERVO_INT_ENABLE      PCIE2
#define SERVO_TIMER_CNT       TCNT1

#define PPM_DDR               DDRB
#define PPM_PORT              PORTB
#define PPM_OUTPUT_PIN        PB2
#define PPM_INT_VECTOR        TIMER1_COMPB_vect
#define PPM_COMPARE           OCR1B
#define PPM_COMPARE_FLAG      COM1B0
#define PPM_COMPARE_ENABLE    OCIE1B

#else
#error NO SUPPORTED DEVICE FOUND! (ATmega16u2 / ATmega32u2 / ATmega328p)
#endif

// Used to indicate invalid SERVO input signals
volatile uint8_t servo_input_errors = 0;

// Used to indicate missing SERVO input signals
volatile bool servo_input_missing = true;

// Used to indicate if PPM generator is active
volatile bool ppm_generator_active = false;

// Used to indicate a brownout restart
volatile bool brownout_reset = false;

// ------------------------------------------------------------------------------
// PPM GENERATOR START - TOGGLE ON COMPARE INTERRUPT ENABLE
// ------------------------------------------------------------------------------
void ppm_start( void )
{
        // Prevent reenabling an already active PPM generator
        if( ppm_generator_active ) return;
        
        // Store interrupt status and register flags
        uint8_t SREG_tmp = SREG;

        // Stop interrupts
        cli();


        // Make sure initial output state is low
        PPM_PORT &= ~(1 << PPM_OUTPUT_PIN);

        // Wait for output pin to settle
        //_delay_us( 1 );

        // Set initial compare toggle to maximum (32ms) to give other parts of the system time to start
        SERVO_TIMER_CNT = 0;
        PPM_COMPARE = 0xFFFF;

        // Set toggle on compare output
        TCCR1A = (1 << PPM_COMPARE_FLAG);

        // Set TIMER1 8x prescaler
        TCCR1B = ( 1 << CS11 );

        // Enable output compare interrupt
        TIMSK1 |= (1 << PPM_COMPARE_ENABLE);

        // Indicate that PPM generator is active
        ppm_generator_active = true;

        // Restore interrupt status and register flags
        SREG = SREG_tmp;
		
		#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)		
		// Turn on TX led if PPM generator is active
		PORTD &= ~( 1<< PD5 );
		#endif
}

// ------------------------------------------------------------------------------
// PPM GENERATOR STOP - TOGGLE ON COMPARE INTERRUPT DISABLE
// ------------------------------------------------------------------------------
void ppm_stop( void )
{
        // Store interrupt status and register flags
        uint8_t SREG_tmp = SREG;

        // Stop interrupts
        cli();

        // Disable output compare interrupt
        TIMSK1 &= ~(1 << PPM_COMPARE_ENABLE);

        // Reset TIMER1 registers
        TCCR1A = 0;
        TCCR1B = 0;

        // Indicate that PPM generator is not active
        ppm_generator_active = false;

        // Restore interrupt status and register flags
        SREG = SREG_tmp;
		
		#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)		
		// Turn off TX led if PPM generator is off
		PORTD |= ( 1<< PD5 );
		#endif
}

// ------------------------------------------------------------------------------
// Watchdog Interrupt (interrupt only mode, no reboot)
// ------------------------------------------------------------------------------
ISR( WDT_vect ) // If watchdog is triggered then enable missing signal flag and copy failsafe values
{
    // Use failsafe values if PPM generator is active or if chip has been reset from a brown-out
    if ( ppm_generator_active || brownout_reset )
    {
        // Copy failsafe values to ppm[..]
        for ( uint8_t i = 0; i < PPM_ARRAY_MAX; i++ )
        {
            ppm[ i ] = failsafe_ppm[ i ];
        }

	}

    // If we are in PPM passtrough mode and a input signal has been detected, or if chip has been reset from a brown_out then start the PPM generator.
    if( ( servo_input_mode == PPM_PASSTROUGH_MODE && servo_input_missing == false ) || brownout_reset )
    {
        // Start PPM generator
        ppm_start();
        brownout_reset = false;
	}

    // Set missing receiver signal flag
    servo_input_missing = true;
    
    // Reset servo input error flag
    servo_input_errors = 0;
		
	#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)		
	// Turn on RX led if failsafe has triggered after ppm generator i active
	if( ppm_generator_active )  PORTD &= ~( 1<< PD4 );
	#endif
}
// ------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
// SERVO/PPM INPUT - PIN CHANGE INTERRUPT
// ------------------------------------------------------------------------------
ISR( SERVO_INT_VECTOR )
{
    // Servo pulse start timing
    static uint16_t servo_start[ SERVO_CHANNELS ] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
	// Toggle LED delay
	static uint8_t led_delay = 0;
	#endif
 
	// Missing throttle signal failsafe
	static uint8_t throttle_timeout = 0;
 
	// Servo input pin storage 
    static uint8_t servo_pins_old = 0;

    // Used to store current servo input pins
    uint8_t servo_pins;

    // Read current servo pulse change time
    uint16_t servo_time = SERVO_TIMER_CNT;

    // ------------------------------------------------------------------------------
    // PPM passtrough mode ( signal passtrough from channel 1 to ppm output pin)
    // ------------------------------------------------------------------------------
    if( servo_input_mode == PPM_PASSTROUGH_MODE )
    {
        // Has watchdog timer failsafe started PPM generator? If so we need to stop it.
        if( ppm_generator_active )
        {
            // Stop PPM generator
            ppm_stop();
        }
        
        // PPM (channel 1) input pin is high
        if( SERVO_INPUT & 1 ) 
        {
            // Set PPM output pin high
            PPM_PORT |= (1 << PPM_OUTPUT_PIN);
        }
        // PPM (channel 1) input pin is low
        else
        {
            // Set PPM output pin low
            PPM_PORT &= ~(1 << PPM_OUTPUT_PIN);
        }

        // Reset Watchdog Timer
        wdt_reset(); 

        // Set servo input missing flag false to indicate that we have received servo input signals
        servo_input_missing = false;

		#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
		// Toggle TX LED at PPM passtrough
		if( ++led_delay > 128 ) // Toggle every 128th pulse
		{
			// Toggle TX led
			PIND |= ( 1<< PD5 ); 
			led_delay = 0;
		}
		#endif
		
        // Leave interrupt
        return;
    }

    // ------------------------------------------------------------------------------
    // SERVO PWM MODE
    // ------------------------------------------------------------------------------
CHECK_PINS_START: // Start of servo input check

    // Store current servo input pins
    servo_pins = SERVO_INPUT;

    // Calculate servo input pin change mask
    uint8_t servo_change = servo_pins ^ servo_pins_old;

    // Set initial servo pin and channel
    uint8_t servo_pin = 1;
    uint8_t servo_channel = 0;

CHECK_PINS_LOOP: // Input servo pin check loop

    // Check for pin change on current servo channel
    if( servo_change & servo_pin )
    {
        // High (raising edge)
        if( servo_pins & servo_pin )
        {
            servo_start[ servo_channel ] = servo_time;
        }
        else
        {
            
            // Get servo pulse width
            uint16_t servo_width = servo_time - servo_start[ servo_channel ] - PPM_PRE_PULSE;
            
            // Check that servo pulse signal is valid before sending to ppm encoder
            if( servo_width > PWM_SERVO_MAX ) goto CHECK_PINS_ERROR;
            if( servo_width < PWM_SERVO_MIN ) goto CHECK_PINS_ERROR;

            // Calculate servo channel position in ppm[..]
            uint8_t _ppm_channel = ( servo_channel << 1 ) + 1;

			//Reset throttle failsafe timeout
			if( _ppm_channel == 5 ) throttle_timeout = 0;

        #ifdef _AVERAGE_FILTER_
            // Average filter to smooth input jitter
            servo_width += ppm[ _ppm_channel ];
            servo_width >>= 1;
        #endif

        #ifdef _JITTER_FILTER_
            // 0.5us cut filter to remove input jitter
            int16_t ppm_tmp = ppm[ _ppm_channel ] - servo_width;
            if( ppm_tmp == 1 ) goto CHECK_PINS_NEXT;
            if( ppm_tmp == -1 ) goto CHECK_PINS_NEXT;
        #endif

            // Update ppm[..]
            ppm[ _ppm_channel ] = servo_width;
        }
    }
    
CHECK_PINS_NEXT:

    // Select next servo pin
    servo_pin <<= 1;

    // Select next servo channel
    servo_channel++;
	
    // Check channel and process if needed
    if( servo_channel < SERVO_CHANNELS ) goto CHECK_PINS_LOOP;
    
    goto CHECK_PINS_DONE;

CHECK_PINS_ERROR:
    
    // Used to indicate invalid servo input signals
    servo_input_errors++;
	
	#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
	// Delay LED toggle
	led_delay = 0;
	#endif

    goto CHECK_PINS_NEXT;
    
    // All servo input pins has now been processed

CHECK_PINS_DONE:
    
    // Reset Watchdog Timer
    wdt_reset(); 

    // Set servo input missing flag false to indicate that we have received servo input signals
    servo_input_missing = false;

    // Store current servo input pins for next check
    servo_pins_old = servo_pins;

    // Start PPM generator if not already running
    if( ppm_generator_active == false ) ppm_start();


	#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
	// Toggle RX LED when finished receiving servo pulses
	if( ++led_delay > 64 ) // Toggle led every 64th time
	{
		PIND |= ( 1<< PD4 );
		led_delay = 0;
	}
	#endif	
	
	// Throttle failsafe
	if( throttle_timeout++ >= 128 )
	{
		// Reset throttle timeout
		throttle_timeout = 0;
		// Set throttle failsafe value
		ppm[ 5 ] = PPM_THROTTLE_FAILSAFE;
	}
	
    //Has servo input changed while processing pins, if so we need to re-check pins
    if( servo_pins != SERVO_INPUT ) goto CHECK_PINS_START;

    // Clear interrupt event from already processed pin changes
    PCIFR |= (1 << SERVO_INT_CLEAR_FLAG);
}
// ------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
// PPM OUTPUT - TIMER1 COMPARE INTERRUPT
// ------------------------------------------------------------------------------
ISR( PPM_INT_VECTOR )  
{
    // Current active ppm channel
    static uint8_t ppm_channel = PPM_ARRAY_MAX - 1;

    // Update timing for next PPM output pin toggle
    PPM_COMPARE += ppm[ ppm_channel ];

    // Select the next ppm channel
    if( ++ppm_channel >= PPM_ARRAY_MAX ) 
	{
		ppm_channel = 0;

		#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
		// Blink TX LED when PPM generator has finished a pulse train
		PIND |= ( 1<< PD5 );
		#endif
	}
}
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
// PPM READ - INTERRUPT SAFE PPM SERVO CHANNEL READ
// ------------------------------------------------------------------------------
uint16_t ppm_read_channel( uint8_t channel )
{
    // Limit channel to valid value
    uint8_t _channel = channel;
    if( _channel == 0 ) _channel = 1;
    if( _channel > SERVO_CHANNELS ) _channel = SERVO_CHANNELS;

    // Calculate ppm[..] position
    uint8_t ppm_index = ( _channel << 1 ) + 1;
    
    // Read ppm[..] in a non blocking interrupt safe manner
    uint16_t ppm_tmp = ppm[ ppm_index ];
    while( ppm_tmp != ppm[ ppm_index ] ) ppm_tmp = ppm[ ppm_index ];

    // Return as normal servo value
    return ppm_tmp + PPM_PRE_PULSE;    
}
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
// PPM ENCODER INIT
// ------------------------------------------------------------------------------
void ppm_encoder_init( void )
{
    // ATmegaXXU2 only init code
    // ------------------------------------------------------------------------------    
    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
        // ------------------------------------------------------------------------------    
        // Reset Source check
        // ------------------------------------------------------------------------------
        if (MCUSR & 1)    // Power-on Reset
        {
            MCUSR=0; // Clear MCU Status register
            // custom code here
        }
        else if (MCUSR & 2)    // External Reset
        {
           MCUSR=0; // Clear MCU Status register
           // custom code here
        }
        else if (MCUSR & 4)    // Brown-Out Reset
        {
           MCUSR=0; // Clear MCU Status register
           brownout_reset=true;
        }
        else    // Watchdog Reset
        {
           MCUSR=0; // Clear MCU Status register
           // custom code here
        }

        // APM USB connection status UART MUX selector pin
        // ------------------------------------------------------------------------------
        USB_DDR |= (1 << USB_PIN); // Set USB pin to output
    #endif
     

    // USE JUMPER TO CHECK FOR INPUT MODE (pin 2&3 or 3&4 shorted)
    // ------------------------------------------------------------------------------
    if( servo_input_mode == JUMPER_SELECT_MODE )
    {
        // channel 3 status counter
        uint8_t channel_2_status = 0;
		uint8_t channel_4_status = 0;
		
		// Set channel 2 to input
        SERVO_DDR &= ~(1 << 1);
		// Enable channel 2 pullup
        SERVO_PORT |= (1 << 1);
		
		// Set channel 4 to input
        SERVO_DDR &= ~(1 << 3);
		// Enable channel 4 pullup
        SERVO_PORT |= (1 << 3);
		
		
		// Set channel 3 to output
        SERVO_DDR |= (1 << 2);
        // Set channel 3 output low
        SERVO_PORT &= ~(1 << 2);
		
		_delay_us (10);
        		
				
        // Increment channel_2_status if channel 2 is set low by channel 3
        if( ( SERVO_INPUT & (1 << 1) ) == 0 ) channel_2_status++;
		// Increment channel_4_status if channel 4 is set low by channel 3
        if( ( SERVO_INPUT & (1 << 3) ) == 0 ) channel_4_status++;

        // Set channel 3 output high
        SERVO_PORT |= (1 << 2);
        
        _delay_us (10);
        
        // Increment channel_2_status if channel 2 is set high by channel 3
        if( ( SERVO_INPUT & (1 << 1) ) != 0 ) channel_2_status++;
		// Increment channel_4_status if channel 4 is set high by channel 3
		if( ( SERVO_INPUT & (1 << 3) ) != 0 ) channel_4_status++;

        // Set channel 3 output low
        SERVO_PORT &= ~(1 << 2);

        _delay_us (10);

        // Increment channel_2_status if channel 2 is set low by channel 3
        if( ( SERVO_INPUT & (1 << 1) ) == 0 ) channel_2_status++;
		// Increment channel_4_status if channel 4 is set low by channel 3
        if( ( SERVO_INPUT & (1 << 3) ) == 0 ) channel_4_status++;
		
		
        // Set servo input mode based on channel_2_status
        if( channel_2_status == 3 ) servo_input_mode = PPM_PASSTROUGH_MODE;
		if( channel_4_status == 3 ) servo_input_mode = PPM_REDUNDANCY_MODE;
        else servo_input_mode = SERVO_PWM_MODE;

    }

    // RESET SERVO/PPM PINS AS INPUTS WITH PULLUPS
    // ------------------------------------------------------------------------------
    SERVO_DDR = 0;
    SERVO_PORT |= 0xFF;

#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
    // on 32U2 set PD0 to be an output, and clear the bit. This tells
    // the 2560 that USB is connected. The USB connection event fires
    // later to set the right value
    DDRD |= 1;
    if (usb_connected) {
	    PORTD     &= ~1;
    } else {
	    PORTD     |= 1;
    }
#endif

    // SERVO/PPM INPUT - PIN CHANGE INTERRUPT
    // ------------------------------------------------------------------------------
    if( servo_input_mode == SERVO_PWM_MODE )
    {
        // Set servo input interrupt pin mask to all 8 servo input channels
        SERVO_INT_MASK = 0xFF;
    }

    if( servo_input_mode == PPM_PASSTROUGH_MODE )
    {
        // Set servo input interrupt pin mask to servo input channel 1
        SERVO_INT_MASK = 0x01;
    }
    
    // Enable servo input interrupt
    PCICR |= (1 << SERVO_INT_ENABLE);

    // PPM OUTPUT PIN
    // ------------------------------------------------------------------------------
    // Set PPM pin to output
    PPM_DDR |= (1 << PPM_OUTPUT_PIN);

    // ------------------------------------------------------------------------------
    // Enable watchdog interrupt mode
    // ------------------------------------------------------------------------------
    // Disable watchdog
    wdt_disable();
     // Reset watchdog timer
    wdt_reset();
     // Start timed watchdog setup sequence
    WDTCSR |= (1<<WDCE) | (1<<WDE );
    // Set 250 ms watchdog timeout and enable interrupt
    WDTCSR = (1<<WDIE) | (1<<WDP2);
	
	
	
}
// ------------------------------------------------------------------------------

#endif // _PPM_ENCODER_H_


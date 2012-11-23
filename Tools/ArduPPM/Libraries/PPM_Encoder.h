// -------------------------------------------------------------
// ArduPPM (PPM Encoder) V2.3.12
// -------------------------------------------------------------
// Improved servo to ppm for ArduPilot MEGA v1.x (ATmega328p),
// PhoneDrone and APM2.x (ATmega32u2)

// By: John Arne Birkeland - 2012
// APM v1.x adaptation and "difficult" receiver testing by Olivier ADLER
// -------------------------------------------------------------


// -------------------------------------------------------------
// ARDUPPM OPERATIONAL DESCRIPTION
// -------------------------------------------------------------

// APM 2.x LED STATUS:
// -------------------
// RX - OFF         = No input signal detected
// RX - SLOW TOGGLE = Input signal OK
// RX - FAST TOGGLE = Invalid input signal(s) detected
// RX - ON          = Input signal(s) lost during flight and fail-safe activated
// TX - OFF         = PPM output disabled
// TX - FAST TOGGLE = PPM output enabled
// TX - SLOW TOGGLE = PPM pass-trough mode

// SERVO INPUT (PWM) MODE:
// -----------------------
// - PPM output will not be enabled unless a input signal has been detected and verified
// - Verified inputs are lost during operaton (lose servo wire or receiver malfunction):
//   + The PPM output channel for the lost input will be set to the default fail-safe value
//   + PPM throttle output (ch3) will be permanently set to fail-safe (900us)
// - Lost channel signal is restored:
//   + PPM output for the restored channel will be updated with the valid signal
//   + PPM throttle output (ch3) will not be restored, and will continue to output fail-safe (900us)

// PPM PASS-THROUGH MODE (signal pin 2&3 shorted):
// -----------------------------------------------
// - PPM output will not be enabled unless a input signal has been detected
// - Active signal on input channel 1 has been detected:
//   + Any input level changes will be passed directly to the PPM output (PPM pass-trough)
//   + If no input level changes are detected withing 250ms:
//     + PPM output is enabled and default fail-safe values for all eight channels transmitted
//     + Input level change detected again, PPM fail-safe output is terminated and normal PPM pass-through operation is restored

// Changelog:

// 01-08-2011
// V2.2.3 - Changed back to BLOCKING interrupts.
//          Assembly PPM compare interrupt can be switch back to non-blocking, but not recommended.
// V2.2.3 - Implemented 0.5us cut filter to remove servo input capture jitter.

// 04-08-2011
// V2.2.4 - Implemented PPM passtrough funtion.
//          Shorting channel 2&3 enabled ppm passtrough on channel 1.

// 04-08-2011
// V2.2.5 - Implemented simple average filter to smooth servo input capture jitter.
//          Takes fewer clocks to execute and has better performance then cut filter.

// 05-08-2011
// V2.2.51 - Minor bug fixes.

// 06-08-2011
// V2.2.6 - PPM passtrough failsafe implemented.
//          The PPM generator will be activated and output failsafe values while ppm passtrough signal is missing.

// 01-09-2011
// V2.2.61 - Temporary MUX pin always high patch for APM beta board

// 22-09-2011
// V2.2.62 - ATmegaXXU2 USB connection status pin (PC2) for APM UART MUX selection (removed temporary high patch)
//         - Removed assembly optimized PPM generator (not usable for production release)

// 23-09-2011
// V2.2.63 - Average filter disabled

// 24-09-2011
// V2.2.64 - Added distincts Power on / Failsafe PPM values
//         - Changed CH5 (mode selection) PPM Power on and Failsafe values to 1555 (Flight mode 4)
//         - Added brownout detection : Failsafe values are copied after a brownout reset instead of power on values

// 25-09-2011
// V2.2.65 - Implemented PPM output delay until input signal is detected (PWM and PPM pass-trough mode)
//         - Changed brownout detection and FailSafe handling to work with XXU2 chips
//         - Minor variable and define naming changes to enhance readability

// 15-03-2012
// V2.2.66  - Added APM2 (ATmega32U2) support for using TX and RX status leds to indicate PWM and PPM traffic 
//            - <RX>: <OFF> = no pwm input detected, <TOGGLE> = speed of toggle indicate how many channel are active, <ON> = input lost (failsafe)
//            - <TX>: <OFF> = ppm output not started, <FAST TOGGLE> = normal PWM->PPM output or PPM passtrough failsafe, <SLOW TOGGLE> = PPM passtrough

// 03-06-2012
// V2.2.67 - Implemented detection and failsafe (throttle = 900us) for missing throttle signal. 

// 04-06-2012
// V2.2.68 - Fixed possible logic flaw in throttle failsafe reset if  _JITTER_FILTER_ is enabled

// 02-11-2012
// V2.2.69 - Added PPM output positive polarity - mainly for standalone PPM encoder board use

// 03-11-2012
// V2.3.0 - Implemented single channel fail-safe detection for all inputs

// 16-11-2012
// V2.3.1 - Improved watchdog timer reset, so that only valid input signals will prevent the watchdog timer from triggering

// 22-11-2012
// V2.3.11 - Very experimental test forcing throttle fail-safe (RTL) on single channel loss. !DO NOT RELEASE TO PUBLIC!
//         - Test for active input channels during init

// 23-11-2012 !VERY EXPERIMENTAL! 
// V2.3.11 - Nested interrupt PWM output interrupt (PPM generator) for greatly improved jitter performance
//         - Active input channel test during init removed
//         - Implemented dynamic testing of active input channels

// 23-12-2012
// V2.3.12 - New improved fail-safe detection and handeling for single or multible signal loss and receiver malfuntion
//         - Improved LED status for APM 2.x
//         - Improved jitter performance (PPM output using nested interrupts)

// -------------------------------------------------------------

#ifndef _PPM_ENCODER_H_
#define _PPM_ENCODER_H_

#include <avr/io.h>

// -------------------------------------------------------------

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>


// -------------------------------------------------------------
// SERVO INPUT FILTERS AND PARAMETERS
// -------------------------------------------------------------
// Using both filters is not recommended and may reduce servo input resolution

//#define _AVERAGE_FILTER_            // Average filter to smooth servo input capture jitter
#define _JITTER_FILTER_ 2           // Cut filter to remove servo input capture jitter (1 unit = 0.5us)
#define _INPUT_ERROR_TRIGGER_ 100   // Number of invalid input signals allowed before triggering alarm
// -------------------------------------------------------------

#ifndef F_CPU
#define F_CPU             16000000UL
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

// Version stamp for firmware hex file ( decode hex file using <avr-objdump -s file.hex> and look for "ArduPPM" string )
const char ver[15] = "ArduPPMv2.3.12"; 

// -------------------------------------------------------------
// INPUT MODE
// -------------------------------------------------------------

#define JUMPER_SELECT_MODE    0    // Default - PPM passtrough mode selected if channel 2&3 shorted. Normal servo input (pwm) if not shorted.
#define SERVO_PWM_MODE        1    // Normal 8 channel servo (pwm) input
#define PPM_PASSTROUGH_MODE   2    // PPM signal passtrough on channel 1
#define JETI_MODE             3    // Reserved
#define SPEKTRUM_MODE         4    // Reserved for Spektrum satelitte on channel 1

// Servo input mode (jumper (default), pwm, ppm, jeti or spektrum)
volatile uint8_t servo_input_mode = JUMPER_SELECT_MODE;
// -------------------------------------------------------------

// Number of Timer1 ticks in one microsecond
#define ONE_US                F_CPU / 8 / 1000 / 1000

// 400us PPM pre pulse
#define PPM_PRE_PULSE         ONE_US * 400


// -------------------------------------------------------------
// SERVO LIMIT VALUES
// -------------------------------------------------------------

// Servo minimum position
#define PPM_SERVO_MIN         ONE_US * 900 - PPM_PRE_PULSE

// Servo center position
#define PPM_SERVO_CENTER      ONE_US * 1500 - PPM_PRE_PULSE

// Servo maximum position
#define PPM_SERVO_MAX         ONE_US * 2100 - PPM_PRE_PULSE

// Throttle default at power on
#define PPM_THROTTLE_DEFAULT  ONE_US * 1100 - PPM_PRE_PULSE

// Throttle during failsafe
#define PPM_THROTTLE_FAILSAFE ONE_US * 900 - PPM_PRE_PULSE

// CH5 power on values (mode selection channel)
#define PPM_CH5_MODE_4        ONE_US * 1555 - PPM_PRE_PULSE

// -------------------------------------------------------------
// PPM OUTPUT SETTINGS
// -------------------------------------------------------------
// #define _POSITIVE_PPM_FRAME_    // Switch to positive pulse PPM
// (the actual timing is encoded in the length of the low between two pulses)

// Number of servo input channels
#define SERVO_CHANNELS        8

// PPM period 18.5ms - 26.5ms (54hz - 37Hz) 
#define PPM_PERIOD            ONE_US * ( 22500 - ( 8 * 1500 ) )

// Size of ppm[..] data array ( servo channels * 2 + 2)
#define PPM_ARRAY_MAX         18

// -------------------------------------------------------------
// SERVO FAILSAFE VALUES
// -------------------------------------------------------------
volatile uint16_t failsafe_ppm[ PPM_ARRAY_MAX ] =                               
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
// Data array for storing ppm (8 channels) pulse widths.
// -------------------------------------------------------------
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

// -------------------------------------------------------------
// Data arraw for storing ppm timeout (missing channel detection)
// -------------------------------------------------------------
#define PPM_TIMEOUT_VALUE 40 // ~1sec before triggering missing channel detection
volatile uint8_t ppm_timeout[ PPM_ARRAY_MAX ];

// Servo input channel connected flag
volatile bool servo_input_connected[ PPM_ARRAY_MAX ];

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
#define PPM_COMPARE_FORCE_MATCH    FOC1A

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
#define PPM_COMPARE_FORCE_MATCH    FOC1B

#else
#error NO SUPPORTED DEVICE FOUND! (ATmega16u2 / ATmega32u2 / ATmega328p)
#endif

// Used to force throttle fail-safe mode (RTL)
volatile bool throttle_failsafe_force = false;

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
        volatile uint8_t SREG_tmp = SREG;

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
        
        #if defined (_POSITIVE_PPM_FRAME_)
        // Force output compare to reverse polarity
        TCCR1C |= (1 << PPM_COMPARE_FORCE_MATCH);
        #endif

        // Enable output compare interrupt
        TIMSK1 |= (1 << PPM_COMPARE_ENABLE);

        // Indicate that PPM generator is active
        ppm_generator_active = true;

        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)        
        // Turn on TX led if PPM generator is active
        PORTD &= ~( 1<< PD5 );
        #endif

        // Restore interrupt status and register flags
        SREG = SREG_tmp;
}

// ------------------------------------------------------------------------------
// PPM GENERATOR STOP - TOGGLE ON COMPARE INTERRUPT DISABLE
// ------------------------------------------------------------------------------
void ppm_stop( void )
{
        // Store interrupt status and register flags
        volatile uint8_t SREG_tmp = SREG;

        // Stop interrupts
        cli();

        // Disable output compare interrupt
        TIMSK1 &= ~(1 << PPM_COMPARE_ENABLE);

        // Reset TIMER1 registers
        TCCR1A = 0;
        TCCR1B = 0;

        // Indicate that PPM generator is not active
        ppm_generator_active = false;

        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)        
        // Turn off TX led if PPM generator is off
        PORTD |= ( 1<< PD5 );
        #endif
        
        // Restore interrupt status and register flags
        SREG = SREG_tmp;
}

// ------------------------------------------------------------------------------
// Watchdog Interrupt (interrupt only mode, no reset)
// ------------------------------------------------------------------------------
ISR( WDT_vect ) // If watchdog is triggered then enable missing signal flag and copy power on or failsafe positions
{
    // Use failsafe values if PPM generator is active or if chip has been reset from a brown-out
    if ( ppm_generator_active || brownout_reset )
    {
        // Copy failsafe values to ppm[..]
        for( volatile uint8_t i = 0; i < PPM_ARRAY_MAX; i++ )
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

    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)        
    // Turn on RX led if signal is missing
    if( !servo_input_missing )
    {
        PORTD &= ~( 1<< PD4 );
    }
    #endif

    // Set missing receiver signal flag
    servo_input_missing = true;
    
    // Reset servo input error flag
    servo_input_errors = 0;
    
}
// ------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
// SERVO/PPM INPUT - PIN CHANGE INTERRUPT
// ------------------------------------------------------------------------------
ISR( SERVO_INT_VECTOR )
{
    // Servo pulse start timing
    static uint16_t servo_start[ PPM_ARRAY_MAX ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // We sacrefice some memory but save instructions by working with ppm index count (18) instead of input channel count (8)

    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
    // Toggle LED delay
    static uint8_t led_delay = 0;
    #endif
 
    // Servo input pin storage 
    static uint8_t servo_pins_old = 0;

    // Used to store current servo input pins
    uint8_t servo_pins;

    uint8_t servo_change;
    uint8_t servo_pin;
    uint8_t ppm_channel;

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
    servo_change = servo_pins ^ servo_pins_old;
    
    // Set initial servo pin and ppm[..]  index
    servo_pin = 1;
    ppm_channel = 1;

CHECK_PINS_LOOP: // Input servo pin check loop

    // Check for pin change on current servo channel
    if( servo_change & servo_pin )
    {
        // Remove processed pin change from bitmask
        servo_change &= ~servo_pin;

        // High (raising edge)
        if( servo_pins & servo_pin )
        {
            servo_start[ ppm_channel ] = servo_time;
        }
        else
        {
            // Get servo pulse width
            uint16_t servo_width = servo_time - servo_start[ ppm_channel ] - PPM_PRE_PULSE;
            
            // Check that servo pulse signal is valid before sending to ppm encoder
            if( servo_width > PPM_SERVO_MAX ) goto CHECK_PINS_ERROR;
            if( servo_width < PPM_SERVO_MIN ) goto CHECK_PINS_ERROR;

            // Reset Watchdog Timer
            wdt_reset(); 

            // Set servo input missing flag false to indicate that we have received servo input signals
            servo_input_missing = false;
            
            // Channel has received a valid signal, so it must be connected
            servo_input_connected [ ppm_channel ] = true;
            
           //Reset ppm single channel fail-safe timeout
            ppm_timeout[ ppm_channel ] = 0;

            // Check for forced throttle fail-safe
            if( throttle_failsafe_force )
            {
                if( ppm_channel == 5 )
                {
                    // Force throttle fail-safe
                    servo_width = PPM_THROTTLE_FAILSAFE;
                }

            #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
                // Turn on RX LED if throttle fail-safe is forced
                PORTD &= ~(1 << PD4);
            #endif    
            }
            
        #ifdef _AVERAGE_FILTER_
            // Average filter to smooth input jitter
            servo_width += ppm[ ppm_channel ];
            servo_width >>= 1;
        #endif

        #ifdef _JITTER_FILTER_
            // 0.5us cut filter to remove input jitter
            int16_t ppm_tmp = ppm[ ppm_channel ] - servo_width;
            if( ppm_tmp <= _JITTER_FILTER_ && ppm_tmp >= -_JITTER_FILTER_ ) goto CHECK_PINS_NEXT;
        #endif

            // Update ppm[..]
            ppm[ ppm_channel ] = servo_width;
        }
    }
    
CHECK_PINS_NEXT:
 
    // Are we done processing pins?
    if( servo_change )
    {
        // Select next ppm[..] index
        ppm_channel += 2;

        // Select next servo pin
        servo_pin <<= 1;
     
        // Check next pin
        goto CHECK_PINS_LOOP;
    }

    // All changed pins have been checked
    goto CHECK_PINS_DONE;

CHECK_PINS_ERROR:
    
    // Used to indicate invalid servo input signals
    servo_input_errors++;

    goto CHECK_PINS_NEXT;
    
// Processing done, cleanup and exit
CHECK_PINS_DONE:

    // Start PPM generator if not already running
    if( !ppm_generator_active ) ppm_start();

    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
   
    // Toggle RX LED when finished receiving servo pulses
    if( ++led_delay == 0 ) // Toggle led every 128th or 255th interval
    {
        PIND |= ( 1<< PD4 );

        // Fast toggle on servo input errors
        if( servo_input_errors > _INPUT_ERROR_TRIGGER_ )
        {
            led_delay = 223;
        }
    }
    #endif    

    // Store current servo input pins for next check
    servo_pins_old = servo_pins;

    //Has servo input changed while processing pins, if so we need to re-check pins
    //if( servo_pins != SERVO_INPUT ) goto CHECK_PINS_START;

    // Clear interrupt event from already processed pin changes
    //PCIFR |= (1 << SERVO_INT_CLEAR_FLAG);
}
// ------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
// PPM OUTPUT - TIMER1 COMPARE INTERRUPT
// ------------------------------------------------------------------------------
// Current active ppm channel
volatile uint8_t ppm_out_channel = PPM_ARRAY_MAX - 1;
ISR( PPM_INT_VECTOR, ISR_NOBLOCK )  
{
    // ------------------------------------------------------------------------------
    // !! NESTED INTERRUPT !!
    // - ALL VARIABLES SHOULD BE GLOBAL VOLATILE 
    // - ACCESSING VARIABLES >8BIT MUST BE DONE ATOMIC USING CLI/SEI
    // ------------------------------------------------------------------------------   

    // Update timing for next compare toggle with either current ppm input value, or fail-safe value if there is a channel timeout.
    if( ppm_timeout[ ppm_out_channel ] > PPM_TIMEOUT_VALUE )
    {
        // Use ppm fail-safe value
        cli();
        PPM_COMPARE += failsafe_ppm[ ppm_out_channel ];
        sei();
        
        // Did we lose an active servo input channel? If so force throttle fail-safe (RTL)
        if( servo_input_connected[ ppm_out_channel ] )
        {
            throttle_failsafe_force = true;
        }
    }
    else
    {
        // Use latest ppm input value   
        cli();
        PPM_COMPARE += ppm[ ppm_out_channel ];
        sei();
        
        // Increment channel timeout (reset to zero in input interrupt each time a valid signal is detected)
        ppm_timeout[ ppm_out_channel ] ++;
    }
    
    // Select the next ppm channel
    if( ++ppm_out_channel >= PPM_ARRAY_MAX ) 
    {
        ppm_out_channel = 0;

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
        // Reset Source checkings
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
        
        DDRD |= (1<<PD4); // RX LED OUTPUT
        DDRD |= (1<<PD5); // TX LED OUTPUT
        
        PORTD |= (1<<PD4); // RX LED OFF
        PORTD |= (1<<PD5); // TX LED OFF
    #endif
     

    // USE JUMPER TO CHECK FOR PWM OR PPM PASSTROUGH MODE (channel 2&3 shorted)
    // ------------------------------------------------------------------------------
    if( servo_input_mode == JUMPER_SELECT_MODE )
    {
        // channel 3 status counter
        uint8_t channel3_status = 0;

        // Set channel 3 to input
        SERVO_DDR &= ~(1 << 2);

        // Enable channel 3 pullup
        SERVO_PORT |= (1 << 2);

        // Set channel 2 to output
        SERVO_DDR |= (1 << 1);

        // Set channel 2 output low
        SERVO_PORT &= ~(1 << 1);

        _delay_us (10);
        
        // Increment channel3_status if channel 3 is set low by channel 2
        if( ( SERVO_INPUT & (1 << 2) ) == 0 ) channel3_status++;

        // Set channel 2 output high
        SERVO_PORT |= (1 << 1);
        
        _delay_us (10);
        
        // Increment channel3_status if channel 3 is set high by channel 2
        if( ( SERVO_INPUT & (1 << 2) ) != 0 ) channel3_status++;

        // Set channel 2 output low
        SERVO_PORT &= ~(1 << 1);

        _delay_us (10);

        // Increment channel3_status if channel 3 is set low by channel 2
        if( ( SERVO_INPUT & (1 << 2) ) == 0 ) channel3_status++;

        // Set servo input mode based on channel3_status
        if( channel3_status == 3 ) servo_input_mode = PPM_PASSTROUGH_MODE;
        else servo_input_mode = SERVO_PWM_MODE;

    }


    // SERVO/PPM INPUT PINS
    // ------------------------------------------------------------------------------
    // Set all servo input pins to inputs
    SERVO_DDR = 0;

    // Activate pullups on all input pins
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

     // PPM PASS-THROUGH MODE
    if( servo_input_mode == PPM_PASSTROUGH_MODE )
    {
        // Set servo input interrupt pin mask to servo input channel 1
        SERVO_INT_MASK = 0x01;
    }

    // SERVO PWM INPUT MODE
    // ------------------------------------------------------------------------------
    if( servo_input_mode == SERVO_PWM_MODE )
    {
        // Interrupt on all input pins
        SERVO_INT_MASK = 0xFF;
    }
    
    // Enable servo input interrupt
    PCICR |= (1 << SERVO_INT_ENABLE);

    // PPM OUTPUT
    // ------------------------------------------------------------------------------
    // PPM generator (PWM output timer/counter) is started either by pin change interrupt or by watchdog interrupt

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


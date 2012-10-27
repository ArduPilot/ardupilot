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

// Not for production - Work in progress

#ifndef _PPM_ENCODER_H_
#define _PPM_ENCODER_H_

#include <avr/io.h>
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
// GLOBAL SETTINGS
// -------------------------------------------------------------

// Number of Timer1 ticks for 1 microsecond
#define TICKS_FOR_ONE_US                F_CPU / 8 / 1000 / 1000

// -------------------------------------------------------------
// INPUT MODE (by jumper selection)
// -------------------------------------------------------------

#define JUMPER_SELECT_MODE    0    // Default - PPM passtrough mode selected if input pins 2&3 shorted. Normal servo input (pwm) if not shorted.
#define SERVO_PWM_MODE        1    // Normal 8 channel servo (pwm) input
#define PPM_PASSTROUGH_MODE   2    // PPM signal passtrough on channel 1 if input pins 2&3 shorted
#define PPM_REDUNDANCY_MODE   3    // PPM redundancy on channels 1 and 2 if input pins 3&4 shorted
#define SPEKTRUM_MODE         4    // Spektrum satelitte on channel 1 (reserved - not yet implemented)

volatile uint8_t servo_input_mode = JUMPER_SELECT_MODE;

// -------------------------------------------------------------
// PPM REDUNDANCY MODE SETTINGS
// -------------------------------------------------------------

#define SWITCHOVER_CHANNEL_A		9	// Receiver 1 PPM channel to force receiver 2. Use 0 for no switchover channel
										// Must be choosed between 6 to 16. Preferabily from 9 to 16 so that APM can use
										// channels 1 to 8.


#define SWITCHOVER_1_to_2_DELAY_MS		50	// Delay for switching to receiver 2
#define SWITCHOVER_2_to_1_DELAY_MS		200	// Delay for switching back to receiver 1

#define CHANNEL_COUNT_DETECTION_THRESHOLD	10 // Valid frames detected before channel count validation


// 0 for standard PPM, 1 for PPMv2 (Futaba 760 us 16 Channels), 2 for PPMv3 (Jeti 1050 us 16 channels), 3 for Hitec 9 channels
// PPM input frame mode receiver 1
// -------------------------------------------------------------
#define PPM_CH1_STANDARD			// Standard PPM : 1500 us +/- 600 us - 8 channels - 20 ms frame period
//#define PPM_CH1_STANDARD_EXTENDED	// Hitec 9 channels hardware : 1500 us +/- 600 us - 9 channels - 22.1 ms slower frame period
//#define PPM_CH1_V2				// PPMv2 used initialy at Futaba - 760 us +/- 300 us - 16 Channels - normal 20 ms frame period
//#define PPM_CH1_V3				// PPMv3 to allow for 16 channels with long sync symbol for hardware compatibility : 1050 us +/- 300 us - 25 ms frame period

// PPM input frame mode receiver 2
// -------------------------------------------------------------
#define PPM_CH2_STANDARD
//#define PPM_CH2_STANDARD_EXTENDED
//#define PPM_CH2_V2
//#define PPM_CH2_V3

// PPM input frame definitions receiver 1
// -------------------------------------------------------------
#if defined (PPM_CH1_STANDARD) || defined (PPM_CH1_STANDARD_EXTENDED)
 
#ifdef PPM_CH1_STANDARD_EXTENDED
 
// Max input PPM channels
#define PPM_CH1_MAX_CHANNELS        9
// Frame period
#define PPM_CH1_FRAME_PERIOD	22100	// frame period (microseconds)

#else

// Max input PPM channels
#define PPM_CH1_MAX_CHANNELS        8
// Frame period
#define PPM_CH1_FRAME_PERIOD	20000	// frame period (microseconds)

#endif

// Negative Shift
//#define SHIFT_CH1_NEGATIVE // Some receivers may use negative shift (zero volt part of the signal carry the PWM information instead of the positive part)

// PPM channels minimum and maximum values
#define PPM_CH1_VAL_MIN         TICKS_FOR_ONE_US * 900
#define PPM_CH1_VAL_MAX         TICKS_FOR_ONE_US * 2100
#define PPM_CH1_VAL_CENTER      TICKS_FOR_ONE_US * 1500
#define PPM_CH1_FORCE_VAL_MIN	1800

// PPM channel pre pulse lenght
#define PPM_CH1_CHANNEL_PREPULSE_LENGHT		400

// PPM frame sync symbol minimum and maximum laps
#define PPM_CH1_MIN_SYNC_LENGHT		PPM_CH1_FRAME_PERIOD - ( PPM_CH1_MAX_CHANNELS * PPM_CH1_VAL_MAX ) // Sync symbol detection
#define PPM_CH1_MAX_SYNC_LENGHT		PPM_CH1_FRAME_PERIOD - ( PPM_CH1_MAX_CHANNELS * PPM_CH1_VAL_MIN ) // Sync timeout

#endif

#ifdef PPM_CH1_V2 (PPMv2 is a 50 Hz 16 channels mode, used initialy on Futaba and later on other manufacturers modules)
  
// Max input PPM channels
#define PPM_CH1_MAX_CHANNELS        16
// Frame period
#define PPM_CH1_FRAME_PERIOD	20000	// frame period (microseconds)

// Negative Shift
//#define SHIFT_CH1_NEGATIVE // Some receivers may use negative shift (zero volt part of the signal carry the PWM information instead of the positive part)

// PPM channels minimum and maximum values
#define PPM_CH1_VAL_MIN         TICKS_FOR_ONE_US * 450
#define PPM_CH1_VAL_MAX         TICKS_FOR_ONE_US * 1050
#define PPM_CH1_VAL_CENTER      TICKS_FOR_ONE_US * 750
#define PPM_CH1_FORCE_VAL_MIN	900

// PPM channel pre pulse lenght
#define PPM_CH1_CHANNEL_PREPULSE_LENGHT		200

// PPM frame sync symbol minimum and maximum laps
#define PPM_CH1_MIN_SYNC_LENGHT		PPM_CH1_FRAME_PERIOD - ( PPM_CH1_MAX_CHANNELS * PPM_CH1_VAL_MAX ) // Sync symbol detection
#define PPM_CH1_MAX_SYNC_LENGHT		PPM_CH1_FRAME_PERIOD - ( PPM_CH1_MAX_CHANNELS * PPM_CH1_VAL_MIN ) // Sync timeout

#endif

#ifdef PPM_CH1_V3 (PPMv3 is a 40 Hz slower refresh frequency 16 channels mode to allow 16 channels with older transmitter modules)
  
// Max input PPM channels
#define PPM_CH1_MAX_CHANNELS        16
// Frame period
#define PPM_CH1_FRAME_PERIOD	25000	// frame period (microseconds)

// Negative Shift
//#define SHIFT_CH1_NEGATIVE // Some receivers may use negative shift (zero volt part of the signal carry the PWM information instead of the positive part)

// PPM channels minimum and maximum values
#define PPM_CH1_VAL_MIN         TICKS_FOR_ONE_US * 750
#define PPM_CH1_VAL_MAX         TICKS_FOR_ONE_US * 1350
#define PPM_CH1_VAL_CENTER      TICKS_FOR_ONE_US * 1050
#define PPM_CH1_FORCE_VAL_MIN	1260

// PPM channel pre pulse lenght
#define PPM_CH1_CHANNEL_PREPULSE_LENGHT		400

// PPM frame sync symbol minimum and maximum laps
#define PPM_CH1_MIN_SYNC_LENGHT		PPM_CH1_FRAME_PERIOD - ( PPM_CH1_MAX_CHANNELS * PPM_CH1_VAL_MAX ) // Sync symbol detection
#define PPM_CH1_MAX_SYNC_LENGHT		PPM_CH1_FRAME_PERIOD - ( PPM_CH1_MAX_CHANNELS * PPM_CH1_VAL_MIN ) // Sync timeout

#endif

// PPM input frame definitions receiver 2
// -------------------------------------------------------------
#if defined (PPM_CH2_STANDARD) || defined (PPM_CH2_STANDARD_EXTENDED)
 
#ifdef PPM_CH2_STANDARD_EXTENDED
 
// Max input PPM channels
#define PPM_CH2_CHANNELS        9
// Frame period
#define PPM_CH2_FRAME_PERIOD	22100	// frame period (microseconds)

#else

// Max input PPM channels
#define PPM_CH2_CHANNELS        8
// Frame period
#define PPM_CH2_FRAME_PERIOD	20000	// frame period (microseconds)

#endif

// Negative Shift
//#define SHIFT_CH2_NEGATIVE // Some receivers may use negative shift (zero volt part of the signal carry the PWM information instead of the positive part)

// PPM channels minimum and maximum values
#define PPM_CH2_VAL_MIN         TICKS_FOR_ONE_US * 900
#define PPM_CH2_VAL_MAX         TICKS_FOR_ONE_US * 2100
#define PPM_CH2_VAL_CENTER      TICKS_FOR_ONE_US * 1500

// PPM channel pre pulse lenght
#define PPM_CH1_CHANNEL_PREPULSE_LENGHT		400

// PPM frame sync symbol minimum and maximum laps
#define PPM_CH2_MIN_SYNC_LENGHT		PPM_CH2_FRAME_PERIOD - ( PPM_CH2_CHANNELS * PPM_CH2_VAL_MAX ) // Sync symbol detection
#define PPM_CH2_MAX_SYNC_LENGHT		PPM_CH2_FRAME_PERIOD - ( PPM_CH2_CHANNELS * PPM_CH2_VAL_MIN ) // Sync timeout

#endif

#ifdef PPM_CH2_V2 (PPMv2 is a 50 Hz 16 channels mode, used initialy on Futaba and later on other manufacturers modules)
  
// Max input PPM channels
#define PPM_CH2_CHANNELS        16
// Frame period
#define PPM_CH2_FRAME_PERIOD	20000	// frame period (microseconds)

// Negative Shift
//#define SHIFT_CH2_NEGATIVE // Some receivers may use negative shift (zero volt part of the signal carry the PWM information instead of the positive part)

// PPM channels minimum and maximum values
#define PPM_CH2_VAL_MIN         TICKS_FOR_ONE_US * 450
#define PPM_CH2_VAL_MAX         TICKS_FOR_ONE_US * 1050
#define PPM_CH2_VAL_CENTER      TICKS_FOR_ONE_US * 750

// PPM channel pre pulse lenght
#define PPM_CH1_CHANNEL_PREPULSE_LENGHT		200

// PPM frame sync symbol minimum and maximum laps
#define PPM_CH2_MIN_SYNC_LENGHT		PPM_CH2_FRAME_PERIOD - ( PPM_CH2_CHANNELS * PPM_CH2_VAL_MAX ) // Sync symbol detection
#define PPM_CH2_MAX_SYNC_LENGHT		PPM_CH2_FRAME_PERIOD - ( PPM_CH2_CHANNELS * PPM_CH2_VAL_MIN ) // Sync timeout

#endif

#ifdef PPM_CH2_V3 (PPMv3 is a 40 Hz slower refresh frequency 16 channels mode to allow 16 channels with older transmitter modules)
  
// Max input PPM channels
#define PPM_CH2_CHANNELS        16
// Frame period
#define PPM_CH2_FRAME_PERIOD	25000	// frame period (microseconds)

// Negative Shift
//#define SHIFT_CH2_NEGATIVE // Some receivers may use negative shift (zero volt part of the signal carry the PWM information instead of the positive part)

// PPM channels minimum and maximum values
#define PPM_CH2_VAL_MIN         TICKS_FOR_ONE_US * 750
#define PPM_CH2_VAL_MAX         TICKS_FOR_ONE_US * 1350
#define PPM_CH2_VAL_CENTER      TICKS_FOR_ONE_US * 1050

// PPM channel pre pulse lenght
#define PPM_CH1_CHANNEL_PREPULSE_LENGHT		400

// PPM frame sync symbol minimum and maximum laps
#define PPM_CH2_MIN_SYNC_LENGHT		PPM_CH2_FRAME_PERIOD - ( PPM_CH2_CHANNELS * PPM_CH2_VAL_MAX ) // Sync symbol detection
#define PPM_CH2_MAX_SYNC_LENGHT		PPM_CH2_FRAME_PERIOD - ( PPM_CH2_CHANNELS * PPM_CH2_VAL_MIN ) // Sync timeout

#endif


// -------------------------------------------------------------
// SERVO PWM MODE input settings
// -------------------------------------------------------------

// Number of input PWM channels
#define PWM_CHANNELS        8

// PWM channels minimum values
#define PWM_VAL_MIN         TICKS_FOR_ONE_US * 900 - PPM_PRE_PULSE

// PWM channels maximum values
#define PWM_VAL_MAX         TICKS_FOR_ONE_US * 2100 - PPM_PRE_PULSE

// PWM input filters
// Using both filters is not recommended and may reduce servo input resolution

// #define _AVERAGE_FILTER_         // Average filter to smooth servo input capture jitter
// #define _JITTER_FILTER_             // Cut filter to remove 0,5us servo input capture jitter

// -------------------------------------------------------------
// PPM output frame format
// -------------------------------------------------------------

// Number of output PPM channels
#define PPM_CHANNELS        8

// 400us PPM pre pulse
#define PPM_PRE_PULSE         TICKS_FOR_ONE_US * 400

// PPM channels center positions
#define PPM_VAL_CENTER      TICKS_FOR_ONE_US * 1500 - PPM_PRE_PULSE

// PPM period 18.5ms - 26.5ms (54hz - 37Hz) 
#define PPM_PERIOD            TICKS_FOR_ONE_US * ( 22500 - ( PPM_CHANNELS * 1500 ) )

// Size of ppm[..] data array
#define PPM_ARRAY_MAX         PPM_CHANNELS * 2 + 2

// -------------------------------------------------------------
// PPM output default values
// -------------------------------------------------------------

// Throttle default at power on
#define PPM_THROTTLE_DEFAULT  TICKS_FOR_ONE_US * 1100 - PPM_PRE_PULSE

// Throttle during failsafe
#define PPM_THROTTLE_FAILSAFE TICKS_FOR_ONE_US * 900 - PPM_PRE_PULSE

// CH5 power on values (mode selection channel)
#define PPM_CH5_MODE_4        TICKS_FOR_ONE_US * 1555 - PPM_PRE_PULSE

// -------------------------------------------------------------
// PPM output frame variables
// -------------------------------------------------------------

// Data array for storing output PPM (8 channels) pulse widths.

volatile uint16_t ppm[ PPM_ARRAY_MAX ] =                                
{
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 1 
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 2
    PPM_PRE_PULSE,
    PPM_THROTTLE_DEFAULT,     // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CH5_MODE_4,           // Channel 5 (flight mode)
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 6
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 7
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_PERIOD
};

// Output PPM FAILSAFE values

const uint16_t failsafe_ppm[ PPM_ARRAY_MAX ] =                               
{
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 1
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 2
    PPM_PRE_PULSE,
    PPM_THROTTLE_FAILSAFE,    // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CH5_MODE_4,           // Channel 5
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 6
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 7
    PPM_PRE_PULSE,
    PPM_VAL_CENTER,         // Channel 8
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

// Invalid SERVO input signals count
volatile uint8_t servo_input_errors = 0;

// Missing SERVO input signals flag
volatile bool servo_input_missing = true;

// PPM generator active flag
volatile bool ppm_generator_active = false;

// Brownout restart flag
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
    // To store current servo input pins
    uint8_t servo_pins;
	
	// Servo input pin storage 
    static uint8_t servo_pins_old = 0;
			
	// PWM Mode pulse start time
	static uint16_t servo_start[ servo_channel ] = { 0, 0, 0, 0, 0, 0, 0, 0 };
		
	// Missing throttle signal failsafe
	static uint8_t throttle_timeout = 0;
	
	#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
	// Toggle LED delay
	static uint8_t led_delay = 0;
	#endif
  
    // Read current servo timer
    uint16_t servo_time = SERVO_TIMER_CNT;

	
    // ------------------------------------------------------------------------------
    // PPM redundancy mode
    // ------------------------------------------------------------------------------
	
	//---------------------------------------------------------------------
	// Todo : Conversion to PPM output format
	// Todo : sync between PPM input and output after switchover
	
	// Todo : rework code from line 950 to end of redundancy mode
			
	// Todo : Add delay inside switchover algo
	// Todo : Add LED code for APM 1.4
//-------------------------------------------------------------------------
	
	if( servo_input_mode == PPM_REDUNDANCY_MODE )
    {
		// -------------------------------------
		// PPM redundancy mode - variables Init
		// -------------------------------------
		
		// PPM1 prepulse start
		static uint16_t ppm1_prepulse_start;
		// PPM2 prepulse start
		static uint16_t ppm2_prepulse_start;
		
		// PPM1 prepulse width
		static uint16_t ppm1_prepulse_width;
		// PPM2 prepulse width
		static uint16_t ppm2_prepulse_width;
		
		// PPM1 pulse start time
		static uint16_t ppm1_start[ 16 ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		// PPM2 pulse start time
		static uint16_t ppm2_start[ 16 ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		
		// PPM1 pulse lenght
		static uint16_t ppm1_width[ 16 ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		// PPM2 pulse lenght
		static uint16_t ppm2_width[ 16 ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
				
		// Reset PPM channels ( 0 = Sync Symbol )
		static uint8_t ppm1_channel = 0; // Channel 0 = sync symbol
		static uint8_t ppm2_channel = 0; // Channel 0 = sync symbol
		
		// Frame sync flag
		static bool sync_ch1 = false;
		static bool sync_ch2 = false;
		
		// Channel error flags
		static bool channel_error_ch1 = true;
		static bool channel_error_ch2 = true;
		
		// Sync error flags
		static bool sync_error_ch1 = true;
		static bool sync_error_ch2 = true;
		
		//---------------------------------------------------------
		
		// ch2 switchover flag
		static bool switchover_ch2 = false;
		
		//---------------------------------------------------------
		
		// Channel count detection ready flag
		static bool channel_count_ready_ch1 = false;
		static bool channel_count_ready_ch2 = false;
		
		// Channel count detected flag
		static bool channel_count_detected_ch1 = false;
		static bool channel_count_detected_ch2 = false;
		
		// Detected Channel count
		static uint8_t channel_count_ch1 = PPM_CH1_MAX_CHANNELS;
		static uint8_t channel_count_ch2 = PPM_CH2_MAX_CHANNELS;
		
		// Detected Channel count previous value
		static uint8_t previous_channel_count_ch1 = 0;
		static uint8_t previous_channel_count_ch2 = 0;
		
		// Channel count detection counter
		static uint8_t channel_count_detection_counter_ch1 = 0;
		static uint8_t channel_count_detection_counter_ch1 = 0;
		
			
		// -----------------------------------
		// PPM redundancy - decoder
		// -----------------------------------
		
	CHECK_START: // Start of PPM inputs check

		// Store current PPM inputs pins
		servo_pins = SERVO_INPUT;

		// Calculate servo input pin change mask
		uint8_t servo_change = servo_pins ^ servo_pins_old;
		
		// -----------------------------------
		// PPM redundancy - Ch1 decoding
		// -----------------------------------
		
	CHECK_LOOP: // Input PPM pins check loop

		// Check if we have a pin change on PPM channel 1
		if( servo_change & 1 )
		{
			// -----------------------------------------------------------------------------------------------------------------------
			// Check if we've got a high level (raising edge, channel start or sync symbol end)
			if( servo_pins & 1 )
			{
				// Check for pre pulse lenght
				ppm1_prepulse_width = servo_time - ppm1_prepulse_start;
				if ( true ) //Todo optionnal: We could add a test here for channel pre pulse lenght check
				{
					//We have a valid pre pulse
					if( ppm1_channel ==  channel_count_ch1 ) // Check for last channel
					{
						// We are at latest PPM channel
						sync_ch1 = false; 		// Reset sync flag
						ppm1_channel = 0; 		// Reset PPM channel counter						
					}
					else // We do not have yet reached the last channel
					{
						// Increment channel counter
						ppm1_channel++;
					}
				}
				else
				{
					//We do not have a valid pre pulse
					sync_error_ch1 = true;	 	// Set sync error flag
					sync_ch1 = false;			// Reset sync flag
					ppm1_channel = 0; 			// Reset PPM channel counter
				}
				ppm1_start[ ppm1_channel ] = servo_time; // Store pulse start time for PPM1 input
			}
			// -----------------------------------------------------------------------------------------------------------------------
			else // We've got a low level (falling edge, channel end or sync symbol start)
			{
				ppm1_width[ ppm1_channel ] = servo_time - ppm1_start[ ppm1_channel ]; // Calculate channel pulse lenght, or sync symbol lenght
				if(sync_ch1 == true) // Are we synchronized ?
				{
					// Check channel pulse lenght validity
					if( ppm1_width[ ppm1_channel ] > ( PPM_CH1_VAL_MAX - PPM_CH1_CHANNEL_PREPULSE_LENGHT ) ) || ( ppm1_width[ ppm1_channel ] < ( PPM_CH1_VAL_MIN - PPM_CH1_CHANNEL_PREPULSE_LENGHT ) ) // If we have a valid pulse lenght
					{
						// Reset channel error flag
						channel_error_ch1 = false;
					}
					else	// We do not have a valid channel lenght
					{
						if( ppm1_width[ ppm1_channel ] > PPM_CH1_MIN_SYNC_LENGHT ) || ( ppm1_width[ ppm1_channel ] < PPM_CH1_MAX_SYNC_LENGHT ) //Check for sync symbol
						{
							// We have a valid sync symbol
							if( channel_count_detected_ch1 == false ) // Check if we do not have yet channel count detected
							{
								// We have detected channels count
								channel_count_ch1 = ppm1_channel; // Store PPM1 channel count
								channel_count_ready_ch1 = true; // Set PPM1 channel count detection ready flag
								
								sync_error_ch1 = false; 	// Reset sync error flag
								sync_ch1 = true;			// Set sync flag
							}
							else	// Channel count had been detected before
							{
								//We should not have a sync symbol here
								sync_error_ch1 = true; 	// Set sync error flag
								sync_ch1 = false;			// Reset sync flag
							}
							ppm1_channel = 0; 			// Reset PPM channel counter
						}
						else // We do not have a valid sync symbol
						{
							channel_error_ch1 = true;	// Set channel error flag
						}
					}
													
				}
				// ------------------------------------------------------------------------------
				else // We are not yet synchronized
				{
					if( ppm1_width[ ppm1_channel ] > PPM_CH1_MIN_SYNC_LENGHT ) || ( ppm1_width[ ppm1_channel ] < PPM_CH1_MAX_SYNC_LENGHT ) //Check for sync symbol
					{
						// We have a valid sync symbol
						sync_error_ch1 = false; 	// Reset sync error flag
						sync_ch1 = true;			// Set sync flag
					}
					else // We did not find a valid sync symbol
					{
					sync_error_ch1 = true; 	// Set sync error flag
					sync_ch1 = false;			// Reset sync flag
					}
					ppm1_channel = 0; // Reset PPM channel counter
				}			
			}
			ppm1_prepulse_start = servo_time; // Store prepulse start time
			// -----------------------------------------------------------------------------------------------------------------------
		}

		// -----------------------------------
		// PPM redundancy - Ch2 decoding
		// -----------------------------------

		// Check if we have a pin change on PPM channel 2
		if( servo_change & 2 )
		{
			// -----------------------------------------------------------------------------------------------------------------------
			// Check if we've got a high level (raising edge, channel start or sync symbol end)
			if( servo_pins & 2 )
			{
				// Check for pre pulse lenght
				ppm2_prepulse_width = servo_time - ppm2_prepulse_start;
				if ( true ) //Todo optionnal: We could add a test here for channel pre pulse lenght check
				{
					//We have a valid pre pulse
					if( ppm2_channel ==  channel_count_ch2 ) // Check for last channel
					{
						// We are at latest PPM channel
						sync_ch2 = false; 		// Reset sync flag
						ppm2_channel = 0; 		// Reset PPM channel counter						
					}
					else // We do not have yet reached the last channel
					{
						// Increment channel counter
						ppm2_channel++;
					}
				}
				else
				{
					//We do not have a valid pre pulse
					sync_error_ch2 = true;	 	// Set sync error flag
					sync_ch2 = false;			// Reset sync flag
					ppm2_channel = 0; 			// Reset PPM channel counter
				}
				ppm2_start[ ppm2_channel ] = servo_time; // Store pulse start time for PPM2 input
			}
			// -----------------------------------------------------------------------------------------------------------------------
			else // We've got a low level (falling edge, channel end or sync symbol start)
			{
				ppm2_width[ ppm2_channel ] = servo_time - ppm2_start[ ppm2_channel ]; // Calculate channel pulse lenght, or sync symbol lenght
				if(sync_ch2 == true) // Are we synchronized ?
				{
					// Check channel pulse lenght validity
					if( ppm2_width[ ppm2_channel ] > ( PPM_CH2_VAL_MAX - PPM_CH2_CHANNEL_PREPULSE_LENGHT ) ) || ( ppm2_width[ ppm2_channel ] < ( PPM_CH2_VAL_MIN - PPM_CH2_CHANNEL_PREPULSE_LENGHT ) ) // If we have a valid pulse lenght
					{
						// Reset channel error flag
						channel_error_ch2 = false;
					}
					else	// We do not have a valid channel lenght
					{
						if( ppm2_width[ ppm2_channel ] > PPM_CH2_MIN_SYNC_LENGHT ) || ( ppm2_width[ ppm2_channel ] < PPM_CH2_MAX_SYNC_LENGHT ) //Check for sync symbol
						{
							// We have a valid sync symbol
							if( channel_count_detected_ch2 == false ) // Check if we do not have yet channel count detected
							{
								// We have detected channels count
								channel_count_ch2 = ppm2_channel; // Store PPM2 channel count
								channel_count_ready_ch2 = true; // Set PPM2 channel count detection ready flag
								
								sync_error_ch2 = false; 	// Reset sync error flag
								sync_ch2 = true;			// Set sync flag
							}
							else	// Channel count had been detected before
							{
								//We should not have a sync symbol here
								sync_error_ch2 = true; 	// Set sync error flag
								sync_ch2 = false;			// Reset sync flag
							}
							ppm2_channel = 0; 			// Reset PPM channel counter
						}
						else // We do not have a valid sync symbol
						{
							channel_error_ch2 = true;	// Set channel error flag
						}
					}
													
				}
				// ------------------------------------------------------------------------------
				else // We are not yet synchronized
				{
					if( ppm2_width[ ppm2_channel ] > PPM_CH2_MIN_SYNC_LENGHT ) || ( ppm2_width[ ppm2_channel ] < PPM_CH2_MAX_SYNC_LENGHT ) //Check for sync symbol
					{
						// We have a valid sync symbol
						sync_error_ch2 = false; 	// Reset sync error flag
						sync_ch2 = true;			// Set sync flag
					}
					else // We did not find a valid sync symbol
					{
					sync_error_ch2 = true; 	// Set sync error flag
					sync_ch2 = false;			// Reset sync flag
					}
					ppm2_channel = 0; // Reset PPM channel counter
				}			
			}
			ppm2_prepulse_start = servo_time; // Store prepulse start time
			// -----------------------------------------------------------------------------------------------------------------------
		}
		
		// -----------------------------------
		// PPM redundancy - Post processing
		// -----------------------------------
		
		// Could be eventually run in the main loop for performances improvements if necessary
		// sync mode between input and ouptput should clear performance problems
		
		// -----------------
		// Switchover code
		// -----------------
		
		// Check for PPM1 validity
		if ( sync_error_ch1 == false ) && ( channel_error_ch1 == false) // PPM1 is valid
		{
			// check for PPM2 forcing (through PPM1 force channel)
			if ( ppm1_width [ SWITCHOVER_CHANNEL ] > PPM_CH1_FORCE_VAL_MIN )	// Channel 2 forcing is alive
			{
				// Check for PPM2 validity
				if ( sync_error_ch2 == false ) && ( channel_error_ch2 == false) // PPM2	is valid
				{
					// Check for PPM2 selected
					if ( switchover_ch2 == true ) // PPM2 is selected
					{
						// Do nothing
					}
					else
					{
						// Switch to PPM2 without delay
						if ( ppm2_channel == channel_count_ch2 )	// Check for last PPM2 channel before switching
						{
							switchover_ch2 == true; // Switch to PPM2
						}
					}
				}
			}
			else // Check for PPM1 selected
			{
				if ( switchover_ch2 == false ) // PPM1 is selected
				{
					// Load PPM Output with PPM1
					ppm[ ppm1_channel ] = ppm1_width;
				}
				else // PPM1 is not selected
				{
					// To Enhance : Optional switchover delay 2 to 1 here
					if ( ppm1_channel == channel_count_ch1 )	// Check for last PPM1 channel before switching
					{
						switchover_ch2 == false; // Switch to PPM1
					}
				}
			}
		}
		else // PPM1 is not valid
		{
			// Check for ppm2 validity
			if ( sync_error_ch2 == false ) && ( channel_error_ch2 == false) // PPM2 is valid
			{
				// Check PPM2 selected
				if ( switchover_ch2 == true ) // PPM2 is selected
				{
					// Load PPM Output with PPM2
					ppm[ ppm2_channel ] = ppm2_width;
				}
				else // Switch to PPM2
				{
					// To Enhance : Optional switchover delay 1 to 2 here
					if ( ppm2_channel == channel_count_ch2 )	// Check for last PPM2 channel before switching
					{
						switchover_ch2 == true; // Switch to PPM2
					}
				}
				
			}
			else // PPM2 is not valid
			{
				// load PPM output with failsafe values
			}
		}
		
		// -----------------------------------
		// Channel count post processing code
		// -----------------------------------
		
		// To enhance: possible global detection flag to avoid 2 channel_count_detected tests
		
		// Ch1
		
		if ( channel_count_detected_ch1 == true ) // Channel count for Ch1 was detected
		{
			// Do nothing
		}
		else // Do we have a channel count detection ready ?
		{
			if ( channel_count_ready_ch1 == true ) // If channel count is ready
			{
				// Check for detection counter
				if ( channel_count_detection_counter_ch1 < CHANNEL_COUNT_DETECTION_THRESHOLD )	// Detection counter < Threshold
				{
					// Compare channel count with previous value
					if ( channel_count_ch1 == previous_channel_count_ch1 )	// We have the same value
					{
						channel_count_detection_counter_ch1++;	// Increment detection counter
					}
					else	// We do not have the same value
					{
						channel_count_detection_counter_ch1 = 0;	// Reset detection counter
					}
					previous_channel_count_ch1 = channel_count_ch1; // Load previous channel count with channel count
				}
				else	// Detection counter >= Threshold
				{
					channel_count_detected_ch1 = true; // Channel count is now detected
				}
				channel_count_ready_ch1 = false; // Reset channel count detection ready flag
			}
		}
		
		// Ch2
		
		if ( channel_count_detected_ch2 == true ) // Channel count for ch2 was detected
		{
			// Do nothing
		}
		else // Do we have a channel count detection ready ?
		{
			if ( channel_count_ready_ch2 == true ) // If channel count is ready
			{
				// Check for detection counter
				if ( channel_count_detection_counter_ch2 < CHANNEL_COUNT_DETECTION_THRESHOLD )	// Detection counter < Threshold
				{
					// Compare channel count with previous value
					if ( channel_count_ch2 == previous_channel_count_ch2 )	// We have the same value
					{
						channel_count_detection_counter_ch2++;	// Increment detection counter
					}
					else	// We do not have the same value
					{
						channel_count_detection_counter_ch2 = 0;	// Reset detection counter
					}
					previous_channel_count_ch2 = channel_count_ch2; // Load previous channel count with channel count
				}
				else	// Detection counter >= Threshold
				{
					channel_count_detected_ch2 = true; // Channel count is now detected
				}
				channel_count_ready_ch2 = false; // Reset channel count detection ready flag
			}
		}
			
		/*
		//Reset throttle failsafe timeout
		if( ppm1_channel == 5 ) throttle_timeout = 0;
		
	CHECK_ERROR:

		#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
		// Delay LED toggle
		led_delay = 0;
		#endif
		
		*/

	CHECK_DONE:
		
		// Reset Watchdog Timer
		wdt_reset(); 

		// Set servo input missing flag false to indicate that we have received servo input signals
		servo_input_missing = false;

		// Store current servo input pins for next check
		servo_pins_old = servo_pins;

		// Start PPM generator if not already running
		if( ppm_generator_active == false ) ppm_start();

		/*

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
		if( servo_pins != SERVO_INPUT ) goto CHECK_START;
		
		*/

		// Clear interrupt event from already processed pin changes
		// PCIFR |= (1 << SERVO_INT_CLEAR_FLAG);
		
		// Leave interrupt
			return;
	}
	// -------------------------
	// PPM redundancy mode END
	// -------------------------

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
    // PWM MODE (8 channels inputs)
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
		// Low (falling edge)
        {
            
            // Get servo pulse width
            uint16_t servo_width = servo_time - servo_start[ servo_channel ] - PPM_PRE_PULSE;
            
            // Check that servo pulse signal is valid before sending to ppm encoder
            if( servo_width > PWM_VAL_MAX ) goto CHECK_PINS_ERROR;
            if( servo_width < PWM_VAL_MIN ) goto CHECK_PINS_ERROR;

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
    if( servo_channel < PWM_CHANNELS ) goto CHECK_PINS_LOOP;
    
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
// PWM MODE END
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
    if( _channel > PWM_CHANNELS ) _channel = PWM_CHANNELS;

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
     

    // USE JUMPER TO CHECK FOR INPUT MODE (pins 2&3 or 3&4 shorted)
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
        SERVO_INT_MASK = 0b11111111;
    }

    if( servo_input_mode == PPM_PASSTROUGH_MODE )
    {
        // Set servo input interrupt pin mask to servo input channel 1
        SERVO_INT_MASK = 0b00000001;
    }
    if( servo_input_mode == PPM_REDUNDANCY_MODE )
    {
        // Set servo input interrupt pin mask to servo input channel 1 and 2
        SERVO_INT_MASK = 0b00000011;
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


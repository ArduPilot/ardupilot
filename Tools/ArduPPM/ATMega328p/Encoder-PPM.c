// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// ArduPPM Version v0.9.89
// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// ARDUCOPTER 2 : PPM ENCODER for AT Mega 328p and APM v1.4 Boards
// By:John Arne Birkeland - 2011
//
//  By: Olivier ADLER - 2011 - APM v1.4 adaptation and testing
//
// Compiled with Atmel AVR Studio 4.0 / AVR GCC
// ------------------------------------------------------------------------------------------------------------------------------------------------------------

// 		Changelog		//
//
// Code based on John Arne PPM v1 encoder. Mux / Led / Failsafe control from Olivier ADLER.
// Adaptation to APM v1.4 / ATMEGA 328p by Olivier ADLER, with great code base, help  and advices from John Arne.
//
//	0.9.0 -> 0.9.4 : experimental versions. Not publicly available. Jitter problems, good reliability.
// 	
//	New PPM code base V2 from John Arne designed for 32u2 AVRs
//
//	0.9.5 : first reliable and jitter free version based on new John PPM V2 code and Olivier interrupt nesting idea.
//	0.9.6 : enhanced jitter free version with non bloking servo interrupt and ultra fast ppm generator interrupt(John's ideas)
//	0.9.7 : mux (passthrough mode) switchover reliability enhancements and error reporting improvements.
//	0.9.75 : implemented ppm_encoder.h library with support for both atmega328p and atmega32u2 chips
//	0.9.76 : timers 0 and 2 replaced by a delayed loop for simplicity. Timer 0 and 2 are now free for use.
//	              reworked error detection with settable time window, errors threshold and Led delay
//	0.9.77 : Implemented ppm_encoder.h into latest version.
//	0.9.78 : Implemented optimzed assembly compare interrupt
//	0.9.79 : Removed Non Blocking attribute for servo input interrupt
//	0.9.80 : Removed non blocking for compare interrupt, added optionnal jitter filter and optionnal non blocking attribute for assembly version of compare interrupt
//	0.9.81 : Added PPM PASSTROUGH Mode and LED Codes function to report special modes
//	0.9.82 : LED codes function simplification
//	0.9.83 : Implemented PPM passtrough failsafe
//	0.9.84 : Corrected pin and port names in Encoder-PPM.c according to #define for Mega32U2 compatibility
//	0.9.85 : Added brownout reset detection flag
//	0.9.86 : Added a #define to disable Radio Passthrough mode (hardware failsafe for Arduplane)
//	0.9.87 : #define correction for radio passthrough (was screwed up).
//  0.9.88 : LED fail-safe indication is on whenever throttle is low
//  0.9.89 : LED fail-safe change can be reverted with a define
//  0.9.90 : Small improvements in library
// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// PREPROCESSOR DIRECTIVES
// ------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "../Libraries/PPM_Encoder.h"
#include <util/delay.h>


#define	ERROR_THRESHOLD		2									// Number of servo input errors before alerting
#define ERROR_DETECTION_WINDOW	3000 * LOOP_TIMER_10MS			// Detection window for error detection (default to 30s)
#define	ERROR_CONDITION_DELAY	500 * LOOP_TIMER_10MS			// Servo error condition LED delay (LED blinking duration)

//#define PASSTHROUGH_MODE_ENABLED	// Comment this line to remove CH8 radio passthrough mode support (hardware failsafe for Arduplane)
#define PASSTHROUGH_CHANNEL		8 * 2	// Channel for passthrough mode selection
#define PASSTHROUGH_CHANNEL_OFF_US		ONE_US * 1600 - PPM_PRE_PULSE	// Passthrough off threshold
#define PASSTHROUGH_CHANNEL_ON_US		ONE_US * 1800 - PPM_PRE_PULSE	// Passthrough on threshold

#define THROTTLE_CHANNEL		3 * 2	// Throttle Channel
#define THROTTLE_CHANNEL_LED_TOGGLE_US		ONE_US * 1200 - PPM_PRE_PULSE	// Throttle Channel Led toggle threshold
#define LED_LOW_BLINKING_RATE	125 * LOOP_TIMER_10MS // Led blink rate for low throttle position (half period)

// Timers

#define TIMER0_10MS		156			// Timer0 ticks for 10 ms duration
#define TIMER1_10MS		20000		// Timer1 ticks for 10 ms duration
#define TIMER2_100MS		1562	// Timer2 ticks for 100 ms duration
#define LOOP_TIMER_10MS	10			// Loop timer ticks for 10 ms duration

// LED Code

#define	SPACE_SHORT_DURATION	40 * LOOP_TIMER_10MS	// Space after short symbol
#define	SPACE_LONG_DURATION	75 * LOOP_TIMER_10MS		// Space after long symbol
#define	SYMBOL_SHORT_DURATION	20 * LOOP_TIMER_10MS	// Short symbol duration
#define	SYMBOL_LONG_DURATION	100 * LOOP_TIMER_10MS	// Long symbol duration
#define	INTER_CODE_DURATION	150 * LOOP_TIMER_10MS		// Inter code duration

#define INTER_CODE		0		// Symbols value for coding
#define SHORT_SYMBOL	1
#define LONG_SYMBOL		2
#define SHORT_SPACE		3
#define LONG_SPACE		4
#define LOOP			5



// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// PPM ENCODER INIT AND AUXILIARY TASKS
// ------------------------------------------------------------------------------------------------------------------------------------------------------------

int main(void)
{
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	// LOCAL VARIABLES
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	bool init = true; // We are inside init sequence
	bool mux_passthrough = false; // Mux passthrough mode status Flag : passthrough is off
	uint16_t led_acceleration; // Led acceleration based on throttle stick position
	bool servo_error_condition = false;	//	Servo signal error condition
	
	static uint16_t servo_error_detection_timer=0;		// Servo error detection timer
	static uint16_t servo_error_condition_timer=0; 		// Servo error condition timer
	static uint16_t blink_led_timer = 0; 		// Blink led timer
	
	#ifdef PASSTHROUGH_MODE_ENABLED
	static uint8_t mux_timer = 0;				// Mux timer
	static uint8_t mux_counter = 0;				// Mux counter
	static int8_t mux_check = 0;
	static uint16_t mux_ppm = 500;
	#endif
	
	static uint16_t led_code_timer = 0;	// Blink Code Timer
	static uint8_t led_code_symbol = 0;	// Blink Code current symbol

	
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	// LOCAL FUNCTIONS
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	// ------------------------------------------------------------------------------
	// Led blinking (non blocking) function
	// ------------------------------------------------------------------------------
		
	uint8_t blink_led ( uint16_t half_period )	// ( half_period max = 65 s )
	{
						
		blink_led_timer++;
		
		if ( blink_led_timer < half_period ) // If half period has not been reached
		{
			return 0; // Exit timer function and return 0
		}
		else	// half period reached - LED Toggle
		{
			PPM_PORT ^= ( 1 << PB0 );	// Toggle status LED
			blink_led_timer = 0;	// Blink led timer reset
						
			return 1;	// half period reached - Exit timer function and return 1
		}
	
	}
	
	// ------------------------------------------------------------------------------
	// Led code (non blocking) function
	// ------------------------------------------------------------------------------
	
	void blink_code_led ( uint8_t code )
	{
		
		const uint8_t coding[2][14] = {
		
		// PPM_PASSTROUGH_MODE
		{ INTER_CODE, LONG_SYMBOL, LONG_SPACE, SHORT_SYMBOL, SHORT_SPACE, SHORT_SYMBOL, LOOP }, 
		
		// JETI_MODE
		{ INTER_CODE, LONG_SYMBOL, LONG_SPACE, SHORT_SYMBOL, SHORT_SPACE, SHORT_SYMBOL, SHORT_SPACE, SHORT_SYMBOL,LOOP }
		
		};
		
		led_code_timer++;		
					
						
			switch ( coding [ code - 2 ] [ led_code_symbol ] )
			{
				case INTER_CODE:
				
				if ( led_code_timer < ( INTER_CODE_DURATION ) ) return;
				else PPM_PORT |= ( 1 << PB0 );		// Enable status LED
				break;
				
				case LONG_SYMBOL:	// Long symbol
				
				if ( led_code_timer < ( SYMBOL_LONG_DURATION ) ) return;
				else PPM_PORT &= ~( 1 << PB0 );	// Disable status LED
				break;
				
				case SHORT_SYMBOL:	// Short symbol
								
				if ( led_code_timer < ( SYMBOL_SHORT_DURATION ) ) return;
				else PPM_PORT &= ~( 1 << PB0 );	// Disable status LED
				break;
				
				case SHORT_SPACE:	// Short space
				
				if ( led_code_timer < ( SPACE_SHORT_DURATION ) ) return;
				else PPM_PORT |= ( 1 << PB0 );		// Enable status LED
				break;
				
				case LONG_SPACE:	// Long space
				
				if ( led_code_timer < ( SPACE_LONG_DURATION ) ) return;
				else PPM_PORT |= ( 1 << PB0 );		// Enable status LED
				break;
				
				case LOOP:	// Loop to code start
				led_code_symbol = 0;
				return;
				break;
				
			}
						
		led_code_timer = 0;	// Code led timer reset
		led_code_symbol++;	// Next symbol
		
		return; // LED code function return
		
	}
	
		
	// ------------------------------------------------------------------------------
	// ppm reading helper - interrupt safe and non blocking function
	// ------------------------------------------------------------------------------
	uint16_t ppm_read( uint8_t channel )
	{
		uint16_t ppm_tmp = ppm[ channel ];
		while( ppm_tmp != ppm[ channel ] ) ppm_tmp = ppm[ channel ];

		return ppm_tmp;
	}

	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	// INITIALISATION CODE
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	// ------------------------------------------------------------------------------	
	// Reset Source checkings
	// ------------------------------------------------------------------------------
	if (MCUSR & 1)	// Power-on Reset
	{
	   MCUSR=0; // Clear MCU Status register
	   // custom code here
	}
	else if (MCUSR & 2)	// External Reset
	{
	   MCUSR=0; // Clear MCU Status register
	   // custom code here
	}
	else if (MCUSR & 4)	// Brown-Out Reset
	{
	   MCUSR=0; // Clear MCU Status register
	   brownout_reset=true;
	}
	else	// Watchdog Reset
	{
	   MCUSR=0; // Clear MCU Status register
	   // custom code here
	}

	
	// ------------------------------------------------------------------------------
	// Servo input and PPM generator init
	// ------------------------------------------------------------------------------
	ppm_encoder_init();

	// ------------------------------------------------------------------------------
	// Outputs init
	// ------------------------------------------------------------------------------
	PPM_DDR |= ( 1 << PB0 );	// Set LED pin (PB0) to output
	PPM_DDR |= ( 1 << PB1 );	// Set MUX pin (PB1) to output
	PPM_DDR |= ( 1 << PPM_OUTPUT_PIN );	// Set PPM pin (PPM_OUTPUT_PIN, OC1B) to output
	
	// ------------------------------------------------------------------------------		
	// Timer0 init (normal mode) used for LED control and custom code
	// ------------------------------------------------------------------------------
	TCCR0A = 0x00;	// Clock source: System Clock
	TCCR0B = 0x05;	// Set 1024x prescaler - Clock value: 15.625 kHz - 16 ms max time
	TCNT0 = 0x00;
	OCR0A = 0x00;		// OC0x outputs: Disconnected
	OCR0B = 0x00;
	TIMSK0 = 0x00;		// Timer 1 interrupt disable
	
	// ------------------------------------------------------------------------------
	// Enable global interrupt
	// ------------------------------------------------------------------------------
	sei();			// Enable Global interrupt flag
	
	// ------------------------------------------------------------------------------
	// Disable radio passthrough (mux chip A/B control)
	// ------------------------------------------------------------------------------
	PPM_PORT |= ( 1 << PB1 );	// Set PIN B1 to disable Radio passthrough (mux)
	
	
	
	
	// ------------------------------------------------------------------------------
	// Check for first valid servo signal
	// ------------------------------------------------------------------------------
	while( 1 )
	{
	#if defined _THROTTLE_LOW_RECOVERY_POSSIBLE
		if ( throttle_failsafe_force )	// We have an error 
	#else
		if ( servo_error_condition || servo_input_missing )	// We have an error 
	#endif
		{
			blink_led ( 6 * LOOP_TIMER_10MS ); // Status LED very fast blink if invalid servo input or missing signal
		}
		else		// We are running normally
		{
			init = false;	// initialisation is done,
			switch ( servo_input_mode )
			{
				case SERVO_PWM_MODE: // Normal PWM mode
				goto PWM_LOOP;
				break;
			
				case PPM_PASSTROUGH_MODE: // PPM_PASSTROUGH_MODE
				goto PPM_PASSTHROUGH_LOOP;
				break;
				
				default:	// Normal PWM mode
				goto PWM_LOOP;
				break;
			}
		}
		_delay_us (970); // Slow down while loop
	}
	

	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	// AUXILIARY TASKS
	// ------------------------------------------------------------------------------------------------------------------------------------------------------------
	PWM_LOOP: // SERVO_PWM_MODE
	while( 1 )
	{
		#ifdef PASSTHROUGH_MODE_ENABLED
		// ------------------------------------------------------------------------------
		// Radio passthrough control (mux chip A/B control)
		// ------------------------------------------------------------------------------
			
		mux_timer++; // Increment mux timer
					
		if ( mux_timer > ( 3 * LOOP_TIMER_10MS ) )	// Check Passthrough Channel every 30ms
		{
		
			mux_timer = 0; // Reset mux timer
																

			if ( mux_counter++ < 5)		// Check passthrough channel position 5 times
			{

				mux_ppm = ppm_read( PASSTHROUGH_CHANNEL - 1 ); // Safely read passthrough channel ppm position

				if ( mux_ppm < ( PASSTHROUGH_CHANNEL_OFF_US ) )	// Check ppm value and update validation counter
				{
					mux_check -= 1;
				}
				else if ( mux_ppm > ( PASSTHROUGH_CHANNEL_ON_US ) )
				{
					mux_check += 1;
				}

			}
			else							// Check 
			{		
				switch ( mux_check )	// If all 5 checks are the same, update mux status flag
				{
					case -5:
					mux_passthrough = false;
					PPM_PORT |= ( 1 << PB1 );	// Set PIN B1 (Mux) to disable Radio passthrough
					break;
			
					case 5:
					mux_passthrough = true;
					PPM_PORT &= ~( 1 << PB1 );	// Reset PIN B1 (Mux) to enable Radio passthrough
					break;
				}
				mux_check = 0; 			// Reset mux validation counter
				mux_counter = 0;		// Reset mux counter
			}
					
		}			
		
		#endif

		// ------------------------------------------------------------------------------
		// Status LED control
		// ------------------------------------------------------------------------------
	#ifdef _THROTTLE_LOW_FAILSAFE_INDICATION
		if ( throttle_failsafe_force ) // We have an error 
	#else
		if ( servo_error_condition || servo_input_missing )	// We have an error
	#endif
		{
			blink_led ( 6 * LOOP_TIMER_10MS ); // Status LED very fast blink if invalid servo input or missing signal
		}
		else		// We are running normally
		{
			if ( mux_passthrough == false ) //  Normal mode : status LED toggle speed from throttle position
			{
				led_acceleration = ( ppm[THROTTLE_CHANNEL - 1] - ( PPM_SERVO_MIN ) ) / 2;
				blink_led ( LED_LOW_BLINKING_RATE - led_acceleration );
			}
			else 	//  Passthrough mode : status LED never flashing
			{
				//  Enable status LED if throttle > THROTTLE_CHANNEL_LED_TOGGLE_US
				if ( ppm[THROTTLE_CHANNEL - 1] > ( THROTTLE_CHANNEL_LED_TOGGLE_US ) )
				{
					PPM_PORT |= ( 1 << PB0 );
				}
				// Disable status LED if throttle <= THROTTLE_CHANNEL_LED_TOGGLE_US
				else if ( ppm[THROTTLE_CHANNEL - 1] <= ( THROTTLE_CHANNEL_LED_TOGGLE_US ) )
				{
					PPM_PORT &= ~( 1 << PB0 );	
				}
			}
		}

		// ------------------------------------------------------------------------------
		// Servo input error detection
		// ------------------------------------------------------------------------------
		
		// If there are too many errors during the detection time window, then trig servo error condition
		
		if ( servo_input_errors > 0 )									// Start error rate checking if an error did occur
		{
			if ( servo_error_detection_timer > ( ERROR_DETECTION_WINDOW ) )	// If 10s delay reached
			{
				servo_error_detection_timer = 0;	 						// Reset error detection timer

				servo_input_errors = 0; 									// Reset servo input error counter
				
			}
			else															// If 10s delay is not reached
			{
				servo_error_detection_timer++;  							// Increment servo error timer value
				
				if ( servo_input_errors >= ( ERROR_THRESHOLD ) )	// If there are too many errors
				{
					servo_error_condition = true;							// Enable error condition flag
					servo_input_errors = 0;									// Reset servo input error counter
					servo_error_detection_timer = 0;						// Reset servo error detection timer
					servo_error_condition_timer = 0;						// Reset servo error condition timer
				}
			}
			
		}

		
		// Servo error condition flag (will control blinking LED)

		if ( servo_error_condition == true )	// We are in error condition
		{	
																	
			if ( servo_error_condition_timer > ( ERROR_CONDITION_DELAY ) )	// If 3s delay reached
			{
				servo_error_condition_timer = 0;	 						// Reset servo error condition timer

				servo_error_condition = false; 	// Reset servo error condition flag (Led will stop very fast blink)
			}

			else servo_error_condition_timer++; // If 3s delay is not reached update servo error condition timer value
		}
		
		_delay_us (950); // Slow down while loop
		
	}	// PWM Loop end

	
	
	PPM_PASSTHROUGH_LOOP: // PPM_PASSTROUGH_MODE
	
	while (1)
	{
		// ------------------------------------------------------------------------------
		// Status LED control
		// ------------------------------------------------------------------------------
		
		if ( servo_input_missing )	// We have an error 
		{
			blink_led ( 6 * LOOP_TIMER_10MS ); // Status LED very fast blink if invalid servo input or missing signal
		}
		else		// We are running normally
		{
			blink_code_led ( PPM_PASSTROUGH_MODE ); // Blink LED according to mode 2 code  (one long, two shorts).
		}
		
		_delay_us (970); // Slow down this loop
		
	
	}	// PPM_PASSTHROUGH Loop end
	
} // main function end



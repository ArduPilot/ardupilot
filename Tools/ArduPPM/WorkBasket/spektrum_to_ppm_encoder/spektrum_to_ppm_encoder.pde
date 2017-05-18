
/*********************************************************************************************************
 Title  :   C  file for the ArduPilotMega PPM encoder
 Author:    Doug Weibel
 Date:      11/11/10
 Comments:  This software is FREE. Use it at your own risk.
			This software has been tested with limited transmitter types and receiver binding types	*/
 
		#include <avr/interrupt.h>
		#include <avr/io.h>

		#define TRUE 1
		#define FALSE 0

        // buffers
        struct Buffer {
				volatile bool AB;
                volatile uint8_t position;
                volatile uint8_t bytesA[16];
                volatile uint8_t bytesB[16];
        };
        volatile Buffer         buffer;
		volatile bool			sync = 0;					// Are we in sync
		volatile unsigned long	last_end_time = 4294966895;	// time last packet ended (or last char received if not sync'd)

		bool				eleven_bit		= 0;
		uint16_t			ch[12];									// PWM values  
		volatile bool		falling_edge;		
		volatile uint8_t	ch_index;
		#define		CH_MAX 8								// Number of channels to form PWM output
		volatile 	uint16_t offsets[CH_MAX];
	
		#define baud_setting 16	// See page 231 of the data sheet

		// Comment - this software rearranges the channel order from the tranmsitter to match APM usage
		//			Ch1 = Aileron = Spectrum Ch2
		//			Ch2 = Elevator = Spectrum Ch3
		//			Ch3 = Throttle = Spectrum Ch1
		//			Ch8 = Flight mode = Spectrum Ch7
		const uint16_t failsafe_values[8] = {1500,1500,1000,1500,1500,1500,1500,1295};
		const uint16_t startup_values[8]  = {1500,1500,1000,1500,1500,1500,1500,1900};
		
		extern unsigned long timer0_millis;
		
		
//********************************************
void setup()
{
	DDRD &= 0B11111110;		// pd0 = RX0
	DDRB |= 0B00000111;		// pb0 = PPM LED, pb1 = MUX OUT, pb2 = PPM OUT
	PORTB |= 0B00000100;	// Set PPM OUT high
	
	TCCR1A = 0;
	TCCR1B = 0B00001001;			// This will differ for the 328?
	TCCR1C = 0;
	OCR1A = 0Xffff;
	TCNT1 = 0;
	TIMSK1 = 0B00000010;			//Output Compare A Match Interrupt Enable
	
       // init buffers
	buffer.position = 0;
	buffer.AB = TRUE;
	sync = FALSE;
 
		// assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
	UCSR0A |= (1<<U2X0);
	UBRR0H = baud_setting >> 8;
	UBRR0L = baud_setting;
    UCSR0B |= (1<<RXEN0) | (1<<RXCIE0);		

	for (int i=0;i<CH_MAX;i++)
		ch[i] = startup_values[i];

	// Set the mux for "auto".  There is no "manual" input to route if using a Spektrum satellite.
	PORTB |= 0B00000010;

}

//*********************************************************
void loop(void)
{
	uint8_t state, ledCount;
	while(1)
	{
			
		delay (20);
		
		state = update();
		ppm_output();
		
		// Set the PPM LED to indicate status
		if(state<2) {					// LED on means signal not received/sync'd yet
			PORTB |= 0B00000001;
		} else if (state == 2) {		// LED slow flash means signal lost / failsafe
			ledCount++;
			if(ledCount>=20) {
				ledCount=0;
				PORTB ^= 0B00000001;
			}
		} else if (state == 3) {		// LED fast flash means receiving signal
			ledCount++;
			if(ledCount>=5) {
				ledCount=0;
				PORTB ^= 0B00000001;
			}
		} else {						// LED off means something is wrong
			PORTB &= 0B11111110;
		}
	
	}
}

// Serial port character received interrupt
//*******************************************
ISR(USART_RX_vect)    //, ISR_BLOCK)
{
		unsigned char c = UDR0;
        uint8_t         i;
		unsigned long m;
		unsigned long m2;
			
		// timer0_millis could change inside interrupt and we don't want to disable interrupts
		// we can do two readings and compare.
		m = timer0_millis;
		m2 = timer0_millis;
		if (m!=m2)               // timer0_millis corrupted?
			m = timer0_millis;   // this should be fine...

		if (sync)
		{
			if(buffer.AB)
				buffer.bytesA[buffer.position] = c;
			else
				buffer.bytesB[buffer.position] = c;

			buffer.position++;
			if(buffer.position == 16)
			{
				sync = FALSE;
				buffer.AB = !buffer.AB;
				last_end_time = m;
			}
		}
		else
		{
			if ((m - last_end_time) > 7)
			{
				sync = TRUE;
				if (buffer.AB)
				{
					buffer.bytesA[0] = c;
					buffer.position = 1;
				}
				else
				{
					buffer.bytesB[0] = c;
					buffer.position = 1;
				}
			} else {
				last_end_time = m;
			}
		}
		
/*
if (sync) {
			PORTB |= 0B00000001;	
} else {
			PORTB &= 0B11111110;
}
*/		

}



// Timer 1 Compare A Match interrupt
//**********************************
ISR(TIMER1_COMPA_vect) {

	if (!falling_edge) {
		PORTB |= 0B00000100;	// Set PPM OUT high
		if(ch_index < CH_MAX ) {
			OCR1A = 16 * offsets[ch_index];
			falling_edge = TRUE;
			ch_index++;
		} else {
			// set timer top to max to minimize interrupt execution when not sending a frame
			OCR1A = 0xffff;
		}
	} else {
		if(ch_index <= CH_MAX ) {
			PORTB &= 0B11111011;		// Set PPM OUT low
			OCR1A = 4800;			//16 * 300,  300 usec negative going pulse width;
			falling_edge = FALSE;
		} else {
			// set timer top to max to minimize interrupt execution when not sending a frame
			OCR1A = 0xffff;
		}
	}
				
//PORTB &= 0B11111110;
}


// Get new RC data.  Call before calling pwm_output
//*************************************************
uint8_t update(void) {

// Returns:
//			0 means that we have not yet received packets
//			1 means we are starting up
//			2 means we are in failsafe
//			3 means radio OK

	static bool setup_done = 0;
	unsigned long m;
	unsigned long m2;
	unsigned long time_now;
	uint8_t packet1[17], packet2[17];
	uint16_t temp;
	bool redo = 0;
 
	// timer0_millis could change and we don't want to disable interrupts
	// we can do two readings and compare.
	time_now = timer0_millis;
	m2 = timer0_millis;
	if (time_now!=m2)               // timer0_millis corrupted?
		time_now = timer0_millis;   // this should be fine...
		
	m = last_end_time;
	m2 = last_end_time;
	if (m!=m2)               // last_end_time corrupted?
		m = last_end_time;   // this should be fine...
	
	if( m > time_now ) {		// We have not gotten in sync yet
		for(int i=0;i<12;i++)
			ch[i] = startup_values[i];
		return 0;
	} else if (time_now < m + 500) {		// We are receiving packets
	
			do {
				packet1[16] = buffer.AB;
				if (buffer.AB)
					for (int i=0; i < 16; i++)	
						packet1[i] = buffer.bytesB[i];
				else 
					for (int i=0; i < 16; i++)	
						packet1[i] = buffer.bytesA[i];
				if(packet1[16] != buffer.AB) {			// Did buffer.AB didn't change while we were working with it?
					redo = 1;
				} else {
					redo = 0;
				}
			} while (redo);
			
			if(!setup_done) {
				for (int i=2;i<15;i=i+2) {
					if(packet1[i]&0B01000000) eleven_bit = 1;
				}
				setup_done = 1;
			}
				
			for(int i=2;i<15;i=i+2) {
				int ch_num;
				temp = packet1[i+1] + (packet1[i]<<8);
				if(eleven_bit) {
					ch_num = (temp&0B0111100000000000)>>11;
					ch[ch_num] = temp&0B0000011111111111;
					ch[ch_num] /= 2;
					ch[ch_num] += 1000;
				} else {
					ch_num = (temp&0B0011110000000000)>>10;
					ch[ch_num] = temp&0B0000001111111111;
					ch[ch_num] += 1000;
				}
			}
			// Rearrange the first three channels from Spektrum order to APM order
			temp = ch[0];
			ch[0]=ch[1];
			ch[1]=ch[2];
			ch[2]=temp;
			// Rearrange the last two channels from Spektrum order to APM order
			temp = ch[6];
			ch[6]=ch[7];
			ch[7]=temp;
			return 3;
		
	} else {		// This is the failsafe case	
		for(int i=0;i<12;i++)
			ch[i] = failsafe_values[i];
		return 2;
	}
}					

// Send a PWM output frame
// ************************************************
void ppm_output(void) {

//	Channels 0 to CH_MAX-1

	// Convert PWM values to offsets
	for(int i=0;i<CH_MAX;i++) {
		offsets[i] = ch[i] - 300;		// Subtract 300 usec for the pulse width
	}
	
	ch_index=0;
	// Set the flag to begin creating falling edges
	falling_edge = 1;	
}

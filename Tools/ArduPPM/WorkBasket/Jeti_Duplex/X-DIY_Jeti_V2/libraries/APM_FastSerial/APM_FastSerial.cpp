/*
	APM_FastSerial.cpp - Fast Serial Output for Ardupilot Mega Hardware (atmega1280)
	It´s also compatible with standard Arduino boards (atmega 168 and 328)
	Interrupt driven Serial output with intermediate buffer
	Code Jose Julio and Jordi Muñoz. DIYDrones.com

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library works as a complement of the standard Arduino Serial
	library. So user must initialize Standard Serial Arduino library first.
	This library works in Serial port 0 and Serial port3(telemetry port)[APM]
	Methods: (the same as standard arduino library, inherits from Print)
	    write()  for bytes or array of bytes (binary output)
		print()  for chars, strings, numbers and floats
		println()
*/

//#include "WProgram.h"
#include "APM_FastSerial.h"
extern "C" {
  // AVR LibC Includes
  #include <inttypes.h>
  #include <avr/interrupt.h>
  #include <avr/io.h>
  #include "WConstants.h"
}
#define TX_BUFFER_SIZE 80   // Serial output buffer size

// Serial0 buffer
uint8_t tx_buffer0[TX_BUFFER_SIZE];
volatile int tx_buffer0_head=0;
volatile int tx_buffer0_tail=0;

#if defined(__AVR_ATmega1280__)
// Serial3 buffer
uint8_t tx_buffer3[TX_BUFFER_SIZE];
volatile int tx_buffer3_head=0;
volatile int tx_buffer3_tail=0;
#endif

#if defined(__AVR_ATmega1280__)  // For atmega1280 we use Serial port 0 and 3
// Serial0 interrupt
ISR(SIG_USART0_DATA)
{
  uint8_t data;

  if (tx_buffer0_tail == tx_buffer0_head)
    UCSR0B &= ~(_BV(UDRIE0));    // Disable interrupt
  else {
    data = tx_buffer0[tx_buffer0_tail];
    tx_buffer0_tail = (tx_buffer0_tail + 1) % TX_BUFFER_SIZE;	
    UDR0 = data;
  }
}

// Serial3 interrupt
ISR(SIG_USART3_DATA)
{
  uint8_t data;

  if (tx_buffer3_tail == tx_buffer3_head)	    
    UCSR3B &= ~(_BV(UDRIE3));  // Disable interrupt
  else {
    data = tx_buffer3[tx_buffer3_tail];
    tx_buffer3_tail = (tx_buffer3_tail + 1) % TX_BUFFER_SIZE;	
    UDR3 = data;
  }
}
#else

// Serial interrupt
ISR(USART_UDRE_vect)
{
  uint8_t data;

  if (tx_buffer0_tail == tx_buffer0_head)
    UCSR0B &= ~(_BV(UDRIE0));    // Disable interrupt
  else {
    data = tx_buffer0[tx_buffer0_tail];
    tx_buffer0_tail = (tx_buffer0_tail + 1) % TX_BUFFER_SIZE;	
    UDR0 = data;
  }
}
#endif

// Constructors ////////////////////////////////////////////////////////////////
APM_FastSerial_Class::APM_FastSerial_Class(uint8_t SerialPort)
{
  SerialPortNumber=SerialPort;   // This could be serial port 0 or 3
}

// Public Methods //////////////////////////////////////////////////////////////

// This is the important function (basic funtion: send a byte)
void APM_FastSerial_Class::write(uint8_t b)
{
  uint8_t Enable_tx_int=0;
  uint8_t new_head;

  if (SerialPortNumber==0)  // Serial Port 0
    {
    // if buffer was empty then we enable Serial TX interrupt
    if (tx_buffer0_tail == tx_buffer0_head)
	  Enable_tx_int=1;
  
    new_head = (tx_buffer0_head + 1) % TX_BUFFER_SIZE;  // Move to next position in the ring buffer
    if (new_head==tx_buffer0_tail)
      return;            // This is an Error : BUFFER OVERFLOW. We lost this character!!

    tx_buffer0[tx_buffer0_head] = b;  // Add data to the tx buffer
    tx_buffer0_head = new_head;              // Update head pointer

    if (Enable_tx_int)
      UCSR0B |= _BV(UDRIE0);   // Enable Serial TX interrupt
    }
#if defined(__AVR_ATmega1280__)
  else   // Serial Port 3
    {
    // if buffer was empty then we enable Serial TX interrupt
    if (tx_buffer3_tail == tx_buffer3_head)
	  Enable_tx_int=1;
  
    new_head = (tx_buffer3_head + 1) % TX_BUFFER_SIZE;  // Move to next position in the ring buffer
    if (new_head==tx_buffer3_tail)
      return;            // This is an Error : BUFFER OVERFLOW. We lost this character!!

    tx_buffer3[tx_buffer3_head] = b;  // Add data to the tx buffer
    tx_buffer3_head = new_head;              // Update head pointer

    if (Enable_tx_int)
      UCSR3B |= _BV(UDRIE3);   // Enable Serial TX interrupt
    }
#endif
}

// Send a buffer of bytes (this is util for binary protocols)
void APM_FastSerial_Class::write(const uint8_t *buffer, int size)
{
	while (size--)
      write(*buffer++);
}

// We create this two instances
APM_FastSerial_Class APM_FastSerial(0);       // For Serial port 0
#if defined(__AVR_ATmega1280__)
	APM_FastSerial_Class APM_FastSerial3(3);  // For Serial port 3  (only Atmega1280)
#endif
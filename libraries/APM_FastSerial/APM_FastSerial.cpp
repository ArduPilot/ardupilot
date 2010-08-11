/*
	APM_FastSerial.cpp - Fast Serial Output for Ardupilot Mega Hardware (Atmega1280)
	Interrupt driven Serial output with intermediate buffer
	Code Jose Julio and Jordi Muñoz. DIYDrones.com

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library works as a complement of the standard Arduino Serial
	library. So user must initialize Standard Serial Arduino library first.
	This library works in Serial port 0 and Serial port3 (telemetry port)
	Methods: (the same as standard arduino library)
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

// Serial3 buffer
uint8_t tx_buffer3[TX_BUFFER_SIZE];
volatile int tx_buffer3_head=0;
volatile int tx_buffer3_tail=0;


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
}

// Send a buffer of bytes (this is util for binary protocols)
void APM_FastSerial_Class::write(const uint8_t *buffer, int size)
{
	while (size--)
      write(*buffer++);
}

void APM_FastSerial_Class::print(uint8_t b)
{
	write(b);
}

void APM_FastSerial_Class::print(const char *s)
{
  while (*s)
    print(*s++);
}

void APM_FastSerial_Class::print(char c)
{
  write((uint8_t) c);
}

void APM_FastSerial_Class::print(int n)
{
  print((long) n);
}

void APM_FastSerial_Class::print(unsigned int n)
{
  print((unsigned long) n);
}

void APM_FastSerial_Class::print(long n)
{
  if (n < 0) {
    print('-');
    n = -n;
  }
  printNumber(n, 10);
}

void APM_FastSerial_Class::print(unsigned long n)
{
  printNumber(n, 10);
}

void APM_FastSerial_Class::print(long n, int base)
{
  if (base == 0)
    print((char) n);
  else if (base == 10)
    print(n);
  else
    printNumber(n, base);
}

void APM_FastSerial_Class::println(void)
{
  print('\r');
  print('\n');  
}

void APM_FastSerial_Class::println(char c)
{
  print(c);
  println();  
}

void APM_FastSerial_Class::println(const char c[])
{
  print(c);
  println();
}

void APM_FastSerial_Class::println(uint8_t b)
{
  print(b);
  println();
}

void APM_FastSerial_Class::println(int n)
{
  print(n);
  println();
}

void APM_FastSerial_Class::println(long n)
{
  print(n);
  println();  
}

void APM_FastSerial_Class::println(unsigned long n)
{
  print(n);
  println();  
}

void APM_FastSerial_Class::println(long n, int base)
{
  print(n, base);
  println();
}

// To print floats
void APM_FastSerial_Class::print(double n, int digits)
{
  printFloat(n, digits);
}

void APM_FastSerial_Class::println(double n, int digits)
{
  print(n, digits);
  println();
}
// Private Methods /////////////////////////////////////////////////////////////

void APM_FastSerial_Class::printNumber(unsigned long n, uint8_t base)
{
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
  unsigned long i = 0;

  if (n == 0) {
    print('0');
    return;
  } 

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    print((char) (buf[i - 1] < 10 ? '0' + buf[i - 1] : 'A' + buf[i - 1] - 10));
}

void APM_FastSerial_Class::printFloat(double number, uint8_t digits) 
{ 
  // Handle negative numbers
  if (number < 0.0)
  {
     print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print(toPrint);
    remainder -= toPrint; 
  } 
}

// We create this two instances
APM_FastSerial_Class APM_FastSerial(0);   // For Serial port 0
APM_FastSerial_Class APM_FastSerial3(3);  // For Serial port 3
/*
  Example of APM_FastSerial library.
  Code by Jose Julio and Jordi Muñoz. DIYDrones.com
*/

// FastSerial is implemented for Serial port 0 (connection to PC) and 3 (telemetry)
#include <APM_FastSerial.h> // ArduPilot Mega Fast Serial Output Library

#if defined(__AVR_ATmega1280__)
  #define LED 35
#else
  #define LED 13
#endif

unsigned long timer;
unsigned long counter;

void setup()
{  
  int myint=14235;        // Examples of data tytpes
  long mylong=-23456432;
  float myfloat=-26.669;
  byte mybyte=0xD1;
  byte bc_bufIn[50];

  for (int i=0;i<10;i++)  
  	bc_bufIn[i]=i*10+30;   // we fill the byte array
  
  pinMode(LED,OUTPUT);
  
  // We use the standard serial port initialization
  Serial.begin(57600);
  //Serial3.begin(57600);  // if we want to use port3 also (Mega boards)...
  delay(100);
  // We can use both methods to write to serial port:
  Serial.println("ArduPilot Mega FastSerial library test");          // Standard serial output
  APM_FastSerial.println("FAST_SERIAL : ArduPilot Mega FastSerial library test");  // Fast serial output
  // We can use the same on serial port3 (telemetry)
  // APM_FastSerial3.println("Serial Port3 : ArduPilot Mega FastSerial library test");
  delay(1000);
  // Examples of data types (same result as standard arduino library)
  APM_FastSerial.println("Numbers:");
  APM_FastSerial.println(myint);
  APM_FastSerial.println(mylong);
  APM_FastSerial.println(myfloat);
  APM_FastSerial.println("Byte:");
  APM_FastSerial.write(mybyte);
  APM_FastSerial.println();
  APM_FastSerial.println("Bytes:");
  APM_FastSerial.write(bc_bufIn,10);
  APM_FastSerial.println();
  delay(4000);
}

void loop()
{
  if((millis()- timer) > 20)  // 50Hz loop
    {
    timer = millis();
    if (counter < 250)  // we use the Normal Serial output for 5 seconds
      {
      // Example of typical telemetry output
      digitalWrite(LED,HIGH);
      Serial.println("!ATT:14.2;-5.5;-26.8");  // 20 bytes
      digitalWrite(LED,LOW); 
      if ((counter%5)==0)   // GPS INFO at 5Hz
        {
        digitalWrite(LED,HIGH);
        Serial.println("!LAT:26.345324;LON:-57.372638;ALT:46.7;GC:121.3;GS:23.1"); // 54 bytes
        digitalWrite(LED,LOW);
        }
      }
    else   // and Fast Serial Output for other 5 seconds
      {
      // The same info...
      digitalWrite(LED,HIGH);
      APM_FastSerial.println("#ATT:14.2;-5.5;-26.8");  // 20 bytes
      digitalWrite(LED,LOW);
      if ((counter%5)==0)   // GPS INFO at 5Hz
        {
        digitalWrite(LED,HIGH);
        
        APM_FastSerial.println("#LAT:26.345324;LON:-57.372638;ALT:46.7;GC:121.3;GS:23.1");  // 54 bytes
        digitalWrite(LED,LOW);
        }        
      if (counter>500)  // Counter reset
        counter=0;
      }
    counter++;  
    }
}
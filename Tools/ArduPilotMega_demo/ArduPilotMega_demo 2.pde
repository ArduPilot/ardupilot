//Realeased under really Creative Commons! 
//This just a basic demo code...
//Reads and pulses the 8 channels... 
//By Jordi Munoz
#include <avr/interrupt.h>

volatile unsigned int Start_Pulse =0;
volatile unsigned int Stop_Pulse =0;
volatile unsigned int Pulse_Width =0;

volatile int Test=0;
volatile int Test2=0;
volatile int Temp=0;
volatile int Counter=0;
volatile byte PPM_Counter=0;
volatile int PWM_RAW[8] = {2400,2400,2400,2400,2400,2400,2400,2400};
int All_PWM=1500;

long timer=0;
long timer2=0;

void setup()
{
  Init_PWM1(); //OUT2&3 
  Init_PWM3(); //OUT6&7
  Init_PWM5(); //OUT0&1
  Init_PPM_PWM4(); //OUT4&5
  Serial.begin(57600);

}
void loop()
{
  
   //Switch low, forward the PPM
   if(InputCh(5) <= 1250) //Reading channel 5 to change mode
   {
     OutputCh(0, InputCh(0));
     OutputCh(1, InputCh(1));
     OutputCh(2, InputCh(2));
     OutputCh(3, InputCh(3));
     OutputCh(4, InputCh(4));
     OutputCh(5, InputCh(5));
     OutputCh(6, InputCh(6));
     OutputCh(7, InputCh(7));
   }
   else
   {
     //Switch High, Move servos
     if(InputCh(5) >= 1750)
     {
         if((millis()- timer2) >= 20)
          {
            timer2=millis();
            if(All_PWM >2100)
            All_PWM=900;
            else
            All_PWM+=20;
          }
     }
     //Switch in the midle, center all servos
     else
     {
        All_PWM=1500;  
     } 
     
     OutputCh(0, All_PWM);
     OutputCh(1, All_PWM);
     OutputCh(2, All_PWM);
     OutputCh(3, All_PWM);
     OutputCh(4, All_PWM);
     OutputCh(5, All_PWM);
     OutputCh(6, All_PWM);
     OutputCh(7, All_PWM);     
   }
  
  //Printing all values. 
  if((millis()- timer) >= 250)
  {
    timer=millis();
    Serial.print("Ch0:");
    Serial.print(InputCh(0));
    Serial.print(" Ch1:");
    Serial.print(InputCh(1));
    Serial.print(" Ch2:");
    Serial.print(InputCh(2));
    Serial.print(" Ch3:");
    Serial.print(InputCh(3));
    Serial.print(" Ch4:");
    Serial.print(InputCh(4));
    Serial.print(" Ch5:");
    Serial.print(InputCh(5));
    Serial.print(" Ch6:");
    Serial.print(InputCh(6));
    Serial.print(" Ch7:");
    Serial.println(InputCh(7));
  }
   delay(20);
}



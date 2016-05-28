
void OutputCh(byte ch, int pwm)
{
  pwm=constrain(pwm,900,2100);
  pwm*=2;
 
 switch(ch)
  {
    case 0:  OCR5B=pwm; break;  //ch0
    case 1:  OCR5C=pwm; break;  //ch1
    case 2:  OCR1B=pwm; break;  //ch2
    case 3:  OCR1C=pwm; break;  //ch3
    case 4:  OCR4C=pwm; break;  //ch4
    case 5:  OCR4B=pwm; break;  //ch5
    case 6:  OCR3C=pwm; break;  //ch6
    case 7:  OCR3B=pwm; break;  //ch7
    case 8:  OCR5A=pwm; break;  //ch8,  PL3
    case 9:  OCR1A=pwm; break;  //ch9,  PB5
    case 10: OCR3A=pwm; break;  //ch10, PE3
  } 
  
}

int InputCh(byte ch)
{
  return (PWM_RAW[ch]+600)/2; 
}

void Init_PWM1(void)
{
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);

  //Remember the registers not declared here remains zero by default... 
  TCCR1A =((1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all... 
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
  OCR1A = 3000; //PB5, none
  OCR1B = 3000; //PB6, OUT2
  OCR1C = 3000; //PB7  OUT3
  ICR1 = 40000; //50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,
}

void Init_PWM3(void)
{
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);

  //Remember the registers not declared here remains zero by default... 
  TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all... 
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
  OCR3A = 3000; //PE3, NONE
  OCR3B = 3000; //PE4, OUT7
  OCR3C = 3000; //PE5,  OUT6
  ICR3 = 40000; //50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,
}

void Init_PWM5(void)
{
  pinMode(44,OUTPUT);
  pinMode(45,OUTPUT);
  pinMode(46,OUTPUT);
  
  TCCR5A =((1<<WGM51)|(1<<COM5A1)|(1<<COM5B1)|(1<<COM5C1)); 
  TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51); //Prescaler set to 8
  OCR5A = 3000; //PL3, 
  OCR5B = 3000; //PL4, OUT0
  OCR5C = 3000; //PL5  OUT1
  ICR5 = 40000;
  //ICR5 = 43910; //So (16000000hz/8)/50hz=40000,
}

/*Note that timer4 is configured to used the Input capture for PPM decoding and to pulse two servos 
  OCR4A is used as the top counter*/
void Init_PPM_PWM4(void)
{
  pinMode(49, INPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
      //Remember the registers not declared here remains zero by default... 
  TCCR4A =((1<<WGM40)|(1<<WGM41)|(1<<COM4C1)|(1<<COM4B1)|(1<<COM4A1));  
  TCCR4B = ((1<<WGM43)|(1<<WGM42)|(1<<CS41)|(1<<ICES4)); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
  OCR4A = 40000; ///50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000, 
  //must be 50hz because is the servo standard (every 20 ms, and 1hz = 1sec) 1000ms/20ms=50hz, elementary school stuff...   
  OCR4B = 3000; //PH4, OUT5
  OCR4C = 3000; //PH5, OUT4
 
  TIMSK4 |= (1<<ICIE4); //Timer interrupt mask
  sei();
  
}
/****************************************************
  Interrupt Vector
 ****************************************************/
ISR(TIMER4_CAPT_vect)//interrupt. 
{
  if(((1<<ICES4)&TCCR4B) >= 0x01)
  { 
    if(Start_Pulse>Stop_Pulse) //Checking if the Stop Pulse overflow the register, if yes i normalize it. 
    {
      Stop_Pulse+=40000; //Nomarlizing the stop pulse.
    }
    Pulse_Width=Stop_Pulse-Start_Pulse; //Calculating pulse 
       if(Pulse_Width>5000) //Verify if this is the sync pulse
       {
        PPM_Counter=0; //If yes restart the counter
       }
       else
       {
        PWM_RAW[PPM_Counter]=Pulse_Width; //Saving pulse. 
        PPM_Counter++; 
       }
    Start_Pulse=ICR4;
    TCCR4B &=(~(1<<ICES4)); //Changing edge detector. 
  }
  else
  {
    Stop_Pulse=ICR4; //Capturing time stop of the drop edge
    TCCR4B |=(1<<ICES4); //Changing edge detector. 
    //TCCR4B &=(~(1<<ICES4));
  }
  //Counter++;
}

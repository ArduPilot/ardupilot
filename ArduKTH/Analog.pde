// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//---------------------------------------------------------------------------
// Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------

void init_analog(){
  hal.console->print("  Analog: ");
  Ch0 = hal.analogin->channel(0);
  Ch1 = hal.analogin->channel(1);
  Ch2 = hal.analogin->channel(2);
  Ch3 = hal.analogin->channel(3);
  Ch4 = hal.analogin->channel(4);
  Vcc = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);
  hal.console->println(" ok. ");
}
//-------------------------------------------------------------------------------

void read_analogue_channels(){
   if (abs(time_ms-last_adc_update)>20){
      last_adc_update = time_ms;
       adc0 =  (float)Ch0->read_average()/1024.*5.;  // [Volt]
       adc1 =  (float)Ch1->read_average()/1024.*5.;  // [Volt]
       adc2 =  (float)Ch2->read_average()/1024.*5.;  // [Volt]
       adc3 =  (float)Ch3->read_average()/1024.*5.;  // [Volt]
       adc4 =  (float)Ch4->read_average()/1024.*5.;  // [Volt]
       vcc  =  (float)Vcc->read_average()/1000.;     // [Volt]
      //hal.console->printf_P(PSTR("Analogue channels:  adc0=%.2f, adc1=%.2f,   adc2=%.2f, vcc: %f\r\n"),adc0,adc1, adc1, vcc);
    }
}//-------------------------------------------------------------------------------










// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//---------------------------------------------------------------------------
// Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------

//  PWM output channels
//   1:  Starboard servo    
//   2:  Port servo                
//   3:                      
//   4:           
//   5:  
//   6:  
//   7:  
//   8:  
//
//  PWM input channels
//   1:  RC Rudder (bygla  mottagare #1 - APM #1)
//   2:  RC Thrust (bygla  mottagare #2 - APM #2)
//   3:  
//   4:  
//   5:  
//   6:  
//   7:  
//   8:  
//   
//   Analogue channels:
//   A0:  
//   A1:  
//   A2:    
//   A3:  
//   A4:  
//   A5:  
//   A6:  
//   A7:  
//   A8:  
//-------------------------------------------------------------------------------
void setup_default_Kite_mission(){
    hal.console->printf("Setting up default Kite mission: ");
    CC_leg leg;
    Nlegs_cc          = 0;
    leg.duration   = 99999; leg.course= ToRad(0);   leg.rpm=1600;    leg.depth=0.0;    CC_mission[Nlegs_cc] = leg;   Nlegs_cc++;
    current_leg_nr = 0;
    hal.console->printf("done\n");

  }
//-------------------------------------------------------------------------------
void Kite_neutral_ctrl(){
  hal.rcout->write(CH_1,1500);
  hal.rcout->write(CH_2,1500); 
  hal.rcout->write(CH_3,1500); 
  hal.rcout->write(CH_4,1500);
  hal.rcout->write(CH_5,1500);
  hal.rcout->write(CH_6,1500);
  hal.rcout->write(CH_7,1500);
  hal.rcout->write(CH_8,1500);
}
//---------------------------------------------------------------------------
void Kite_craft_setup(){
  hal.console->printf_P(PSTR("=============================\n"));
  hal.console->printf_P(PSTR("Setting up Kite craft\n"));
  hal.console->printf_P(PSTR("=============================\n"));
  craft_type='K';

  pid_1.kP(800);  
  pid_1.kI(0.0);   
  pid_1.kD(0.0);    
  pid_1.imax(0.0);   
  pid_1.save_gains();
  pid_2.kP(800);   
  pid_2.kI(0.0);   
  pid_2.kD(0.0);    
  pid_2.imax(0.0);   
  pid_2.save_gains();
  pid_3.kP(500);   
  pid_3.kI(0.0);   
  pid_3.kD(0.0);    
  pid_3.imax(0.0);   
  pid_3.save_gains();
  pid_4.kP(500);   
  pid_4.kI(0.0);   
  pid_4.kD(0.0);    
  pid_4.imax(0.0);   
  pid_4.save_gains(); 
  Kite_neutral_ctrl();
  sog_threshold = 999;      // [m/s] speed when to switch from Heading=Compass to Heading=SOG 
  k_xtrack      = 0.0;      // 0:steer directly to wp, 1:steer to closest point on rhumb-line
  setup_default_Kite_mission();
  print_CC_mission();
}
//-------------------------------------------------------------------------------
void write_Kite_telementry_data(){
  if ((time_ms-last_data_sent_ms)>100)  {
    last_data_sent_ms = time_ms; 
    hal.console->printf("#Kite:   rpc:%.1f %.1f %.1f,   PWM:%i %i   SOG:%.1fm/s  %.5f %.5f \n",
                        ToDeg(roll),ToDeg(pitch),ToDeg(heading),
                        pwm_port,pwm_stbd,  // Ctrl
                        gps.sog,ToDeg(gps.lon),ToDeg(gps.lat));  // AHRS
  }
}
//-------------------------------------------------------------------------------
void Kite_Control_Laws(){
  if ((time_ms-last_ctrl_ms)>20)  {
    last_ctrl_ms = time_ms;
      
    int RC_roll_pwm  = (int)(hal.rcin->read(CH_1)); // From RC receiver
    int RC_pitch_pwm = (int)(hal.rcin->read(CH_2)); // From RC receiver
    
    //Roll
    float rollAngle_from_RC = ToRad( (RC_roll_pwm-1500)/7.0); // [rad]
    err_roll      = roll+ rollAngle_from_RC ;                 // [rad]
    pwm_roll      = (int) pid_1.get_pid(err_roll);            // Control value from PID-regulator 
     
    //Pitch
    float thrustAngle_from_RC = ToRad( (RC_pitch_pwm-1500)/7.0); // [rad]
    err_pitch     = pitch-ToRad(25)-thrustAngle_from_RC;         // [rad]
    pwm_pitch     = (int) pid_2.get_pid(err_pitch);              // Control value from PID-regulator 
     
    // Mix it all
    pwm_stbd   = 1000 - pwm_roll - pwm_pitch;       // Mixing law
    pwm_port   = 2000 - pwm_roll + pwm_pitch;       // Mixing law
    pwm_stbd   =  min(max(pwm_stbd,1000),2000);     // Limit 
    pwm_port   =  min(max(pwm_port,1000),2000);     // Limit 
    hal.rcout->write(CH_1,pwm_stbd);                // send to servo 
    hal.rcout->write(CH_2,pwm_port);                // send to servo 
  }
}
//-------------------------------------------------------------------------------
void Kite(){

  if (ctrl_mode=='i')   { 
    Kite_neutral_ctrl(); 
  }          // Idle mode
  else                  { 
    Kite_Control_Laws();  
  }

  if (ctrl_mode!='i')   {
    write_Kite_telementry_data();
  }
}
//-------------------------------------------------------------------------------









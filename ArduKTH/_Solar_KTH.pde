// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//---------------------------------------------------------------------------
// Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------

//  PWM output channels
//   1:  Solar rudder                    PID 1 - On Heading
//   2:  Motor throttle                  PID 2 - on RPM
//   3:  Solar roll servo  (port)        PID 3 - on roll angle
//   4:  Solar roll servo  (Starboard)   PID 3 - on roll angle
//   5:  Solar pitch servo               PID 4 - on center wand
//   6:  
//   7:  
//   8:  
//
//  PWM input channels
//   1:  RC Rudder
//   2:  RC throttle
//   3:  
//   4:  RC switch (auto/manual)
//   5:  
//   6:  
//   7:  Measuring of power from the solar panels
//   8:  
//   
//   Analogue channels:
//   A0:  Wand (flex sensor or potentiometer starboard)
//   A1:  Wand (flex sensor or potentiometer port)
//   A2:  Wand (flex sensor or potentiometer center)  
//   A3:  
//   A4:  
//   A5:  
//   A6:  
//   A7:  
//   A8:  
//-------------------------------------------------------------------------------
void setup_default_Solar_mission(){
  hal.console->printf("Setting up default SOLAR GPS mission: ");
  Nlegs_GPS = 0;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.26580); // Grannen Erik
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.31301);
  GPS_mission[Nlegs_GPS].rpm       = 10.0;
  GPS_mission[Nlegs_GPS].depth     = 0.0;
  GPS_mission[Nlegs_GPS].wp_radius = 60.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.26528); // Ryttarv.
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.31368);
  GPS_mission[Nlegs_GPS].rpm       = 10.0;
  GPS_mission[Nlegs_GPS].depth     = 0.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.26582);  // Home
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.31257);
  GPS_mission[Nlegs_GPS].rpm       = 10.0;
  GPS_mission[Nlegs_GPS].depth     = 1.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.26528); // Ryttarv.
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.31368);
  GPS_mission[Nlegs_GPS].rpm       = 10.0;
  GPS_mission[Nlegs_GPS].depth     = 0.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.26582);  // Home
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.31257);
  GPS_mission[Nlegs_GPS].rpm       = 10.0;
  GPS_mission[Nlegs_GPS].depth     = 1.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.26528); // Ryttarv.
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.31368);
  GPS_mission[Nlegs_GPS].rpm       = 10.0;
  GPS_mission[Nlegs_GPS].depth     = 0.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.26582);  // Home
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.31257);
  GPS_mission[Nlegs_GPS].rpm       = 10.0;
  GPS_mission[Nlegs_GPS].depth     = 1.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.26528); // Ryttarv.
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.31368);
  GPS_mission[Nlegs_GPS].rpm       = 10.0;
  GPS_mission[Nlegs_GPS].depth     = 0.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.26582);  // Home
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.31257);
  GPS_mission[Nlegs_GPS].rpm       = 10.0;
  GPS_mission[Nlegs_GPS].depth     = 1.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.26528); // Ryttarv.
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.31368);
  GPS_mission[Nlegs_GPS].rpm       = 10.0;
  GPS_mission[Nlegs_GPS].depth     = 0.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.26582);  // Home
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.31257);
  GPS_mission[Nlegs_GPS].rpm       = 10.0;
  GPS_mission[Nlegs_GPS].depth     = 1.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  current_leg_nr = 0;
  hal.console->printf("done\n");
  print_GPS_mission();
}
//-------------------------------------------------------------------------------
void Solar_neutral_ctrl(){
  hal.rcout->write(CH_1,1500);
  hal.rcout->write(CH_2,1000);// Note
  hal.rcout->write(CH_3,1500); 
  hal.rcout->write(CH_4,1500);
  hal.rcout->write(CH_5,1500);
  hal.rcout->write(CH_6,1500);
  hal.rcout->write(CH_7,1500);
  hal.rcout->write(CH_8,1500);
}
//---------------------------------------------------------------------------
void Solar_craft_setup(){
  hal.console->printf_P(PSTR("=============================\n"));
  hal.console->printf_P(PSTR("Setting up Solar powered craft\n"));
  hal.console->printf_P(PSTR("=============================\n"));
  craft_type='S';

  pid_1.kP(-500);  
  pid_1.kI(0.0);   
  pid_1.kD(0.0);    
  pid_1.imax(0.0);   
  pid_1.save_gains();
  pid_2.kP(500);   
  pid_2.kI(0.0);   
  pid_2.kD(0.0);    
  pid_2.imax(0.0);   
  pid_2.save_gains();
  pid_3.kP(800);   
  pid_3.kI(0.0);   
  pid_3.kD(0.0);    
  pid_3.imax(0.0);   
  pid_3.save_gains();
  pid_4.kP(500);   
  pid_4.kI(0.0);   
  pid_4.kD(0.0);    
  pid_4.imax(0.0);   
  pid_4.save_gains(); 
  Solar_neutral_ctrl();
  setup_default_Solar_mission();
  sog_threshold = 1.5;      // [m/s] speed when to switch from Heading=Compass to Heading=SOG 
  k_xtrack      = 0.0;      // 0:steer directly to wp, 1:steer to closest point on rhumb-line
}
//-------------------------------------------------------------------------------
void write_Solar_telementry_data(){
  if ((time_ms-last_data_sent_ms)>200)  {
    last_data_sent_ms = time_ms; 
    hal.console->printf("#S, %.1f, Leg=%i, ctt=%.1f, dtt=%.0fm    heading=%.1f,  roll=%.1f,  pitch=%.2f,  Acc=[%.1f,%.1f,%.1f],   sog=%.1f,  cog=%.1f,   %i,%i\n",
    (float)mission_ms/1000,current_leg_nr,
    ToDeg(target_ctt), target_dtt*1852,      // Mission
    ToDeg(heading),ToDeg(roll),ToDeg(pitch), // Attitudes
    accel.x, accel.y, accel.z,               // Accelerations
    gps.sog,ToDeg(gps.cog),                  // GPS
    pwm_roll,pwm_pitch);  // Control
  }
}
//-------------------------------------------------------------------------------
void Solar_Control_Laws(){
  if ((time_ms-last_ctrl_ms)>20)  {
    last_ctrl_ms = time_ms;

    //Roll
    err_roll  = 0.0 - roll;
    pwm_roll = 1500 + (int) pid_3.get_pid(err_roll);     // Control value from PID-regulator 
    pwm_roll = min(max(pwm_roll,1010),1990);             // Limit 
    hal.rcout->write(CH_3,pwm_roll);                     // send to servo 
    hal.rcout->write(CH_4,pwm_roll);                     // send to servo 
    //hal.console->printf_P(PSTR("err_roll=%f  pwm_roll  %i \n"),err_roll,pwm_roll);

    //Pitch
    err_pitch  = 441.0 - adc2;//pitch;
    err_pitch  = 0 - pitch;
    pwm_pitch = 1500 + (int) pid_4.get_pid(err_pitch);    // Control value from PID-regulator 
    pwm_pitch = min(max(pwm_pitch,1010),1990);            // Limit
    hal.rcout->write(CH_5,pwm_pitch);                     // send to servo 
    //hal.console->printf_P(PSTR("err_pitch=%f  pwm_pitch  %i \n"),err_pitch,pwm_pitch);

    if (hal.rcin->read(CH_4)>1900) {
      RC_feedthrough=true;
    } 
    else {
      RC_feedthrough=false;
    }
    if (RC_feedthrough)
    {
      // Feed-through  RC rudder & RPM
      pwm_rudder       =  (int)(hal.rcin->read(CH_1));     // From Transmitting RC channel
      pwm_rpm          =  (int)(hal.rcin->read(CH_2));     // From Transmitting RC channel
      pwm_rudder       =  min(max(pwm_rudder,1050),1980);  // Limit 
      pwm_rpm          =  min(max(pwm_rpm ,900),2020);     // Limit 
      hal.rcout->write(CH_1,pwm_rudder);                   // send to servo 
      hal.rcout->write(CH_2,pwm_rpm);                      // send to servo 
    } 
    else {
      int chthrottle_temp = hal.rcin->read(CH_2);
      if (chthrottle_temp < 1500 && chthrottle_temp > 1460) {
        // Motor control
        //          err_rpm  = target_rpm - rpm;                         // Control error to feedback
        //          pwm_rpm = 1483 + (int) pid_2.get_pid(err_rpm);       // Control value from PID-regulator 
        pwm_rpm = 1500+533*target_rpm/100;
        pwm_rpm = min(max(pwm_rpm,1450),1800);               // Limit
        hal.rcout->write(CH_2,pwm_rpm);                      // send to motor 
        //hal.console->printf_P(PSTR("err_rpm=%f  pwm_rpm  %i \n"),err_rpm,pwm_rpm);
      }
      else{
        pwm_rpm = chthrottle_temp;
        pwm_rpm = min(max(pwm_rpm,900),2033);         // Limit deflection
        hal.rcout->write(CH_2,pwm_rpm);                   // send to Servo
      }
    }
    // Rudder control
    int chrudder_temp = hal.rcin->read(CH_1);
    if (chrudder_temp < 1484 && chrudder_temp > 1450) {
      err_cc     = unwrap_pi(target_ctt    - heading);     // Control error to feedback
      pwm_cc     = pid_1.get_pid(err_cc);                  // Local help variable
      pwm_rudder = 1467 + ( pwm_cc + 0*pwm_depth);         //  Mix
      pwm_rudder = min(max(pwm_rudder,1050),1980);         // Limit deflection
      hal.rcout->write(CH_1,pwm_rudder);                   // send to Servo
    }
    else{
      pwm_rudder = chrudder_temp;
      pwm_rudder = min(max(pwm_rudder,1010),1990);         // Limit deflection
      hal.rcout->write(CH_1,pwm_rudder);                   // send to Servo
    }
  }
}
//-------------------------------------------------------------------------------
void Solar(){

  if (ctrl_mode=='i')   { 
    Solar_neutral_ctrl(); 
  }          // Idle mode
  else                  { 
    Solar_Control_Laws();  
  }

  if (ctrl_mode!='i')   {
    write_Solar_telementry_data();
  }
}
//-------------------------------------------------------------------------------









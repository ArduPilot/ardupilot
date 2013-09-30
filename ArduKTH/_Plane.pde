// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//---------------------------------------------------------------------------
// Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------

//  PWM output channels
//   1:  Rudder                          PID 1 - On rudder
//   2:  Elevator/pitch                  PID 2 - on Elevator
//   3:  Roll                            PID 3 - on roll 
//   4:  Motor PWM            
//   5:  
//   6:  
//   7:  
//   8:  
//
//  PWM input channels
//   1:  RC Rudder
//   2:  RC Elevator
//   3:  RC Roll
//   4:  RC Motor
//   5:  RC switch (auto/manual)
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
void setup_default_Plane_mission(){
  hal.console->printf("Setting up default Plane GPS mission: ");
  Nlegs_GPS = 0;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.287188); // Närmast skolan
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.310051);
  GPS_mission[Nlegs_GPS].rpm       = 1700;
  GPS_mission[Nlegs_GPS].depth     = 100.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.288730); // Bortre planen.
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.310392);
  GPS_mission[Nlegs_GPS].rpm       = 1700;
  GPS_mission[Nlegs_GPS].depth     = 70;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.287395);  // Grusplan
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.310682);
  GPS_mission[Nlegs_GPS].rpm       = 1700;
  GPS_mission[Nlegs_GPS].depth     = 50.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.0;
  Nlegs_GPS++;

  GPS_mission[Nlegs_GPS].lon       = ToRad(18.287188); // Närmast skolan
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.310051);
  GPS_mission[Nlegs_GPS].rpm       = 1700;
  GPS_mission[Nlegs_GPS].depth     = 20.0;
  GPS_mission[Nlegs_GPS].wp_radius = 20.;
  Nlegs_GPS++;
  
  GPS_mission[Nlegs_GPS].lon       = ToRad(18.287188); // Närmast skolan
  GPS_mission[Nlegs_GPS].lat       = ToRad(59.310051);
  GPS_mission[Nlegs_GPS].rpm       = 1500;
  GPS_mission[Nlegs_GPS].depth     = 0.0;
  GPS_mission[Nlegs_GPS].wp_radius = 0.001;
  Nlegs_GPS++;

  current_leg_nr = 0;
  hal.console->printf("done\n");
  print_GPS_mission();
}
//-------------------------------------------------------------------------------
void Plane_neutral_ctrl(){
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
void Plane_craft_setup(){
  hal.console->printf_P(PSTR("=============================\n"));
  hal.console->printf_P(PSTR("Setting up Plane craft\n"));
  hal.console->printf_P(PSTR("=============================\n"));
  craft_type='P';

  pid_1.kP(500);  
  pid_1.kI(0.0);   
  pid_1.kD(0.0);    
  pid_1.imax(0.0);   
  pid_1.save_gains();
  pid_2.kP(20);   
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
  Plane_neutral_ctrl();
  setup_default_Plane_mission();
  sog_threshold = 4.0;      // [m/s] speed when to switch from Heading=Compass to Heading=SOG 
  k_xtrack      = 0.0;      // 0:steer directly to wp, 1:steer to closest point on rhumb-line
}
//-------------------------------------------------------------------------------
void write_Plane_telementry_data(){
  if ((time_ms-last_data_sent_ms)>200)  {
    last_data_sent_ms = time_ms; 
    hal.console->printf("#P, %.1f, Leg=%i, ctt=%.1f, dtt=%.0fm  heading=%.1f,  roll=%.1f,  pitch=%.2f,  Acc=[%.1f,%.1f,%.1f],   sog=%.1f,  cog=%.1f, h=%.1f,   %i,%i\n",
    (float)mission_ms/1000,current_leg_nr,
    ToDeg(target_ctt), target_dtt*1852,      // Mission
    ToDeg(heading),ToDeg(roll),ToDeg(pitch), // Attitudes
    accel.x, accel.y, accel.z,               // Accelerations
    gps.sog,ToDeg(gps.cog),gps.alt,                // GPS
    pwm_roll,pwm_pitch);  // Control
  }
}
//-------------------------------------------------------------------------------
void Plane_Control_Laws(){
  if ((time_ms-last_ctrl_ms)>20)  {
    last_ctrl_ms = time_ms;
       
    if (hal.rcin->read(CH_4)>1900) {RC_feedthrough=true;} 
    else {RC_feedthrough=false;}
    
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
      // Rudder
      err_cc     = unwrap_pi(target_ctt  - heading);       // Control error to feedback
      pwm_rudder = 1500 + pid_1.get_pid(err_cc);           // Local help variable
      pwm_rudder = min(max(pwm_rudder,1050),1980);         // Limit deflection
      hal.rcout->write(CH_1,pwm_rudder);                   // send to Servo
      
      //Roll
      err_roll  = 0-roll + 0*min(ToDeg(20),max(err_cc,ToDeg(-20)));
      pwm_roll = 1500 + (int) pid_3.get_pid(err_roll);     // Control value from PID-regulator 
      pwm_roll = min(max(pwm_roll,1010),1990);             // Limit 
      hal.rcout->write(CH_3,pwm_roll);                     // send to servo 
    
      //Pitch
      float deltaH = gps.alt-(target_depth+mission_start_pos.alt); // [m]
      err_pitch  = ToDeg(pitch) + min(10,max(deltaH/2 ,-10));
      pwm_pitch = 1500 + (int) pid_2.get_pid(err_pitch);    // Control value from PID-regulator 
      pwm_pitch = min(max(pwm_pitch,1010),1990);            // Limit
      hal.rcout->write(CH_2,pwm_pitch);                     // send to servo 
    
      // Throttle
      pwm_rpm = target_rpm;
      pwm_rpm = min(max(pwm_rpm,1010),1800);   // 
      hal.rcout->write(CH_4,pwm_rpm);            // send to motor 
      
    }
  }
}
//-------------------------------------------------------------------------------
void Plane(){

  if (ctrl_mode=='i')   { 
    Plane_neutral_ctrl(); 
  }          // Idle mode
  else                  { 
    Plane_Control_Laws();  
  }

  if (ctrl_mode!='i')   {
    write_Plane_telementry_data();
  }
}
//-------------------------------------------------------------------------------









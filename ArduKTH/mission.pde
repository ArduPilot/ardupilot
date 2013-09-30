// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//---------------------------------------------------------------------------
// Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------

void start_CC_mission(){ 
   hal.console->printf("Starting CC mission\n");
   arm_RC();
   mission_start_ms = time_ms; 
   mission_ms       = 0;
   current_leg_nr = 0;
   ctrl_mode = 'c';
}
//-------------------------------------------------------------------------------
void start_GPS_mission(){
   if (gps.status!=2){
     hal.console->printf("I refuse to start GPS mission since gps status =0\n");
     return;
   }
   mission_start_pos = current_pos;
   hal.console->printf_P(PSTR("\nStarting GPS mission (at position %.5f,  %.5f)  \n"),ToDeg(mission_start_pos.lon), ToDeg(mission_start_pos.lat));
   arm_RC();
   mission_start_ms  = time_ms; 
   mission_ms        = 0;
   current_leg_nr    = 0;
   ctrl_mode = 'g';
  
   hal.console->printf_P(PSTR("Flirting with motor control..."));
   hal.rcout->write(CH_3,1500); // To make Motor-control happy
   hal.scheduler->delay(2000); // Wait for Motor-control
   hal.console->printf_P(PSTR(" done. \n"));   
}


//-------------------------------------------------------------------------------
void setup_default_CC_mission(){
    hal.console->printf("Setting up default CC mission: ");
    CC_leg leg;
    Nlegs_cc          = 0;
    leg.duration   = 10; leg.course= ToRad(0);   leg.rpm=1600;    leg.depth=0.0;    CC_mission[Nlegs_cc] = leg;   Nlegs_cc++;
    leg.duration   = 10; leg.course= ToRad(90);  leg.rpm=1650;    leg.depth=0.0;    CC_mission[Nlegs_cc] = leg;   Nlegs_cc++;
    leg.duration   = 10; leg.course= ToRad(180); leg.rpm=1600;    leg.depth=0.0;    CC_mission[Nlegs_cc] = leg;   Nlegs_cc++;
    leg.duration   = 10; leg.course= ToRad(270); leg.rpm=1650;    leg.depth=0.0;    CC_mission[Nlegs_cc] = leg;   Nlegs_cc++;
    leg.duration   = 10; leg.course= ToRad(0);   leg.rpm=1500;    leg.depth=0.0;    CC_mission[Nlegs_cc] = leg;   Nlegs_cc++;
    current_leg_nr = 0;
    hal.console->printf("done\n");
  }
//-------------------------------------------------------------------------------
void print_CC_mission(){
   hal.console->printf("\n     Compass course type Mission:  %i legs\n",Nlegs_cc);
   hal.console->printf("     Leg    Duration      CC       Depth        RPM \n");
    for (int ii=0; ii<(Nlegs_cc); ii++) {
       hal.console->printf_P(PSTR("    %3i %10.1f %10.1f %10.1f %10.0f  \n"),ii,CC_mission[ii].duration,ToDeg(CC_mission[ii].course), CC_mission[ii].depth, CC_mission[ii].rpm);
    }
}
//-------------------------------------------------------------------------------

void setup_default_GPS_mission(){
    hal.console->printf("Setting up default GPS mission: ");
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
    
    current_leg_nr = 0;
    hal.console->printf("done\n");
    //print_GPS_mission();
  }
//-------------------------------------------------------------------------------
void print_GPS_mission(){
   hal.console->printf("\n     GPS type Mission:   %i legs\n",Nlegs_GPS);
   hal.console->printf_P(PSTR("     k_Xtrack      = %.1f\n"),k_xtrack);
   hal.console->printf_P(PSTR("     sog_threshold = %.1f\n"),sog_threshold);
   hal.console->printf("     Leg     Lon     Lat         Depth   Radius   rpm  \n");
    for (int ii=0; ii<(Nlegs_GPS); ii++) {
       hal.console->printf_P(PSTR("    %3i   %8.5f %8.5f %8.0f %8.0f %10.0f \n"),
           ii,
           ToDeg(GPS_mission[ii].lon),
           ToDeg(GPS_mission[ii].lat),
           GPS_mission[ii].depth,
           GPS_mission[ii].wp_radius, 
           GPS_mission[ii].rpm);
   }
}
//-------------------------------------------------------------------------------
void CC_mission_manager(){
  mission_ms   = time_ms-mission_start_ms;
  if ((time_ms-mission_update_timer_ms)>1000)  {
     mission_update_timer_ms = time_ms;
  
    float mission_sec = (float)mission_ms/1000.0; // [sec] Time since mission start
    // calc what leg we are on
    current_leg_nr     = 0;
    target_ctt    =  CC_mission[current_leg_nr].course;      // [rad] Compass course
    target_depth  =  CC_mission[current_leg_nr].depth;       // [m]   
    target_rpm    =  CC_mission[current_leg_nr].rpm;         // [m]   
    float legs_accumulated_time_sec = CC_mission[current_leg_nr].duration;
    while(mission_sec > legs_accumulated_time_sec){
      current_leg_nr = current_leg_nr+1;
      if (current_leg_nr<Nlegs_cc){
        target_ctt    =  CC_mission[current_leg_nr].course;  // [rad] Compass course
        target_depth  =  CC_mission[current_leg_nr].depth;   // [m]   
        target_rpm    =  CC_mission[current_leg_nr].rpm;     // [m]   
      }
      else
     { kill_mission();
       break;
     }
      //hal.console->printf("Current leg: %i\n",current_leg_nr );
      legs_accumulated_time_sec = legs_accumulated_time_sec + CC_mission[current_leg_nr].duration;
    } 
  }
}
//-------------------------------------------------------------------------------
void GPS_mission_manager(){
   mission_ms   = time_ms-mission_start_ms;
   if ((time_ms-mission_update_timer_ms)>1000)  {
     mission_update_timer_ms = time_ms;
     
     //wp_class kth_wp      = { ToRad(18.07205),ToRad(59.34837)}; 
     //wp_class home_wp     = { ToRad(18.26582),ToRad(59.31257)};

     wp_class target_WP  = {GPS_mission[current_leg_nr].lon,  GPS_mission[current_leg_nr].lat};  
     CCD   tmp           =  distance_and_bearing(current_pos, target_WP);
     float ctt           = tmp.course;
     
     struct wp_class wpA;
     if (current_leg_nr==0){wpA=mission_start_pos;} else {wpA.lon = GPS_mission[current_leg_nr-1].lon;wpA.lat = GPS_mission[current_leg_nr-1].lat;}
     ctt = calc_course_to_wp_wrpt_Xtrack_error( wpA, target_WP ,current_pos);
     
     float dtt           = tmp.dist;  // [m]
     //hal.console->printf("Target        %.5f  %.5f \n",ToDeg(target_WP.lon),ToDeg(target_WP.lat) );
     //hal.console->printf("Current pos   %.5f  %.5f \n",ToDeg(current_pos.lon),ToDeg(current_pos.lat) );
     //hal.console->printf("GPS-mission  Leg=%i(%i) CC=%.1f  COG = %.1f,  CTT=%.1fdeg,   DTT=%.1fm  \n ",current_leg_nr,Nlegs_GPS,ToDeg(heading),gps.sog,ToDeg(ctt),dtt );

     if (dtt*1852<GPS_mission[current_leg_nr].wp_radius){ 
       current_leg_nr++; 
       if (current_leg_nr==Nlegs_GPS){kill_mission();return;}
       hal.console->printf("Waypoint reached, now turning to leg  %i(%i)\n",current_leg_nr,Nlegs_GPS);
     }
   
     target_ctt           = ctt;  // [rad] Compass course
     target_dtt           = dtt;  // [rad] Compass course
     target_depth        = GPS_mission[current_leg_nr].depth;   // [m]   
     target_rpm          = GPS_mission[current_leg_nr].rpm;
  }
}
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
void Deviation_mission_manager(){
      //hal.console->printf("Deviation mission: ");
 mission_ms   = time_ms-mission_start_ms;
  if ((time_ms-mission_update_timer_ms)>1000)  {
     mission_update_timer_ms = time_ms;
  
    float mission_sec = (float)mission_ms/1000.0; // [sec] Time since mission start
    target_rpm    = 1600;
    target_depth  = 0.0;
    
    if (mission_sec< 30){
       target_ctt    = 0.0;   
    }
    else if (mission_sec <(30+180)){
      // Right loop
      target_ctt    = (mission_sec-30.0)*2.0*(pi/180.0) ;
    }else if (mission_sec<(30+180+180)){
      // Left loop 
      target_ctt    = (360.0-(mission_sec-30.0-180.0)*2.0)*(pi/180.0) ; 
    }
    else{
      kill_mission();
    }
  }
}

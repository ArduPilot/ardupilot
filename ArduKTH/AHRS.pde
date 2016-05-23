// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//---------------------------------------------------------------------------
//  Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------

void init_AHRS()
{
 hal.console->printf("  Init AHRS:  ");
  compass.init();
  ahrs.set_compass(&compass);
  ins.init(AP_InertialSensor::COLD_START,AP_InertialSensor::RATE_100HZ);
  ahrs.init();
  hal.console->println("  Done :-)");
}

//-------------------------------------------------------------------------------
static void print_AHRS()
{
   hal.console->printf("== cc=%4.1fdeg    roll=%0.1fdeg   pitch= %0.1fdeg \n",
                       ToDeg(heading),ToDeg(roll),ToDeg(pitch));
   hal.console->printf("Acc=%4.2f,%4.2f,%4.2f (norm:%4.2f)  Gyro: %4.3f,%4.3f,%4.3f\n",
                       accel.x, accel.y, accel.z, accel.length(), gyro.x, gyro.y, gyro.z);
}
//-------------------------------------------------------------------------------
static void update_heading(){
   if (abs(time_ms-last_GPS_fix)<5000 && gps.sog>sog_threshold) {
      heading = gps.cog;
    }else{
      heading = cc;  
    }
  //hal.console->printf_P(PSTR("heading= %.1f \n"),ToDeg(heading));
}
//-------------------------------------------------------------------------------

static void update_AHRS()
{
  compass.read();
  ahrs.update();
  roll    = ahrs.roll;    // [rad] Euler angle
  pitch   = ahrs.pitch;   // [rad] Euler angle
  cc      = unwrap_2pi(compass.calculate_heading(ahrs.get_dcm_matrix())); 
  
  //ins.update();             // read samples 
  accel    = ins.get_accel(); // Read accelerations x, y, z
  gyro     = ins.get_gyro();  // Read rates x, y, z
}
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------



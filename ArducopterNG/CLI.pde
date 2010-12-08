/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : CLI.pde
 Version  : v1.0, Oct 18, 2010
 Author(s): ArduCopter Team
 Jani Hirvinen, Jose Julio, Jordi Muñoz,
 Ken McEwans, Roberto Navoni, Sandro Benigno, Chris Anderson, Randy McEvans
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 
 * ************************************************************** *
 ChangeLog:
 - 19-10-10 Initial CLI
 
 * ************************************************************** *
 TODO:
 - Full menu systems, debug, settings
 
 * ************************************************************** */


boolean ShowMainMenu;


// CLI Functions
// This can be moved later to CLI.pde
void RunCLI () {

  //  APM_RC.Init();             // APM Radio initialization

  readUserConfig();          // Read memory values from EEPROM

  ShowMainMenu = TRUE;
  // We need to initialize Serial again due it was not initialized during startup. 
  SerBeg(SerBau);   
  SerPrln();
  SerPrln("Welcome to ArduCopter CLI"); 
  SerPri("Firmware: ");
  SerPrln(VER);
  SerPrln();
  SerPrln("Make sure that you have Carriage Return active");

  if(ShowMainMenu) Show_MainMenu();

  // Our main loop that never ends. Only way to get away from here is to reboot APM
  for (;;) { 

    if(ShowMainMenu) Show_MainMenu();   

    delay(50); 
    if (SerAva()) {
      ShowMainMenu = TRUE;
      queryType = SerRea();
      switch (queryType) {
      case 'c':
        CALIB_CompassOffset();
        break;
      case 'i':
        CALIB_AccOffset();
        break;
      case 't':
        CALIB_Throttle();  
        break;
      case 'e':
        CALIB_Esc();
        break; 
      case 's':
        Show_Settings();
        break;
      }
      SerFlu();
    }

    // Changing LED statuses to inform that we are in CLI mode
    // Blinking Red, Yellow, Green when in CLI mode
    if(millis() - cli_timer > 1000) {
      cli_timer = millis();
      CLILedStep();
    }
  } // Mainloop ends
}




void Show_MainMenu() {
  ShowMainMenu = FALSE; 
  SerPrln();
  SerPrln("CLI Menu - Type your command on command prompt");
  SerPrln("----------------------------------------------");
  SerPrln(" c - Show compass offsets");
  SerPrln(" e - ESC Throttle calibration (Works with official ArduCopter ESCs)");
  SerPrln(" i - Initialize and calibrate Accel offsets");
  SerPrln(" t - Calibrate MIN Throttle value");
  SerPrln(" s - Show settings");
  SerPrln(" ");
  SerFlu();
}


/* ************************************************************** */
// Compass/Magnetometer Offset Calibration
void CALIB_CompassOffset() {

#ifdef IsMAG  
  SerPrln("Rotate/Pitch/Roll your ArduCopter untill offset variables are not");
  SerPrln("anymore changing, write down your offset values and update them ");
  SerPrln("to their correct location.  Starting in..");
  SerPrln("2 secs.");
  delay(1000);
  SerPrln("1 secs.");
  delay(1000);
  SerPrln("starting now....");

  APM_Compass.Init();   // Initialization
  APM_Compass.SetOrientation(MAGORIENTATION);         // set compass's orientation on aircraft
  APM_Compass.SetOffsets(0,0,0);                      // set offsets to account for surrounding interference
  APM_Compass.SetDeclination(ToRad(DECLINATION));  // set local difference between magnetic north and true north
  int counter = 0; 

  SerFlu();
  while(1) {
    static float min[3], max[3], offset[3];
    if((millis()- timer) > 100)  {
      timer = millis();
      APM_Compass.Read();
      APM_Compass.Calculate(0,0);  // roll = 0, pitch = 0 for this example

      // capture min
      if( APM_Compass.Mag_X < min[0] ) min[0] = APM_Compass.Mag_X;
      if( APM_Compass.Mag_Y < min[1] ) min[1] = APM_Compass.Mag_Y;
      if( APM_Compass.Mag_Z < min[2] ) min[2] = APM_Compass.Mag_Z;

      // capture max
      if( APM_Compass.Mag_X > max[0] ) max[0] = APM_Compass.Mag_X;
      if( APM_Compass.Mag_Y > max[1] ) max[1] = APM_Compass.Mag_Y;
      if( APM_Compass.Mag_Z > max[2] ) max[2] = APM_Compass.Mag_Z;

      // calculate offsets
      offset[0] = -(max[0]+min[0])/2;
      offset[1] = -(max[1]+min[1])/2;
      offset[2] = -(max[2]+min[2])/2;

      // display all to user
      SerPri("Heading:");
      SerPri(ToDeg(APM_Compass.Heading));
      SerPri("  \t(");
      SerPri(APM_Compass.Mag_X);
      SerPri(",");
      SerPri(APM_Compass.Mag_Y);
      SerPri(",");    
      SerPri(APM_Compass.Mag_Z);
      SerPri(")");

      // display offsets
      SerPri("\t offsets(");
      SerPri(offset[0]);
      SerPri(",");
      SerPri(offset[1]);
      SerPri(",");
      SerPri(offset[2]);
      SerPri(")");
      SerPrln();

      if(counter == 200) {
        counter = 0;
        SerPrln();
        SerPrln("Roll and Rotate your quad untill offsets are not changing!");
        //        SerPrln("to exit from this loop, reboot your APM");        
        SerPrln();        
        delay(500);
      }
      counter++;
    }
    if(SerAva() > 0){

      mag_offset_x = offset[0];
      mag_offset_y = offset[1];
      mag_offset_z = offset[2];

      SerPriln("Saving Offsets to EEPROM");
      writeEEPROM(mag_offset_x, mag_offset_x_ADR);
      writeEEPROM(mag_offset_y, mag_offset_y_ADR);
      writeEEPROM(mag_offset_z, mag_offset_z_ADR);
      delay(500);
      SerPriln("Saved...");
      SerPrln();
      break;
    }

  }
#else
  SerPrln("Magneto module is not activated on Arducopter.pde");
  SerPrln("Check your program #definitions, upload firmware and try again!!");
  //  SerPrln("Now reboot your APM");
  //  for(;;)
  //    delay(10);
#endif      

}


/* ************************************************************** */
// Accell calibration
void CALIB_AccOffset() {

  int loopy;
  long xx = 0, xy = 0, xz = 0; 

  SerPrln("Initializing Accelerometers..");
  adc.Init();            // APM ADC library initialization
  //  delay(250);                // Giving small moment before starting

  calibrateSensors();         // Calibrate neutral values of gyros  (in Sensors.pde)

  SerPrln();
  SerPrln("Sampling 50 samples from Accelerometers, don't move your ArduCopter!");
  SerPrln("Sample:\tAcc-X\tAxx-Y\tAcc-Z");

  for(loopy = 1; loopy <= 50; loopy++) {
    SerPri("  ");
    SerPri(loopy);
    SerPri(":");
    tab();
    SerPri(xx += adc.Ch(4));
    tab();
    SerPri(xy += adc.Ch(5));
    tab();
    SerPrln(xz += adc.Ch(6));
    delay(20);
  }

  xx = xx / (loopy - 1);
  xy = xy / (loopy - 1);
  xz = xz / (loopy - 1);
  xz = xz - 407;               // Z-Axis correction
  //  xx += 42;

  acc_offset_y = xy;
  acc_offset_x = xx;
  acc_offset_z = xz;

  AN_OFFSET[3] = acc_offset_x;
  AN_OFFSET[4] = acc_offset_y;
  AN_OFFSET[5] = acc_offset_z;

  SerPrln();
  SerPrln("Offsets as follows: ");  
  SerPri("  ");
  tab();
  SerPri(acc_offset_x);
  tab();
  SerPri(acc_offset_y);
  tab();
  SerPrln(acc_offset_z);

  SerPrln("Final results as follows: ");  
  SerPri("  ");
  tab();
  SerPri(adc.Ch(4) - acc_offset_x);
  tab();
  SerPri(adc.Ch(5) - acc_offset_y);
  tab();
  SerPrln(adc.Ch(6) - acc_offset_z);
  SerPrln();
  SerPrln("Final results should be close to 0, 0, 408 if they are far (-+10) from ");
  SerPrln("those values, redo initialization or use Configurator for finetuning");
  SerPrln();
  SerPrln("Saving values to EEPROM");
  writeEEPROM(acc_offset_x, acc_offset_x_ADR);
  writeEEPROM(acc_offset_y, acc_offset_y_ADR);
  writeEEPROM(acc_offset_z, acc_offset_z_ADR);
  delay(200);
  SerPrln("Saved..");
  SerPrln();
}



void CALIB_Throttle() {
  int tmpThrottle = 1100;

  SerPrln("Move your throttle to MIN, reading starts in 3 seconds");
  delay(1000);
  SerPrln("2. ");
  delay(1000);
  SerPrln("1. ");
  delay(1000);
  SerPrln("Reading Throttle value, hit enter to exit");

  SerFlu();
  while(1) {
    ch_throttle = APM_RC.InputCh(CH_THROTTLE);
    SerPri("Throttle channel: ");
    SerPrln(ch_throttle);
    if(tmpThrottle > ch_throttle) tmpThrottle = ch_throttle;
    delay(50);
    if(SerAva() > 0){
      break; 
    }
  }

  SerPriln();
  SerPri("Lowest throttle value: ");
  SerPrln(tmpThrottle);
  SerPrln();
  SerPrln("Saving MIN_THROTTLE to EEPROM");
  writeEEPROM(tmpThrottle, MIN_THROTTLE_ADR);
  delay(200);
  SerPrln("Saved..");
  SerPrln();
}


void CALIB_Esc() {

  SerPrln("Disconnect your battery if you have it connected, keep your USB cable/Xbee connected!");
  SerPrln("After battery is disconnected hit enter key and wait more instructions:");
  SerPrln("As safety measure, unmount all your propellers before continuing!!");

  WaitSerialEnter();      

  SerPrln("Move your Throttle to maximum and connect your battery. ");
  SerPrln("after you hear beep beep tone, move your throttle to minimum and");
  SerPrln("hit enter after you are ready to disarm motors.");
  SerPrln("Arming now all motors");
  delay(500);
  SerPrln("Motors: ARMED");
  delay(200);
  SerPrln("Connect your battery and let ESCs to reboot!");
  while(1) {
    ch_throttle = APM_RC.InputCh(CH_THROTTLE);  
    APM_RC.OutputCh(0, ch_throttle);
    APM_RC.OutputCh(1, ch_throttle);
    APM_RC.OutputCh(2, ch_throttle);
    APM_RC.OutputCh(3, ch_throttle);

    // InstantPWM => Force inmediate output on PWM signals
    APM_RC.Force_Out0_Out1();
    APM_RC.Force_Out2_Out3();
    delay(20);
    if(SerAva() > 0){
      break; 
    }
  }

  APM_RC.OutputCh(0, 900);
  APM_RC.OutputCh(1, 900);
  APM_RC.OutputCh(2, 900);
  APM_RC.OutputCh(3, 90);
  APM_RC.Force_Out0_Out1();
  APM_RC.Force_Out2_Out3();

  SerPrln("Motors: DISARMED");
  SerPrln();
}


void Show_Settings() {
  // Reading current EEPROM values
  
  SerPrln("ArduCopter - Current settings");
  SerPrln("-----------------------------");
  SerPri("Firmware: ");
  SerPri(VER);
  SerPrln();
  SerPrln();
  
  readUserConfig();
  delay(50);

  SerPri("Magnetom. offsets (x,y,z): ");
  SerPri(mag_offset_x);
  cspc();
  SerPri(mag_offset_y);
  cspc();
  SerPri(mag_offset_z);
  SerPrln();

  SerPri("Accel offsets (x,y,z): ");
  SerPri(acc_offset_x);
  cspc();
  SerPri(acc_offset_y);
  cspc();
  SerPri(acc_offset_z);
  SerPrln();

  SerPri("Min Throttle: ");
  SerPriln(MIN_THROTTLE);

  SerPri("Magnetometer 1-ena/0-dis: ");
  SerPriln(MAGNETOMETER, DEC);

  SerPri("Camera mode: ");
  SerPriln(cam_mode, DEC);

  SerPrln();
  SerPrln();

}


void cspc() {
  SerPri(", ");
}

void WaitSerialEnter() {
  // Flush serials
  SerFlu();
  delay(50);
  while(1) {
    if(SerAva() > 0){
      break; 
    }
    delay(20);
  }
  delay(250);
  SerFlu();  
}





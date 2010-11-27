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

  if(ShowMainMenu) Show_MainMenu();

  // Our main loop that never ends. Only way to get away from here is to reboot APM
  for (;;) { 
    if(ShowMainMenu) Show_MainMenu();    
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
      }
    }
  } // Mainloop ends

}

void Show_MainMenu() {
  ShowMainMenu = FALSE; 
  SerPrln("CLI Menu - Type your command on command prompt");
  SerPrln("----------------------------------------------");
  SerPrln(" c - Show compass offsets (no return, reboot)");
  SerPrln(" i - Initialize and calibrate Accel offsets");
  SerPrln(" ");
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

  for(;;) {
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

      if(counter == 50) {
        counter = 0;
        SerPrln();
        SerPrln("Roll and Rotate your quad untill offsets are not changing!");
        SerPrln("to exit from this loop, reboot your APM");        
        SerPrln();        
        delay(500);
      }
      counter++;
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

  uint8_t loopy;
  uint16_t xx = 0, xy = 0, xz = 0; 

  adc.Init();            // APM ADC library initialization
//  delay(250);                // Giving small moment before starting

  calibrateSensors();         // Calibrate neutral values of gyros  (in Sensors.pde)

  SerPrln();
  SerPrln("Sampling 10 samples from Accelerometers, don't move your ArduCopter!");
  SerPrln("Sample:\tAcc-X\tAxx-Y\tAcc-Z");

  for(loopy = 1; loopy <= 5; loopy++) {
    SerPri("  ");
    SerPri(loopy);
    SerPri(":");
    tab();
/*    SerPri(xx += read_adc(4));
    tab();
    SerPri(xy += -read_adc(3));
    tab();
    SerPrln(xz += read_adc(5));
*/
    SerPri(xx += adc.Ch(4));
    tab();
    SerPri(xy += adc.Ch(5));
    tab();
    SerPrln(xz += adc.Ch(3));



  }
  
  xx = xx / (loopy - 1);
  xy = xy / (loopy - 1);
  xz = xz / (loopy - 1) ;
  
  SerPriln("Averages as follows");
  SerPri("  ");
  tab();
  SerPri(xx);
  tab();
  SerPri(xy);
  tab();
  SerPri(xz);
  SerPriln();



}



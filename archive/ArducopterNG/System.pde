/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : System.pde
 Version  : v1.0, Aug 27, 2010
 Author(s): ArduCopter Team
 Ted Carancho (aeroquad), Jose Julio, Jordi Muñoz,
 Jani Hirvinen, Ken McEwans, Roberto Navoni,          
 Sandro Benigno, Chris Anderson
 
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
 
 
 * ************************************************************** *
 TODO:
 
 
 * ************************************************************** */

// General Initialization for all APM electronics
void APM_Init() {

  // Setup proper PIN modes for our switched, LEDs, Relays etc on IMU Board
  pinMode(LED_Yellow,OUTPUT); // Yellow LED A  (PC1)
  pinMode(LED_Red,OUTPUT);    // Red LED    B  (PC2)
  pinMode(LED_Green,OUTPUT);  // Green LED  C  (PC0)
  pinMode(RELAY,OUTPUT);      // Relay output  (PL2)
  pinMode(SW1,INPUT);         // Switch SW1    (PG0)
  pinMode(SW2,INPUT);         // Switch SW2    (PG1)

  // Because DDRE and DDRL Ports are not included to normal Arduino Libraries, we need to
  // initialize them with a special command
  APMPinMode(DDRE,7,INPUT);   // DIP1, (PE7), Closest DIP to sliding SW2 switch
  APMPinMode(DDRE,6,INPUT);   // DIP2, (PE6)
  APMPinMode(DDRL,6,INPUT);   // DIP3, (PL6)
  APMPinMode(DDRL,7,INPUT);   // DIP4, (PL7), Furthest DIP from sliding SW2 switch


  /* ********************************************************* */
  ///////////////////////////////////////////////////////// 
  // Normal Initialization sequence starts from here.
  readUserConfig();          // Load user configurable items from EEPROM

  APM_RC.Init();             // APM Radio initialization

#if AIRFRAME == QUAD
  // RC channels Initialization (Quad motors)  
  APM_RC.OutputCh(0,MIN_THROTTLE);  // Motors stoped
  APM_RC.OutputCh(1,MIN_THROTTLE);
  APM_RC.OutputCh(2,MIN_THROTTLE);
  APM_RC.OutputCh(3,MIN_THROTTLE);
#endif  

#if AIRFRAME == HELI
  // RC channels Initialization (heli servos)  
  APM_RC.OutputCh(0,CHANN_CENTER);  // mid position
  APM_RC.OutputCh(1,CHANN_CENTER);
  APM_RC.OutputCh(2,CHANN_CENTER);
  APM_RC.OutputCh(3,CHANN_CENTER);
#endif 
  // Make sure that Relay is switched off.
  digitalWrite(RELAY,LOW);

  // Wiggle LEDs while ESCs are rebooting  
  FullBlink(50,20);

  adc.Init();            // APM ADC library initialization
  DataFlash.Init();          // DataFlash log initialization

#ifdef IsGPS  
  GPS.Init();                // GPS Initialization

#ifdef IsNEWMTEK  
  delay(250);

  // DIY Drones MTEK GPS needs binary sentences activated if you upgraded to latest firmware.
  // If your GPS shows solid blue but LED C (Red) does not go on, your GPS is on NMEA mode
  Serial1.print("$PMTK220,200*2C\r\n");          // 5Hz update rate
  delay(200);
  Serial1.print("$PGCMD,16,0,0,0,0,0*6A\r\n"); 

#endif
#endif

  // Read DIP Switches and other important values. DIP switches needs special functions to 
  // read due they are not defined as normal pins like other GPIO's are. 
  SW_DIP1 = APMPinRead(PINE, 7);
  SW_DIP2 = APMPinRead(PINE, 6);
  SW_DIP3 = APMPinRead(PINL, 6);
  SW_DIP4 = APMPinRead(PINL, 7);

  // Is CLI mode active or not, if it is fire it up and never return.
  if(!digitalRead(SW2)) {
    RunCLI();
    // Btw.. We never return from this....
  }


  flightOrientation = SW_DIP1;    // DIP1 off = we are in + mode, DIP1 on = we are in x mode

  // readUserConfig moved to up to ensure min throttle is read from eeprom 
  //readUserConfig();               // Load user configurable items from EEPROM 

  // Safety measure for Channel mids
  if(roll_mid < 1400 || roll_mid > 1600) roll_mid = 1500;
  if(pitch_mid < 1400 || pitch_mid > 1600) pitch_mid = 1500;
  if(yaw_mid < 1400 || yaw_mid > 1600) yaw_mid = 1500;

#if AIRFRAME == QUAD
  // RC channels Initialization (Quad motors)  
  APM_RC.OutputCh(0,MIN_THROTTLE);  // Motors stoped
  APM_RC.OutputCh(1,MIN_THROTTLE);
  APM_RC.OutputCh(2,MIN_THROTTLE);
  APM_RC.OutputCh(3,MIN_THROTTLE);
#endif  

#if AIRFRAME == HELI
  // RC channels Initialization (heli servos)  
  APM_RC.OutputCh(0,CHANN_CENTER);  // mid position
  APM_RC.OutputCh(1,CHANN_CENTER);
  APM_RC.OutputCh(2,CHANN_CENTER);
  APM_RC.OutputCh(3,CHANN_CENTER);
#endif 

  // Initialise Wire library used by Magnetometer and Barometer
  Wire.begin();

#ifdef IsMAG
  if (MAGNETOMETER == 1) {
    AP_Compass.init(FALSE);  // I2C initialization
    AP_Compass.set_orientation(MAGORIENTATION);
    AP_Compass.set_offsets(mag_offset_x, mag_offset_y, mag_offset_z);
    AP_Compass.set_declination(ToRad(DECLINATION));
  }
#endif

  DataFlash.StartWrite(1);   // Start a write session on page 1

  // Proper Serial port/baud are defined on main .pde and then Arducopter.h with
  // Choises of Xbee or normal serial port
  SerBeg(SerBau);

  // Check if we enable the DataFlash log Read Mode (switch)
  // If we press switch 1 at startup we read the Dataflash eeprom
  while (digitalRead(SW1)==0)  // LEGACY remove soon by jp, 30-10-10
  {
    Serial.println("Entering Log Read Mode...");    // This will be obsole soon due moving to CLI system
    Log_Read(1,1000);
    delay(30000);
  }

  calibrateSensors();         // Calibrate neutral values of gyros  (in Sensors.pde)

  //  Neutro_yaw = APM_RC.InputCh(3); // Take yaw neutral radio value
#ifndef CONFIGURATOR
  for(i=0;i<6;i++)
  {
    SerPri("AN[]:");
    SerPrln(AN_OFFSET[i]);
  }
  SerPri("Yaw neutral value:");
  SerPri(yaw_mid);
#endif

#ifdef UseBMP
  APM_BMP085.Init(FALSE);
#endif

  // Sonar for Altitude hold
#ifdef IsSONAR
  AP_RangeFinder_down.init(AP_RANGEFINDER_PITOT_TUBE, &adc);  AP_RangeFinder_down.set_orientation(AP_RANGEFINDER_ORIENTATION_DOWN);
  //AP_RangeFinder_down.init(AN5);  AP_RangeFinder_down.set_orientation(AP_RANGEFINDER_ORIENTATION_DOWN);
  sonar_threshold = AP_RangeFinder_down.max_distance * 0.8;
  sonar_status = SONAR_STATUS_OK;  // assume sonar is ok to start with
#endif

  // RangeFinders for obstacle avoidance
#ifdef IsRANGEFINDER  
  AP_RangeFinder_frontRight.init(AN5);  AP_RangeFinder_frontRight.set_orientation(AP_RANGEFINDER_ORIENTATION_FRONT_RIGHT);
  AP_RangeFinder_backRight.init(AN4);  AP_RangeFinder_backRight.set_orientation(AP_RANGEFINDER_ORIENTATION_BACK_RIGHT);
  AP_RangeFinder_backLeft.init(AN3);  AP_RangeFinder_backLeft.set_orientation(AP_RANGEFINDER_ORIENTATION_BACK_LEFT);
  AP_RangeFinder_frontLeft.init(AN2);  AP_RangeFinder_frontLeft.set_orientation(AP_RANGEFINDER_ORIENTATION_FRONT_LEFT);
#endif

  delay(1000);

  DataFlash.StartWrite(1);   // Start a write session on page 1

  // initialise helicopter
#if AIRFRAME == HELI
  heli_setup();
#endif

#ifdef IsAM
  // Switch Left & Right lights on
  digitalWrite(RI_LED, HIGH);
  digitalWrite(LE_LED, HIGH); 
#endif

}



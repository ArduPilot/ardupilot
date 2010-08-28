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
  
  int i, j;                  // Temporary variables used to count things
  float aux_float[3];

  pinMode(LED_Yellow,OUTPUT); //Yellow LED A  (PC1)
  pinMode(LED_Red,OUTPUT);    //Red LED B     (PC2)
  pinMode(LED_Green,OUTPUT);  //Green LED C   (PC0)

  pinMode(SW1_pin,INPUT);     //Switch SW1 (pin PG0)

  pinMode(RELE_pin,OUTPUT);   // Rele output
 
  digitalWrite(RELE_pin,LOW);

  //  delay(1000); // Wait until frame is not moving after initial power cord has connected
  FullBlink(50,20);


  APM_RC.Init();             // APM Radio initialization
  APM_ADC.Init();            // APM ADC library initialization
  DataFlash.Init();          // DataFlash log initialization

#ifdef IsGPS  
  GPS.Init();                // GPS Initialization
#ifdef IsNEWMTEK  
  delay(250);
  // DIY Drones MTEK GPS needs binary sentences activated if you upgraded to latest firmware.
  // If your GPS shows solid blue but LED C (Red) does not go on, your GPS is on NMEA mode
  Serial1.print("$PGCMD,16,0,0,0,0,0*6A\r\n"); 
#endif
#endif

  readUserConfig(); // Load user configurable items from EEPROM

  // Safety measure for Channel mids
  if(roll_mid < 1400 || roll_mid > 1600) roll_mid = 1500;
  if(pitch_mid < 1400 || pitch_mid > 1600) pitch_mid = 1500;
  if(yaw_mid < 1400 || yaw_mid > 1600) yaw_mid = 1500;

  // RC channels Initialization (Quad motors)  
  APM_RC.OutputCh(0,MIN_THROTTLE);  // Motors stoped
  APM_RC.OutputCh(1,MIN_THROTTLE);
  APM_RC.OutputCh(2,MIN_THROTTLE);
  APM_RC.OutputCh(3,MIN_THROTTLE);

  if (MAGNETOMETER == 1)
    APM_Compass.Init();  // I2C initialization

  DataFlash.StartWrite(1);   // Start a write session on page 1

  Serial.begin(115200);
  //Serial.println("ArduCopter Quadcopter v1.0");

  // Check if we enable the DataFlash log Read Mode (switch)
  // If we press switch 1 at startup we read the Dataflash eeprom
  while (digitalRead(SW1_pin)==0)
  {
    Serial.println("Entering Log Read Mode...");
    Log_Read(1,1000);
    delay(30000);
  }

  Read_adc_raw();
  delay(20);

  // Offset values for accels and gyros...
  AN_OFFSET[3] = acc_offset_x;
  AN_OFFSET[4] = acc_offset_y;
  AN_OFFSET[5] = acc_offset_z;
  aux_float[0] = gyro_offset_roll;
  aux_float[1] = gyro_offset_pitch;
  aux_float[2] = gyro_offset_yaw;

  j = 0;
  // Take the gyro offset values
  for(i=0;i<300;i++)
  {
    Read_adc_raw();
    for(int y=0; y<=2; y++)   // Read initial ADC values for gyro offset.
    {
      aux_float[y]=aux_float[y]*0.8 + AN[y]*0.2;
      //Serial.print(AN[y]);
      //Serial.print(",");
    }
    //Serial.println();
    Log_Write_Sensor(AN[0],AN[1],AN[2],AN[3],AN[4],AN[5],ch_throttle);
    delay(10);
    
    RunningLights(j);
    // Runnings lights effect to let user know that we are taking mesurements
    if((i % 5) == 0) j++;
    if(j >= 3) j = 0;
  }
  
  // Switch off all ABC lights
  LightsOff();

  for(int y=0; y<=2; y++)   
    AN_OFFSET[y]=aux_float[y];

  //  Neutro_yaw = APM_RC.InputCh(3); // Take yaw neutral radio value
#ifndef CONFIGURATOR
  for(i=0;i<6;i++)
  {
    Serial.print("AN[]:");
    Serial.println(AN_OFFSET[i]);
  }
  Serial.print("Yaw neutral value:");
  //  Serial.println(Neutro_yaw);
  Serial.print(yaw_mid);
#endif

#ifdef RADIO_TEST_MODE    // RADIO TEST MODE TO TEST RADIO CHANNELS
  while(1)
  {
    if (APM_RC.GetState()==1)
    {
      Serial.print("AIL:");
      Serial.print(APM_RC.InputCh(0));
      Serial.print("ELE:");
      Serial.print(APM_RC.InputCh(1));
      Serial.print("THR:");
      Serial.print(APM_RC.InputCh(2));
      Serial.print("YAW:");
      Serial.print(APM_RC.InputCh(3));
      Serial.print("AUX(mode):");
      Serial.print(APM_RC.InputCh(4));
      Serial.print("AUX2:");
      Serial.print(APM_RC.InputCh(5));
      Serial.println();
      delay(200);
    }
  } 
#endif  

  delay(1000);

  DataFlash.StartWrite(1);   // Start a write session on page 1
  timer = millis();
  tlmTimer = millis();
  Read_adc_raw();        // Initialize ADC readings...
  delay(20);

#ifdef IsAM
  // Switch Left & Right lights on
  digitalWrite(RI_LED, HIGH);
  digitalWrite(LE_LED, HIGH); 
#endif

  motorArmed = 0;
  digitalWrite(LED_Green,HIGH);     // Ready to go...
}


void resetPerfData(void) {
	mainLoop_count = 0;
	G_Dt_max = 0;
	gyro_sat_count = 0;
	adc_constraints = 0;
	renorm_sqrt_count = 0;  
	renorm_blowup_count = 0;
	gps_fix_count = 0;
	perf_mon_timer = millis();
}


